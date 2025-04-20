#include "user.h"

#include <stdio.h>

USART_info_t UART7_info = {.USART_handle = UART7, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 0},
             UART5_info = {.USART_handle = UART5, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream1, .DMA_ID = 1};

enum STATE state = IDLE;
struct STATE_R state_R = {
    .fitting = 1};
struct STATE_W state_W;
timer_t runtime;

#define nDATA_OUTPUT

#ifdef DATA_OUTPUT
unsigned char VOFA[32];
#endif

struct
{
    struct
    {
        float timeout, spd;
    } init;
    struct
    {
        float curr, time;
    } lock;
    struct
    {
        float timeout, spd;
    } back;
    struct
    {
        float curr_pct, spd, spd_ctrl_pct, brake_curr_pct, timeout, brake_time;
    } shot;
} VESC_param = {
    .init.timeout = 3,
    .init.spd = -200,

    .lock.curr = -5,
    .lock.time = 1,

    .back.timeout = 2,
    .back.spd = -150,

    .shot.curr_pct = 0.1,
    .shot.spd = 810,
    .shot.spd_ctrl_pct = 0.9, // 0.9375
    .shot.brake_curr_pct = 0,
    .shot.timeout = 0.5,
    .shot.brake_time = 0.2};

struct
{
    float pos_0, gain;
} HighTorque_param = {
    .pos_0 = 0,
    .gain = 1};

// gimbal limit
#define YAW_MIN -160
#define YAW_MAX 160

#define Gimbal_GR (11 * HighTorque_param.gain)

struct target_info basket_info = {.dist_fltr.size = 8, .yaw_fltr.size = 8},
                   R2_info = {.dist_fltr.size = 16, .yaw_fltr.size = 16};

struct pos_info R1_pos_lidar, R1_pos_chassis, R2_pos;

timer_t HighTorque_time, gimbal_time;

float R2_yaw_prev, basket_yaw_prev;
float basket_dist_offset = -5,
      R2_dist_offset = 0;

MovAvgFltr_t yaw_fltr;

float Fitting_Calc_Basket(float dist_cm)
{
    return -3.269900772135022e-10 * pow(dist_cm, 5) +
           6.354993084822791e-7 * pow(dist_cm, 4) -
           0.00047688496297837446 * pow(dist_cm, 3) +
           0.17250630300259218 * pow(dist_cm, 2) -
           29.232875826768577 * dist_cm + 2515.470017191885;
}

float Fitting_Calc_R2(float dist_cm)
{
    return 0;
}

void State(void *argument)
{
    GPIOD->ODR |= 0x80; // fan for wireless bridge

    // default param
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos = -HighTorque_param.pos_0;
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.Kp = 2;
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.Kd = 1;

    while (1)
    {
#ifdef DATA_OUTPUT
        if (VESC[1 - VESC_ID_OFFSET].fdbk.spd > 10 && !state_R.brake)
        {
            sprintf((char *)VOFA, "T:%.2f\n", VESC[1 - VESC_ID_OFFSET].fdbk.spd);
            UART_SendArray(&UART7_info, VOFA, 12);
        }
#endif
        switch (state)
        {
        // spin to top
        case INIT:
        {
            // timeout
            if (Timer_CheckTimeout(&runtime, VESC_param.init.timeout))
            {
                Timer_Clear(&runtime);
                state = IDLE;
                break;
            }

            static MovAvgFltr_t VESC_fltr;

            // ball plate detected
            if (MovAvgFltr(&VESC_fltr, VESC[1 - VESC_ID_OFFSET].fdbk.curr <= 50 ? VESC[1 - VESC_ID_OFFSET].fdbk.curr : 0) >= 20 && // current large enough
                runtime.intvl >= 0.125)                                                                                            // time long enough
            {
                MovAvgFltr_Clear(&VESC_fltr);
                Timer_Clear(&runtime);
                state = IDLE;
                break;
            }
            // go down
            else
            {
                VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.init.spd;
            }

            // control
            {
                VESC_SendCmd(&hfdcan2, 1, VESC_SET_SPD, &HOBBYWING_V9626_KV160);
            }
            break;
        }
        case IDLE:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.curr = 0;

            // control
            {
                VESC_SendCmd(&hfdcan2, 1, VESC_SET_CURR, &HOBBYWING_V9626_KV160);
            }
            break;
        }
        // spin to bottom
        case BACK:
        {
            // timeout
            if (Timer_CheckTimeout(&runtime, VESC_param.back.timeout))
            {
                Timer_Clear(&runtime);
                state = IDLE;
                break;
            }

            // bottom reached
            if (GPIOE->IDR & 0x4)
            {
                Timer_Clear(&runtime);
                state = LOCK;
                break;
            }
            // go down
            else
                VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.back.spd;

            // control
            {
                VESC_SendCmd(&hfdcan2, 1, VESC_SET_SPD, &HOBBYWING_V9626_KV160);
            }
            break;
        }
        // stay at pos, wait for ball
        case LOCK:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.curr = VESC_param.lock.curr;

            // time limit
            if (Timer_CheckTimeout(&runtime, VESC_param.lock.time))
            {
                Timer_Clear(&runtime);
                state = IDLE;
                break;
            }

            // control
            {
                VESC_SendCmd(&hfdcan2, 1, VESC_SET_CURR, &HOBBYWING_V9626_KV160);
            }
            break;
        }
        case SHOT:
        {
            // not ready for shot
            if (!state_R.shot_ready)
            {
                state = IDLE;
                break;
            }

            // timeout
            if (Timer_CheckTimeout(&runtime, VESC_param.shot.timeout))
            {
                state_R.brake = 1;

                if (runtime.intvl >= VESC_param.shot.timeout + VESC_param.shot.brake_time) // total duration
                {
                    basket_info.yaw = state_R.shot_ready = state_W.ball = state_R.brake = state_R.spd_ctrl = 0;
                    Timer_Clear(&runtime);
                    Timer_Clear(&HighTorque_time);
                    state = IDLE;
                    break;
                }
            }

            if (state_R.fitting)
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2(R2_info.dist_cm + R2_dist_offset)
                                                     : Fitting_Calc_Basket(basket_info.dist_cm + basket_dist_offset);

            LIMIT(VESC_param.shot.spd, HOBBYWING_V9626_KV160.spd_max);

            VESC[1 - VESC_ID_OFFSET].ctrl.curr = VESC_param.shot.spd * (state_R.brake ? VESC_param.shot.brake_curr_pct // curr for brake
                                                                                      : VESC_param.shot.curr_pct);     // curr for acceleration

            LIMIT_RANGE(VESC[1 - VESC_ID_OFFSET].ctrl.curr, 120, 50); // ESC current limit

            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.shot.spd; // target speed

            // switch control mode
            if (VESC[1 - VESC_ID_OFFSET].fdbk.spd / VESC_param.shot.spd >= VESC_param.shot.spd_ctrl_pct)
                state_R.spd_ctrl = 1;

            // control
            {
                if (state_R.brake)
                    VESC_SendCmd(&hfdcan2, 1, VESC_SET_CURR_BRAKE, &HOBBYWING_V9626_KV160);
                else if (state_R.spd_ctrl)
                    VESC_SendCmd(&hfdcan2, 1, VESC_SET_SPD, &HOBBYWING_V9626_KV160);
                else
                    VESC_SendCmd(&hfdcan2, 1, VESC_SET_CURR, &HOBBYWING_V9626_KV160);
            }
            break;
        }
        // test only
        case READY:
        {
            state_W.ball = 1;
            state = BACK;
            break;
        }
        }

        // delay after shot
        if (Timer_CheckTimeout(&HighTorque_time, 0.1))
            HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos = -HighTorque_param.pos_0 - (state_W.ball
                                                                                           // aim at R2
                                                                                           ? (state_W.aim_R2 ? R2_yaw_prev + (R2_info.yaw - R2_yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / 25.f)
                                                                                                             // aim at basket when close to
                                                                                                             // yaw from chassis, no smooth compensation
                                                                                                             : (basket_info.dist_cm <= 900 ? (err.basket_camera && err.basket_lidar && err.pos_lidar ? basket_info.yaw
                                                                                                                                                                                                     // yaw from camera or lidar, smooth compensation
                                                                                                                                                                                                     : basket_yaw_prev + (basket_info.yaw - basket_yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / (err.basket_camera ? 10.f : 30.f)))
                                                                                                                                           // stay at middle when far from
                                                                                                                                           : 0)) *
                                                                                                 Gimbal_GR
                                                                                           // stay at middle
                                                                                           : 0);

        LIMIT_RANGE(HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MAX, YAW_MIN); // gimbal limit

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, 2);
        }

        if (state_W.ball &&
            (state_W.aim_R2 ? MovAvgFltr_GetStatus(&R2_info.dist_fltr, 5.f / 100)                          // position ready for R2
                            : MovAvgFltr_GetStatus(&basket_info.dist_fltr, 5.f / 100)) &&                  // position ready for basket
            MovAvgFltr_GetNewTargetStatus(&yaw_fltr, HighTorque[2 - HIGHTORQUE_ID_OFFSET].fdbk.spd, 0, 8)) // yaw ready
            state_R.shot_ready = 1;
        else if (state != SHOT)
            state_R.shot_ready = 0;

        osDelay(1);
    }
}