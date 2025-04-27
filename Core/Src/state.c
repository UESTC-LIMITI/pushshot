#include "user.h"

USART_info_t UART7_info = {.USART_handle = UART7, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 0},
             UART5_info = {.USART_handle = UART5, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream1, .DMA_ID = 1};

enum STATE state;
struct STATE_R state_R = {
    .fitting = 0};
struct STATE_W state_W;
timer_t runtime;

#define nDATA_OUTPUT

#ifdef DATA_OUTPUT
#include <stdio.h>
unsigned char VOFA[32];
#endif

struct
{
    struct
    {
        float timeout, spd;
    } back;
    struct
    {
        float acc_curr_pct, spd, spd_ctrl_pct, brake_curr_pct, timeout, brake_time;
    } shot;
} VESC_param = {
    .back.timeout = 2,
    .back.spd = -150,

    .shot.acc_curr_pct = 0.1,
    .shot.spd = 810,
    .shot.spd_ctrl_pct = 0.9,
    .shot.brake_curr_pct = 0.05,
    .shot.timeout = 1.5,
    .shot.brake_time = 0.25};

struct
{
    float pos_0, gain;
} HighTorque_param = {
    .pos_0 = -2,
    .gain = 1};

// gimbal limit
#define YAW_MIN (-115 + HighTorque_param.pos_0)
#define YAW_MAX (115 + HighTorque_param.pos_0)

#define Gimbal_GR (14.45f * HighTorque_param.gain)

struct target_info basket_info = {.dist_fltr.size = 2},
                   R2_info = {.dist_fltr.size = 4};

struct pos_info R1_pos_lidar, R1_pos_chassis, R2_pos;

timer_t HighTorque_time, gimbal_time;

float yaw_prev;
float basket_dist_offset = 4,
      R2_dist_offset = 0;

MovAvgFltr_t yaw_fltr;

float Fitting_Calc_Basket(float dist_cm)
{
    return 0.9125 * dist_cm +
           596.25;
}

float Fitting_Calc_R2(float dist_cm)
{
    return 0;
}

void State(void *argument)
{
    GPIOD->ODR |= 0x80; // fan for wireless bridge

    // default param
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.pos_0;
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.Kp = 2;
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.Kd = 1;

    while (1)
    {
#ifdef DATA_OUTPUT
        if (state == SHOT && !state_R.brake)
        {
            sprintf((char *)VOFA, "T:%.2f\n", VESC[1 - VESC_ID_OFFSET].fdbk.spd);
            UART_SendArray(&UART7_info, VOFA, 12);
        }
#endif
        switch (state)
        {
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
            if (Timer_CheckTimeout(&runtime, VESC_param.back.timeout) || // timeout
                GPIOE->IDR & 0x4)                                        // bottom reached
            {
                Timer_Clear(&runtime);
                state = IDLE;
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
                                                                                      : VESC_param.shot.acc_curr_pct); // curr for acceleration

            LIMIT_RANGE(VESC[1 - VESC_ID_OFFSET].ctrl.curr, 50, 120); // ESC current limit

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
        }

        // delay after action
        if (Timer_CheckTimeout(&HighTorque_time, 0.25))
            HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.pos_0 + (state_W.ball
                                                                                          // aim at R2
                                                                                          ? (state_W.aim_R2 ? yaw_prev + (R2_info.yaw - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / 25.f)
                                                                                                            // aim at basket when close to
                                                                                                            : (basket_info.dist_cm <= 900 ? (yaw_prev + (basket_info.yaw - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / (err.basket_lidar && err.pos_lidar ? (err.basket_camera ? 500.f // chassis
                                                                                                                                                                                                                                                                                : 30.f) // camera
                                                                                                                                                                                                                                                           : 10.f)))                    // lidar
                                                                                                                                          // stay at middle when far from
                                                                                                                                          : 0)) *
                                                                                                Gimbal_GR
                                                                                          // stay at middle
                                                                                          : 0);

        LIMIT_RANGE(HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MIN, YAW_MAX); // gimbal limit

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, 2);
        }

        if (state_W.ball &&
            (state_W.aim_R2 ? MovAvgFltr_GetNewStatus(&R2_info.dist_fltr, R2_info.dist_cm, 2.5)             // position ready for R2
                            : MovAvgFltr_GetNewStatus(&basket_info.dist_fltr, basket_info.dist_cm, 2.5)) && // position ready for basket
            MovAvgFltr_GetNewTargetStatus(&yaw_fltr, HighTorque[2 - HIGHTORQUE_ID_OFFSET].fdbk.spd, 0, 10)) // yaw ready
            state_R.shot_ready = 1;
        else if (state != SHOT)
            state_R.shot_ready = 0;

        osDelay(1);
    }
}