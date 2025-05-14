#include "user.h"

USART_info_t UART7_info = {.USART_handle = UART7, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 0}, // USB to TTL
    UART5_info = {.USART_handle = UART5, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream1, .DMA_ID = 1};          // dual robot communication Tx

enum STATE state;
struct STATE_R state_R = { // internal-change state
    .fitting = 0};
struct STATE_W state_W; // external-change state
timer_t runtime;

#define DATA_OUTPUT

#ifdef DATA_OUTPUT
#include <stdio.h>
unsigned char VOFA[32];
#endif

struct
{
    struct
    {
        float timeout, curr_detect, fltr_time, spd;
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
        float acc_curr_pct, spd, spd_ctrl_pct, brake_curr_pct, timeout, brake_time;
    } shot;
} VESC_param = {
    .init.timeout = 5,
    .init.curr_detect = 5,
    .init.fltr_time = 0.125,
    .init.spd = -250,

    .lock.curr = -5,
    .lock.time = 1,

    .back.timeout = 2,
    .back.spd = -250,

    .shot.acc_curr_pct = 0.1,
    .shot.spd = 1000,
    .shot.spd_ctrl_pct = 0.8,
    .shot.brake_curr_pct = 0.03125,
    .shot.timeout = 0.5,
    .shot.brake_time = 0.25};

struct
{
    float basket_pos_0, R2_pos_0;
} HighTorque_param = {
    .basket_pos_0 = (YAW_MAX + YAW_MIN) / 2 + 0,
    .R2_pos_0 = (YAW_MAX + YAW_MIN) / 2 + 0};

struct pos_t R1_pos_lidar, R1_pos_chassis, R2_pos, basket_pos = {.x = 14.05, .y = -4};

struct target_info basket_info,
    R2_info = {.dist_fltr.size = 8};

timer_t HighTorque_time, gimbal_time;

float yaw_prev, yaw_curr = (YAW_MAX + YAW_MIN) / 2;

float basket_spd_offset = 0,
      R2_spd_offset = 0;

MovAvgFltr_t yaw_fltr;

float Fitting_Calc_Basket(float dist_cm)
{
    return 0;
}

float Fitting_Calc_R2(float dist_cm)
{
    return 0;
}

void State(void *argument)
{
    // fan for lidar
    GPIOG->ODR |= 0x400;

    // default param
    HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = (YAW_MIN + YAW_MAX) / 2;
    HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.Kp = 2;
    HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.Kd = 1;

    while (1)
    {
#ifdef DATA_OUTPUT
        if (state == SHOT && !state_R.brake)
        {
            sprintf((char *)VOFA, "T:%.2f,%d\n", VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.spd, state_R.spd_ctrl ? 1000 : 0);
            UART_SendArray(&UART7_info, VOFA, 16);
        }
#endif
        switch (state)
        {
        case IDLE:
        {
            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr = 0;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_CURR, &CUBEMARS_R100_KV90);
            }
            break;
        }
        // spin to top
        case INIT:
        {
            static MovAvgFltr_t VESC_fltr;

            if (Timer_CheckTimeout(&runtime, VESC_param.init.timeout) ||                                               // timeout
                                                                                                                       // ball plate detected
                MovAvgFltr(&VESC_fltr, VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.curr) >= VESC_param.init.curr_detect && // current large enough
                    runtime.intvl >= VESC_param.init.fltr_time)                                                        // time long enough
            {
                MovAvgFltr_Clear(&VESC_fltr);
                Timer_Clear(&runtime);
                state = IDLE;
                break;
            }

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.spd = VESC_param.init.spd; // go down

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_SPD, &CUBEMARS_R100_KV90);
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
                if (GPIOE->IDR & 0x4)
                {
                    state_W.ball = 1;
                    state = LOCK;
                    break;
                }
                state = IDLE;
                break;
            }

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.spd = VESC_param.back.spd; // go down

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_SPD, &CUBEMARS_R100_KV90);
            }
            break;
        }
        // stay at pos, wait for ball
        case LOCK:
        {
            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr = VESC_param.lock.curr;

            // time limit
            if (Timer_CheckTimeout(&runtime, VESC_param.lock.time))
            {
                Timer_Clear(&runtime);
                state = IDLE;
                break;
            }

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_CURR, &CUBEMARS_R100_KV90);
            }
            break;
        }
        case SHOT:
        {
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

            // use fitting data
            if (state_R.fitting)
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2(R2_info.dist_cm)
                                                     : Fitting_Calc_Basket(basket_info.dist_cm);

            LIMIT(VESC_param.shot.spd, CUBEMARS_R100_KV90.spd_max); // speed limit

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr = VESC_param.shot.spd * (state_R.brake ? VESC_param.shot.brake_curr_pct // curr for brake
                                                                                                : VESC_param.shot.acc_curr_pct); // curr for acceleration

            LIMIT_RANGE(VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr, 70, 120); // ESC current limit

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.spd = VESC_param.shot.spd; // target speed

            // switch control mode
            if (VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.spd / VESC_param.shot.spd >= VESC_param.shot.spd_ctrl_pct)
                state_R.spd_ctrl = 1;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, state_R.brake ? VESC_SET_CURR_BRAKE : (state_R.spd_ctrl ? VESC_SET_SPD : VESC_SET_CURR), &CUBEMARS_R100_KV90);
            }
            break;
        }
        }

        // delay after action
        if (Timer_CheckTimeout(&HighTorque_time, 0.25))
        {
            // stay at middle
            if (!state_W.ball || !state_W.gimbal ||               // no ball or gimbal disabled
                !state_W.aim_R2 && basket_info.dist_cm >= 1200 || // aim at basket but too far
                state_W.aim_R2 && R2_info.dist_cm >= 1200)        // aim at R2 but too far
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = (YAW_MIN + YAW_MAX) / 2;
            // aim at basket
            else if (!state_W.aim_R2)
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.basket_pos_0 + (err.basket_info ? yaw_curr                                                                   // chassis
                                                                                                                         : yaw_prev + (yaw_curr - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / 10.f)) // lidar
                                                                                                            * Gimbal_GR;
            // aim at R2
            else if (state_W.aim_R2)
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.R2_pos_0 + (yaw_prev + (yaw_curr - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / (err.basket_info ? 50.f      // chassis, err.basket_info as lidar position error flag
                                                                                                                                                                                          : 10.f))) * // lidar
                                                                                                        Gimbal_GR;
        }

        LIMIT_RANGE(HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MIN, YAW_MAX); // gimbal limit

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, GIMBAL_ID);
        }

        if (state_W.ball &&
            (state_W.aim_R2 ? MovAvgFltr_GetStatus(&R2_info.dist_fltr, 1) && R2_info.dist_cm <= 900                               // position ready for R2
                            : MovAvgFltr_GetStatus(&basket_info.dist_fltr, basket_info.dist_cm) && basket_info.dist_cm <= 750) && // position ready for basket
            MovAvgFltr_GetNewStatus(&yaw_fltr, HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos, 1))                         // yaw ready
            state_R.shot_ready = 1;
        else if (state != SHOT) // SHOT process protection
            state_R.shot_ready = 0;

        osDelay(1);
    }
}