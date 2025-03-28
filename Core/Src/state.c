#include "user.h"

#include <stdio.h>

USART_info_t UART7_info = {.USART_handle = UART7},
             UART5_info = {.USART_handle = UART5},
             USART3_info = {.USART_handle = USART3};

enum STATE state = IDLE;
struct STATE_R state_R;
struct STATE_W state_W = {
    .fitting = 0};
timer_t runtime;

#ifdef DATA_OUTPUT
unsigned char VOFA[32];
#endif
struct
{
    float back_spd, init_spd, shot_curr_pct, shot_spd, spd_ctrl_pct, brake_curr_pct;
} VESC_param = {
    .back_spd = -100,
    .init_spd = 250,
    .shot_curr_pct = 0.12,
    .shot_spd = 1000,
    .spd_ctrl_pct = 0.92,
    .brake_curr_pct = 0.05};

struct
{
    float pos_0, pos_target;
} HighTorque_param = {
    .pos_0 = 10};

// gimbal limit
#define YAW_MIN -168
#define YAW_MAX 172

struct target_info basket_info = {.yaw_fltr.len = 10},
                   R2_info = {.yaw_fltr.len = 10};
timer_t HighTorque_time;

float test_curr;

float Fitting_Calc_Basket(void)
{
    return 0;
}

float Fitting_Calc_R2(void)
{
    return 0;
}

void State(void *argument)
{
    // default param
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.pos_0;
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.Kp = 5;
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.Kd = 1;

    while (1)
    {
#ifdef DATA_OUTPUT
        if (VESC[1 - VESC_ID_OFFSET].fdbk.spd > VESC_param.init_spd + 5 && !state_R.brake)
        {
            sprintf(VOFA, "T:%.2f,%d\n", VESC[1 - VESC_ID_OFFSET].fdbk.spd, state_R.spd_ctrl ? 1000 : 0);
            UART_SendArray(&UART5_info, VOFA, 16);
        }
#endif
        switch (state)
        {
        case BACK:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.back_spd; // spin to init pos

            // down to pos
            if (GPIOE->IDR & 0x4)
                state = INIT;
            break;
        }
        case INIT:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.init_spd; // spin to init pos

            // up to pos
            if (!(GPIOE->IDR & 0x4))
            {
                state_R.shot_ready = 1;
                state = IDLE;
            }
            break;
        }
        case IDLE:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.curr = 0;
            VESC[1 - VESC_ID_OFFSET].ctrl.spd = 0;

            // automatic init
            if (state_W.ball && !state_R.shot_ready)
                state = INIT;
            break;
        }
        case SHOT:
        {
            if (state_W.fitting &&
                !VESC[1 - VESC_ID_OFFSET].ctrl.spd) // spd not set
            {
                VESC_param.shot_spd = state_W.aim_R2 ? Fitting_Calc_R2()      // fit for pass ball
                                                     : Fitting_Calc_Basket(); // fit for shoot ball
            }

            VESC[1 - VESC_ID_OFFSET].ctrl.curr = VESC_param.shot_spd * (state_R.brake ? VESC_param.brake_curr_pct  // curr for brake
                                                                                      : VESC_param.shot_curr_pct); // curr for acceleration
            LIMIT(VESC[1 - VESC_ID_OFFSET].ctrl.curr, 120);                                                        // ESC curr limit

            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.shot_spd; // target spd

            // switch ctrl mode
            if (VESC[1 - VESC_ID_OFFSET].fdbk.spd / VESC[1 - VESC_ID_OFFSET].ctrl.spd >= VESC_param.spd_ctrl_pct)
                state_R.spd_ctrl = 1;

            if (Timer_CheckTimeout(&runtime, 0.5)) // shot duration
            {
                state_R.brake = 1;

                if (runtime.intvl >= 0.7) // total duration
                {
                    state_R.shot_ready = state_W.ball = state_R.brake = state_R.spd_ctrl = 0;
                    Timer_Clear(&runtime);
                    Timer_Clear(&HighTorque_time);
                    state = IDLE;
                }
            }
            break;
        }
        }

        // delay after shot
        if (Timer_CheckTimeout(&HighTorque_time, 0.1))
            HighTorque[2 - HIGHTORQUE_ID_OFFSET]
                .ctrl.pos = HighTorque_param.pos_0 + (state_W.ball ||            // auto init
                                                              state_R.shot_ready // manual init
                                                          ? (state_W.aim_R2 ? R2_info.yaw
                                                                            : basket_info.yaw) *
                                                                Gimbal_GR // aim at target
                                                          : 0);           // stay mid

        LIMIT_RANGE(HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MAX, YAW_MIN); // gimbal limit

        osDelay(1);
    }
}