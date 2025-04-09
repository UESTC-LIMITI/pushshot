#include "user.h"

#include <stdio.h>

USART_info_t UART7_info = {.USART_handle = UART7},
             UART5_info = {.USART_handle = UART5};

enum STATE state = IDLE;
struct STATE_R state_R = {
    .fitting = 0};
struct STATE_W state_W;
timer_t runtime;

#ifdef DATA_OUTPUT
unsigned char VOFA[32];
#endif
struct
{
    struct
    {
        float time;
    } rst;
    struct
    {
        float spd, time;
    } back;
    struct
    {
        float spd;
    } init;
    struct
    {
        float curr_pct, spd, spd_ctrl_pct, brake_curr_pct;
    } shot;
} VESC_param = {
    .rst.time = 1,

    .back.spd = -100,
    .back.time = 0.4,

    .init.spd = 100,

    .shot.curr_pct = 0.1,
    .shot.spd = 800,
    .shot.spd_ctrl_pct = 0.9,
    .shot.brake_curr_pct = 0.05};

struct
{
    float pos_0, gain;
} HighTorque_param = {
    .pos_0 = 0,
    .gain = 0};

// gimbal limit
#define YAW_MIN -160
#define YAW_MAX 160

#define Gimbal_GR (11 * HighTorque_param.gain)

struct target_info basket_info = {.dist_cm_fltr.num = 10, .yaw_fltr.num = 10},
                   R2_info = {.dist_cm_fltr.num = 10, .yaw_fltr.num = 10};
struct pos_info R1_pos_lidar, R1_pos_chassis, R2_pos;
timer_t HighTorque_time;

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
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos = -HighTorque_param.pos_0;
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.Kp = 5;
    HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.Kd = 2;

    while (1)
    {
#ifdef DATA_OUTPUT
        if (VESC[1 - VESC_ID_OFFSET].fdbk.spd > VESC_param.init.spd + 20 && !state_R.brake)
        {
            sprintf(VOFA, "T:%.2f,%d\n", VESC[1 - VESC_ID_OFFSET].fdbk.spd, state_R.spd_ctrl ? 1000 : 0);
            UART_SendArray(&UART7_info, VOFA, 16);
        }
#endif
        switch (state)
        {
        // spin to rst pos
        case RST:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.spd = -VESC_param.back.spd;

            if (Timer_CheckTimeout(&runtime, VESC_param.rst.time))
            {
                Timer_Clear(&runtime);
                state = IDLE;
            }
            break;
        }
        // spin back some distance
        case BACK:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.back.spd;

            if (Timer_CheckTimeout(&runtime, VESC_param.back.time))
            {
                Timer_Clear(&runtime);
                state = IDLE;
            }
            break;
        }
        // spin to init pos
        case INIT:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.init.spd;

            // timeout exception
            if (Timer_CheckTimeout(&runtime, 5))
            {
                Timer_Clear(&runtime);
                state_W.ball = 0;
                state = IDLE;
            }

            // up to pos
            if (!(GPIOE->IDR & 0x4))
            {
                if (state_W.RST)
                {
                    Timer_Clear(&runtime);
                    state_W.RST = 0;
                    state = RST;
                }
                else
                {
                    Timer_Clear(&runtime);
                    state_R.shot_ready = 1;
                    state = IDLE;
                }
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
            if (!state_R.shot_ready)
                state = INIT;

            if (state_R.fitting &&
                !VESC[1 - VESC_ID_OFFSET].ctrl.spd) // spd not set
            {
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2()      // fitting for pass ball
                                                     : Fitting_Calc_Basket(); // fitting for shoot ball
            }

            VESC[1 - VESC_ID_OFFSET].ctrl.curr = VESC_param.shot.spd * (state_R.brake ? VESC_param.shot.brake_curr_pct // curr for brake
                                                                                      : VESC_param.shot.curr_pct);     // curr for acceleration

            LIMIT_RANGE(VESC[1 - VESC_ID_OFFSET].ctrl.curr, 200, 50); // ESC curr limit

            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.shot.spd; // target spd

            // switch ctrl mode
            if (VESC[1 - VESC_ID_OFFSET].fdbk.spd / VESC[1 - VESC_ID_OFFSET].ctrl.spd >= VESC_param.shot.spd_ctrl_pct)
                state_R.spd_ctrl = 1;

            if (Timer_CheckTimeout(&runtime, 0.5)) // shot duration
            {
                state_R.brake = 1; // timeout protection

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
            HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos = -HighTorque_param.pos_0 + (state_W.ball ||                                           // auto init
                                                                                               state_R.shot_ready                                // manual init
                                                                                           ? (state_W.aim_R2 && !err.R2_pos ? R2_info.yaw        // aim at R2 && R2 online
                                                                                                                            : basket_info.yaw) * // aim at basket
                                                                                                 Gimbal_GR
                                                                                           : 0); // stay mid

        LIMIT_RANGE(HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MAX, YAW_MIN); // gimbal limit

        osDelay(1);
    }
}