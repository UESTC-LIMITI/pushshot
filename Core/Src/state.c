#include "user.h"

#include <stdio.h>

USART_info_t UART7_info = {.USART_handle = UART7},
             UART5_info = {.USART_handle = UART5, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 0};

enum STATE state = IDLE;
struct STATE_R state_R = {
    .fitting = 1};
struct STATE_W state_W;
timer_t runtime;

float basket_yaw_prev;

#define DATA_OUTPUT

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
        float curr;
    } lock;
    struct
    {
        float timeout, spd;
    } back;
    struct
    {
        float curr_pct, spd, spd_ctrl_pct, brake_curr_pct;
    } shot;
} VESC_param = {
    .init.timeout = 3,
    .init.spd = -200,

    .lock.curr = -5,

    .back.timeout = 2.5,
    .back.spd = -100,

    .shot.curr_pct = 0.1,
    .shot.spd = 810,
    .shot.spd_ctrl_pct = 0.9, // 0.9375
    .shot.brake_curr_pct = 0};

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

struct target_info basket_info = {.dist_cm_fltr.size = 10, .yaw_fltr.size = 10},
                   R2_info = {.dist_cm_fltr.size = 10, .yaw_fltr.size = 10};
struct pos_info R1_pos_lidar, R1_pos_chassis, R2_pos;
timer_t HighTorque_time, gimbal_time;

float Fitting_Calc_Basket(float dist_cm)
{
    return -3.269900772135022e-10 * pow(dist_cm, 5) +
           6.354993084822791e-7 * pow(dist_cm, 4) -
           0.00047688496297837446 * pow(dist_cm, 3) +
           0.17250630300259218 * pow(dist_cm, 2) -
           29.232875826768577 * dist_cm + 2515.470017191885;
}

float Fitting_Calc_R2(void)
{
    return 0;
}

void State(void *argument)
{
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
            UART_SendArray(&UART7_info, VOFA, 16);
        }
#endif
        switch (state)
        {
        case IDLE:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.curr = 0;
            break;
        }
        // spin to init pos
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
            }
            // go down
            else
            {
                VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.init.spd;
            }

            break;
        }
        case BACK:
        {
            if (Timer_CheckTimeout(&runtime, VESC_param.back.timeout))
            {
                Timer_Clear(&runtime);
                state = IDLE;

                break;
            }

            if (GPIOE->IDR & 0x4)
            {
                Timer_Clear(&runtime);
                state = LOCK;
            }
            else
                VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.back.spd;

            break;
        }
        // stay at pos
        case LOCK:
        {
            // current control
            VESC[1 - VESC_ID_OFFSET].ctrl.curr = VESC_param.lock.curr;

            break;
        }
        case SHOT:
        {
            if (!state_R.shot_ready)
            {
                state = LOCK;
                break;
            }

            if (state_R.fitting)
            {
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2()                             // fitting for pass ball
                                                     : Fitting_Calc_Basket(basket_info.dist_cm - 6); // fitting for shoot ball
            }

            VESC[1 - VESC_ID_OFFSET].ctrl.curr = VESC_param.shot.spd * (state_R.brake ? VESC_param.shot.brake_curr_pct // curr for brake
                                                                                      : VESC_param.shot.curr_pct);     // curr for acceleration

            LIMIT_RANGE(VESC[1 - VESC_ID_OFFSET].ctrl.curr, 120, 50); // ESC curr limit

            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.shot.spd; // target spd

            // switch ctrl mode
            if (VESC[1 - VESC_ID_OFFSET].fdbk.spd / VESC[1 - VESC_ID_OFFSET].ctrl.spd >= VESC_param.shot.spd_ctrl_pct)
                state_R.spd_ctrl = 1;

            if (Timer_CheckTimeout(&runtime, 0.5)) // shot duration
            {
                state_R.brake = 1; // timeout protection

                if (runtime.intvl >= 0.7) // total duration
                {
                    basket_info.yaw = state_R.shot_ready = state_W.ball = state_R.brake = state_R.spd_ctrl = 0;
                    Timer_Clear(&runtime);
                    Timer_Clear(&HighTorque_time);
                    state = IDLE;
                }
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
            HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos = -HighTorque_param.pos_0 - (state_W.ball                                                                                                                 // already init
                                                                                           ? (state_W.aim_R2 ? R2_info.yaw                                                                                          // aim at R2
                                                                                                             : basket_yaw_prev + (basket_info.yaw - basket_yaw_prev) * Timer_GetRatio(&gimbal_time, 1000 / 30.f)) * // aim at basket
                                                                                                 Gimbal_GR
                                                                                           : 0); // stay mid

        LIMIT_RANGE(HighTorque[2 - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MAX, YAW_MIN); // gimbal limit

        static MovAvgFltr_t HighTorque_fltr;
        if (state_W.ball && MovAvgFltr_GetTargetStatus(&HighTorque_fltr, HighTorque[2 - HIGHTORQUE_ID_OFFSET].fdbk.spd, 0, 2))
            state_R.shot_ready = 1;
        else if (state != SHOT)
            state_R.shot_ready = 0;

        osDelay(1);
    }
}