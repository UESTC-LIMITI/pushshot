#include "user.h"

USART_info_t UART7_info = {.USART_handle = UART7, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 0}, // USB to TTL
    UART5_info = {.USART_handle = UART5, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream1, .DMA_ID = 1};          // wireless bridge Tx

enum STATE state;
struct STATE_R state_R = { // internal-change state
    .fitting = 1};
struct STATE_W state_W; // external-change state
timer_t runtime;

#define DATA_OUTPUT

#ifdef DATA_OUTPUT
#include <stdio.h>
unsigned char VOFA[32];
#endif

#define PUSHSHOT_ID 1 - VESC_ID_OFFSET

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
    .back.spd = -200,

    .shot.acc_curr_pct = 0.1,
    .shot.spd = 1000,
    .shot.spd_ctrl_pct = 0.875,
    .shot.brake_curr_pct = 0.05,
    .shot.timeout = 1.5,
    .shot.brake_time = 0.25};

#define GIMBAL_ID 2 - HIGHTORQUE_ID_OFFSET

struct
{
    float pos_0, gain;
} HighTorque_param = {
    .pos_0 = -2,
    .gain = 1};

#define YAW_MIN (-108 + HighTorque_param.pos_0)
#define YAW_MAX (122 + HighTorque_param.pos_0)

#define Gimbal_GR (14.45f * HighTorque_param.gain)

struct target_info basket_info = {.dist_fltr.size = 2},
                   R2_info = {.dist_fltr.size = 4};

struct pos_info R1_pos_lidar, R1_pos_chassis, R2_pos;

timer_t HighTorque_time, gimbal_time;

float yaw_prev;
float basket_dist_offset = 0,
      basket_spd_offset = -4,
      R2_dist_offset = 0,
      R2_spd_offset = 0;

MovAvgFltr_t yaw_fltr;

float Fitting_Calc_Basket(float dist_cm)
{
    // e2 sum: 89.77
    // return 5.140813755997965e-12 * pow(dist_cm, 5) +
    //        -8.718216121250677e-9 * pow(dist_cm, 4) +
    //        0.000004339723124857642 * pow(dist_cm, 3) +
    //        0.00022797951896791346 * pow(dist_cm, 2) +
    //        0.17669564730022103 * dist_cm +
    //        728.6753135213174 + basket_spd_offset;

    // e2 sum: 77.96
    return -7.969408091887571e-10 * pow(dist_cm, 5) +
           0.0000017742687568045312 * pow(dist_cm, 4) +
           -0.0015595432989812252 * pow(dist_cm, 3) +
           0.6748970650078263 * pow(dist_cm, 2) +
           -142.63446494936943 * dist_cm +
           12597.700893643732 + basket_spd_offset;

    // e2 sum: 59.86
    // return 9.105900798539739e-10 * pow(dist_cm, 5) +
    //        -0.0000016365813943902685 * pow(dist_cm, 4) +
    //        0.0011434304287192276 * pow(dist_cm, 3) +
    //        -0.3866560404894699 * pow(dist_cm, 2) +
    //        63.979260883294046 * dist_cm +
    //        -3312.014752839768;
}

float Fitting_Calc_R2(float dist_cm)
{
    // e2 sum: 0
    return 0.8 * dist_cm +
           600 + R2_spd_offset;

    // e2 sum: 21.88
    // return -2.0606060378153268e-7 * pow(dist_cm, 4) +
    //        0.00045232322764832134 * pow(dist_cm, 3) +
    //        -0.36987120850244537 * pow(dist_cm, 2) +
    //        134.43093313649297 * dist_cm +
    //        -17330.890011500775 + R2_spd_offset;
}

void State(void *argument)
{
    // default param
    HighTorque[GIMBAL_ID].ctrl.pos = HighTorque_param.pos_0;
    HighTorque[GIMBAL_ID].ctrl.Kp = 2;
    HighTorque[GIMBAL_ID].ctrl.Kd = 1;

    while (1)
    {
#ifdef DATA_OUTPUT
        if (state == SHOT && !state_R.brake)
        {
            sprintf((char *)VOFA, "T:%.2f\n", VESC[PUSHSHOT_ID].fdbk.spd);
            UART_SendArray(&UART7_info, VOFA, 12);
        }
#endif
        switch (state)
        {
        case IDLE:
        {
            VESC[PUSHSHOT_ID].ctrl.curr = 0;

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
                VESC[PUSHSHOT_ID].ctrl.spd = VESC_param.back.spd;

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

            // use fitting data
            if (state_R.fitting)
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2(R2_info.dist_cm + R2_dist_offset)
                                                     : Fitting_Calc_Basket(basket_info.dist_cm + basket_dist_offset);

            LIMIT(VESC_param.shot.spd, HOBBYWING_V9626_KV160.spd_max); // speed limit

            VESC[PUSHSHOT_ID].ctrl.curr = VESC_param.shot.spd * (state_R.brake ? VESC_param.shot.brake_curr_pct // curr for brake
                                                                               : VESC_param.shot.acc_curr_pct); // curr for acceleration

            LIMIT_RANGE(VESC[PUSHSHOT_ID].ctrl.curr, 70, 120); // ESC current limit

            VESC[PUSHSHOT_ID].ctrl.spd = VESC_param.shot.spd; // target speed

            // switch control mode
            if (VESC[PUSHSHOT_ID].fdbk.spd / VESC_param.shot.spd >= VESC_param.shot.spd_ctrl_pct)
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
            HighTorque[GIMBAL_ID].ctrl.pos = HighTorque_param.pos_0 + (state_W.ball
                                                                           // aim at R2 && R2 online
                                                                           ? (state_W.aim_R2 && !err.R2_pos ? yaw_prev + (R2_info.yaw - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / 25.f)
                                                                                                            // aim at basket when close to
                                                                                                            : (basket_info.dist_cm <= 900 ? (yaw_prev + (basket_info.yaw - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / (err.basket_lidar && err.pos_lidar ? (err.basket_camera ? 500.f // chassis
                                                                                                                                                                                                                                                                                : 30.f) // camera
                                                                                                                                                                                                                                                           : 10.f)))                    // lidar
                                                                                                                                          // stay at middle when far from
                                                                                                                                          : 0)) *
                                                                                 Gimbal_GR
                                                                           // stay at middle
                                                                           : 0);

        LIMIT_RANGE(HighTorque[GIMBAL_ID].ctrl.pos, YAW_MIN, YAW_MAX); // gimbal limit

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, 2);
        }

        if (state_W.ball &&
            (state_W.aim_R2 ? MovAvgFltr_GetNewStatus(&R2_info.dist_fltr, R2_info.dist_cm, 1.5)             // position ready for R2
                            : MovAvgFltr_GetNewStatus(&basket_info.dist_fltr, basket_info.dist_cm, 1.5)) && // position ready for basket
            MovAvgFltr_GetNewStatus(&yaw_fltr, HighTorque[GIMBAL_ID].fdbk.pos, 2))                          // yaw ready
            state_R.shot_ready = 1;
        else if (state != SHOT) // SHOT process protection
            state_R.shot_ready = 0;

        osDelay(1);
    }
}