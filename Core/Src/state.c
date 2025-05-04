#include "user.h"

USART_info_t UART7_info = {.USART_handle = UART7, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 0}, // USB to TTL
    UART5_info = {.USART_handle = UART5, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream1, .DMA_ID = 1};          // wireless bridge Tx

enum STATE state;
struct STATE_R state_R = { // internal-change state
    .fitting = 1};
struct STATE_W state_W; // external-change state
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
    .back.spd = -200,

    .shot.acc_curr_pct = 0.11,
    .shot.spd = 1000,
    .shot.spd_ctrl_pct = 0.89,
    .shot.brake_curr_pct = 0.05,
    .shot.timeout = 1,
    .shot.brake_time = 0.25};

struct
{
    float pos_0, gain;
} HighTorque_param = {
    .pos_0 = -8,
    .gain = 1};

#define YAW_MIN (-101 + HighTorque_param.pos_0)
#define YAW_MAX (115 + HighTorque_param.pos_0)

#define Gimbal_GR (14.45f * HighTorque_param.gain)

struct target_info basket_info = {.dist_fltr.size = 4},
                   R2_info = {.dist_fltr.size = 4};

struct pos_info R1_pos_lidar, R1_pos_chassis, R2_pos;

timer_t HighTorque_time, gimbal_time;

float yaw_prev;
float basket_dist_offset = 0,
      basket_spd_offset = 0,
      R2_dist_offset = 0,
      R2_spd_offset = -8;

MovAvgFltr_t yaw_fltr;

float Fitting_Calc_Basket(float dist_cm)
{
    if (dist_cm <= 440)
        // e2 sum: 9.16
        return -0.0000015861742424794212 * pow(dist_cm, 4) +
               0.002400568181997187 * pow(dist_cm, 3) +
               -1.3551231060409918 * pow(dist_cm, 2) +
               339.0213744863868 * dist_cm +
               -30837.94807418798 + basket_spd_offset;
    else if (dist_cm <= 560)
        // e2 sum: 0.00
        return -3.907911882983228e-9 * pow(dist_cm, 5) +
               0.000010101219118041627 * pow(dist_cm, 4) +
               -0.010351922363042831 * pow(dist_cm, 3) +
               5.256966412067413 * pow(dist_cm, 2) +
               -1321.8689270019531 * dist_cm +
               132554.23063334628 + basket_spd_offset;
    else
        // e2 sum: 1.94
        return 7.339014634899499e-7 * pow(dist_cm, 4) +
               -0.0018235478512451664 * pow(dist_cm, 3) +
               1.6927176844328642 * pow(dist_cm, 2) +
               -695.037252664566 * dist_cm +
               107565.29744557396 + basket_spd_offset;
}

float Fitting_Calc_R2(float dist_cm)
{
    // e2 sum: 0
    return 0.8 * dist_cm +
           600 + R2_spd_offset;
}

void State(void *argument)
{
    // default param
    HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.pos_0;
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
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_CURR, &HOBBYWING_V9626_KV160);
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
                VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.spd = VESC_param.back.spd;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_SPD, &HOBBYWING_V9626_KV160);
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
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2(R2_info.dist_cm + R2_dist_offset)
                                                     : Fitting_Calc_Basket(basket_info.dist_cm + basket_dist_offset);

            LIMIT(VESC_param.shot.spd, HOBBYWING_V9626_KV160.spd_max); // speed limit

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr = VESC_param.shot.spd * (state_R.brake ? VESC_param.shot.brake_curr_pct // curr for brake
                                                                                                : VESC_param.shot.acc_curr_pct); // curr for acceleration

            LIMIT_RANGE(VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr, 70, 120); // ESC current limit

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.spd = VESC_param.shot.spd; // target speed

            // switch control mode
            if (VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.spd / VESC_param.shot.spd >= VESC_param.shot.spd_ctrl_pct)
                state_R.spd_ctrl = 1;

            // control
            {
                if (state_R.brake)
                    VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_CURR_BRAKE, &HOBBYWING_V9626_KV160);
                else if (state_R.spd_ctrl)
                    VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_SPD, &HOBBYWING_V9626_KV160);
                else
                    VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_CURR, &HOBBYWING_V9626_KV160);
            }
            break;
        }
        }

        // delay after action
        if (Timer_CheckTimeout(&HighTorque_time, 0.25))
            HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.pos_0 + (state_W.ball
                                                                                                  // aim at R2 && R2 online
                                                                                                  ? (state_W.aim_R2 && !err.R2_pos ? yaw_prev + (R2_info.yaw - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / 25.f)
                                                                                                                                   // aim at basket when close to
                                                                                                                                   : (basket_info.dist_cm <= 900 ? (yaw_prev + (basket_info.yaw - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / (err.basket_info && err.pos_lidar ? 500.f   // chassis
                                                                                                                                                                                                                                                                                 : 10.f))) // lidar
                                                                                                                                                                 // stay at middle when far from
                                                                                                                                                                 : 0)) *
                                                                                                        Gimbal_GR
                                                                                                  // stay at middle
                                                                                                  : 0);

        LIMIT_RANGE(HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MIN, YAW_MAX); // gimbal limit

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, GIMBAL_ID);
        }

        if (state_W.ball &&
            (state_W.aim_R2 ? MovAvgFltr_GetNewStatus(&R2_info.dist_fltr, R2_info.dist_cm, 1)             // position ready for R2
                            : MovAvgFltr_GetNewStatus(&basket_info.dist_fltr, basket_info.dist_cm, 1)) && // position ready for basket
            MovAvgFltr_GetNewStatus(&yaw_fltr, HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos, 1)) // yaw ready
            state_R.shot_ready = 1;
        else if (state != SHOT) // SHOT process protection
            state_R.shot_ready = 0;

        osDelay(1);
    }
}