#include "user.h"

USART_info_t UART7_info = {.USART_handle = UART7, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 0}, // USB to TTL
    UART5_info = {.USART_handle = UART5, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream1, .DMA_ID = 1};          // dual robot communication Tx

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

struct
{
    struct
    {
        float timeout, curr, spd, spd_ctrl_pct;
    } back;
    struct
    {
        float acc_curr_pct, spd, spd_ctrl_pct, brake_curr_pct, timeout, brake_time;
    } shot;
} VESC_param = {
    .back.timeout = 2,
    .back.curr = -32,
    .back.spd = -250,
    .back.spd_ctrl_pct = 0.75,

    .shot.acc_curr_pct = 0.11,
    .shot.spd = 1000,
    .shot.spd_ctrl_pct = 0.9,
    .shot.brake_curr_pct = 0.05,
    .shot.timeout = 0.5,
    .shot.brake_time = 0.25};

struct
{
    float basket_pos_0, R2_pos_0;
} HighTorque_param = {
    .basket_pos_0 = (YAW_MAX + YAW_MIN) / 2 + 12,
    .R2_pos_0 = (YAW_MAX + YAW_MIN) / 2 + 14};

struct pos_t R1_pos_lidar, R1_pos_chassis, R2_pos, basket_pos = {.x = 14.05, .y = -3.99};

struct target_info basket_info,
    R2_info = {.dist_fltr.size = 8};

timer_t HighTorque_time, gimbal_time;

float yaw_prev, yaw_curr = (YAW_MAX + YAW_MIN) / 2;

float basket_spd_offset = -26,
      R2_spd_offset = 0;

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
    if (dist_cm <= 475)
        // e2 sum: 2.48
        return -6.933333092928873e-7 * pow(dist_cm, 4) +
               0.0011339258859948131 * pow(dist_cm, 3) +
               -0.693499975343002 * pow(dist_cm, 2) +
               188.9634324684739 * dist_cm +
               -18521.29298183975 + R2_spd_offset;
    else if (dist_cm <= 600)
        // e2 sum: 0.09
        return 9.457479466234986e-9 * pow(dist_cm, 5) +
               -0.000025470330683674547 * pow(dist_cm, 4) +
               0.027390942675992846 * pow(dist_cm, 3) +
               -14.704026699066162 * pow(dist_cm, 2) +
               3941.3226318359375 * dist_cm +
               -421155.14626460464 + R2_spd_offset;
    else
        // e2 sum: 0.13
        return -2.9887550212492897e-9 * pow(dist_cm, 5) +
               0.00001051371634730458 * pow(dist_cm, 4) +
               -0.014733505202457309 * pow(dist_cm, 3) +
               10.282764434814453 * pow(dist_cm, 2) +
               -3573.9356079101562 * dist_cm +
               495922.22327146993 + R2_spd_offset;
}

void State(void *argument)
{
    // default param
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
                state_R.spd_ctrl = 0;
                if (GPIOE->IDR & 0x4)
                    state_W.ball = 1;
                state = IDLE;
                break;
            }
            else
                state_R.spd_ctrl = VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.spd / VESC_param.back.spd >= VESC_param.back.spd_ctrl_pct; // stall protection

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr = VESC_param.back.curr;
            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.spd = VESC_param.back.spd;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, state_R.spd_ctrl ? VESC_SET_SPD : VESC_SET_CURR, &HOBBYWING_V9626_KV160);
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
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2(R2_info.dist_cm - 12.5)
                                                     : Fitting_Calc_Basket(basket_info.dist_cm);

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
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, state_R.brake ? VESC_SET_CURR_BRAKE : (state_R.spd_ctrl ? VESC_SET_SPD : VESC_SET_CURR), &HOBBYWING_V9626_KV160);
            }
            break;
        }
        }

        // delay after action
        if (Timer_CheckTimeout(&HighTorque_time, 0.25))
        {
            // stay at middle
            if (!state_W.ball || !state_W.gimbal ||              // no ball or gimbal disabled
                !state_W.aim_R2 && basket_info.dist_cm >= 900 || // aim at R2 but too far
                state_W.aim_R2 && R2_info.dist_cm >= 900)        // aim at basket but too far
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = 0;
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
                            : MovAvgFltr_GetStatus(&basket_info.dist_fltr, basket_info.dist_cm) && basket_info.dist_cm <= 900) && // position ready for basket
            MovAvgFltr_GetNewStatus(&yaw_fltr, HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos, 1))                         // yaw ready
            state_R.shot_ready = 1;
        else if (state != SHOT) // SHOT process protection
            state_R.shot_ready = 0;

        osDelay(1);
    }
}