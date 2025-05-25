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
        float spd, curr_detect, OC_time;
    } back;
    struct
    {
        float acc_curr_pct, spd, spd_ctrl_pct, brake_curr, timeout, brake_time;
    } shot;
} VESC_param = {
    .back.spd = -200,
    .back.curr_detect = 42.875,
    .back.OC_time = 0.5,

    .shot.acc_curr_pct = 0.11,
    .shot.spd = 1000,
    .shot.spd_ctrl_pct = 0.9,
    .shot.brake_curr = 42.875,
    .shot.timeout = 0.5,
    .shot.brake_time = 0.25};

struct
{
    float basket_pos_0, R2_pos_0;
} HighTorque_param = {
    .basket_pos_0 = 12,
    .R2_pos_0 = 12};

struct pos_t R1_pos_lidar, R1_pos_chassis, R2_pos, basket_pos = {.x = 14.05, .y = -4};

struct target_info basket_info = {.dist_fltr.size = 3},
                   R2_info = {.dist_fltr.size = 8};

timer_t HighTorque_time, gimbal_time;

float yaw_prev = (YAW_MAX + YAW_MIN) / 2,
      yaw_curr = (YAW_MAX + YAW_MIN) / 2;

float basket_spd_offset = -2,
      R2_spd_offset = -9;

MovAvgFltr_t yaw_fltr;

float Fitting_Calc_Basket(float dist_cm)
{
    if (dist_cm <= 400)
        // e2 sum: 0.00
        return 5.7307949385965173e-8 * pow(dist_cm, 5) +
               -0.00009898683288156462 * pow(dist_cm, 4) +
               0.06819698377512395 * pow(dist_cm, 3) +
               -23.42859649658203 * pow(dist_cm, 2) +
               4015.0483322143555 * dist_cm +
               -273855.3417188975 + basket_spd_offset;
    else if (dist_cm <= 500)
        // e2 sum: 0.01
        return -3.381361945997696e-8 * pow(dist_cm, 5) +
               0.00007595042006869335 * pow(dist_cm, 4) +
               -0.06812112824991345 * pow(dist_cm, 3) +
               30.496549129486084 * pow(dist_cm, 2) +
               -6813.6507568359375 * dist_cm +
               608651.4141721731 + basket_spd_offset;
    else if (dist_cm <= 600)
        // e2 sum: 0.48
        return 3.906248680074498e-7 * pow(dist_cm, 4) +
               -0.0008605321195318538 * pow(dist_cm, 3) +
               0.7086282325908542 * pow(dist_cm, 2) +
               -257.9252437353134 * dist_cm +
               36005.98091615922 + basket_spd_offset;
    else
        // e2 sum: 0.00
        return 0.55 * dist_cm +
               777 + basket_spd_offset;
}

float Fitting_Calc_R2(float dist_cm)
{
    if (dist_cm <= 475)
        // e2 sum: 0.00
        return -2.133408949500648e-8 * pow(dist_cm, 5) +
               0.000043308226054250554 * pow(dist_cm, 4) +
               -0.03505994862644002 * pow(dist_cm, 3) +
               14.147859066724777 * pow(dist_cm, 2) +
               -2844.7474670410156 * dist_cm +
               228775.7340842169 + R2_spd_offset;
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
    GPIOG->ODR |= 0x400; // fan for lidar

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
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_CURR, &HOBBYWING_V9626_KV160);
            }
            break;
        }
        // spin to bottom
        case BACK:
        {
            if (GPIOE->IDR & 0x4) // bottom reached
            {
                state = IDLE;
                break;
            }

            // stall protection
            static MovAvgFltr_t curr_fltr;
            static timer_t OC_time;
            if (MovAvgFltr(&curr_fltr, VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.curr) >= VESC_param.back.curr_detect)
            {
                if (Timer_CheckTimeout(&OC_time, VESC_param.back.OC_time))
                {
                    Timer_Clear(&OC_time);
                    state = IDLE;
                    break;
                }
            }
            else
                Timer_Clear(&OC_time);

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
                    Timer_Clear(&runtime);
                    Timer_Clear(&HighTorque_time);
                    state_R.shot_ready = state_W.ball = state_R.brake = state_R.spd_ctrl = 0;
                    state = IDLE;
                    break;
                }
            }

            // use fitting data
            if (state_R.fitting)
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2(MovAvgFltr_GetData(&R2_info.dist_fltr))
                                                     : Fitting_Calc_Basket(MovAvgFltr_GetData(&basket_info.dist_fltr));
            LIMIT(VESC_param.shot.spd, HOBBYWING_V9626_KV160.spd_max);         // speed limit
            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.spd = VESC_param.shot.spd; // target speed

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr = state_R.brake ? VESC_param.shot.brake_curr                          // current for brake
                                                                         : VESC_param.shot.spd * VESC_param.shot.acc_curr_pct; // current for acceleration
            LIMIT(VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr, 120);                                                          // ESC current limit

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
            if (!state_W.ball || !state_W.gimbal ||               // no ball or gimbal disabled
                !state_W.aim_R2 && basket_info.dist_cm >= 1200 || // aim at basket but too far
                state_W.aim_R2 && R2_info.dist_cm >= 1200)        // aim at R2 but too far
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = (YAW_MIN + YAW_MAX) / 2;
            // aim at basket
            else if (!state_W.aim_R2)
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = (YAW_MAX + YAW_MIN) / 2 + HighTorque_param.basket_pos_0 + (err.basket_info ? yaw_curr                                                                   // chassis
                                                                                                                                                   : yaw_prev + (yaw_curr - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / 10.f)) // lidar
                                                                                                                                      * Gimbal_GR;
            // aim at R2
            else if (state_W.aim_R2)
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = (YAW_MAX + YAW_MIN) / 2 + HighTorque_param.R2_pos_0 + (yaw_prev + (yaw_curr - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / (err.basket_info ? 50.f      // chassis, err.basket_info as lidar position error flag
                                                                                                                                                                                                                    : 10.f))) * // lidar
                                                                                                                                  Gimbal_GR;
        }

        LIMIT_RANGE(HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MIN, YAW_MAX); // gimbal limit

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, GIMBAL_ID);
        }

        state_R.shot_ready = state_W.ball &&
                             (state_W.aim_R2 ? MovAvgFltr_GetStatus(&R2_info.dist_fltr, 1) && R2_info.dist_cm <= 900             // position ready for R2
                                             : MovAvgFltr_GetStatus(&basket_info.dist_fltr, 1) && basket_info.dist_cm <= 750) && // position ready for basket
                             MovAvgFltr_GetNewStatus(&yaw_fltr, HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos, 1);       // yaw ready

        osDelay(1);
    }
}