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
    } init;
    struct
    {
        float curr;
    } lock;
    struct
    {
        float acc_curr, spd, spd_ctrl_err, brake_curr, timeout, brake_time;
    } shot;
} VESC_param = {
    .init.spd = -150,
    .init.curr_detect = 26,
    .init.OC_time = 0.5,

    .lock.curr = -6.5,

    .shot.acc_curr = 40,
    .shot.spd = 800,
    .shot.spd_ctrl_err = 50,
    .shot.brake_curr = 26,
    .shot.timeout = 0.5,
    .shot.brake_time = 0.25};

struct
{
    float pos_0, basket_offset, R2_offset;
} HighTorque_param = {
    .pos_0 = (YAW_MAX + YAW_MIN) / 2,
    .basket_offset = 16.5,
    .R2_offset = 11};

struct pos_info R1_pos_lidar, R1_pos_chassis;

struct pos_t R2_pos = {.x = 12.5, .y = -4},
             basket_pos = {.x = 14.05, .y = -4};

struct target_info basket_info,
    R2_info = {.dist_fltr.size = 4};

timer_t R2_yaw_time;

float R2_yaw_prev = (YAW_MAX + YAW_MIN) / 2,
      R2_yaw_curr = (YAW_MAX + YAW_MIN) / 2;

char spd_offset;

MovAvgFltr_t yaw_fltr;

float Fitting_Calc_AccCurr(float spd)
{
    if (spd <= 700)
        return (spd - 400) * 0.1;
    else
        return (spd - 500) * 0.15;
}

float Fitting_Calc_Basket(float dist_cm)
{
    if (dist_cm <= 300)
        return -0.000018518518518167992 * pow(dist_cm, 3) +
               0.01692460317434552 * pow(dist_cm, 2) +
               -4.404100529041898 * dist_cm +
               974.7142857102483 + basket_spd_offset;
    else if (dist_cm <= 500)
        return 0.5022727272727273 * dist_cm +
               527.1818181818181 + basket_spd_offset;
    else
        return 0.44285714285714284 * dist_cm +
               554.7619047619274 + basket_spd_offset;
}

float Fitting_Calc_R2(float dist_cm)
{
    if (dist_cm <= 350)
        return 0.68 * dist_cm +
               422 + R2_spd_offset;
    else if (dist_cm <= 450)
        return 5.333331518642126e-7 * pow(dist_cm, 4) +
               -0.0008799997108326352 * pow(dist_cm, 3) +
               0.5428664927603677 * pow(dist_cm, 2) +
               -147.70995393395424 * dist_cm +
               15583.995539364472 + R2_spd_offset;
    else if (dist_cm <= 550)
        return -0.00007999999999364071 * pow(dist_cm, 3) +
               0.12011428570497173 * pow(dist_cm, 2) +
               -59.484285709564574 * dist_cm +
               10460.028570601304 + R2_spd_offset;
    else if (dist_cm <= 650)
        return 5.333345618474539e-7 * pow(dist_cm, 4) +
               -0.0011893362825503573 * pow(dist_cm, 3) +
               0.9916693158447742 * pow(dist_cm, 2) +
               -365.85772466659546 * dist_cm +
               51082.158915299995 + R2_spd_offset;
    else
        return 0.46 * dist_cm +
               536.8333333333969 + R2_spd_offset;
}

void State(void *argument)
{
    GPIOG->ODR |= 0x400; // fan for lidar

    // default param
    HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.Kp = 2;
    HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.Kd = 1;

    while (1)
    {
#ifdef DATA_OUTPUT
        if (state == SHOT && !state_R.brake)
        {
            // sprintf((char *)VOFA, "T:%.2f,%d\n", VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.spd, state_R.spd_ctrl ? 1000 : 0);
            sprintf((char *)VOFA, "T:%.1f\n", VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.volt);
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
        // spin to bottom
        case INIT:
        {
            // stall protection
            static MovAvgFltr_t curr_fltr;
            static timer_t OC_time;
            if (MovAvgFltr(&curr_fltr, VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.curr) >= VESC_param.init.curr_detect)
            {
                if (Timer_CheckTimeout(&OC_time, VESC_param.init.OC_time))
                {
                    MovAvgFltr_Clear(&curr_fltr);
                    Timer_Clear(&OC_time);
                    state_R.shot_ready = state_W.ball = 0;
                    state = IDLE;
                    break;
                }
            }
            else
                Timer_Clear(&OC_time);

            // bottom photogate
            if (GPIOE->IDR & 0x4)
            {
                MovAvgFltr_Clear(&curr_fltr);
                Timer_Clear(&OC_time);
                state = LOCK;
                break;
            }

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.spd = VESC_param.init.spd;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_SPD, &CUBEMARS_R100_KV90);
            }
            break;
        }
        // stay at position
        case LOCK:
        {
            // ball plate go up itself
            if (state_W.ball && !(GPIOE->IDR & 0x4))
            {
                state = INIT;
                break;
            }

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr = VESC_param.lock.curr;

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
                state_R.shot_ready = state_W.ball = state_R.spd_ctrl = 0;
                state_R.brake = 1;

                if (runtime.intvl >= VESC_param.shot.timeout + VESC_param.shot.brake_time) // total duration
                {
                    Timer_Clear(&runtime);
                    state_R.brake = 0;
                    state = IDLE;
                    break;
                }
            }

            // use fitting data
            if (state_R.fitting)
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2(MovAvgFltr_GetData(&R2_info.dist_fltr))
                                                     : Fitting_Calc_Basket(MovAvgFltr_GetData(&basket_info.dist_fltr));
            LIMIT(VESC_param.shot.spd, CUBEMARS_R100_KV90.spd_max);            // speed limit
            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.spd = VESC_param.shot.spd; // target speed

            VESC_param.shot.acc_curr = Fitting_Calc_AccCurr(VESC_param.shot.spd);
            LIMIT(VESC_param.shot.acc_curr, 120);                                                     // ESC current limit
            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr = state_R.brake ? VESC_param.shot.brake_curr // current for brake
                                                                         : VESC_param.shot.acc_curr;  // current for acceleration

            // switch control mode
            if (VESC_param.shot.spd - VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.spd <= VESC_param.shot.spd_ctrl_err)
                state_R.spd_ctrl = 1;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, state_R.brake ? VESC_SET_CURR_BRAKE : (state_R.spd_ctrl ? VESC_SET_SPD : VESC_SET_CURR), &CUBEMARS_R100_KV90);
            }
            break;
        }
        }

        if (state_W.gimbal) // gimbal enabled
        {
            // stay at middle
            if (!state_W.ball && state != SHOT) // no ball and not shooting
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.pos_0;
            // aim at basket
            else if (!state_W.aim_R2)
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.pos_0 + HighTorque_param.basket_offset + basket_info.yaw * Gimbal_GR;
            // aim at R2
            else if (state_W.aim_R2)
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.pos_0 + HighTorque_param.R2_offset + (R2_yaw_prev + (R2_yaw_curr - R2_yaw_prev) * Timer_GetRatio(&R2_yaw_time, 1 / (err.R2_pos ? 20.f : 50.f))) * Gimbal_GR;
            LIMIT_RANGE(HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MIN, YAW_MAX); // gimbal limit
        }

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, GIMBAL_ID);
        }

        state_R.shot_ready = state_W.ball &&
                             (!err.yaw_lim && MovAvgFltr_GetNewStatus(&yaw_fltr, HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos, 1) && // yaw ready
                                  (state_W.aim_R2 ? MovAvgFltr_GetStatus(&R2_info.dist_fltr, 2) && R2_info.dist_cm <= 750 && state_W.R2_ready // position ready for R2
                                                  : MovAvgFltr_GetStatus(&basket_info.dist_fltr, 1.5) && basket_info.dist_cm <= 750) ||       // position ready for basket
                              !state_R.fitting);                                                                                              // test

        osDelay(1);
    }
}