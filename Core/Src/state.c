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
        float curr_nball, curr_ball;
    } lock;
    struct
    {
        float acc_curr, spd, spd_ctrl_err, brake_curr, timeout, brake_time;
    } shot;
} VESC_param = {
    .init.spd = -100,
    .init.curr_detect = 26,
    .init.OC_time = 0.5,

    .lock.curr_nball = -6.5,
    .lock.curr_ball = -3.25,

    .shot.acc_curr = 40,
    .shot.spd = 800,
    .shot.spd_ctrl_err = 50,
    .shot.brake_curr = 26,
    .shot.timeout = 0.5,
    .shot.brake_time = 0.5};

struct
{
    float basket_pos_0, R2_pos_0;
} HighTorque_param = {
    .basket_pos_0 = 2,
    .R2_pos_0 = 1};

struct pos_t R1_pos_lidar, R1_pos_chassis, R2_pos, basket_pos = {.x = 14.05, .y = -4};

struct target_info basket_info,
    R2_info = {.dist_fltr.size = 8};

timer_t HighTorque_time, gimbal_time;

float yaw_prev = (YAW_MAX + YAW_MIN) / 2,
      yaw_curr = (YAW_MAX + YAW_MIN) / 2;

char basket_spd_offset = 5,
     R2_spd_offset = 5;

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
        // e2 sum: 0.89
        return -6.510416640237437e-7 * pow(dist_cm, 4) +
               0.0006128472195428003 * pow(dist_cm, 3) +
               -0.2110937490433571 * pow(dist_cm, 2) +
               31.972420471720397 * dist_cm +
               -1182.7857016678831 + basket_spd_offset;
    else if (dist_cm <= 400)
        // e2 sum: 0.34
        return 0.5271428571428571 * dist_cm +
               526 + basket_spd_offset;
    else if (dist_cm <= 500)
        // e2 sum: 0.57
        return -5.208332252149006e-7 * pow(dist_cm, 4) +
               0.0009305553608101036 * pow(dist_cm, 3) +
               -0.6222915353719145 * pow(dist_cm, 2) +
               185.0920243859291 * dist_cm +
               -19955.424248173396 + basket_spd_offset;
    else if (dist_cm <= 600)
        // e2 sum: 0.57
        return 5.208331681494371e-7 * pow(dist_cm, 4) +
               -0.0011319440795887203 * pow(dist_cm, 3) +
               0.9218746987171471 * pow(dist_cm, 2) +
               -332.99116003513336 * dist_cm +
               45752.7946803008 + basket_spd_offset;
    else
        // e2 sum: 0.00
        return 0.6 * dist_cm +
               473 + basket_spd_offset;
}

float Fitting_Calc_R2(float dist_cm)
{
    if (dist_cm <= 450)

        return 0.0000019199993880336663 * pow(dist_cm, 4) +
               -0.0030079990210651886 * pow(dist_cm, 3) +
               1.7595994137227535 * pow(dist_cm, 2) +
               -454.8198445737362 * dist_cm +
               44451.9846487936 + R2_spd_offset;
    else if (dist_cm <= 550)

        return -0.00005333333327284251 * pow(dist_cm, 3) +
               0.08239999990655633 * pow(dist_cm, 2) +
               -41.82666661916301 * dist_cm +
               7734.999992055529 + R2_spd_offset;
    else if (dist_cm <= 650)

        return -0.00009599999999920783 * pow(dist_cm, 3) +
               0.1791999999950349 * pow(dist_cm, 2) +
               -110.61999999918044 * dist_cm +
               23400.000001181426 + R2_spd_offset;
    else

        return 0.2 * dist_cm +
               715 + R2_spd_offset;
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
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_CURR, &CUBEMARS_R100_KV90);
            }
            break;
        }
        // spin to bottom, before dribble start
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

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].ctrl.curr = state_W.ball ? VESC_param.lock.curr_ball
                                                                        : VESC_param.lock.curr_nball;

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
                    Timer_Clear(&runtime);
                    Timer_Clear(&HighTorque_time);
                    state_R.shot_ready = state_W.ball = state_R.brake = state_R.spd_ctrl = 0;
                    state = IDLE;
                    break;
                }
            }

            // use fitting data
            if (state_R.fitting)
                VESC_param.shot.spd = state_W.aim_R2 ? Fitting_Calc_R2(R2_info.dist_cm)
                                                     : Fitting_Calc_Basket(basket_info.dist_cm);
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
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = (YAW_MAX + YAW_MIN) / 2 + HighTorque_param.basket_pos_0 + basket_info.yaw * Gimbal_GR;
            // aim at R2
            else if (state_W.aim_R2)
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.pos = (YAW_MAX + YAW_MIN) / 2 + HighTorque_param.R2_pos_0 + (yaw_prev + (yaw_curr - yaw_prev) * Timer_GetRatio(&gimbal_time, 1 / 50.f)) * Gimbal_GR;
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