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
    .init.curr_detect = 22.625,
    .init.OC_time = 0.5,

    .lock.curr = -5.65625,

    .shot.acc_curr = 40,
    .shot.spd = 800,
    .shot.spd_ctrl_err = 50,
    .shot.brake_curr = 22.625,
    .shot.timeout = 0.5,
    .shot.brake_time = 0.25};

struct
{
    float pos_0, basket_offset, R2_offset;
} HighTorque_param = {
    .pos_0 = (YAW_MAX + YAW_MIN) / 2,
    .basket_offset = 11,
    .R2_offset = 11};

struct pos_info R1_pos_lidar, R1_pos_chassis;

struct pos_t R2_pos = {.x = 12.5, .y = -4},
             basket_pos = {.x = 14.05, .y = -4};

struct target_info basket_info,
    R2_info = {.dist_fltr.size = 4};

timer_t R2_yaw_time, R2_yaw_intvl;

float R2_yaw_prev = (YAW_MAX + YAW_MIN) / 2,
      R2_yaw_curr = (YAW_MAX + YAW_MIN) / 2;

char basket_spd_offset, R2_spd_offset;

MovAvgFltr_t yaw_fltr;

float Fitting_Calc_AccCurr(float spd)
{
    if (spd <= 600)
        return 20;
    else if (spd <= 700)
        return (spd - 400) * 0.1;
    else
        return (spd - 500) * 0.15;
}

float Fitting_Calc_Basket(float dist_cm)
{
    if (dist_cm <= 300)
        return 0.55 * dist_cm +
               510 + basket_spd_offset;
    else if (dist_cm <= 400)
        return 0.6 * dist_cm +
               495 + basket_spd_offset;
    else
        return 0.5 * dist_cm +
               535 + basket_spd_offset;
}

float Fitting_Calc_R2(float dist_cm)
{
    if (dist_cm <= 350)
        return 0.68 * dist_cm +
               426 + R2_spd_offset;
    else if (dist_cm <= 450)
        return 5.333331518642126e-7 * pow(dist_cm, 4) +
               -0.0008799997108326352 * pow(dist_cm, 3) +
               0.5428664927603677 * pow(dist_cm, 2) +
               -147.70995393395424 * dist_cm +
               15587.995539364472 + R2_spd_offset;
    else if (dist_cm <= 550)
        return -0.00007999999999364071 * pow(dist_cm, 3) +
               0.12011428570497173 * pow(dist_cm, 2) +
               -59.484285709564574 * dist_cm +
               10464.028570601304 + R2_spd_offset;
    else if (dist_cm <= 650)
        return 5.333345618474539e-7 * pow(dist_cm, 4) +
               -0.0011893362825503573 * pow(dist_cm, 3) +
               0.9916693158447742 * pow(dist_cm, 2) +
               -365.85772466659546 * dist_cm +
               51086.158915299995 + R2_spd_offset;
    else
        return 0.46 * dist_cm +
               540.8333333333969 + R2_spd_offset;
}

void State(void *argument)
{
    GPIOG->ODR |= 0x400; // fan for lidar

    // default param
    HighTorque[GIMBAL_arrID].ctrl.Kp = 2;
    HighTorque[GIMBAL_arrID].ctrl.Kd = 1;

    while (1)
    {
#ifdef DATA_OUTPUT
        if (state == SHOT && !state_R.brake)
        {
            sprintf((char *)VOFA, "T:%.2f,%d\n", VESC[PUSHSHOT_arrID].fdbk.spd, state_R.spd_ctrl ? 1000 : 0);
            UART_SendArray(&UART7_info, VOFA, 16);
        }
#endif
        switch (state)
        {
        case IDLE:
        {
            VESC[PUSHSHOT_arrID].ctrl.curr = 0;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_CURR, &MOTOR);
            }
            break;
        }
        // spin to bottom
        case INIT:
        {
            // stall protection
            static MovAvgFltr_t curr_fltr = {.size = 48};
            static timer_t OC_time;
            if (MovAvgFltr(&curr_fltr, VESC[PUSHSHOT_arrID].fdbk.curr) >= VESC_param.init.curr_detect)
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

            VESC[PUSHSHOT_arrID].ctrl.spd = VESC_param.init.spd;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_SPD, &MOTOR);
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

            VESC[PUSHSHOT_arrID].ctrl.curr = VESC_param.lock.curr;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, VESC_SET_CURR, &MOTOR);
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

            // switch control mode
            if (VESC_param.shot.spd - VESC[PUSHSHOT_arrID].fdbk.spd <= VESC_param.shot.spd_ctrl_err)
                state_R.spd_ctrl = 1;

            if (state_R.brake)
                VESC[PUSHSHOT_arrID].ctrl.curr = VESC_param.shot.brake_curr;
            else if (!state_R.spd_ctrl)
            {
                if (state_R.fitting)
                    VESC_param.shot.spd = state_W.aim_R2 ? state_W.R2_NetUp ? Fitting_Calc_R2_NetUp(MovAvgFltr_GetData(&R2_info.dist_fltr))
                                                                            : Fitting_Calc_R2_NetDown(MovAvgFltr_GetData(&R2_info.dist_fltr))
                                                         : Fitting_Calc_Basket(MovAvgFltr_GetData(&basket_info.dist_fltr));
                LIMIT(VESC_param.shot.spd, MOTOR.spd_max);
                VESC[PUSHSHOT_arrID].ctrl.spd = VESC_param.shot.spd;

                VESC_param.shot.acc_curr = Fitting_Calc_AccCurr(VESC_param.shot.spd);
                LIMIT(VESC_param.shot.acc_curr, MOTOR.curr_max);
                VESC[PUSHSHOT_arrID].ctrl.curr = VESC_param.shot.acc_curr;
            }

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_ID, state_R.brake ? VESC_SET_CURR_BRAKE : (state_R.spd_ctrl ? VESC_SET_SPD : VESC_SET_CURR), &MOTOR);
            }
            break;
        }
        }

        if (state_W.gimbal) // gimbal enabled
        {
            // stay at middle
            if (!state_W.ball && state != SHOT) // no ball and not shooting
                HighTorque[GIMBAL_arrID].ctrl.pos = HighTorque_param.pos_0;
            // aim at basket
            else if (!state_W.aim_R2)
                HighTorque[GIMBAL_arrID].ctrl.pos = HighTorque_param.pos_0 + HighTorque_param.basket_offset + basket_info.yaw * Gimbal_GR;
            // aim at R2
            else if (state_W.aim_R2)
                HighTorque[GIMBAL_arrID].ctrl.pos = HighTorque_param.pos_0 + HighTorque_param.R2_offset + (R2_yaw_prev + (R2_yaw_curr - R2_yaw_prev) * Timer_GetRatio(&R2_yaw_time, R2_yaw_intvl.intvl)) * Gimbal_GR;
            LIMIT_RANGE(HighTorque[GIMBAL_arrID].ctrl.pos, YAW_MIN, YAW_MAX); // gimbal limit
        }

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, GIMBAL_ID);
        }

        state_R.shot_ready = state_W.ball &&
                             (MovAvgFltr_GetNewStatus(&yaw_fltr, HighTorque[GIMBAL_arrID].fdbk.pos, 1) &&                               // yaw ready
                                  (state_W.aim_R2 ? MovAvgFltr_GetStatus(&R2_info.dist_fltr, 2) && R2_info.dist_cm <= 750               // position ready for R2
                                                  : MovAvgFltr_GetStatus(&basket_info.dist_fltr, 1.5) && basket_info.dist_cm <= 750) && // position ready for basket
                                  !err.yaw_lim ||
                              !state_R.fitting); // test

        osDelay(1);
    }
}