#include "usr.h"

static enum STATE state_last;

#define DATA_OUTPUT
#ifdef DATA_OUTPUT
#include <stdio.h>
#endif

struct
{
    const float high_spd, low_spd, OC_curr, OC_time;
    struct
    {
        const float time;
    } mid;
    struct
    {
        const float high_spd_time, decelerate_time;
    } init;
    struct
    {
        const float curr;
    } lock;
    struct
    {
        float acc_curr, spd;
        const float spd_ctrl_err, brake_curr, timeout, brake_time;
    } shot;
} VESC_param = {
    .high_spd = -300,
    .low_spd = -150,
    .OC_curr = 30.1,
    .OC_time = 0.25,

    .mid.time = 1,

    .init.high_spd_time = 0.375,
    .init.decelerate_time = 0.25,

    .lock.curr = -5.65625,

    .shot.spd_ctrl_err = 50,
    .shot.brake_curr = 30.1,
    .shot.timeout = 0.5,
    .shot.brake_time = 0.25};

struct
{
    const float pos0, basket_offset, R2_offset;
} HighTorque_param = {
    .pos0 = (YAW_MAX + YAW_MIN) / 2,
    .basket_offset = 4,
    .R2_offset = -4};

float Fitting_AccCurr_Basket(float spd)
{
    // 10A, 400RPM
    if (spd <= 400)
        return 10;
    // 15A, 500RPM
    // 20A, 600RPM
    else if (spd <= 600)
        return (spd - 200) * 0.05;
    // 30A, 700RPM
    // 40A, 800RPM
    else if (spd <= 800)
        return (spd - 400) * 0.1;
    // 50A, 850RPM
    // 60A, 900RPM
    else
        return (spd - 600) * 0.2;
}

float Fitting_AccCurr_R2(float spd)
{
    // 10A, 400RPM
    if (spd <= 400)
        return 10;
    // 15A, 500RPM
    // 20A, 600RPM
    else if (spd <= 600)
        return (spd - 200) * 0.05;
    // 30A, 700RPM
    // 40A, 800RPM
    else if (spd <= 800)
        return (spd - 400) * 0.1;
    // 50A, 850RPM
    // 60A, 900RPM
    else
        return (spd - 600) * 0.2;
}

float Fitting_Spd_Basket(float dist_cm)
{
    if (dist_cm <= 260)
        return 0.55 * dist_cm +
               495 + spd_offset;
    else if (dist_cm <= 360)
        return 0.55 * dist_cm +
               495 + spd_offset;
    else if (dist_cm <= 460)
        return 0.6 * dist_cm +
               477 + spd_offset;
    else if (dist_cm <= 560)
        return 0.5 * dist_cm +
               523 + spd_offset;
    else
        return 0.55 * dist_cm +
               495 + spd_offset;

    return 0.55 * dist_cm +
           495 + spd_offset;
}

float Fitting_Spd_R2_NetDown(float dist_cm)
{
    if (dist_cm <= 275)
        return 0.92 * dist_cm + 340 + spd_offset;
    else if (dist_cm <= 375)
        return 0.76 * dist_cm + 384 + spd_offset;
    else
        return 0.6 * dist_cm + 444 + spd_offset;
}

float Fitting_Spd_R2_NetUp(float dist_cm)
{
    return -5.750327304436271e-11 * pow(dist_cm, 5) +
           1.4975612224951695e-7 * pow(dist_cm, 4) +
           -0.0001507099019644187 * pow(dist_cm, 3) +
           0.07246624426352355 * pow(dist_cm, 2) +
           -15.975384144345298 * dist_cm +
           1895.5957471418012 + spd_offset;
}

bool VESC_Stall(void)
{
    static MovAvgFltr_t curr_fltr = {.size = 48};
    static TIMsw_t OC_time;

    if (state != state_last)
    {
        MovAvgFltr_Clear(&curr_fltr);
        TIMsw_Clear(&OC_time);
    }

    if (MovAvgFltr(&curr_fltr, VESC[PUSHSHOT_arrID].fdbk.curr) >= VESC_param.OC_curr)
    {
        if (TIMsw_CheckTimeout(&OC_time, VESC_param.OC_time))
            return true;
    }
    else
        TIMsw_Clear(&OC_time);

    return false;
}

static float brake_trigger_time;
void Brake_Trigger(void)
{
    brake_trigger_time = runtime.intvl;
    state_W.ball = state_R.spd_ctrl = 0;
    state_R.brake = 1;

    *(float *)&R1_Data[9] = VESC[PUSHSHOT_arrID].fdbk.spd;
    *(float *)&R1_Data[13] = brake_trigger_time;
}

void State(void *argument)
{
    CYL3_PORT->ODR |= CYL3_PIN; // fan for lidar

    while (1)
    {
#ifdef DATA_OUTPUT
        if (state == SHOT && !state_R.brake)
        {
            static unsigned char VOFA[32];
            static USART_handle_t UART7_handle = {.USART_handle = UART7, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 0}; // USB to TTL

            sprintf((char *)VOFA, "T:%.2f,%d\n", VESC[PUSHSHOT_arrID].fdbk.spd, state_R.spd_ctrl ? 500 : 0);
            UART_SendArray(&UART7_handle, VOFA, 16);
        }
#endif
        switch (state)
        {
        case IDLE:
        {
            // state initialization
            if (state != state_last)
            {
                state_last = state;
            }

            VESC[PUSHSHOT_arrID].ctrl.curr = 0;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_arrID, VESC_SET_CURR);
            }
            break;
        }
        // stay at middle
        case MID:
        {
            static bool revert;

            // state initialization
            if (state != state_last)
            {
                state_last = state;

                TIMsw_Clear(&runtime);
                revert = false;
            }

            // stall protection
            if (VESC_Stall())
            {
                state = IDLE;
                break;
            }

            if (TIMsw_CheckTimeout(&runtime, VESC_param.mid.time))
            {
                state = LOCK;
                break;
            }

            if (PG_BTM)
            {
                TIMsw_Clear(&runtime);
                revert = true;
            }

            VESC[PUSHSHOT_arrID].ctrl.spd = revert ? -VESC_param.low_spd
                                                   : VESC_param.low_spd;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_arrID, VESC_SET_SPD);
            }
            break;
        }
        // spin to bottom fast
        case INIT_FAST:
        {
            // state initialization
            if (state != state_last)
            {
                state_last = state;

                TIMsw_Clear(&runtime);
            }

            // stall protection
            if (VESC_Stall())
            {
                state = IDLE;
                break;
            }

            // bottom photogate
            if (PG_BTM)
            {
                state = LOCK;
                break;
            }

            // low speed initialization
            if (TIMsw_CheckTimeout(&runtime, VESC_param.init.high_spd_time))
            {
                VESC[PUSHSHOT_arrID].ctrl.spd = VESC_param.high_spd + (VESC_param.low_spd - VESC_param.high_spd) *
                                                                          (runtime.intvl - VESC_param.init.high_spd_time < VESC_param.init.decelerate_time ? (runtime.intvl - VESC_param.init.high_spd_time) / VESC_param.init.decelerate_time
                                                                                                                                                           : 1);
            }
            // high speed initialization
            else
            {
                VESC[PUSHSHOT_arrID].ctrl.spd = VESC_param.high_spd;
            }

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_arrID, VESC_SET_SPD);
            }
            break;
        }
        // spin to bottom slowly
        case INIT_SLOW:
        {
            // state initialization
            if (state != state_last)
            {
                state_last = state;
            }

            // stall protection
            if (VESC_Stall())
            {
                state = IDLE;
                break;
            }

            // bottom photogate
            if (PG_BTM)
            {
                state = LOCK;
                break;
            }

            VESC[PUSHSHOT_arrID].ctrl.spd = VESC_param.low_spd;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_arrID, VESC_SET_SPD);
            }
            break;
        }
        // stay at position
        case LOCK:
        {
            // state initialization
            if (state != state_last)
            {
                state_last = state;
            }

            // ball plate go up
            if (state_W.ball && !PG_BTM)
            {
                state = INIT_SLOW;
                break;
            }

            VESC[PUSHSHOT_arrID].ctrl.curr = VESC_param.lock.curr;

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_arrID, VESC_SET_CURR);
            }
            break;
        }
        case SHOT:
        {
            static bool pg_abuse;

            // state initialization
            if (state != state_last)
            {
                state_last = state;

                TIMsw_Clear(&runtime);
                pg_abuse = false;
            }

            if (TIMsw_CheckTimeout(&runtime, VESC_param.shot.timeout) || // timeout
                state_R.brake ||                                         // brake
                PG_TOP && !pg_abuse && runtime.intvl >= 0.1875)          // top photogate not abuse
            {
                if (!state_R.brake)
                    Brake_Trigger();
                else if (runtime.intvl >= brake_trigger_time + VESC_param.shot.brake_time) // total duration
                {
                    state_R.brake = 0;
                    state = IDLE;
                    break;
                }
            }
            else if (PG_TOP)
                pg_abuse = true;

            // shot parameter calculation
            if (state_R.brake)
                VESC[PUSHSHOT_arrID].ctrl.curr = VESC_param.shot.brake_curr;
            else if (!state_R.spd_ctrl)
            {
                if (state_R.fitting &&
                    (VESC_param.shot.spd = state_W.aim_R2 ? state_W.R2_NetUp ? Fitting_Spd_R2_NetUp(R2_info.dist_cm)
                                                                             : Fitting_Spd_R2_NetDown(R2_info.dist_cm)
                                                          : Fitting_Spd_Basket(basket_info.dist_cm)) < 0)
                    VESC_param.shot.spd = 0;
                VESC[PUSHSHOT_arrID].ctrl.spd = VESC_param.shot.spd;

                VESC_param.shot.acc_curr = state_W.aim_R2 ? Fitting_AccCurr_R2(VESC_param.shot.spd)
                                                          : Fitting_AccCurr_Basket(VESC_param.shot.spd);
                LIMIT(VESC_param.shot.acc_curr, PUSHSHOT_MOTOR.curr_max);
                VESC[PUSHSHOT_arrID].ctrl.curr = VESC_param.shot.acc_curr;

                state_R.spd_ctrl = VESC_param.shot.spd - VESC[PUSHSHOT_arrID].fdbk.spd <= VESC_param.shot.spd_ctrl_err; // switch control mode
            }

            // control
            {
                VESC_SendCmd(&hfdcan2, PUSHSHOT_arrID, state_R.brake ? VESC_SET_CURR_BRAKE : (state_R.spd_ctrl ? VESC_SET_SPD : VESC_SET_CURR));
            }
            break;
        }
        }

        if (state_W.gimbal) // gimbal enabled
        {
            // aim at R2
            if (state_W.aim_R2 && R2_info.dist_cm <= 900)
                HighTorque[GIMBAL_arrID].ctrl.pos = HighTorque_param.pos0 + HighTorque_param.R2_offset +
                                                    (R2_yaw_prev + (R2_info.yaw - R2_yaw_prev) * TIMsw_GetRatio(&R2_yaw_time, R2_msg_intvl.intvl)) * Gimbal_GR;
            // aim at basket
            else if (!state_W.aim_R2 && basket_info.dist_cm <= 900)
                HighTorque[GIMBAL_arrID].ctrl.pos = HighTorque_param.pos0 + HighTorque_param.basket_offset +
                                                    basket_info.yaw * Gimbal_GR;
            LIMIT_RANGE(HighTorque[GIMBAL_arrID].ctrl.pos, YAW_MIN, YAW_MAX); // gimbal limit
        }

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, GIMBAL_arrID);
        }

        osDelay(1);
    }
}