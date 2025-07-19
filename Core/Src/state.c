#include "usr.h"

static enum STATE state_last;

static MovAvgFltr_t curr_fltr = {.size = 48};
static TIMsw_t OC_time;

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
        const float spd_ctrl_err, brake_curr, timeout, pg_abuse_detection_time, brake_time;
    } shot;
} VESC_param = {
    .high_spd = -300,
    .low_spd = -150,
    .OC_curr = 30.1,
    .OC_time = 0.25,

    .mid.time = 0.375,

    .init.high_spd_time = 0.375,
    .init.decelerate_time = 0.25,

    .lock.curr = -5.65625,

    .shot.spd_ctrl_err = 50,
    .shot.brake_curr = 30.1,
    .shot.timeout = 0.5,
    .shot.pg_abuse_detection_time = 0.125,
    .shot.brake_time = 0.25,
};

struct
{
    const float basket_offset, R2_offset;
} HighTorque_param = {
    .basket_offset = 8,
    .R2_offset = 0,
};

static float brake_trigger_time;
float end_spd;

float Fitting_AccCurr(float spd)
{
    // 10A, 400RPM
    if (spd <= 400)
        return 10;
    // 15A, 500RPM
    else if (spd <= 500)
        return spd * 0.05 - 10;
    // 25A, 600RPM
    // 35A, 700RPM
    // 45A, 800RPM
    else if (spd <= 800)
        return spd * 0.1 - 35;
    // 55A, 850RPM
    // 65A, 900RPM
    else
        return spd * 0.2 - 115;
}

float Fitting_Spd_Basket(float dist_cm)
{
    if (dist_cm <= 300)
        return 0.6 * dist_cm +
               480 + spd_offset;
    else if (dist_cm <= 400)
        return 0.65 * dist_cm +
               465 + spd_offset;
    else if (dist_cm <= 500)
        return 0.55 * dist_cm +
               505 + spd_offset;
    else if (dist_cm <= 600)
        return 0.5 * dist_cm +
               530 + spd_offset;
    else
        return 0.5 * dist_cm +
               530 + spd_offset;
}

float Fitting_Spd_R2_NetDown(float dist_cm)
{
    if (dist_cm <= 181)
        return 0.842 * dist_cm + 361.8 + spd_offset;
    else if (dist_cm <= 206)
        return 0.94 * dist_cm + 342.86 + spd_offset;
    else if (dist_cm <= 231)
        return 0.82 * dist_cm + 367.58 + spd_offset;
    else if (dist_cm <= 256)
        return 0.8 * dist_cm + 372.2 + spd_offset;
    else if (dist_cm <= 281)
        return 0.84 * dist_cm + 361.96 + spd_offset;
    else if (dist_cm <= 306)
        return 0.6 * dist_cm + 429.4 + spd_offset;
    else if (dist_cm <= 331)
        return 0.64 * dist_cm + 417.16 + spd_offset;
    else if (dist_cm <= 356)
        return 0.72 * dist_cm + 390.68 + spd_offset;
    else if (dist_cm <= 381)
        return 0.88 * dist_cm + 333.72 + spd_offset;
    else if (dist_cm <= 406)
        return 0.72 * dist_cm + 394.68 + spd_offset;
    else if (dist_cm <= 431)
        return 0.76 * dist_cm + 378.44 + spd_offset;
    else if (dist_cm <= 456)
        return 0.8 * dist_cm + 361.2 + spd_offset;
    else if (dist_cm <= 481)
        return 0.68 * dist_cm + 415.92 + spd_offset;
    else if (dist_cm <= 506)
        return 0.64 * dist_cm + 435.16 + spd_offset;
    else if (dist_cm <= 531)
        return 0.52 * dist_cm + 495.88 + spd_offset;
    else if (dist_cm <= 556)
        return 0.52 * dist_cm + 495.88 + spd_offset;
    else if (dist_cm <= 581)
        return 0.48 * dist_cm + 518.12 + spd_offset;
    else if (dist_cm <= 606)
        return 0.48 * dist_cm + 518.12 + spd_offset;
    else if (dist_cm <= 631)
        return 0.52 * dist_cm + 493.88 + spd_offset;
    else
        return 0.5006 * dist_cm + 506.10 + spd_offset;
}

float Fitting_Spd_R2_NetUp(float dist_cm)
{
    if (dist_cm <= 275)
        return 0.66 * dist_cm + 438 + spd_offset;
    else if (dist_cm <= 300)
        return 0.8 * dist_cm + 398 + spd_offset;
    else if (dist_cm <= 325)
        return 0.6 * dist_cm + 458 + spd_offset;
    else if (dist_cm <= 350)
        return 0.6 * dist_cm + 458 + spd_offset;
    else if (dist_cm <= 375)
        return 0.68 * dist_cm + 430 + spd_offset;
    else if (dist_cm <= 400)
        return 0.68 * dist_cm + 430 + spd_offset;
    else if (dist_cm <= 425)
        return 0.68 * dist_cm + 430 + spd_offset;
    else if (dist_cm <= 450)
        return 0.68 * dist_cm + 430 + spd_offset;
    else if (dist_cm <= 475)
        return 0.76 * dist_cm + 394 + spd_offset;
    else if (dist_cm <= 500)
        return 0.76 * dist_cm + 394 + spd_offset;
    else if (dist_cm <= 525)
        return 0.44 * dist_cm + 394 + spd_offset;
    else if (dist_cm <= 550)
        return 0.44 * dist_cm + 554 + spd_offset;
    else if (dist_cm <= 575)
        return 0.44 * dist_cm + 554 + spd_offset;
    else if (dist_cm <= 600)
        return 0.44 * dist_cm + 554 + spd_offset;
    else if (dist_cm <= 625)
        return 0.44 * dist_cm + 554 + spd_offset;
    else if (dist_cm <= 650)
        return 0.44 * dist_cm + 554 + spd_offset;
    else
        return 0.44 * dist_cm + 554 + spd_offset;
}

void VESC_Stall_Init(void)
{
    MovAvgFltr_Clear(&curr_fltr);
    TIMsw_Clear(&OC_time);
}

bool VESC_Stall(void)
{
    if (MovAvgFltr(&curr_fltr, VESC[PUSHSHOT_arrID].fdbk.curr) >= VESC_param.OC_curr)
    {
        if (TIMsw_CheckTimeout(&OC_time, VESC_param.OC_time))
            return true;
    }
    else
        TIMsw_Clear(&OC_time);

    return false;
}

void Brake_Trigger(void)
{
    if (state_R.brake)
        return;

    brake_trigger_time = runtime.intvl;
    state_W.ball = state_R.spd_ctrl = 0;
    state_R.brake = 1;

    *(float *)&R1_data[9] = end_spd = VESC[PUSHSHOT_arrID].fdbk.spd;
    *(float *)&R1_data[13] = brake_trigger_time;
}

void State(void)
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
            if (state_last != MID)
                state_last = state;
        }

        if (state_last != MID && PG_TOP)
        {
            state = MID;
            break;
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
        // state initialization
        if (state != state_last)
        {
            state_last = state;

            VESC_Stall_Init();
            TIMsw_Clear(&runtime);
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
        else if (PG_TOP)
        {
            TIMsw_Clear(&runtime);
        }

        VESC[PUSHSHOT_arrID].ctrl.spd = VESC_param.low_spd;

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

            VESC_Stall_Init();
            TIMsw_Clear(&runtime);
        }

        // stall protection
        if (VESC_Stall())
        {
            state = IDLE;
            break;
        }

        if (PG_BTM)
        {
            state = LOCK;
            break;
        }
        else if (PG_TOP)
        {
            TIMsw_Clear(&runtime);
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

            VESC_Stall_Init();
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
            if (state_last != MID)
                state_last = state;
        }

        // ball plate go up
        if (state_last != MID && !PG_BTM)
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

        if (TIMsw_CheckTimeout(&runtime, VESC_param.shot.timeout) ||                         // timeout
            state_R.brake ||                                                                 // brake
            PG_TOP && !pg_abuse && runtime.intvl >= VESC_param.shot.pg_abuse_detection_time) // top photogate not abuse
        {
            Brake_Trigger();

            if (runtime.intvl >= brake_trigger_time + VESC_param.shot.brake_time) // total duration
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

            VESC_param.shot.acc_curr = Fitting_AccCurr(VESC_param.shot.spd);
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

    // gimbal
    {
        // aim at R2
        if (state_W.aim_R2 && R2_info.dist_cm <= 900)
            HighTorque[GIMBAL_arrID].ctrl.pos = GIMBAL_0 + HighTorque_param.R2_offset +
                                                (R2_yaw_prev + (R2_info.yaw - R2_yaw_prev) * TIMsw_GetRatio(&R2_yaw_time, R2_msg_intvl.intvl)) * GIMBAL_GR;
        // aim at basket
        else if (!state_W.aim_R2 && basket_info.dist_cm <= 900)
            HighTorque[GIMBAL_arrID].ctrl.pos = GIMBAL_0 + HighTorque_param.basket_offset +
                                                basket_info.yaw * GIMBAL_GR;
        LIMIT_RANGE(HighTorque[GIMBAL_arrID].ctrl.pos, GIMBAL_MIN, GIMBAL_MAX); // gimbal limit

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, GIMBAL_arrID);
        }
    }
}