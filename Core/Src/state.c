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
    .basket_offset = 13,
    .R2_offset = 13,
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

#define SPD_BASKET_k(SEG_START, SEG_END) (float)(SPD_BASKET_##SEG_END - SPD_BASKET_##SEG_START) / (SEG_END - SEG_START)
#define SPD_BASKET_b(SEG_START, SEG_END) (SPD_BASKET_##SEG_START - SPD_BASKET_k(SEG_START, SEG_END) * SEG_START)

#define SPD_BASKET_LIN_CALC(SEG_START, SEG_END, dist_cm) SPD_BASKET_k(SEG_START, SEG_END) * dist_cm + SPD_BASKET_b(SEG_START, SEG_END)

#define SPD_BASKET_200 623.5
#define SPD_BASKET_300 688.5
#define SPD_BASKET_400 761
#define SPD_BASKET_500 833.5
#define SPD_BASKET_600 896

float Fitting_Spd_Basket(float dist_cm)
{
    if (dist_cm <= 300)
        return SPD_BASKET_LIN_CALC(200, 300, dist_cm);
    else if (dist_cm <= 400)
        return SPD_BASKET_LIN_CALC(300, 400, dist_cm);
    else if (dist_cm <= 500)
        return SPD_BASKET_LIN_CALC(400, 500, dist_cm);
    else
        return SPD_BASKET_LIN_CALC(500, 600, dist_cm);
}

#define SPD_R2_NETDOWN_k(SEG_START, SEG_END) (float)(SPD_R2_NETDOWN_##SEG_END - SPD_R2_NETDOWN_##SEG_START) / (SEG_END - SEG_START)
#define SPD_R2_NETDOWN_b(SEG_START, SEG_END) (SPD_R2_NETDOWN_##SEG_START - SPD_R2_NETDOWN_k(SEG_START, SEG_END) * SEG_START)

#define SPD_R2_NETDOWN_LIN_CALC(SEG_START, SEG_END, dist_cm) SPD_R2_NETDOWN_k(SEG_START, SEG_END) * dist_cm + SPD_R2_NETDOWN_b(SEG_START, SEG_END)

#define SPD_R2_NETDOWN_150 522
#define SPD_R2_NETDOWN_170 541
#define SPD_R2_NETDOWN_190 555
#define SPD_R2_NETDOWN_205 564
#define SPD_R2_NETDOWN_220 574
#define SPD_R2_NETDOWN_240 590
#define SPD_R2_NETDOWN_260 606
#define SPD_R2_NETDOWN_280 622
#define SPD_R2_NETDOWN_300 637
#define SPD_R2_NETDOWN_320 652
#define SPD_R2_NETDOWN_340 664
#define SPD_R2_NETDOWN_360 677
#define SPD_R2_NETDOWN_370 683
#define SPD_R2_NETDOWN_380 693
#define SPD_R2_NETDOWN_400 709
#define SPD_R2_NETDOWN_420 727
#define SPD_R2_NETDOWN_440 745
#define SPD_R2_NETDOWN_460 763
#define SPD_R2_NETDOWN_470 769
#define SPD_R2_NETDOWN_480 776
#define SPD_R2_NETDOWN_500 788
#define SPD_R2_NETDOWN_520 797
#define SPD_R2_NETDOWN_540 817
#define SPD_R2_NETDOWN_560 831
#define SPD_R2_NETDOWN_580 846
#define SPD_R2_NETDOWN_605 864
#define SPD_R2_NETDOWN_630 880
#define SPD_R2_NETDOWN_650 895

float Fitting_Spd_R2_NetDown(float dist_cm)
{
    if (dist_cm <= 170)
        return SPD_R2_NETDOWN_LIN_CALC(150, 170, dist_cm);
    else if (dist_cm <= 190)
        return SPD_R2_NETDOWN_LIN_CALC(170, 190, dist_cm);
    else if (dist_cm <= 205)
        return SPD_R2_NETDOWN_LIN_CALC(190, 205, dist_cm);
    else if (dist_cm <= 220)
        return SPD_R2_NETDOWN_LIN_CALC(205, 220, dist_cm);
    else if (dist_cm <= 240)
        return SPD_R2_NETDOWN_LIN_CALC(220, 240, dist_cm);
    else if (dist_cm <= 260)
        return SPD_R2_NETDOWN_LIN_CALC(240, 260, dist_cm);
    else if (dist_cm <= 280)
        return SPD_R2_NETDOWN_LIN_CALC(260, 280, dist_cm);
    else if (dist_cm <= 300)
        return SPD_R2_NETDOWN_LIN_CALC(280, 300, dist_cm);
    else if (dist_cm <= 320)
        return SPD_R2_NETDOWN_LIN_CALC(300, 320, dist_cm);
    else if (dist_cm <= 340)
        return SPD_R2_NETDOWN_LIN_CALC(320, 340, dist_cm);
    else if (dist_cm <= 360)
        return SPD_R2_NETDOWN_LIN_CALC(340, 360, dist_cm);
    else if (dist_cm <= 370)
        return SPD_R2_NETDOWN_LIN_CALC(360, 370, dist_cm);
    else if (dist_cm <= 380)
        return SPD_R2_NETDOWN_LIN_CALC(370, 380, dist_cm);
    else if (dist_cm <= 400)
        return SPD_R2_NETDOWN_LIN_CALC(380, 400, dist_cm);
    else if (dist_cm <= 420)
        return SPD_R2_NETDOWN_LIN_CALC(400, 420, dist_cm);
    else if (dist_cm <= 440)
        return SPD_R2_NETDOWN_LIN_CALC(420, 440, dist_cm);
    else if (dist_cm <= 460)
        return SPD_R2_NETDOWN_LIN_CALC(440, 460, dist_cm);
    else if (dist_cm <= 470)
        return SPD_R2_NETDOWN_LIN_CALC(460, 470, dist_cm);
    else if (dist_cm <= 480)
        return SPD_R2_NETDOWN_LIN_CALC(470, 480, dist_cm);
    else if (dist_cm <= 500)
        return SPD_R2_NETDOWN_LIN_CALC(480, 500, dist_cm);
    else if (dist_cm <= 520)
        return SPD_R2_NETDOWN_LIN_CALC(500, 520, dist_cm);
    else if (dist_cm <= 540)
        return SPD_R2_NETDOWN_LIN_CALC(520, 540, dist_cm);
    else if (dist_cm <= 560)
        return SPD_R2_NETDOWN_LIN_CALC(540, 560, dist_cm);
    else if (dist_cm <= 580)
        return SPD_R2_NETDOWN_LIN_CALC(560, 580, dist_cm);
    else if (dist_cm <= 605)
        return SPD_R2_NETDOWN_LIN_CALC(580, 605, dist_cm);
    else if (dist_cm <= 630)
        return SPD_R2_NETDOWN_LIN_CALC(605, 630, dist_cm);
    else
        return SPD_R2_NETDOWN_LIN_CALC(630, 650, dist_cm);
}

#define SPD_R2_NETUP_k(SEG_START, SEG_END) (float)(SPD_R2_NETUP_##SEG_END - SPD_R2_NETUP_##SEG_START) / (SEG_END - SEG_START)
#define SPD_R2_NETUP_b(SEG_START, SEG_END) (SPD_R2_NETUP_##SEG_START - SPD_R2_NETUP_k(SEG_START, SEG_END) * SEG_START)

#define SPD_R2_NETUP_LIN_CALC(SEG_START, SEG_END, dist_cm) SPD_R2_NETUP_k(SEG_START, SEG_END) * dist_cm + SPD_R2_NETUP_b(SEG_START, SEG_END)

#define SPD_R2_NETUP_250 621
#define SPD_R2_NETUP_275 639.5
#define SPD_R2_NETUP_300 658
#define SPD_R2_NETUP_325 673
#define SPD_R2_NETUP_350 688
#define SPD_R2_NETUP_375 705
#define SPD_R2_NETUP_400 730
#define SPD_R2_NETUP_425 747
#define SPD_R2_NETUP_450 766
#define SPD_R2_NETUP_475 794
#define SPD_R2_NETUP_500 809
#define SPD_R2_NETUP_525 815
#define SPD_R2_NETUP_550 846
#define SPD_R2_NETUP_575 860
#define SPD_R2_NETUP_600 879
#define SPD_R2_NETUP_625 892
#define SPD_R2_NETUP_650 950
#define SPD_R2_NETUP_675 971
#define SPD_R2_NETUP_700 992

float Fitting_Spd_R2_NetUp(float dist_cm)
{
    if (dist_cm <= 275)
        return SPD_R2_NETUP_LIN_CALC(250, 275, dist_cm);
    else if (dist_cm <= 300)
        return SPD_R2_NETUP_LIN_CALC(275, 300, dist_cm);
    else if (dist_cm <= 325)
        return SPD_R2_NETUP_LIN_CALC(300, 325, dist_cm);
    else if (dist_cm <= 350)
        return SPD_R2_NETUP_LIN_CALC(325, 350, dist_cm);
    else if (dist_cm <= 375)
        return SPD_R2_NETUP_LIN_CALC(350, 375, dist_cm);
    else if (dist_cm <= 400)
        return SPD_R2_NETUP_LIN_CALC(375, 400, dist_cm);
    else if (dist_cm <= 425)
        return SPD_R2_NETUP_LIN_CALC(400, 425, dist_cm);
    else if (dist_cm <= 450)
        return SPD_R2_NETUP_LIN_CALC(425, 450, dist_cm);
    else if (dist_cm <= 475)
        return SPD_R2_NETUP_LIN_CALC(450, 475, dist_cm);
    else if (dist_cm <= 500)
        return SPD_R2_NETUP_LIN_CALC(475, 500, dist_cm);
    else if (dist_cm <= 525)
        return SPD_R2_NETUP_LIN_CALC(500, 525, dist_cm);
    else if (dist_cm <= 550)
        return SPD_R2_NETUP_LIN_CALC(525, 550, dist_cm);
    else if (dist_cm <= 575)
        return SPD_R2_NETUP_LIN_CALC(550, 575, dist_cm);
    else if (dist_cm <= 600)
        return SPD_R2_NETUP_LIN_CALC(575, 600, dist_cm);
    else if (dist_cm <= 625)
        return SPD_R2_NETUP_LIN_CALC(600, 625, dist_cm);
    else if (dist_cm <= 650)
        return SPD_R2_NETUP_LIN_CALC(625, 650, dist_cm);
    else if (dist_cm <= 675)
        return SPD_R2_NETUP_LIN_CALC(650, 675, dist_cm);
    else
        return SPD_R2_NETUP_LIN_CALC(675, 700, dist_cm);
}

void VESC_Stall_Init(void)
{
    MovAvgFltr_Clear(&curr_fltr);
    TIMsw_Clear(&OC_time);
}

bool VESC_Stall(void)
{
    if (MovAvgFltr(&curr_fltr, VESC[PUSHSHOT_idx].fdbk.curr) >= VESC_param.OC_curr)
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
    state_W.ball = state_R.spd_ctrl = false;
    state_R.brake = true;

    *(float *)&R1_data[9] = end_spd = VESC[PUSHSHOT_idx].fdbk.spd;
    *(float *)&R1_data[13] = brake_trigger_time;
}

void State(void)
{
#ifdef DATA_OUTPUT
    if (state == SHOT && !state_R.brake)
    {
        static unsigned char VOFA[32];
        static USART_handle_t UART7_handle = {.USART_handle = UART7, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 0}; // USB to TTL

        sprintf((char *)VOFA, "T:%.2f,%d\n", VESC[PUSHSHOT_idx].fdbk.spd, state_R.spd_ctrl ? 500 : 0);
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

            TIMsw_Clear(&runtime);
        }

        if (state_last != MID)
        {
            if (PG_TOP && TIMsw_CheckTimeout(&runtime, 0.125))
            {
                state = MID;
                break;
            }
            else if (!PG_TOP)
            {
                TIMsw_Clear(&runtime);
            }
        }

        VESC[PUSHSHOT_idx].ctrl.curr = 0;

        // control
        {
            VESC_SendCmd(&hfdcan2, PUSHSHOT_idx, VESC_SET_CURR);
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

        VESC[PUSHSHOT_idx].ctrl.spd = VESC_param.low_spd;

        // control
        {
            VESC_SendCmd(&hfdcan2, PUSHSHOT_idx, VESC_SET_SPD);
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

        VESC[PUSHSHOT_idx].ctrl.spd = VESC_param.low_spd;

        // control
        {
            VESC_SendCmd(&hfdcan2, PUSHSHOT_idx, VESC_SET_SPD);
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
            VESC[PUSHSHOT_idx].ctrl.spd = VESC_param.high_spd + (VESC_param.low_spd - VESC_param.high_spd) *
                                                                    (runtime.intvl - VESC_param.init.high_spd_time < VESC_param.init.decelerate_time ? (runtime.intvl - VESC_param.init.high_spd_time) / VESC_param.init.decelerate_time
                                                                                                                                                     : 1);
        }
        // high speed initialization
        else
        {
            VESC[PUSHSHOT_idx].ctrl.spd = VESC_param.high_spd;
        }

        // control
        {
            VESC_SendCmd(&hfdcan2, PUSHSHOT_idx, VESC_SET_SPD);
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

        VESC[PUSHSHOT_idx].ctrl.curr = VESC_param.lock.curr;

        // control
        {
            VESC_SendCmd(&hfdcan2, PUSHSHOT_idx, VESC_SET_CURR);
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
                state_R.brake = false;
                state = IDLE;
                break;
            }
        }
        else if (PG_TOP)
            pg_abuse = true;

        // shot parameter calculation
        if (state_R.brake)
            VESC[PUSHSHOT_idx].ctrl.curr = VESC_param.shot.brake_curr;
        else if (!state_R.spd_ctrl)
        {
            // speed <= 0
            if (state_R.fitting &&
                    (VESC_param.shot.spd = state_W.aim_R2 ? state_W.R2_NetUp ? Fitting_Spd_R2_NetUp(R2_info.dist_cm)
                                                                             : Fitting_Spd_R2_NetDown(R2_info.dist_cm)
                                                          : Fitting_Spd_Basket(basket_info.dist_cm)) <= 0 ||
                VESC_param.shot.spd <= 0)
            {
                state = IDLE;
                break;
            }

            VESC_param.shot.acc_curr = Fitting_AccCurr(VESC[PUSHSHOT_idx].ctrl.spd = VESC_param.shot.spd + spd_offset);
            VESC[PUSHSHOT_idx].ctrl.curr = LIMIT(VESC_param.shot.acc_curr, PUSHSHOT_MOTOR.curr_max);

            state_R.spd_ctrl = VESC_param.shot.spd - VESC[PUSHSHOT_idx].fdbk.spd <= VESC_param.shot.spd_ctrl_err; // switch control mode
        }

        // control
        {
            VESC_SendCmd(&hfdcan2, PUSHSHOT_idx, state_R.brake ? VESC_SET_CURR_BRAKE : (state_R.spd_ctrl ? VESC_SET_SPD : VESC_SET_CURR));
        }
        break;
    }
    }

    // gimbal
    {
        // aim at R2
        if (state_W.aim_R2 && R2_info.dist_cm <= 900)
            HighTorque[GIMBAL_idx].ctrl.pos = GIMBAL_0 + HighTorque_param.R2_offset +
                                              (R2_yaw_prev + (R2_info.yaw - R2_yaw_prev) * TIMsw_GetRatio(&R2_yaw_time, R2_msg_intvl.intvl)) * GIMBAL_GR;
        // aim at basket
        else if (!state_W.aim_R2 && basket_info.dist_cm <= 900)
            HighTorque[GIMBAL_idx].ctrl.pos = GIMBAL_0 + HighTorque_param.basket_offset +
                                              basket_info.yaw * GIMBAL_GR;
        LIMIT_RANGE(HighTorque[GIMBAL_idx].ctrl.pos, GIMBAL_MIN, GIMBAL_MAX); // gimbal limit

        // control
        {
            HighTorque_SetMixParam_f(&hfdcan1, GIMBAL_idx);
        }
    }
}