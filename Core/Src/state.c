#include "user.h"

USART_info_t UART7_info = {.USART_handle = UART7},
             USART3_info = {.USART_handle = USART3, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream0, .DMA_ID = 1},
             USART6_info = {.USART_handle = USART6};

enum STATE state = -1;
struct STATE_R state_R;
struct STATE_W state_W = {
    .fitting = 0};
timer_t runtime;

#ifdef DATA_OUTPUT
unsigned char VOFA[32];
#endif
struct
{
    float init_spd, shot_curr_pct, shot_spd, spd_ctrl_pct, brake_curr_pct;
} VESC_param = {
    .init_spd = 100,
    .shot_curr_pct = 0.09,
    .shot_spd = 1020,
    .spd_ctrl_pct = 0.91,
    .brake_curr_pct = 0.03};

struct
{
    float pos_0, pos_target;
} HighTorque_param = {
    .pos_0 = 0};

// gimbal limit
#define YAW_MIN -175
#define YAW_MAX 175

struct
{
    float M2006_trq;
} M2006_param = {
    .M2006_trq = 0.2};

struct target_info basket_info_pos, basket_info_pointcloud, basket_info, R2_info;
float weight_pointcloud = 0;
MovAvgFltr_t M2006_fltr;
timer_t M2006_time, HighTorque_time;

float test_curr;

float Fitting_Calc_Basket(void)
{
    if (basket_info.dist_cm >= 650)
    {
        float offset = basket_info.dist_cm - 650;
        return 1240 + 0.914 * offset + 0.00258 * pow(offset, 2) - 0.0000172 * pow(offset, 3);
    }
    else if (basket_info.dist_cm >= 600)
    {
        float offset = basket_info.dist_cm - 600;
        return 1200 + 0.7016 * offset + 0.001662 * pow(offset, 2) + 0.0000061233 * pow(offset, 3);
    }
    else if (basket_info.dist_cm >= 550)
    {
        float offset = basket_info.dist_cm - 550;
        return 1170 + 0.48 * offset + 0.002769 * pow(offset, 2) - 0.0000073848 * pow(offset, 3);
    }
    else if (basket_info.dist_cm >= 500)
    {
        float offset = basket_info.dist_cm - 500;
        return 1145 + 0.6785 * offset - 0.006738 * pow(offset, 2) + 0.000063385 * pow(offset, 3);
    }
    else if (basket_info.dist_cm >= 450)
    {
        float offset = basket_info.dist_cm - 450;
        return 1100 + 1.0062 * offset + 0.0001846 * pow(offset, 2) - 0.000046154 * pow(offset, 3);
    }
    else
    {
        float offset = basket_info.dist_cm - 400;
        return 1050 + 0.9969 * offset + 0.0000012308 * pow(offset, 3);
    }
}

// @todo fitting for pass ball
float Fitting_Calc_R2(void)
{
    return 0;
}

void State(void *argument)
{
    // GPIOD->ODR |= 0x80; // fan for lidar

    // default param
    HighTorque[1 - HIGHTORQUE_ID_OFFSET].ctrl.pos = HighTorque_param.pos_0;
    HighTorque[1 - HIGHTORQUE_ID_OFFSET].ctrl.Kp = 5;
    HighTorque[1 - HIGHTORQUE_ID_OFFSET].ctrl.Kd = 1;

    while (1)
    {
#ifdef DATA_OUTPUT
        if (VESC[1 - VESC_ID_OFFSET].fdbk.spd > 105 && !state_R.brake)
        {
            sprintf(VOFA, "T:%.2f,%d\n", VESC[1 - VESC_ID_OFFSET].fdbk.spd, state_R.spd_ctrl ? 1000 : 0);
            UART_SendArray(&UART7_info, VOFA, 16);
        }
#endif
        switch (state)
        {
        case INIT:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.init_spd; // spin to init pos

            // at pos
            if (!(GPIOE->IDR & 0x4))
            {
                state_R.shot_ready = 1;
                state = IDLE;
            }
            break;
        }
        case IDLE:
        {
            VESC[1 - VESC_ID_OFFSET].ctrl.curr = 0;
            VESC[1 - VESC_ID_OFFSET].ctrl.spd = 0;

            // automatic init
            if (state_W.ball && !state_R.shot_ready)
                state = INIT;
            break;
        }
        case SHOT:
        {
            if (state_W.fitting &&
                !VESC[1 - VESC_ID_OFFSET].ctrl.spd) // spd not set
            {
                VESC_param.shot_spd = state_W.aim_R2 ? Fitting_Calc_R2()      // fit for pass ball
                                                     : Fitting_Calc_Basket(); // fit for shoot ball
            }

            VESC[1 - VESC_ID_OFFSET].ctrl.curr = VESC_param.shot_spd * (state_R.brake ? VESC_param.brake_curr_pct  // curr for brake
                                                                                      : VESC_param.shot_curr_pct); // curr for acceleration
            LIMIT(VESC[1 - VESC_ID_OFFSET].ctrl.curr, 120);                                                        // ESC curr limit

            VESC[1 - VESC_ID_OFFSET].ctrl.spd = VESC_param.shot_spd; // target spd

            // switch ctrl mode
            if (VESC[1 - VESC_ID_OFFSET].fdbk.spd / VESC[1 - VESC_ID_OFFSET].ctrl.spd >= VESC_param.spd_ctrl_pct)
                state_R.spd_ctrl = 1;

            if (Timer_CheckTimeout(&runtime, 0.3)) // shot duration
            {
                state_R.brake = 1;

                if (runtime.intvl >= 0.5) // total duration
                {
                    state_R.shot_ready = state_W.ball = state_R.brake = state_R.spd_ctrl = 0;
                    Timer_Clear(&runtime);
                    Timer_Clear(&HighTorque_time);
                    state = IDLE;
                }
            }
            break;
        }
        }

        basket_info.dist_cm = basket_info_pointcloud.dist_cm * (CAN_fault.basket_info_pos ? 1 : weight_pointcloud) + basket_info_pos.dist_cm * (CAN_fault.basket_info_pointcloud ? 1 : 1 - weight_pointcloud);
        basket_info.yaw = basket_info_pointcloud.yaw * (CAN_fault.basket_info_pos ? 1 : weight_pointcloud) + basket_info_pos.yaw * (CAN_fault.basket_info_pointcloud ? 1 : 1 - weight_pointcloud);
        basket_info.height_cm = basket_info_pointcloud.height_cm * (CAN_fault.basket_info_pos ? 1 : weight_pointcloud) + basket_info_pos.height_cm * (CAN_fault.basket_info_pointcloud ? 1 : 1 - weight_pointcloud);

        // delay after shot
        if (Timer_CheckTimeout(&HighTorque_time, 0.1))
            HighTorque[1 - HIGHTORQUE_ID_OFFSET]
                .ctrl.pos = HighTorque_param.pos_0 + (state_W.ball ||            // auto init
                                                              state_R.shot_ready // manual init
                                                          ? (state_W.aim_R2 ? R2_info.yaw
                                                                            : basket_info.yaw) *
                                                                Gimbal_GR // aim at target
                                                          : 0);           // stay mid

        LIMIT_RANGE(HighTorque[1 - HIGHTORQUE_ID_OFFSET].ctrl.pos, YAW_MAX, YAW_MIN); // gimbal limit

        // use with controller
        C610[1 - 1].ctrl.trq = state_W.defense ? -M2006_param.M2006_trq : M2006_param.M2006_trq; // turn to spd ctrl?
        if (!CAN_fault.M2006 &&
            !C610[1 - 1].ctrl.pos &&                                             // pos not set
            Timer_CheckTimeout(&M2006_time, 0.5) &&                              // time for startup
            MovAvgFltr_GetTargetStatus(&M2006_fltr, C610[1 - 1].fdbk.spd, 0, 5)) // almost stop
            C610[1 - 1].ctrl.pos = C610[1 - 1].fdbk.pos + (state_W.defense ? 5 : -5);

        osDelay(1);
    }
}