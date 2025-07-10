#include "usr.h"

HighTorque_t HighTorque[HIGHTORQUE_NUM + 1] = {
    {.ID = 1,
     .ctrl.pos = (YAW_MAX + YAW_MIN) / 2,
     .ctrl.Kp = 2,
     .ctrl.Kd = 1},
    {.ID = HIGHTORQUE_ID_BCAST},
};

VESC_t VESC[VESC_NUM] = {
    {.ID = 1, .motor = &PUSHSHOT_MOTOR},
};

enum STATE state;

struct STATE_R state_R = { // internal-change state
    .fitting = 1};

struct STATE_W state_W; // external-change state

struct target_info basket_info = {.dist_fltr.size = 16, .yaw_fltr.size = 16},
                   R2_info = {.dist_fltr.size = 4, .yaw_fltr.size = 4};

struct ERR err;

struct ERR_CNT err_cnt;

TIMsw_t runtime, R2_yaw_time, R2_msg_intvl;

float R2_yaw_prev;

char spd_offset;

// call each function under corresponding init function created by CubeMX

void FDCAN1_Init(void)
{
    FDCAN_FilterTypeDef FDCAN_Filter;
    FDCAN_Filter.IdType = FDCAN_STANDARD_ID;
    FDCAN_Filter.FilterIndex = 0;
    FDCAN_Filter.FilterType = FDCAN_FILTER_MASK;
    FDCAN_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_Filter.FilterID1 = 0;
    FDCAN_Filter.FilterID2 = 0;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter);

    FDCAN_Filter.IdType = FDCAN_EXTENDED_ID;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
}

void FDCAN2_Init(void)
{
    FDCAN_FilterTypeDef FDCAN_Filter;
    FDCAN_Filter.IdType = FDCAN_STANDARD_ID;
    FDCAN_Filter.FilterIndex = 0;
    FDCAN_Filter.FilterType = FDCAN_FILTER_MASK;
    FDCAN_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_Filter.FilterID1 = 0;
    FDCAN_Filter.FilterID2 = 0;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_Filter);

    FDCAN_Filter.IdType = FDCAN_EXTENDED_ID;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_Filter);

    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);
}

void FDCAN3_Init(void)
{
    FDCAN_FilterTypeDef FDCAN_Filter;
    FDCAN_Filter.IdType = FDCAN_STANDARD_ID;
    FDCAN_Filter.FilterIndex = 0;
    FDCAN_Filter.FilterType = FDCAN_FILTER_RANGE;
    FDCAN_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_Filter.FilterID1 = 0xA;
    FDCAN_Filter.FilterID2 = 0xC;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter);

    FDCAN_Filter.FilterIndex = 1;
    FDCAN_Filter.FilterType = FDCAN_FILTER_DUAL;
    FDCAN_Filter.FilterID1 = 0xE;
    FDCAN_Filter.FilterID2 = 0xF;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter);

    FDCAN_Filter.FilterIndex = 2;
    FDCAN_Filter.FilterID1 = 0x14;
    FDCAN_Filter.FilterID2 = 0xA3;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter);

    FDCAN_Filter.FilterIndex = 3;
    FDCAN_Filter.FilterType = FDCAN_FILTER_RANGE;
    FDCAN_Filter.FilterID1 = 0xA5;
    FDCAN_Filter.FilterID2 = 0xA7;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter);

    FDCAN_Filter.FilterIndex = 4;
    FDCAN_Filter.FilterType = FDCAN_FILTER_DUAL;
    FDCAN_Filter.FilterID1 = 0xDF;
    FDCAN_Filter.FilterID2 = 0x105;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter);

    FDCAN_Filter.FilterIndex = 5;
    FDCAN_Filter.FilterID1 = 0x201;
    FDCAN_Filter.FilterID2 = 0x21F;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter);

    FDCAN3->GFC = 0x3F;

    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan3);
}

// TIMER
void TIM7_Init(void)
{
    TIM7->CR1 |= 1;
}

void UART5_Init(void)
{
    UART5->CR1 |= 0x20;
    UART5->CR3 |= 0x1;
}