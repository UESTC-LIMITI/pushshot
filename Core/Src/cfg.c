#include "usr.h"
#include <stdlib.h>

HighTorque_t HighTorque[HIGHTORQUE_NUM + 1] = {
    {
        .ID = 1,
        .ctrl.pos = (YAW_MAX + YAW_MIN) / 2,
        .ctrl.Kp = 2,
        .ctrl.Kd = 1,
    },
    {.ID = HIGHTORQUE_ID_BCAST},
};

VESC_t VESC[VESC_NUM] = {
    {.ID = 1, .motor = &PUSHSHOT_MOTOR},
};

unsigned char task_intvl_ms_cnt_State = TASK_INTVL_ms_State, task_intvl_ms_cnt_Err = TASK_INTVL_ms_Err, task_intvl_ms_cnt_Comm = TASK_INTVL_ms_Comm;

bool task_timeout;

enum STATE volatile state;

struct STATE_R state_R = {.fitting = 1}; // internal-change state

struct STATE_W state_W; // external-change state

struct target_info basket_info = {.dist_fltr.size = 16, .yaw_fltr.size = 16},
                   R2_info = {.dist_fltr.size = 4, .yaw_fltr.size = 4};

struct ERR err = {
    .VESC = 1,
    .HighTorque = 1,
    .basket_pos = 1,
    .R1_pos = 1,
    .R2_pos = 1,
};

struct ERR_CNT err_cnt;

TIMsw_t runtime, R2_yaw_time, R2_msg_intvl;

float R2_yaw_prev;

char spd_offset;

__attribute__((section(".ARM.__at_0x24000000"))) unsigned char R1_Data[19] = {0xA5};

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

void TIMsw_Init(void)
{
    TIM7->CR1 |= 0x1;
}

void UART5_Init(void)
{
    UART5->CR1 |= 0x20;
    UART5->CR3 |= 0x1;
}

void TIM6_Init(void)
{
    TIM6->DIER |= 0x1;
    TIM6->CR1 |= 0x5;
}

void PeriphInit(void)
{
    FDCAN1_Init();
    FDCAN2_Init();
    FDCAN3_Init();
    TIMsw_Init();
    UART5_Init();

    {
        TIMsw_t *init_time = malloc(sizeof(TIMsw_t));
        while (!TIMsw_CheckTimeout(init_time, 1))
            ;
        free(init_time);
    }
}

void Scheduler(void)
{
    CYL3_PORT->ODR |= CYL3_PIN; // fan for lidar

    TIM6_Init();

    while (1)
    {
        if (task_intvl_ms_cnt_State == TASK_INTVL_ms_State)
        {
            task_intvl_ms_cnt_State = 0;
            State();
        }

        if (task_intvl_ms_cnt_Err == TASK_INTVL_ms_Err)
        {
            task_intvl_ms_cnt_Err = 0;
            Err();
        }

        if (task_intvl_ms_cnt_Comm == TASK_INTVL_ms_Comm)
        {
            task_intvl_ms_cnt_Comm = 0;
            Comm();
        }
    }
}