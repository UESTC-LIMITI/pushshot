#include "user.h"

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
    FDCAN_Filter.FilterIndex = 0;
    FDCAN_Filter.FilterType = FDCAN_FILTER_MASK;
    FDCAN_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_Filter.FilterID1 = 0;
    FDCAN_Filter.FilterID2 = 0;
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
    FDCAN_Filter.FilterIndex = 0;
    FDCAN_Filter.FilterType = FDCAN_FILTER_MASK;
    FDCAN_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_Filter.FilterID1 = 0;
    FDCAN_Filter.FilterID2 = 0;
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
    FDCAN_Filter.FilterID2 = 0xF6;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter);

    FDCAN_Filter.FilterIndex = 3;
    FDCAN_Filter.FilterID1 = 0x104;
    FDCAN_Filter.FilterID2 = 0x105;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter);

    FDCAN_Filter.FilterIndex = 4;
    FDCAN_Filter.FilterID1 = 0x106;
    FDCAN_Filter.FilterID2 = 0x201;
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

// RGB
void TIM16_Init(void)
{
    TIM16->EGR |= 0x1;
    TIM16->CCER |= 1;
    TIM16->CR1 |= 1;
}

void UART5_Init(void)
{
    UART5->CR3 |= 0x40;

    DMA1_Stream2->NDTR = 10;
    DMA1_Stream2->PAR = (unsigned)&UART5->RDR;
    DMA1_Stream2->M0AR = (unsigned)RxData_D1S2;
    DMA1_Stream2->CR |= 0x11;
}