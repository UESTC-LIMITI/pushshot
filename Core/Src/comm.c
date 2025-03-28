#include "user.h"

void Comm(void *argument)
{
    while (1)
    {
        unsigned char TxData[] = {state_W.ball, CAN_fault.VESC, CAN_fault.HighTorque, CAN_fault.basket_info, CAN_fault.R2_info};
        FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA0, TxData, 5);
        osDelay(10);
    }
}

void FDCAN3_IT0_IRQHandler(void)
{
    if (FDCAN3->IR & 0x1)
    {
        FDCAN3->IR |= 0x1;

        FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
        uint8_t RxData[64];
        HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxData);

        switch (FDCAN_RxHeader.Identifier)
        {
        case 0xA:
        {
            if (state_R.shot_ready)
                state = SHOT;
            break;
        }
        case 0xB:
        {
            state_W.aim_R2 = 0; // switch target
            break;
        }
        case 0xC:
        {
            state_W.aim_R2 = 1; // switch target
            break;
        }
        case 0xE:
        {
            state = BACK;
            break;
        }
        case 0xF:
        {
            state_W.ball = 1;
            break;
        }
        // case 0x11: // dist and yaw of R2
        // {
        //     CAN_fault.R2_info = 0; // clear CAN fault flag

        //     R2_info.dist_cm = MovAvgFltr(&R2_info.dist_cm_fltr, *(float *)&RxData[8]) / 10;
        //     R2_info.yaw = MovAvgFltr(&R2_info.yaw_fltr, *(float *)&RxData[12]);
        //     break;
        // }
        case 0x14:
        {
            if (state == IDLE)
                state = INIT;
            break;
        }
        case 0x105: // dist and yaw of basket from pos
        {
            CAN_fault.basket_info = 0; // clear CAN fault flag

            basket_info.dist_cm = MovAvgFltr(&basket_info.dist_cm_fltr, *(float *)RxData) * 100;
            basket_info.yaw = MovAvgFltr(&basket_info.yaw_fltr, *(float *)&RxData[4]);
            basket_info.height_cm = MovAvgFltr(&basket_info.height_cm_fltr, *(float *)&RxData[8]) * 100;
            break;
        }
        }
    }
}