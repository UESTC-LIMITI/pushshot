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

float test;
void FDCAN3_IT0_IRQHandler(void)
{
    if (FDCAN3->IR & 0x1)
    {
        FDCAN3->IR |= 0x1;

        FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
        unsigned char RxData[64];
        HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxData);

        switch (FDCAN_RxHeader.Identifier)
        {
        // shot
        case 0xA:
        {
            if (state_R.shot_ready && state == IDLE)
                state = SHOT;
            break;
        }
        // switch target to basket
        case 0xB:
        {
            R2_info.dist_cm_fltr.len = R2_info.yaw_fltr.len = 0;
            state_W.aim_R2 = 0;
            break;
        }
        // switch target to R2
        case 0xC:
        {
            basket_info.dist_cm_fltr.len = basket_info.yaw_fltr.len = 0;
            state_W.aim_R2 = 1;
            break;
        }
        case 0xE:  // dribble done
        case 0x12: // dribble start
        {
            if (state == IDLE)
                state = BACK;
            break;
        }
        case 0xF:  // ball passed
        case 0x14: // manual init
        {
            state_W.ball = 1;
            if (state == IDLE)
                state = INIT;
            break;
        }
        // rst
        case 0xF6:
        {
            if (state == IDLE &&
                GPIOE->IDR & 0x4)
            {
                state_R.shot_ready = state_W.ball = 0;
                state_W.RST = 1;
                state = INIT;
            }
            break;
        }
        case 0x104:
        {
            break;
        }
        // dist and yaw of basket
        case 0x105:
        {
            CAN_fault.basket_info = 0; // clear CAN fault flag

            basket_info.dist_cm = MovAvgFltr(&basket_info.dist_cm_fltr, *(float *)RxData) * 100;
            basket_info.yaw = MovAvgFltr(&basket_info.yaw_fltr, *(float *)&RxData[4]);
            break;
        }
        // dist and yaw of R2
        case 0x106:
        {
            CAN_fault.R2_info = 0; // clear CAN fault flag

            float dist_x = *(float *)RxData - *(float *)&RxData[12],
                  dist_y = *(float *)&RxData[4] - *(float *)&RxData[16],
                  dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2)),
                  angle = atan(dist_y / dist_x) - *(float *)&RxData[20];

            // dist exception
            if (dist > 0 && dist <= 15)
                R2_info.dist_cm = MovAvgFltr(&R2_info.dist_cm_fltr, dist) * 100;

            // mini angle
            if (angle > PI)
                angle = 2 * PI - angle;
            else if (angle < -PI)
                angle = 2 * PI + angle;

            R2_info.yaw = MovAvgFltr(&R2_info.yaw_fltr, angle) * R2D;
            break;
        }
        }
    }
}