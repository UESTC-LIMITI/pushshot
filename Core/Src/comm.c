#include "user.h"

// basket pos
#define BASKET_X 13.9f
#define BASKET_Y -4.03f

#define CENTRE_OFFSET 0.2f

void Comm(void *argument)
{
    while (1)
    {
        unsigned char TxData[] = {state_W.ball, err.VESC, err.HighTorque, err.pos_lidar, err.basket_info, err.pos_chassis, err.R2_pos};
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
        unsigned char RxData[64];
        HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxData);

        switch (FDCAN_RxHeader.Identifier)
        {
        // shot
        case 0xA:
        {
            if (state == IDLE)
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
        // pos from lidar, for data collection only
        case 0x104:
        {
            err.pos_lidar = 0; // clear err flag

            R1_pos_lidar.x = *(float *)RxData - CENTRE_OFFSET * cos(*(float *)&RxData[8]);
            R1_pos_lidar.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(*(float *)&RxData[8]);
            R1_pos_lidar.yaw = *(float *)&RxData[8] * R2D;

            float dist_x = BASKET_X - R1_pos_lidar.x,
                  dist_y = BASKET_Y - R1_pos_lidar.y;

            // basket_info.dist_cm = MovAvgFltr(&basket_info.dist_cm_fltr, sqrt(pow(dist_x, 2) + pow(dist_y, 2))) * 100;
            // basket_info.yaw = MovAvgFltr(&basket_info.yaw_fltr, -(atan(dist_y / dist_x) - *(float *)&RxData[8])) * R2D;

            // mini angle
            if (basket_info.yaw > 180)
                basket_info.yaw = 360 - basket_info.yaw;
            else if (basket_info.yaw < -180)
                basket_info.yaw = 360 + basket_info.yaw;

            break;
        }
        // basket info from lidar
        case 0x105:
        {
            err.basket_info = 0; // clear err flag

            // basket_info.dist_cm = MovAvgFltr(&basket_info.dist_cm_fltr, *(float *)RxData) * 100;
            // basket_info.yaw = MovAvgFltr(&basket_info.yaw_fltr, *(float *)&RxData[4]);
            break;
        }
        // pos from chassis, for normal use
        case 0x201:
        {
            err.pos_chassis = 0; // clear err flag

            R1_pos_chassis.x = *(float *)RxData - CENTRE_OFFSET * cos(*(float *)&RxData[16]);
            R1_pos_chassis.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(*(float *)&RxData[16]);
            R1_pos_chassis.yaw = *(float *)&RxData[16] * R2D;

            float dist_x = BASKET_X - R1_pos_chassis.x,
                  dist_y = BASKET_Y - R1_pos_chassis.y;

            basket_info.dist_cm = MovAvgFltr(&basket_info.dist_cm_fltr, sqrt(pow(dist_x, 2) + pow(dist_y, 2))) * 100;
            // basket_info.yaw = MovAvgFltr(&basket_info.yaw_fltr, -(atan(dist_y / dist_x) - *(float *)&RxData[16])) * R2D;

            // mini angle
            if (basket_info.yaw > 180)
                basket_info.yaw = 360 - basket_info.yaw;
            else if (basket_info.yaw < -180)
                basket_info.yaw = 360 + basket_info.yaw;

            break;
        }
        }
    }
}

// info of R2
__attribute__((section(".ARM.__at_0x24000000"))) unsigned char RxData_D1S0[12];
void DMA1_Stream0_IRQHandler(void)
{
    if (DMA1->LISR & 0x20)
    {
        DMA1->LIFCR |= 0x20;

        err.R2_pos = 0; // clear err flag

        R2_pos.x = *(float *)RxData_D1S0,
        R2_pos.y = *(float *)&RxData_D1S0[4],
        R2_pos.yaw = *(float *)&RxData_D1S0[8];

        float dist_x = R2_pos.x - R1_pos_chassis.x,
              dist_y = R2_pos.y - R1_pos_chassis.y;

        R2_info.dist_cm = MovAvgFltr(&R2_info.dist_cm_fltr, sqrt(pow(dist_x, 2) + pow(dist_y, 2))) * 100,
        R2_info.yaw = MovAvgFltr(&R2_info.yaw_fltr, atan(dist_y / dist_x) * R2D - *(float *)&RxData_D1S0[8]);

        // mini angle
        if (R2_info.yaw > 180)
            R2_info.yaw = 360 - R2_info.yaw;
        else if (R2_info.yaw < -180)
            R2_info.yaw = 360 + R2_info.yaw;
    }
}