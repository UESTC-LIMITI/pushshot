#include "user.h"

// basket pos
#define BASKET_X 14.05f
#define BASKET_Y -3.99f

#define CENTRE_OFFSET 0.2f

float camera_Kp = 0.02, // 0.016
    camera_Ki = 0.002;

void Comm(void *argument)
{
    while (1)
    {
        static bool basket_lock;
        if (basket_info.dist_cm <= 1000 && !basket_lock)
        {
            FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0x116, NULL, 0);
            basket_lock = true;
        }

        unsigned char R1_info[10] = {0xA5, state_W.aim_R2 && state_R.brake};
        *(float *)&R1_info[2] = R1_pos_chassis.x;
        *(float *)&R1_info[6] = R1_pos_chassis.y;
        UART_SendArray(&UART5_info, R1_info, 10);

        osDelay(10);
    }
}

float Gimbal_PID(float err)
{
    static float iterm, err_prev;

    if (ABS(err) >= 20)
        iterm = 0;
    else
        iterm += err / 30;

    float yaw = (ABS(err) > 1 ? err : 0) * camera_Kp + LIMIT_ABS(iterm, 0.25) * camera_Ki;

    err_prev = err;

    return yaw;
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
            if (state == LOCK)
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
            break;
        }
        // rst
        case 0xF6:
        {
            if (state == IDLE)
            {
                state_R.shot_ready = state_W.ball = 0;
                state = INIT;
            }
            else if (state == LOCK)
            {
                state_R.shot_ready = state_W.ball = 0;
                state = IDLE;
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

            basket_info.dist_cm = MovAvgFltr(&basket_info.dist_cm_fltr, sqrt(pow(dist_x, 2) + pow(dist_y, 2))) * 100;
            // basket_info.yaw = MovAvgFltr(&basket_info.yaw_fltr, -(atan(dist_y / dist_x) - *(float *)&RxData[8])) * R2D;

            // mini angle
            if (basket_info.yaw > 180)
                basket_info.yaw = 360 - basket_info.yaw;
            else if (basket_info.yaw < -180)
                basket_info.yaw = 360 + basket_info.yaw;

            break;
        }
        case 0x105:
        {
            // Timer_Clear(&gimbal_time);
            // basket_yaw_prev = basket_info.yaw;

            // basket_info.dist_cm = MovAvgFltr(&basket_info.dist_cm_fltr, *(float *)RxData) * 100;
            // basket_info.yaw = MovAvgFltr(&basket_info.yaw_fltr, -*(float *)&RxData[4]);

            break;
        }
        // basket yaw from camera
        case 0x106:
        {
            err.basket_yaw = 0; // clear err flag

            if (state_W.ball)
            {
                Timer_Clear(&gimbal_time);
                basket_yaw_prev = basket_info.yaw;
                basket_info.yaw -= Gimbal_PID(*(float *)RxData);
            }
            LIMIT_ABS(basket_info.yaw, 16);

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

            // basket_info.dist_cm = MovAvgFltr(&basket_info.dist_cm_fltr, sqrt(pow(dist_x, 2) + pow(dist_y, 2))) * 100;
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
// __attribute__((section(".ARM.__at_0x24000000"))) unsigned char RxData_D1S0[16];
// void DMA1_Stream0_IRQHandler(void)
// {
//     if (DMA1->LISR & 0x20)
//     {
//         DMA1->LIFCR |= 0x20;

//         err.R2_pos = 0; // clear err flag

//         R2_pos.x = *(float *)RxData_D1S0,
//         R2_pos.y = *(float *)&RxData_D1S0[4],
//         R2_pos.yaw = *(float *)&RxData_D1S0[8];

//         float dist_x = R2_pos.x - R1_pos_chassis.x,
//               dist_y = R2_pos.y - R1_pos_chassis.y;

//         R2_info.dist_cm = MovAvgFltr(&R2_info.dist_cm_fltr, sqrt(pow(dist_x, 2) + pow(dist_y, 2))) * 100,
//         R2_info.yaw = MovAvgFltr(&R2_info.yaw_fltr, atan(dist_y / dist_x) * R2D - *(float *)&RxData_D1S0[8]);

//         // mini angle
//         if (R2_info.yaw > 180)
//             R2_info.yaw = 360 - R2_info.yaw;
//         else if (R2_info.yaw < -180)
//             R2_info.yaw = 360 + R2_info.yaw;
//     }
// }

void UART5_IRQHandler(void)
{
    static unsigned char RxData[12], cnt;
    if (UART5->ISR & 0x20)
    {
        RxData[cnt++] = UART5->RDR;

        if (RxData[0] != 0xA5)
            cnt = 0;
        else if (cnt == 8)
        {
            err.R2_pos = 0; // clear err flag
            cnt = 0;
        }
    }
    // else if (UART5->ISR & 0x10) // idle
    // {
    //     UART5->ICR |= 0x10;

    //     cnt = 0;
    //     // DMA1_Stream0->NDTR = 8;
    //     // DMA1_Stream0->CR |= 1;
    // }
    else if (UART5->ISR & 0x8) // overrun
    {
        UART5->ICR |= 0x8;
    }
}