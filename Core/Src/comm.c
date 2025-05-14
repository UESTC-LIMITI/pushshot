#include "user.h"

#define CENTRE_OFFSET 0.069f

__attribute__((section(".ARM.__at_0x24000000"))) unsigned char R1_Data[10] = {0xA5},
                                                               RxData_D1S2[10];

void Comm(void *argument)
{
    while (1)
    {
        *(float *)&R1_Data[1] = err.basket_info ? R1_pos_chassis.x : R1_pos_lidar.x,     // err.basket_info as lidar position error flag
            *(float *)&R1_Data[5] = err.basket_info ? R1_pos_chassis.y : R1_pos_lidar.y, // err.basket_info as lidar position error flag
            R1_Data[9] = state_W.aim_R2 && state_R.brake;
        UART_SendArray(&UART5_info, R1_Data, 10); // dual robot communication

        static timer_t dual_robo_comm_time;

        // restart DMA after 0.1s
        if (!(DMA1_Stream2->CR & 1) && Timer_CheckTimeout(&dual_robo_comm_time, 0.1))
        {
            Timer_Clear(&dual_robo_comm_time);

            DMA1_Stream2->NDTR = 10;
            DMA1_Stream2->CR |= 1;
        }
        osDelay(10);
    }
}

float Lidar_Yaw_Stablization(float yaw)
{
    static MovAvgFltr_t lidar_yaw_fltr = {.size = 4};
    return MovAvgFltr_GetNewStatus(&lidar_yaw_fltr, yaw, 0.05) ? MovAvgFltr_GetData(&lidar_yaw_fltr) // tiny fluctuation, filter before output
                                                               : yaw;                                // large error, direct output
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
            if (state == IDLE && state_R.shot_ready)
                state = SHOT;
            break;
        }
        // switch target to basket
        case 0xB:
        {
            state_W.aim_R2 = 0;
            break;
        }
        // switch target to R2
        case 0xC:
        {
            state_W.aim_R2 = 1;
            break;
        }
        case 0xE:  // dribble end
        case 0x14: // manual init, for test only
        {
            if (state == IDLE)
                state = BACK;
            break;
        }
        // enable gimbal
        case 0xA6:
        {
            state_W.gimbal = 1;
            break;
        }
        // disable gimbal
        case 0xA7:
        {
            state_W.gimbal = 0;
            break;
        }
        // reset
        case 0xF6:
        {
            if (state == IDLE)
            {
                state_R.shot_ready = state_W.ball = 0;
                state = INIT;
            }
            else if (state == LOCK)
            {
                Timer_Clear(&runtime);
                state_R.shot_ready = state_W.ball = 0;
                state = IDLE;
            }
            break;
        }
        // position info from lidar
        case 0x104:
        {
            err_cnt.pos_lidar = err.pos_lidar = 0; // clear error flag

            // R1_pos_lidar.x = *(float *)RxData - CENTRE_OFFSET * cos(*(float *)&RxData[8] / R2D),
            // R1_pos_lidar.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(*(float *)&RxData[8] / R2D),
            R1_pos_lidar.yaw = *(float *)&RxData[8];

            break;
        }
        // basket info from lidar
        case 0x105:
        {
            err_cnt.basket_info = err.basket_info = 0; // clear error flag

            basket_pos.x = *(float *)RxData,
            basket_pos.y = *(float *)&RxData[4];

            basket_info.dist_cm = *(float *)&RxData[8] * 100;
            MovAvgFltr(&basket_info.dist_fltr, basket_info.dist_cm);

            basket_info.yaw = Lidar_Yaw_Stablization(-*(float *)&RxData[12]);

            if (!state_W.aim_R2 &&
                Timer_CheckTimeout(&gimbal_time, 1 / 10.f))
            {
                Timer_Clear(&gimbal_time);
                yaw_prev = yaw_curr,
                yaw_curr = basket_info.yaw;
            }
            break;
        }
        // position info from chassis
        case 0x201:
        {
            err_cnt.pos_chassis = err.pos_chassis = 0; // clear error flag

            R1_pos_lidar.x = *(float *)RxData - CENTRE_OFFSET * cos(R1_pos_lidar.yaw / R2D),
            R1_pos_lidar.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(R1_pos_lidar.yaw / R2D);

            R1_pos_chassis.x = *(float *)&RxData[24] - CENTRE_OFFSET * cos(*(float *)&RxData[16] / R2D),
            R1_pos_chassis.y = *(float *)&RxData[28] - CENTRE_OFFSET * sin(*(float *)&RxData[16] / R2D),
            R1_pos_chassis.yaw = *(float *)&RxData[16];

            // gimbal feedforward associated with angular velocity of chassis
            if (state_W.ball)
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.spd = -*(float *)&RxData[20] * Gimbal_GR *
                                                                        // feedforward gain associated with available angle
                                                                        (-*(float *)&RxData[20] >= 0 ? YAW_MAX - HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos
                                                                                                     : HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos - YAW_MIN) /
                                                                        (YAW_MAX - YAW_MIN);

            if (err.basket_info)
            {
                float dist_x = basket_pos.x - R1_pos_chassis.x,
                      dist_y = basket_pos.y - R1_pos_chassis.y;

                basket_info.dist_cm = sqrt(pow(dist_x, 2) + pow(dist_y, 2)) * 100;
                MovAvgFltr(&basket_info.dist_fltr, basket_info.dist_cm);

                // absolute angle
                basket_info.yaw = atan2(dist_y, dist_x) * R2D - R1_pos_chassis.yaw;

                // mini angle
                if (basket_info.yaw > 180)
                    basket_info.yaw -= 360;
                else if (basket_info.yaw < -180)
                    basket_info.yaw += 360;

                yaw_prev = yaw_curr,
                yaw_curr = basket_info.yaw;
            }
            break;
        }
        }
    }
}

// info of R2
void DMA1_Stream2_IRQHandler(void)
{
    if (DMA1->LISR & 0x20 << 16)
    {
        DMA1->LIFCR |= 0x20 << 16;

        if (RxData_D1S2[0] == 0xAA) // data check
        {
            err_cnt.R2_pos = err.R2_pos = 0; // clear error flag

            R2_pos.x = *(float *)&RxData_D1S2[1] / 1000,
            R2_pos.y = *(float *)&RxData_D1S2[5] / 1000;

            state_W.R2_ready = RxData_D1S2[9];

            R2_Pos_Process();
        }
        // disable DMA
        else
        {
            DMA1_Stream2->CR &= ~1;
        }
    }
}

void R2_Pos_Process(void)
{
    float dist_x = R2_pos.x - (err.basket_info ? R1_pos_chassis.x : R1_pos_lidar.x), // err.basket_info as lidar position error flag
        dist_y = R2_pos.y - (err.basket_info ? R1_pos_chassis.y : R1_pos_lidar.y);   // err.basket_info as lidar position error flag

    R2_info.dist_cm = sqrt(pow(dist_x, 2) + pow(dist_y, 2)) * 100;
    MovAvgFltr(&R2_info.dist_fltr, R2_info.dist_cm);

    // absolute angle
    R2_info.yaw = err.basket_info ? atan2(dist_y, dist_x) * R2D - R1_pos_chassis.yaw // err.basket_info as lidar position error flag
                                  : Lidar_Yaw_Stablization(atan2(dist_y, dist_x) * R2D - R1_pos_lidar.yaw);

    // mini angle
    if (R2_info.yaw > 180)
        R2_info.yaw -= 360;
    else if (R2_info.yaw < -180)
        R2_info.yaw += 360;

    if (state_W.aim_R2 &&
        (err.basket_info || Timer_CheckTimeout(&gimbal_time, 1 / 10.f))) // err.basket_info as lidar position error flag
    {
        Timer_Clear(&gimbal_time);
        yaw_prev = yaw_curr,
        yaw_curr = R2_info.yaw;
    }

    FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA1, (unsigned char *)&R2_pos, 8);
}