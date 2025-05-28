#include "user.h"

#define CENTRE_OFFSET 0.069

__attribute__((section(".ARM.__at_0x24000000"))) unsigned char R1_Data[10] = {0xA5},
                                                               RxData_D1S2[10];

void Comm(void *argument)
{
    while (1)
    {
        *(float *)&R1_Data[1] = R1_pos_lidar.x,
                *(float *)&R1_Data[5] = R1_pos_lidar.y,
                R1_Data[9] = state_W.aim_R2 && state_R.brake;
        UART_SendArray(&UART5_info, R1_Data, 10); // dual robot communication

        // restart DMA after 0.1s
        static timer_t dual_robo_comm_time;
        if (!(DMA1_Stream2->CR & 1) && Timer_CheckTimeout(&dual_robo_comm_time, 0.1))
        {
            Timer_Clear(&dual_robo_comm_time);

            DMA1_Stream2->NDTR = 10;
            DMA1_Stream2->CR |= 1;
        }

        unsigned char TxData[12];
        *(float *)TxData = R2_info.dist_cm,
                *(float *)&TxData[4] = basket_spd_offset,
                *(float *)&TxData[8] = R2_spd_offset;
        FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA2, TxData, 9);

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
        case 0xA: // shoot
        {
            if (state == IDLE && state_R.shot_ready)
                state = SHOT;
            break;
        }
        case 0xB: // switch target to basket
        {
            state_W.aim_R2 = 0;
            break;
        }
        case 0xC: // switch target to R2
        {
            state_W.aim_R2 = 1;
            break;
        }
        case 0x12: // dribble start
        {
            if (state == IDLE)
            {
                state_R.shot_ready = state_W.ball = 0;
                state = INIT;
            }
            break;
        }
        case 0xF: // pass ball
        {
            state_W.ball = 1;
            break;
        }
        case 0x14: // manual initialization, for test only
        {
            if (state == IDLE)
            {
                state_W.ball = 1;
                state = INIT;
            }
            break;
        }
        case 0xA3: // speed offset ++
        {
            state_W.aim_R2 ? ++R2_spd_offset : ++basket_spd_offset;
            break;
        }
        case 0xA4: // speed offset --
        {
            state_W.aim_R2 ? --R2_spd_offset : --basket_spd_offset;
            break;
        }
        case 0xA6: // enable gimbal
        {
            state_W.gimbal = 1;
            break;
        }
        case 0xA7: // disable gimbal
        {
            state_W.gimbal = 0;
            break;
        }
        case 0x104: // position info from lidar
        {
            err_cnt.pos_lidar = err.pos_lidar = 0; // clear error flag

            // R1_pos_lidar.x = *(float *)RxData - CENTRE_OFFSET * cos(*(float *)&RxData[8] / R2D),
            // R1_pos_lidar.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(*(float *)&RxData[8] / R2D),
            // R1_pos_lidar.yaw = *(float *)&RxData[8];

            break;
        }
        case 0x105: // basket info from lidar
        {
            err_cnt.basket_info = err.basket_info = 0; // clear error flag

            basket_pos.x = *(float *)RxData,
            basket_pos.y = *(float *)&RxData[4];

            basket_info.dist_cm = *(float *)&RxData[8] * 100,
            basket_info.yaw = -*(float *)&RxData[12];

            MovAvgFltr(&basket_info.dist_fltr, basket_info.dist_cm),
                MovAvgFltr(&basket_info.yaw_fltr, basket_info.yaw);

            break;
        }
        case 0x201: // position info from chassis
        {
            err_cnt.pos_chassis = err.pos_chassis = 0; // clear error flag

            R1_pos_lidar.x = *(float *)RxData - CENTRE_OFFSET * cos(*(float *)&RxData[16] / R2D),
            R1_pos_lidar.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(*(float *)&RxData[16] / R2D),
            R1_pos_lidar.yaw = *(float *)&RxData[16];

            R1_pos_chassis.x = *(float *)&RxData[24] - CENTRE_OFFSET * cos(*(float *)&RxData[16] / R2D),
            R1_pos_chassis.y = *(float *)&RxData[28] - CENTRE_OFFSET * sin(*(float *)&RxData[16] / R2D);
            // R1_pos_chassis.yaw = *(float *)&RxData[16];

            // gimbal feedforward associated with angular velocity of chassis
            if (state_W.ball)
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.spd = -*(float *)&RxData[20] * Gimbal_GR *
                                                                        // feedforward gain associated with available angle
                                                                        (-*(float *)&RxData[20] >= 0 ? YAW_MAX - HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos
                                                                                                     : HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos - YAW_MIN) /
                                                                        (YAW_MAX - YAW_MIN);

            if (err.basket_info)
            {
                float dist_x = basket_pos.x - R1_pos_lidar.x,
                      dist_y = basket_pos.y - R1_pos_lidar.y;

                basket_info.dist_cm = sqrt(pow(dist_x, 2) + pow(dist_y, 2)) * 100,
                basket_info.yaw = atan2(dist_y, dist_x) * R2D - R1_pos_lidar.yaw;

                if (basket_info.yaw > 180)
                    basket_info.yaw -= 360;
                else if (basket_info.yaw < -180)
                    basket_info.yaw += 360;

                MovAvgFltr(&basket_info.dist_fltr, basket_info.dist_cm),
                    MovAvgFltr(&basket_info.yaw_fltr, basket_info.yaw);
            }
            break;
        }
        }
    }
}

// R2 info
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
    float dist_x = R2_pos.x - R1_pos_lidar.x,
          dist_y = R2_pos.y - R1_pos_lidar.y;

    R2_info.dist_cm = sqrt(pow(dist_x, 2) + pow(dist_y, 2)) * 100,
    R2_info.yaw = atan2(dist_y, dist_x) * R2D - R1_pos_lidar.yaw;

    if (R2_info.yaw > 180)
        R2_info.yaw -= 360;
    else if (R2_info.yaw < -180)
        R2_info.yaw += 360;

    MovAvgFltr(&R2_info.dist_fltr, R2_info.dist_cm),
        MovAvgFltr(&R2_info.yaw_fltr, R2_info.yaw);

    if (state_W.aim_R2)
    {
        Timer_Clear(&gimbal_time);
        yaw_prev = yaw_curr,
        yaw_curr = MovAvgFltr_GetData(&R2_info.yaw_fltr);
    }

    FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA1, (unsigned char *)&R2_pos, 8);
}