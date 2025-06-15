#include "user.h"

#define CENTRE_OFFSET 0.069

__attribute__((section(".ARM.__at_0x24000000"))) unsigned char R1_Data[11] = {0xA5};

unsigned char CheckSum(unsigned char *data)
{
    unsigned char temp = 0;
    for (unsigned char cnt = 0; cnt < 10; ++cnt)
    {
        temp += data[cnt];
    }
    return temp;
}

void Comm(void *argument)
{
    while (1)
    {
        // dual robot communication
        *(float *)&R1_Data[1] = R1_pos_lidar.x + 0.24 * cos(R1_pos_lidar.yaw / R2D),
                *(float *)&R1_Data[5] = R1_pos_lidar.y + 0.24 * sin(R1_pos_lidar.yaw / R2D),
                R1_Data[9] = state_W.aim_R2 && state_R.brake,
                R1_Data[10] = CheckSum(R1_Data);
        UART_SendArray(&UART5_info, R1_Data, 11);

        osDelay(20);
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
            if (state == LOCK && state_R.shot_ready)
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
            if (state != SHOT)
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
        case 0x14: // manual initialization
        {
            if (state == IDLE)
            {
                state_W.ball = 1;
                state = INIT;
            }
            break;
        }
        case 0xA3: // modify speed offset
        {
            state_W.aim_R2 ? (R2_spd_offset = *(float *)RxData)
                           : (basket_spd_offset = *(float *)RxData);
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
        case 0xDF: // reset
        {
            if (state != SHOT)
            {
                state_R.shot_ready = state_W.ball = 0;
                state = IDLE;
            }
            break;
        }
        case 0x105: // basket info from lidar
        {
            err_cnt.basket_info = err.basket_info = 0; // clear error flag

            basket_pos.x = *(float *)RxData,
            basket_pos.y = *(float *)&RxData[4];

            basket_info.dist_cm = *(float *)&RxData[8] * 100,
            basket_info.yaw = -*(float *)&RxData[12];

            MovAvgFltr(&basket_info.dist_fltr, basket_info.dist_cm);

            break;
        }
        case 0x201: // position info from chassis
        {
            err_cnt.pos = err.pos = 0; // clear error flag

            R1_pos_lidar.x = *(float *)RxData - CENTRE_OFFSET * cos(*(float *)&RxData[16] / R2D),
            R1_pos_lidar.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(*(float *)&RxData[16] / R2D),
            R1_pos_lidar.yaw = *(float *)&RxData[16];

            R1_pos_chassis.x = *(float *)&RxData[24] - CENTRE_OFFSET * cos(*(float *)&RxData[16] / R2D),
            R1_pos_chassis.y = *(float *)&RxData[28] - CENTRE_OFFSET * sin(*(float *)&RxData[16] / R2D);
            // R1_pos_chassis.yaw = *(float *)&RxData[16];

            // gimbal feedforward associated with angular velocity of chassis
            HighTorque[GIMBAL_arrID].ctrl.spd = state_W.gimbal && state_W.ball ? -*(float *)&RxData[20] * Gimbal_GR *
                                                                                     // feedforward gain associated with available angle
                                                                                     (-*(float *)&RxData[20] >= 0 ? YAW_MAX - HighTorque[GIMBAL_arrID].fdbk.pos
                                                                                                                  : HighTorque[GIMBAL_arrID].fdbk.pos - YAW_MIN) /
                                                                                     (YAW_MAX - YAW_MIN)
                                                                               : 0;

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

                MovAvgFltr(&basket_info.dist_fltr, basket_info.dist_cm);
            }
            break;
        }
        }
    }
}

// R2 info
void UART5_IRQHandler(void)
{
    static unsigned char RxData[11], cnt;

    if (UART5->ISR & 0x20)
    {
        RxData[cnt++] = UART5->RDR;

        if (RxData[0] != 0xAA ||
            cnt == 11 && RxData[10] != CheckSum(RxData))
            cnt = 0;
        else if (cnt == 11)
        {
            err_cnt.R2_pos = err.R2_pos = 0; // clear error flag

            cnt = 0;

            R2_pos.x = *(float *)&RxData[1] / 1000,
            R2_pos.y = *(float *)&RxData[5] / 1000;

            switch (RxData[9])
            {
            case 0:
            {
                state_W.R2_NetUp = 0;
                break;
            }
            case 1:
            {
                state_W.R2_NetUp = 1;
                break;
            }
            case 2:
            {
                if (state_W.aim_R2 && state == LOCK && state_R.shot_ready)
                    state = SHOT;
                break;
            }
            }

            R2_Pos_Process();
        }
    }
    // error
    else if (UART5->ISR & 0xA)
    {
        UART5->ICR |= 0xA;
    }
}

void R2_Pos_Process(void)
{
    Timer_Clear(&R2_yaw_time);
    Timer_GetIntvl(&R2_yaw_intvl);

    float dist_x = R2_pos.x - R1_pos_lidar.x,
          dist_y = R2_pos.y - R1_pos_lidar.y;

    R2_info.dist_cm = sqrt(pow(dist_x, 2) + pow(dist_y, 2)) * 100,
    R2_info.yaw = atan2(dist_y, dist_x) * R2D - R1_pos_lidar.yaw;

    if (R2_info.yaw > 180)
        R2_info.yaw -= 360;
    else if (R2_info.yaw < -180)
        R2_info.yaw += 360;

    MovAvgFltr(&R2_info.dist_fltr, R2_info.dist_cm);

    R2_yaw_prev = R2_yaw_curr,
    R2_yaw_curr = R2_info.yaw;

    FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA1, (unsigned char *)&R2_pos, 8);
}