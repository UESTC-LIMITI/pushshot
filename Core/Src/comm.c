#include "user.h"

// basket pos
#define BASKET_X 14.05f
#define BASKET_Y -3.99f

#define CENTRE_OFFSET 0.2f

float camera_Kp = 0.02,
      camera_Ki = 0.002;

__attribute__((section(".ARM.__at_0x24000000"))) unsigned char R1_Data[10] = {0xA5},
                                                               RxData_D1S2[10];

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
        else if (basket_info.dist_cm > 1000)
            basket_lock = false;

        *(float *)&R1_Data[1] = err.pos_chassis ? R1_pos_lidar.x : R1_pos_chassis.x;
        *(float *)&R1_Data[5] = err.pos_chassis ? R1_pos_lidar.y : R1_pos_chassis.y;
        R1_Data[9] = state_W.aim_R2 && state_R.brake;
        UART_SendArray(&UART5_info, R1_Data, 10);
        osDelay(10);
    }
}

float Gimbal_PID(float err)
{
    static float iterm, err_prev;

    if (ABS(err) >= 16)
        iterm = 0;
    else
        iterm += err / 30;

    float yaw = (ABS(err) > 2 ? err : 0) * camera_Kp + LIMIT_ABS(iterm, 0.5) * camera_Ki;

    err_prev = err;

    return yaw;
}

float Yaw_Stablize(float yaw)
{
    static MovAvgFltr_t lidar_yaw_fltr = {.size = 4};

    // tiny fluctuation, filter before output
    if (MovAvgFltr_GetNewStatus(&lidar_yaw_fltr, yaw, 0.5))
        return lidar_yaw_fltr.sum / lidar_yaw_fltr.len;
    // error, direct output
    else
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
            if (state == IDLE)
                state = SHOT;
            break;
        }
        // switch target to basket
        case 0xB:
        {
            if (state_W.aim_R2)
            {
                MovAvgFltr_Clear(&yaw_fltr);
                state_W.aim_R2 = 0;
            }
            break;
        }
        // switch target to R2
        case 0xC:
        {
            if (!state_W.aim_R2)
            {
                MovAvgFltr_Clear(&yaw_fltr);
                state_W.aim_R2 = 1;
            }
            break;
        }
        case 0xE: // dribble end
        {
            if (state == IDLE)
                state = BACK;
            break;
        }
        // ball passed
        case 0xF:
        {
            Timer_Clear(&HighTorque_time);
            state_W.ball = 1;
            break;
        }
        // manual init, for test only
        case 0x14:
        {
            if (state == IDLE)
            {
                state_W.ball = 1;
                state = BACK;
            }
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
        // position info from lidar, 1
        case 0x104:
        {
            err_cnt.pos_lidar = err.pos_lidar = 0; // clear error flag

            R1_pos_lidar.x = *(float *)RxData - CENTRE_OFFSET * cos(*(float *)&RxData[8]);
            R1_pos_lidar.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(*(float *)&RxData[8]);
            R1_pos_lidar.yaw = *(float *)&RxData[8] * R2D;

            if (err.basket_lidar)
            {
                if (!state_W.aim_R2)
                {
                    Timer_Clear(&gimbal_time);
                    yaw_prev = basket_info.yaw;
                }

                float dist_x = BASKET_X - R1_pos_lidar.x,
                      dist_y = BASKET_Y - R1_pos_lidar.y;

                basket_info.dist_cm = sqrt(pow(dist_x, 2) + pow(dist_y, 2)) * 100;

                // absolute angle
                float yaw = (dist_x >= 0 ? atan(dist_y / dist_x) * R2D                                   // basket at front
                                         : (atan(dist_y / dist_x) * R2D + (dist_y >= 0 ? 180 : -180))) - // basket behind
                            R1_pos_lidar.yaw;

                // mini angle
                if (yaw > 180)
                    yaw -= 360;
                else if (yaw < -180)
                    yaw += 360;

                basket_info.yaw = Yaw_Stablize(yaw);
            }
            break;
        }
        // basket info from lidar, 0
        case 0x105:
        {
            err_cnt.basket_lidar = err.basket_lidar = 0; // clear error flag

            if (!state_W.aim_R2)
            {
                Timer_Clear(&gimbal_time);
                yaw_prev = basket_info.yaw;
            }

            basket_info.dist_cm = *(float *)RxData * 100,
            basket_info.yaw = Yaw_Stablize(-*(float *)&RxData[4]);

            break;
        }
        // basket info from camera, 2
        case 0x106:
        {
            err_cnt.basket_camera = err.basket_camera = 0; // clear error flag

            if (state_W.ball && basket_info.dist_cm <= 900 && // start PID calculation
                err.basket_lidar && err.pos_lidar &&
                !state_W.aim_R2)
            {
                Timer_Clear(&gimbal_time);
                yaw_prev = basket_info.yaw;

                basket_info.yaw -= Gimbal_PID(*(float *)RxData);
                LIMIT_ABS(basket_info.yaw, 16); // max angle for PID
            }

            break;
        }
        // pos from chassis, 3
        case 0x201:
        {
            err_cnt.pos_chassis = err.pos_chassis = 0; // clear error flag

            R1_pos_chassis.x = *(float *)RxData - CENTRE_OFFSET * cos(*(float *)&RxData[16]);
            R1_pos_chassis.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(*(float *)&RxData[16]);
            R1_pos_chassis.yaw = *(float *)&RxData[16] * R2D;

            if (err.basket_lidar && err.pos_lidar)
            {
                float dist_x = BASKET_X - R1_pos_chassis.x,
                      dist_y = BASKET_Y - R1_pos_chassis.y;

                basket_info.dist_cm = sqrt(pow(dist_x, 2) + pow(dist_y, 2)) * 100;

                if (err.basket_camera)
                {
                    if (!state_W.aim_R2)
                    {
                        Timer_Clear(&gimbal_time);
                        yaw_prev = basket_info.yaw;
                    }

                    // absolute angle
                    basket_info.yaw = (dist_x >= 0 ? atan(dist_y / dist_x) * R2D                                   // basket at front
                                                   : (atan(dist_y / dist_x) * R2D + (dist_y >= 0 ? 180 : -180))) - // basket behind
                                      R1_pos_chassis.yaw;

                    // mini angle
                    if (basket_info.yaw > 180)
                        basket_info.yaw -= 360;
                    else if (basket_info.yaw < -180)
                        basket_info.yaw += 360;
                }
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

            float dist_x = R2_pos.x - (err.pos_chassis ? R1_pos_lidar.x : R1_pos_chassis.x),
                  dist_y = R2_pos.y - (err.pos_chassis ? R1_pos_lidar.y : R1_pos_chassis.y);

            R2_info.dist_cm = sqrt(pow(dist_x, 2) + pow(dist_y, 2)) * 100;

            if (state_W.aim_R2)
            {
                Timer_Clear(&gimbal_time);
                yaw_prev = R2_info.yaw;
            }

            // absolute angle
            R2_info.yaw = (dist_x >= 0 ? atan(dist_y / dist_x) * R2D                                   // R2 at front
                                       : (atan(dist_y / dist_x) * R2D + (dist_y >= 0 ? 180 : -180))) - // R2 behind
                          (err.pos_lidar ? R1_pos_chassis.yaw                                          // position from lidar offline
                                         : R1_pos_lidar.yaw);                                          // position from lidar online

            // mini angle
            if (R2_info.yaw > 180)
                R2_info.yaw -= 360;
            else if (R2_info.yaw < -180)
                R2_info.yaw += 360;

            FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA1, (unsigned char *)&R2_pos, 8);
        }
    }
}