#include "usr.h"
#include "arm_math.h"

#define CENTRE_OFFSET 0.138

struct
{
    float x, y, yaw;
} R1_pos;

struct
{
    float x, y;
} R2_pos,
    basket_pos = {.x = 14, .y = -4},
    basket_pos_R2;

unsigned char CheckSum_1B(unsigned char *data, unsigned char len)
{
    unsigned char temp = 0;
    for (unsigned char cnt = 0; cnt < len - 1; ++cnt)
    {
        temp += data[cnt];
    }
    return temp;
}

void Comm(void)
{
    static USART_handle_t UART5_handle = {.USART_handle = UART5, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Stream1, .DMA_ID = 1}; // dual robot communication Tx

    // dual robot communication
    float temp;
    *(float *)&R1_data[1] = temp = R1_pos.x + 0.24 * cos(R1_pos.yaw / R2D),     // offset required by R2
        *(float *)&R1_data[5] = temp = R1_pos.y + 0.24 * sin(R1_pos.yaw / R2D); // offset required by R2
    R1_data[17] = state_W.aim_R2 && state_R.brake,
    R1_data[sizeof(R1_data) - 1] = CheckSum_1B(R1_data, sizeof(R1_data));
    UART_SendArray(&UART5_handle, R1_data, sizeof(R1_data));
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
            if (state == LOCK && state_W.ball &&
                ((state_W.aim_R2 ? R2_info.dist_cm <= 800 : basket_info.dist_cm <= 750) ||
                 !state_R.fitting))
                state = SHOT;
            break;
        }
        case 0xB: // switch target to basket
        {
            if (state != SHOT)
                state_W.aim_R2 = false;
            break;
        }
        case 0xC: // switch target to R2
        {
            if (state != SHOT)
                state_W.aim_R2 = true;
            break;
        }
        case 0xE: // dribble end
        {
            if (state == LOCK)
                state = INIT_SLOW;
            else if (state == IDLE || state == MID)
                state = INIT_FAST;
            break;
        }
        case 0xF: // pass ball
        {
            if (PG_BTM)
                state_W.ball = true;
            break;
        }
        case 0x14: // manual initialization
        {
            if (state != SHOT)
            {
                state_W.ball = true;
                state = INIT_SLOW;
            }
            break;
        }
        // reset
        case 0xA2:
        case 0xDF:
        {
            if (state != SHOT)
            {
                state_W.ball = false;
                state = IDLE;
            }
            break;
        }
        case 0xA3: // speed offset
        {
            spd_offset = *(float *)RxData;
            break;
        }
        case 0xA4: // skill competition: ready to shoot
        {
            state_W.shot_ready = true;
            break;
        }
        case 0xA5: // skill competition: automatic initialization
        {
            if (state == IDLE && PG_TOP || state == MID)
            {
                state_W.ball = true;
                state = INIT_FAST;
            }
            else if (state == LOCK)
            {
                state_W.ball = true;
                state = INIT_SLOW;
            }
            break;
        }
        case 0xA6: // enable gimbal
        {
            if (!err.HighTorque_startup && !err.HighTorque_OH)
            {
                HighTorque[GIMBAL_idx].ctrl.Kp = 0.375,
                HighTorque[GIMBAL_idx].ctrl.Kd = 0.5;
            }
            break;
        }
        case 0xA7: // disable gimbal
        {
            HighTorque[GIMBAL_idx].ctrl.Kd = HighTorque[GIMBAL_idx].ctrl.Kp = 0;
            break;
        }
        case 0x105: // basket info from lidar
        {
            err_cnt.basket_pos = 0; // clear error flag

            basket_pos.x = *(float *)RxData,
            basket_pos.y = *(float *)&RxData[4];
            break;
        }
        case 0x201: // position info from chassis
        {
            err_cnt.R1_pos = 0; // clear error flag

            R1_pos.x = *(float *)RxData - CENTRE_OFFSET * cos(*(float *)&RxData[16] / R2D),
            R1_pos.y = *(float *)&RxData[4] - CENTRE_OFFSET * sin(*(float *)&RxData[16] / R2D),
            R1_pos.yaw = *(float *)&RxData[16];

            // gimbal feedforward gain factor
            float factor = (-*(float *)&RxData[20] >= 0 ? GIMBAL_MAX - HighTorque[GIMBAL_idx].fdbk.pos
                                                        : HighTorque[GIMBAL_idx].fdbk.pos - GIMBAL_MIN) /
                           ((GIMBAL_MAX - GIMBAL_MIN) * 0.5);
            // gimbal feedforward velocity opposed to angular velocity of chassis
            HighTorque[GIMBAL_idx].ctrl.spd = err.HighTorque_OH ? 0
                                                                : -*(float *)&RxData[20] * GIMBAL_GR * LIMIT(factor, 1);

            float dist_x = basket_pos.x - R1_pos.x,
                  dist_y = basket_pos.y - R1_pos.y;

            basket_info.dist_cm = MovAvgFltr(&basket_info.dist_fltr, hypot(dist_x, dist_y)) * 100,
            basket_info.yaw = atan2(dist_y, dist_x) * R2D - R1_pos.yaw;

            if (basket_info.yaw > 180)
                basket_info.yaw -= 360;
            else if (basket_info.yaw < -180)
                basket_info.yaw += 360;

            basket_info.yaw = MovAvgFltr(&basket_info.yaw_fltr, basket_info.yaw);

            break;
        }
        }
    }
}

// R2 info
void UART5_IRQHandler(void)
{
    static unsigned char RxData[19], cnt;

    if (UART5->ISR & 0x20)
    {
        RxData[cnt++] = UART5->RDR;

        if (RxData[0] != 0xAA ||
            cnt == sizeof(RxData) && CheckSum_1B(RxData, sizeof(RxData)) != RxData[sizeof(RxData) - 1])
            cnt = 0;
        else if (cnt == sizeof(RxData))
        {
            err_cnt.R2_pos = 0; // clear error flag

            cnt = 0;

            R2_pos.x = *(float *)&RxData[1] / 1000,
            R2_pos.y = *(float *)&RxData[5] / 1000;
            basket_pos_R2.x = *(float *)&RxData[9] / 1000,
            basket_pos_R2.y = *(float *)&RxData[13] / 1000;

            err.coor_unmatch = ABS(basket_pos.x - basket_pos_R2.x) >= 0.07;

            state_W.R2_AtPos = RxData[17] & 0x1;
            state_W.R2_NetUp = (RxData[17] & 0x4) >> 2;

            // shot request
            if (RxData[17] & 0x10 && state_W.shot_ready &&
                state == LOCK && state_W.ball &&
                state_W.aim_R2 && R2_info.dist_cm <= 800)
            {
                state_W.shot_ready = false;
                state = SHOT;
            }

            unsigned char R2_data[9];
            *(float *)R2_data = R2_pos.x,
                    *(float *)&R2_data[4] = R2_pos.y,
                    R2_data[8] = !err.coor_unmatch && state_W.aim_R2 && state_W.R2_AtPos;
            FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA1, R2_data, 9);

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
    R2_yaw_prev = R2_info.yaw;

    TIMsw_Clear(&R2_yaw_time);
    TIMsw_GetIntvl(&R2_msg_intvl);

    float dist_x = R2_pos.x - R1_pos.x,
          dist_y = R2_pos.y - R1_pos.y;

    R2_info.dist_cm = MovAvgFltr(&R2_info.dist_fltr, hypot(dist_x, dist_y)) * 100,
    R2_info.yaw = atan2(dist_y, dist_x) * R2D - R1_pos.yaw;

    if (R2_info.yaw > 180)
        R2_info.yaw -= 360;
    else if (R2_info.yaw < -180)
        R2_info.yaw += 360;

    R2_info.yaw = MovAvgFltr(&R2_info.yaw_fltr, R2_info.yaw);
}