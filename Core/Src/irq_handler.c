#include "user.h"

void FDCAN1_IT0_IRQHandler(void)
{
    if (FDCAN1->IR & 0x1)
    {
        FDCAN1->IR |= 0x1;

        FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
        uint8_t RxData[32];
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxData);

        switch (FDCAN_RxHeader.Identifier)
        {
        case 0x100:
        {
            CAN_fault.HighTorque = 0; // clear CAN fault flag

            if (RxData[0] == (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 3) &&
                RxData[1] == HIGHTORQUE_REG_POS_FDBK &&
                RxData[14] == (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 1) &&
                RxData[15] == HIGHTORQUE_REG_TEMP)
            {
                HighTorque[1 - HIGHTORQUE_ID_OFFSET].fdbk.pos = *(float *)&RxData[2] * 360;
                HighTorque[1 - HIGHTORQUE_ID_OFFSET].fdbk.spd = *(float *)&RxData[6] * 360;
                HighTorque[1 - HIGHTORQUE_ID_OFFSET].fdbk.trq = *(float *)&RxData[10];
                HighTorque[1 - HIGHTORQUE_ID_OFFSET].fdbk.temp = *(float *)&RxData[16];
            }
            break;
        }
        }
    }
}

void FDCAN2_IT0_IRQHandler(void)
{
    if (FDCAN2->IR & 0x1)
    {
        FDCAN2->IR |= 0x1;

        FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
        uint8_t RxData[8];
        HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxData);

        switch (FDCAN_RxHeader.Identifier)
        {
        case (VESC_STATUS_1 | 1):
        {
            CAN_fault.VESC = 0; // clear CAN fault flag

            VESC[1 - VESC_ID_OFFSET].fdbk.spd = (float)(RxData[0] << 24 | RxData[1] << 16 | RxData[2] << 8 | RxData[3]) / HOBBYWING_V9626_KV160.PP;
            break;
        }
        case 0x201:
        {
            CAN_fault.M2006 = 0; // clear CAN fault flag

            static float pos_prev, pos_curr;

            pos_curr = ((RxData[0] << 8) | RxData[1]) * C610_fPOS;
            if (ABS(pos_prev - pos_curr) >= 180) // lap count
                C610[0].fdbk.pos += ((pos_prev > pos_curr ? 360 : -360) + pos_curr - pos_prev) / M2006_GR;
            else
                C610[0].fdbk.pos += (pos_curr - pos_prev) / M2006_GR;
            pos_prev = pos_curr;

            C610[0].fdbk.spd = (short)((RxData[2] << 8) | RxData[3]) / M2006_GR;
            C610[0].fdbk.curr = (short)((RxData[4] << 8) | RxData[5]) * C610_fCURR;
            C610[0].fdbk.trq = C610[0].fdbk.curr * M2006_fTRQ;

            C610_PID_spd[0].deprev = C610_PID_spd[0].decurr;
            C610_PID_spd[0].decurr = C610[0].ctrl.spd - C610[0].fdbk.spd;

            C610_PID_pos[0].deprev = C610_PID_pos[0].decurr;
            C610_PID_pos[0].decurr = C610[0].ctrl.pos - C610[0].fdbk.pos;

            break;
        }
        }
    }
}

// top pg
void EXTI1_IRQHandler(void)
{
    EXTI->PR1 |= 0x2;
    if (GPIOF->IDR & 0x2 && state == SHOT)
        state_R.brake = 1;
}