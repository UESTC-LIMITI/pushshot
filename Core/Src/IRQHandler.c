#include "user.h"

// gimbal
void FDCAN1_IT0_IRQHandler(void)
{
    if (FDCAN1->IR & 0x1)
    {
        FDCAN1->IR |= 0x1;

        FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
        unsigned char RxData[32];
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxData);

        switch (FDCAN_RxHeader.Identifier)
        {
        case (GIMBAL_ID << 8):
        {
            if (RxData[0] == (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 3) &&
                RxData[1] == HIGHTORQUE_REG_POS_FDBK &&
                RxData[14] == (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 1) &&
                RxData[15] == HIGHTORQUE_REG_TEMP)
            {
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.pos = *(float *)&RxData[2] * 360;
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.spd = *(float *)&RxData[6] * 360;
                HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.trq = *(float *)&RxData[10];

                // not over heat
                if ((HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.temp = *(float *)&RxData[16]) <= 50)
                    err_cnt.HighTorque = err.HighTorque = 0; // clear error flag
                // over heat
                else if (HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].fdbk.temp >= 60)
                    HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.Kd = HighTorque[GIMBAL_ID - HIGHTORQUE_ID_OFFSET].ctrl.Kp = 0;
            }
            break;
        }
        }
    }
}

// pushshot ESC
void FDCAN2_IT0_IRQHandler(void)
{
    if (FDCAN2->IR & 0x1)
    {
        FDCAN2->IR |= 0x1;

        FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
        unsigned char RxData[8];
        HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxData);

        switch (FDCAN_RxHeader.Identifier)
        {
        case (VESC_STATUS_1 << 8 | PUSHSHOT_ID):
        {
            err_cnt.VESC = err.VESC = 0; // clear error flag

            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.spd = (float)(RxData[0] << 24 | RxData[1] << 16 | RxData[2] << 8 | RxData[3]) / CUBEMARS_R100_KV90.PP;
            VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.curr = (float)(RxData[4] << 8 | RxData[5]) / VESC_fCURR_R;
            break;
        }
        case (VESC_STATUS_5 << 8 | PUSHSHOT_ID):
        {
            // not under voltage
            if ((VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.volt = (float)(RxData[4] << 8 | RxData[5]) / VESC_fVOLT) >= 23.4)
                err_cnt.VESC = err.VESC = 0; // clear error flag
            break;
        }
        }
    }
}

// top photogate, trigger brake
void EXTI1_IRQHandler(void)
{
    EXTI->PR1 |= 0x2;
    if (GPIOF->IDR & 0x2 && state == SHOT)
    {
        state_R.shot_ready = state_W.ball = state_R.spd_ctrl = 0;
        state_R.brake = 1;
    }
}