#include "usr.h"

// gimbal
void FDCAN1_IT0_IRQHandler(void)
{
    if (FDCAN1->IR & 0x1)
    {
        FDCAN1->IR |= 0x1;

        FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
        unsigned char RxData[20];
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxData);

        // HighTorque_MsgHandler(FDCAN_RxHeader.Identifier, GIMBAL_arrID, RxData);

        if (FDCAN_RxHeader.Identifier == (HighTorque[GIMBAL_arrID].ID << 8 | HIGHTORQUE_ID_SRC) &&
            RxData[0] == (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 3) &&
            RxData[1] == HIGHTORQUE_REG_POS_FDBK &&
            RxData[14] == (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 1) &&
            RxData[15] == HIGHTORQUE_REG_TEMP)
        {
            HighTorque[GIMBAL_arrID].fdbk.pos = *(float *)&RxData[2] * 360;
            HighTorque[GIMBAL_arrID].fdbk.spd = *(float *)&RxData[6] * 360;
            HighTorque[GIMBAL_arrID].fdbk.trq = *(float *)&RxData[10];
            HighTorque[GIMBAL_arrID].fdbk.temp = *(float *)&RxData[16];
        }

        // gimbal not over heat
        if (HighTorque[GIMBAL_arrID].fdbk.temp <= 55)
            err_cnt.HighTorque = err.HighTorque = 0; // clear error flag
        // gimbal over heat
        else if (HighTorque[GIMBAL_arrID].fdbk.temp >= 60)
            HighTorque[GIMBAL_arrID].ctrl.Kd = HighTorque[GIMBAL_arrID].ctrl.Kp = 0;
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

        if (VESC_MsgHandler(FDCAN_RxHeader.Identifier, PUSHSHOT_arrID, RxData))
            err_cnt.VESC = err.VESC = 0; // clear error flag
    }
}

// top photogate
void EXTI1_IRQHandler(void)
{
    EXTI->PR1 |= 0x2;
    if (PG_TOP && state == SHOT)
        Brake_Trigger();
}