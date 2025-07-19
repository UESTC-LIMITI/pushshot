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

        // HighTorque_MsgHandler(FDCAN_RxHeader.Identifier, GIMBAL_idx, RxData);

        if (FDCAN_RxHeader.Identifier == (HighTorque[GIMBAL_idx].ID << 8 | HIGHTORQUE_ID_SRC) &&
            RxData[0] == (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 3) &&
            RxData[1] == HIGHTORQUE_REG_POS_FDBK &&
            RxData[14] == (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 1) &&
            RxData[15] == HIGHTORQUE_REG_TEMP)
        {
            HighTorque[GIMBAL_idx].fdbk.pos = *(float *)&RxData[2] * 360;
            HighTorque[GIMBAL_idx].fdbk.spd = *(float *)&RxData[6] * 360;
            HighTorque[GIMBAL_idx].fdbk.trq = *(float *)&RxData[10];
            HighTorque[GIMBAL_idx].fdbk.temp = *(float *)&RxData[16];
        }

        // gimbal not over heat
        if (HighTorque[GIMBAL_idx].fdbk.temp <= 55)
            err_cnt.HighTorque = 0; // clear error flag
        // gimbal over heat
        else if (HighTorque[GIMBAL_idx].fdbk.temp > HIGHTORQUE_TEMP_LIM)
            HighTorque[GIMBAL_idx].ctrl.Kd = HighTorque[GIMBAL_idx].ctrl.Kp = 0;
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

        if (VESC_MsgHandler(FDCAN_RxHeader.Identifier, PUSHSHOT_idx, RxData))
            err_cnt.VESC = 0; // clear error flag
    }
}

// top photogate
void EXTI1_IRQHandler(void)
{
    EXTI->PR1 |= 0x2;
    if (PG_TOP && state == SHOT)
        Brake_Trigger();
}

// timer for scheduler
void TIM6_DAC_IRQHandler(void)
{
    TIM6->SR &= ~0x1;

    if (task_intvl_ms_cnt_State == TASK_INTVL_ms_State)
        task_timeout = true;
    else
        ++task_intvl_ms_cnt_State;

    if (task_intvl_ms_cnt_Err == TASK_INTVL_ms_Err)
        task_timeout = true;
    else
        ++task_intvl_ms_cnt_Err;

    if (task_intvl_ms_cnt_Comm == TASK_INTVL_ms_Comm)
        task_timeout = true;
    else
        ++task_intvl_ms_cnt_Comm;
}