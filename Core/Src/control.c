#include "user.h"

void Control(void *argument)
{
    while (1)
    {
        switch (state)
        {
        case IDLE:
        {
            VESC_SendCmd(&hfdcan2, 1, VESC_SET_CURR, &HOBBYWING_V9626_KV160);
            break;
        }
        case INIT:
        {
            VESC_SendCmd(&hfdcan2, 1, VESC_SET_SPD, &HOBBYWING_V9626_KV160);
            break;
        }
        case SHOT:
        {
            if (state_R.brake)
                VESC_SendCmd(&hfdcan2, 1, VESC_SET_CURR_BRAKE, &HOBBYWING_V9626_KV160);
            else if (state_R.spd_ctrl)
                VESC_SendCmd(&hfdcan2, 1, VESC_SET_SPD, &HOBBYWING_V9626_KV160);
            else
                VESC_SendCmd(&hfdcan2, 1, VESC_SET_CURR, &HOBBYWING_V9626_KV160);
            break;
        }
        }

        HighTorque_SetMixParam_f(&hfdcan1, 1);

        if (!CAN_fault.M2006 && C610[1 - 1].ctrl.pos)
            C610_SetPos(&hfdcan2, C610_ID1);
        else
            C610_SetTrq(&hfdcan2, C610_ID1);
        osDelay(1);
    }
}