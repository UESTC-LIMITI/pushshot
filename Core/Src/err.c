#include "usr.h"

struct ERR err;
struct ERR_CNT err_cnt;

void Err(void *argument)
{
    while (1)
    {
        for (unsigned char cnt = 0; cnt < 5; cnt++)
        {
            // maximum communication failure time, 160~200ms
            if (((unsigned char *)&err_cnt)[cnt] == 4)
                *(unsigned char *)&err |= 1 << cnt;
            else
                ++((unsigned char *)&err_cnt)[cnt];
        }

        if (err.R1_pos)
            HighTorque[GIMBAL_arrID].ctrl.spd = 0;

        if (err.R2_pos)
            R2_Pos_Process();

        err.yaw_lim = HighTorque[GIMBAL_arrID].ctrl.pos == YAW_MAX || HighTorque[GIMBAL_arrID].ctrl.pos == YAW_MIN;

        FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA0, (unsigned char *)&err, 1);

        osDelay(40);
    }
}