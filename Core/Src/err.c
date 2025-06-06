#include "user.h"

struct ERR err;
struct ERR_CNT err_cnt;

void Error(void *argument)
{
    while (1)
    {
        for (unsigned char cnt = 0; cnt < 5; cnt++)
        {
            // max failure, 200ms
            if (((unsigned char *)&err_cnt)[cnt] == 10)
                *(unsigned char *)&err |= 1 << cnt;
            else
                ((unsigned char *)&err_cnt)[cnt]++;
        }

        if (err.R2_pos)
        {
            if (!state_W.R2_ready)
                R2_pos.x = 12.5,
                R2_pos.y = -4;

            R2_Pos_Process();
        }

        err.yaw_lim = HighTorque[GIMBAL_ID].ctrl.pos == YAW_MAX || HighTorque[GIMBAL_ID].ctrl.pos == YAW_MIN;

        FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA0, (unsigned char *)&err, 1);

        osDelay(50);
    }
}