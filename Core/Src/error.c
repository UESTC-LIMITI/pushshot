#include "user.h"

struct ERR err = {
    .VESC = 1,
    .HighTorque = 1,
    .pos_lidar = 1,
    .pos_chassis = 1,
    .R2_pos = 1,
    .basket_yaw = 1};

void Error(void *argument)
{
    while (1)
    {
        static struct ERR err_prev, err_curr;

        *(unsigned char *)&err_prev = *(unsigned char *)&err_curr;
        *(unsigned char *)&err_curr = *(unsigned char *)&err;
        *(unsigned char *)&err &= *(unsigned char *)&err_prev;

        FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA0, (unsigned char *)&err, 1);

        // set err flag
        state_R.err = *(unsigned char *)&err ? true
                                             : false;

        *(unsigned char *)&err = 0xFF;

        osDelay(500);
    }
}