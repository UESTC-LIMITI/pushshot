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
        // set err flag
        state_R.err = err.VESC |
                      err.HighTorque |
                      err.pos_lidar |
                      err.pos_chassis |
                      err.R2_pos |
                      err.basket_yaw;

        err.VESC = err.HighTorque = err.pos_lidar = err.pos_chassis = err.R2_pos = err.basket_yaw = 1;

        osDelay(100);
    }
}