#include "user.h"

struct CAN_FAULT CAN_fault = {
    .VESC = 1,
    .HighTorque = 1,
    .basket_info = 1,
    .R2_info = 1};

void Error(void *argument)
{
    while (1)
    {
        // set fault flag
        state_R.CAN_fault = CAN_fault.VESC |
                            CAN_fault.HighTorque |
                            CAN_fault.basket_info |
                            CAN_fault.R2_info;

        CAN_fault.VESC = CAN_fault.HighTorque = CAN_fault.basket_info = CAN_fault.R2_info = 1;

        osDelay(100);
    }
}