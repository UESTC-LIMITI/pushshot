#include "user.h"

struct ERR err;
struct ERR_CNT err_cnt;

void Error(void *argument)
{
    while (1)
    {
        for (unsigned char cnt = 0; cnt < 6; cnt++)
        {
            // max failure
            if (((unsigned char *)&err_cnt)[cnt] == 9)
                *(unsigned char *)&err |= 1 << cnt;
            else
                ((unsigned char *)&err_cnt)[cnt]++;
        }

        // under voltage
        if (VESC[PUSHSHOT_ID - VESC_ID_OFFSET].fdbk.volt <= 22.8)
            err.UV = 1;

        FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA0, (unsigned char *)&err, 1);

        // set err flag
        state_R.err = *(unsigned char *)&err ? 1
                                             : 0;

        osDelay(40);
    }
}