#include "usr.h"

void Err(void)
{
    static const struct
    {
        unsigned short VESC,
            HighTorque,
            basket_pos,
            R1_pos,
            R2_pos;
    } timeout_ms = {
        .VESC = 1,
        .HighTorque = 1,
        .basket_pos = 125,
        .R1_pos = 1,
        .R2_pos = 40,
    };

    for (unsigned char cnt = 0; cnt < 5; cnt++) // 5 timeout error
    {
        if (((unsigned short *)&err_cnt)[cnt] == ((unsigned short *)&timeout_ms)[cnt]) // reach timeout limit
            *(unsigned char *)&err |= 1 << cnt;                                        // set error
        else
        {
            if (!((unsigned short *)&err_cnt)[cnt])
                *(unsigned char *)&err &= ~(1 << cnt); // reset error

            ++((unsigned short *)&err_cnt)[cnt];
        }
    }

    {
        static unsigned char timeout_ms_cnt;
        if (err.R1_pos && timeout_ms_cnt++ == timeout_ms.R1_pos)
            HighTorque[GIMBAL_arrID].ctrl.spd = 0;
        else
            timeout_ms_cnt = 0;
    }

    {
        static unsigned char timeout_ms_cnt;
        if (err.R2_pos && timeout_ms_cnt++ == timeout_ms.R2_pos)
            R2_Pos_Process();
        else
            timeout_ms_cnt = 0;
    }

    err.yaw_lim_exceed = HighTorque[GIMBAL_arrID].ctrl.pos == GIMBAL_MAX || HighTorque[GIMBAL_arrID].ctrl.pos == GIMBAL_MIN;

    err.UV = VESC[PUSHSHOT_arrID].fdbk.volt < 24; // under voltage

    {
        static unsigned char intvl_ms_cnt;
        if (++intvl_ms_cnt == ERR_CODE_INTVL_ms)
        {
            FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA0, (unsigned char *)&err, 1);
            intvl_ms_cnt = 0;
        }
    }
}