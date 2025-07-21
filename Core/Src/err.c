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
        .VESC = VESC_TIMEOUT_ms,
        .HighTorque = HIGHTORQUE_TIMEOUT_ms,
        .basket_pos = BASKET_POS_TIMEOUT_ms,
        .R1_pos = R1_POS_TIMEOUT_ms,
        .R2_pos = R2_POS_TIMEOUT_ms,
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

    if (err.R1_pos)
        HighTorque[GIMBAL_idx].ctrl.spd = 0;

    if (err.R2_pos)
        R2_Pos_Process();

    err.yaw_lim_exceed = HighTorque[GIMBAL_idx].ctrl.Kp && (HighTorque[GIMBAL_idx].ctrl.pos == GIMBAL_MAX || HighTorque[GIMBAL_idx].ctrl.pos == GIMBAL_MIN);

    err.VESC_UV = VESC[PUSHSHOT_idx].fdbk.volt < VESC_VOLTAGE_LIM_MIN; // under voltage

    {
        static bool check;
        if (!check && !err.HighTorque)
        {
            check = true;

            err.HighTorque_startup = HighTorque[GIMBAL_idx].fdbk.pos > 360 - GIMBAL_MAX - 9 || HighTorque[GIMBAL_idx].fdbk.pos < -360 - GIMBAL_MIN + 9;
        }
        if (check && err.HighTorque_startup)
            err.HighTorque = true;
    }

    // gimbal over heat
    if (HighTorque[GIMBAL_idx].fdbk.temp > HIGHTORQUE_TEMP_WARNING)
        err.HighTorque = true;
    if ((err.HighTorque_OH = HighTorque[GIMBAL_idx].fdbk.temp > HIGHTORQUE_TEMP_LIM))
        HighTorque[GIMBAL_idx].ctrl.Kd = HighTorque[GIMBAL_idx].ctrl.Kp = 0;

    {
        static unsigned char intvl_ms_cnt;
        if (++intvl_ms_cnt == ERR_CODE_INTVL_ms)
        {
            FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA0, (unsigned char *)&err, 1);
            intvl_ms_cnt = 0;
        }
    }
}