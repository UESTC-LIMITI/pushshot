#include "usr.h"

void Err(void *argument)
{
    while (1)
    {
        static const struct
        {
            unsigned short VESC,
                HighTorque,
                basket_pos,
                R1_pos,
                R2_pos;
        } timeout_ms = {
            .VESC = 2,
            .HighTorque = 5,
            .basket_pos = 200,
            .R1_pos = 5,
            .R2_pos = 50};

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
            static unsigned char cnt_err_R1_pos;
            if (err.R1_pos && cnt_err_R1_pos++ == timeout_ms.R1_pos)
                HighTorque[GIMBAL_arrID].ctrl.spd = 0;
            else
                cnt_err_R1_pos = 0;
        }

        {
            static unsigned char cnt_err_R2_pos;
            if (err.R2_pos && cnt_err_R2_pos++ == timeout_ms.R2_pos)
                R2_Pos_Process();
            else
                cnt_err_R2_pos = 0;
        }

        err.yaw_lim_exceed = HighTorque[GIMBAL_arrID].ctrl.pos == YAW_MAX || HighTorque[GIMBAL_arrID].ctrl.pos == YAW_MIN;

        err.UV = VESC[PUSHSHOT_arrID].fdbk.volt < 24.3; // under voltage

        {
            static unsigned char cnt_err_code;
            if (cnt_err_code++ == 100) // send error code per 100ms
            {
                FDCAN_BRS_SendData(&hfdcan3, FDCAN_STANDARD_ID, 0xA0, (unsigned char *)&err, 1);
                cnt_err_code = 0;
            }
        }

        osDelay(1);
    }
}