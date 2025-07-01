#ifndef __USER_H
#define __USER_H

#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"

#define TIMsw TIM7
#define FDCAN_SUPPORT

// all the following file can be found at https://github.com/Chosen-Zeng/lib

// from hw
#include "TIM.h"
#include "USART.h"
#include "CAN.h"

// from algorithm
#include "algorithm.h"
#include "fltr.h"

// from periph
#define VESC_NUM 1
#define VESC_ID_OFFSET 1
#include "VESC.h"
#define PUSHSHOT_ID 1
#define PUSHSHOT_arrID 0
#define MOTOR CUBEMARS_R100_KV90

#define HIGHTORQUE_NUM 1
#define HIGHTORQUE_ID_OFFSET 1
#include "HighTorque.h"
#define GIMBAL_ID 1
#define GIMBAL_arrID 0

extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;

extern USART_info_t UART7_info, UART5_info;

enum STATE
{
    IDLE,
    INIT,
    LOCK,
    SHOT
};
extern enum STATE state;

struct STATE_R
{
    unsigned char spd_ctrl : 1,
        brake : 1,
        fitting : 1;
};
extern struct STATE_R state_R;

struct STATE_W
{
    unsigned char ball : 1,
        gimbal : 1,
        aim_R2 : 1,
        R2_NetUp : 1;
};
extern struct STATE_W state_W;

struct ERR
{
    unsigned char VESC : 1,
        HighTorque : 1,
        basket_pos : 1,
        R1_pos : 1,
        R2_pos : 1,
        yaw_lim : 1;
};
extern struct ERR err;

struct ERR_CNT
{
    unsigned char VESC,
        HighTorque,
        basket_pos,
        R1_pos,
        R2_pos;
};
extern struct ERR_CNT err_cnt;

struct target_info
{
    float dist_cm, yaw;
    MovAvgFltr_t dist_fltr, yaw_fltr;
};
extern struct target_info basket_info, R2_info;

struct pos_info
{
    float x, y, yaw;
};
extern struct pos_info R1_pos;

struct pos_t
{
    float x, y;
};
extern struct pos_t R2_pos, basket_pos;

extern TIMsw_t runtime, R2_yaw_time, R2_yaw_intvl;

extern float R2_yaw_prev;

extern char spd_offset;

#define Gimbal_GR (14.6875 * 1) // real gear ratio * gain

#define YAW_MIN -127
#define YAW_MAX 135

void FDCAN1_Init(void);
void FDCAN2_Init(void);
void FDCAN3_Init(void);
void TIM7_Init(void);
void UART5_Init(void);

void R2_Pos_Process(void);

#endif