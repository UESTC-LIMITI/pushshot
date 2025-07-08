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
#include "VESC.h"
#define PUSHSHOT_arrID 0
#define PUSHSHOT_MOTOR CUBEMARS_R100_KV90

#define HIGHTORQUE_NUM 1
#include "HighTorque.h"
#define GIMBAL_arrID 0

#define PG_BTM (GPIOE->IDR & GPIO_PIN_2)
// #define PG_MID (GPIO->IDR & GPIO_PIN)
// #define PG_TOP (GPIO->IDR & GPIO_PIN)
#define PG_BREAK (GPIOF->IDR & GPIO_PIN_1)

#define CYL1_PORT GPIOD
#define CYL1_PIN GPIO_PIN_7
#define CYL2_PORT GPIOG
#define CYL2_PIN GPIO_PIN_9
#define CYL3_PORT GPIOG
#define CYL3_PIN GPIO_PIN_10

extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;

enum STATE
{
    IDLE,
    MID,
    INIT_SLOW,
    INIT_FAST,
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

struct pos_info
{
    float x, y, yaw;
};

struct pos_t
{
    float x, y;
};

struct target_info
{
    float dist_cm, yaw;
    MovAvgFltr_t dist_fltr, yaw_fltr;
};
extern struct target_info basket_info, R2_info;

struct ERR
{
    unsigned char VESC : 1,
        HighTorque : 1,
        basket_pos : 1,
        R1_pos : 1,
        R2_pos : 1,
        yaw_lim : 1,
        coor : 1;
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

extern TIMsw_t runtime, R2_yaw_time, R2_msg_intvl;

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