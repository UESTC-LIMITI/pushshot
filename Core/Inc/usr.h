#ifndef __USER_H
#define __USER_H

#include <stdbool.h>

#include "main.h"
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;

#define TIMsw TIM7
#define FDCAN_SUPPORT

// following file at https://github.com/Chosen-Zeng/lib

// hw
#include "TIM.h"
#include "USART.h"
#include "CAN.h"

// algorithm
#include "algorithm.h"
#include "fltr.h"

// periph
#define HIGHTORQUE_NUM 1
#include "HighTorque.h"
#define VESC_NUM 1
#include "VESC.h"

#define PUSHSHOT_arrID 0
#define PUSHSHOT_MOTOR CUBEMARS_R100_KV90
#define GIMBAL_arrID 0

#define PG_BTM (GPIOE->IDR & GPIO_PIN_2)
// #define PG_MID (GPIO->IDR & GPIO_PIN)
#define PG_TOP (GPIOF->IDR & GPIO_PIN_1)

#define CYL1_PORT GPIOD
#define CYL1_PIN GPIO_PIN_7
#define CYL2_PORT GPIOG
#define CYL2_PIN GPIO_PIN_9
#define CYL3_PORT GPIOG
#define CYL3_PIN GPIO_PIN_10

// following variable and function at Core/Src/cfg.c

extern unsigned char task_intvl_ms_cnt_State, task_intvl_ms_cnt_Err, task_intvl_ms_cnt_Comm;
#define TASK_INTVL_ms_State 1
#define TASK_INTVL_ms_Err 1
#define TASK_INTVL_ms_Comm 20

extern bool task_timeout;

#define ERR_CODE_INTVL_ms 10

enum STATE
{
    IDLE,
    MID,
    INIT_SLOW,
    INIT_FAST,
    LOCK,
    SHOT
};
extern enum STATE volatile state;

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
        aim_R2 : 1,
        R2_NetUp : 1;
};
extern struct STATE_W state_W;

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
        yaw_lim_exceed : 1,
        coor_unmatch : 1,
        UV : 1;
};
extern struct ERR err;

struct ERR_CNT
{
    unsigned short VESC,
        HighTorque,
        basket_pos,
        R1_pos,
        R2_pos;
};
extern struct ERR_CNT err_cnt;

extern TIMsw_t runtime, R2_yaw_time, R2_msg_intvl;

extern float R2_yaw_prev;

extern char spd_offset;

extern unsigned char R1_Data[19];

#define GIMBAL_MIN -209
#define GIMBAL_MAX 203

#define GIMBAL_0 ((GIMBAL_MAX + GIMBAL_MIN) / 2)
#define GIMBAL_GR (507 / 23.f)

#define HIGHTORQUE_TEMP_LIM 60

void PeriphInit(void); // call after initialization function created by CubeMX
void Scheduler(void);  // call in loop in main.c

// task function
void State(void);
void Err(void);
void Comm(void);

void R2_Pos_Process(void); // at Core/Src/comm.c

void Brake_Trigger(void); // at Core/Src/state.c

#endif