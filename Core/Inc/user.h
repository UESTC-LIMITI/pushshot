#ifndef __USER_H
#define __USER_H

#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"

#define TIMER TIM7
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

#define HIGHTORQUE_NUM 1
#define HIGHTORQUE_ID_OFFSET 1
#include "HighTorque.h"
#define GIMBAL_ID 1

extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;

extern USART_info_t UART7_info, UART5_info;

enum STATE
{
    IDLE,
    BACK,
    SHOT
};
extern enum STATE state;

struct STATE_R
{
    unsigned char err : 1,
        shot_ready : 1,
        spd_ctrl : 1,
        brake : 1,
        fitting : 1;
};
extern struct STATE_R state_R;

struct STATE_W
{
    unsigned char ball : 1,
        aim_R2 : 1;
};
extern struct STATE_W state_W;

struct ERR
{
    unsigned char VESC : 1,
        HighTorque : 1,
        pos_lidar : 1,
        pos_chassis : 1,
        R2_pos : 1,
        basket_info : 1;
};
extern struct ERR err;

struct ERR_CNT
{
    unsigned char VESC,
        HighTorque,
        pos_lidar,
        pos_chassis,
        R2_pos,
        basket_info;
};
extern struct ERR_CNT err_cnt;

struct target_info
{
    float dist_cm, yaw;
    MovAvgFltr_t dist_fltr;
};
extern struct target_info basket_info, R2_info;

struct pos_info
{
    float x, y, yaw;
};
extern struct pos_info R1_pos_lidar, R1_pos_chassis, R2_pos;

extern timer_t runtime, HighTorque_time, gimbal_time;

extern float yaw_prev;

extern MovAvgFltr_t yaw_fltr;

extern unsigned char RxData_D1S2[];

void FDCAN1_Init(void);
void FDCAN2_Init(void);
void FDCAN3_Init(void);
void TIM7_Init(void);
void TIM16_Init(void);
void UART5_Init(void);

#endif