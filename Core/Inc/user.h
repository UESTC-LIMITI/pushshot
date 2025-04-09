#ifndef __USER_H
#define __USER_H

#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"

#define TIMER TIM7
#define FDCAN_SUPPORT

#define DATA_OUTPUT

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

#define HIGHTORQUE_NUM 1
#define HIGHTORQUE_ID_OFFSET 2
#include "HighTorque.h"

extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;

enum STATE
{
    RST,
    BACK,
    INIT,
    IDLE,
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
        aim_R2 : 1,
        RST : 1;
};
extern struct STATE_W state_W;

struct ERR
{
    unsigned char VESC : 1,
        HighTorque : 1,
        pos_lidar : 1,
        basket_info : 1,
        pos_chassis : 1,
        R2_pos : 1;
};
extern struct ERR err;

struct target_info
{
    float dist_cm, yaw;
    MovAvgFltr_t dist_cm_fltr, yaw_fltr;
};
extern struct target_info basket_info, R2_info;

struct pos_info
{
    float x, y, yaw;
};
extern struct pos_info R1_pos_lidar, R1_pos_chassis, R2_pos;

extern timer_t runtime;

extern unsigned char RxData_D1S0[12];

void FDCAN1_Init(void);
void FDCAN2_Init(void);
void FDCAN3_Init(void);
void TIM7_Init(void);
void TIM16_Init(void);
void UART5_Init(void);

#endif