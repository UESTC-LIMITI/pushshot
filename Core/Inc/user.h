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

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

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
    unsigned char CAN_fault : 1;
    unsigned char shot_ready : 1;
    unsigned char spd_ctrl : 1;
    unsigned char brake : 1;
};
extern struct STATE_R state_R;

struct STATE_W
{
    unsigned char ball : 1;
    unsigned char fitting : 1;
    unsigned char aim_R2 : 1;
    unsigned char RST : 1;
};
extern struct STATE_W state_W;

struct CAN_FAULT
{
    unsigned char VESC : 1;
    unsigned char HighTorque : 1;
    unsigned char basket_info : 1;
    unsigned char R2_info : 1;
};
extern struct CAN_FAULT CAN_fault;

struct target_info
{
    float dist_cm, yaw;
    MovAvgFltr_t dist_cm_fltr, yaw_fltr;
};
extern struct target_info basket_info, R2_info;

extern timer_t runtime;

void FDCAN1_Init(void);
void FDCAN2_Init(void);
void FDCAN3_Init(void);
void TIM7_Init(void);
void TIM16_Init(void);

#endif