#ifndef FDCAN_H
#define FDCAN_H


#include "main.h"
#include "DJI_Transform_master.h"
#include "DM_J4310.h"

//舵轮电机回零
typedef enum
{
	eHOMING_IDLE,
	eHOMING_READY,
    eHOMING_MOVING,
    eHOMING_DONE	
}Steer_Homing_State;

typedef struct
{
	DJI_HandleTypeDef *steer;
	GPIO_TypeDef *port;           // 光电门所在的 GPIO 组 (如 GPIOC)
    uint16_t pin;                 // 光电门对应的引脚号 (如 GPIO_PIN_1)
    uint8_t is_homed;             // 该单轮是否已完成回零	
}Steer_Homing_TypeDef;


//void my_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* HI14 IMU 驱动（bsp 层）：数据结构与解析都在 bsp/chassis/gyro_HI14.c，
 * 这里只做转发，方便 app 层包含。 */
#include "gyro_HI14.h"

extern uint32_t hi14_bitrate_kbps;   // HI14 当前尝试/锁定的 CAN 速率（kbps）

/* 收到的 CAN 帧按 ID 统计（诊断用：看清陀螺仪实际开了哪几路 TPDO） */
#define HI14_ID_MAX 12
typedef struct
{
    uint32_t id;        // CAN ID
    uint32_t cnt;       // 该 ID 的帧数
    uint8_t  idtype;    // 1=29位扩展 0=11位标准
    uint8_t  dlc;       // 数据长度
    uint8_t  d[8];      // 最近一帧数据
} HI14_IdStat;

extern HI14_IdStat hi14_ids[HI14_ID_MAX];
extern uint8_t hi14_id_num;

/* yaw 环遥测（整定采数用） */
typedef struct
{
	float target;   // 目标航向 deg
	float actual;   // 实际航向 deg
	float err;      // 偏差 deg
	float vw;       // yaw 环输出的角速度指令
	float gz;       // 陀螺 Z 轴角速度 deg/s
} YawTel_TypeDef;

extern YawTel_TypeDef yaw_tel;

void Chassis_C620_Init(void);
void App_SBUS_Chassis(void *argument);

/* DM-J4310-2EC 关节电机（CAN2）：两台，编号 CAN_ID = 1、2，句柄定义在 app/fdcan_it.c
 * 上电 500ms 后由 DM_J4310_Task 使能并各自转到“当前位置 + 30°”（见 DM_J4310_START_MOVE） */
#define DM_J4310_NUM    2

extern DM_J4310_HandleTypeDef dm_joint[DM_J4310_NUM];

/** @brief 初始化 DM-J4310：启动 CAN2（1Mbps）+ 建句柄（不使能、不发帧） */
void DM_J4310_App_Init(void);

/** @brief DM-J4310 心跳任务：500ms 后使能并转 +30°；之后每 5ms 重发上一帧防止 CAN Timeout */
void DM_J4310_Task(void *argument);

#endif
