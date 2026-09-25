#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"



//定时器
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;

//队列
extern osMessageQueueId_t Handle_Data_ToChassisHandle;
extern osMessageQueueId_t Handle_Data_ToUpperHandle;
extern osMessageQueueId_t Chassis_To_CanHandle;

//信号量
extern osSemaphoreId_t Joy_DataHandle;
extern osSemaphoreId_t Gyro_DataHandle;
extern osSemaphoreId_t Radar_DataHandle;
extern osSemaphoreId_t DT35_DataHandle;
extern osSemaphoreId_t Laser_L_DataHandle;
extern osSemaphoreId_t Laser_R_DataHandle;
extern osSemaphoreId_t Air_DataHandle;

extern osSemaphoreId_t Chassis_Low_SpeedHandle;
extern osSemaphoreId_t ready_connectHandle;         //准备对接信号量
extern osSemaphoreId_t transform_weaponHandle;      //存武器信号量
extern osSemaphoreId_t recycle_armHandle;           //三区回收机械臂信号量
extern osSemaphoreId_t ready_captureHandle;         //准备抓取信号量
extern osSemaphoreId_t ready_suctionHandle;         //准备吸块信号量
extern osSemaphoreId_t start_captureHandle;         //开始抓取信号量
extern osSemaphoreId_t start_suctionHandle;          //开始吸块信号量
extern osSemaphoreId_t start_centerHandle;          //吸块之后开始归中信号量，未使用
extern osSemaphoreId_t Take_blockHandle;            //从归中取块信号量
extern osSemaphoreId_t close_suckerHandle;          //三区到点之后自动关闭吸盘
extern osSemaphoreId_t transmit_blockHandle;        //递块信号量
extern osSemaphoreId_t place_blockHandle;            //三区放置哪一列信号量
extern osSemaphoreId_t arm_initHandle;                  //机械臂初始摆放(防守)
extern osSemaphoreId_t Close_R_SuckerHandle;            //递块关闭吸盘




void My_Init(void);


#endif
