/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis.h"
#include "handle_rev.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for print */
osThreadId_t printHandle;
const osThreadAttr_t print_attributes = {
  .name = "print",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for FDCAN */
osThreadId_t FDCANHandle;
const osThreadAttr_t FDCAN_attributes = {
  .name = "FDCAN",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DM_J4310 */
osThreadId_t DM_J4310Handle;
const osThreadAttr_t DM_J4310_attributes = {
  .name = "DM_J4310",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Unitree GO */
osThreadId_t GO_JointHandle;
const osThreadAttr_t GO_Joint_attributes = {
  .name = "GO_Joint",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Handle_Data_ToChassis */
osMessageQueueId_t Handle_Data_ToChassisHandle;
const osMessageQueueAttr_t Handle_Data_ToChassis_attributes = {
  .name = "Handle_Data_ToChassis"
};
/* Definitions for Handle_Data_ToUpper */
osMessageQueueId_t Handle_Data_ToUpperHandle;
const osMessageQueueAttr_t Handle_Data_ToUpper_attributes = {
  .name = "Handle_Data_ToUpper"
};
/* Definitions for Chassis_To_Can */
osMessageQueueId_t Chassis_To_CanHandle;
const osMessageQueueAttr_t Chassis_To_Can_attributes = {
  .name = "Chassis_To_Can"
};
/* Definitions for Joy_Data */
osSemaphoreId_t Joy_DataHandle;
const osSemaphoreAttr_t Joy_Data_attributes = {
  .name = "Joy_Data"
};
/* Definitions for Gyro_Data */
osSemaphoreId_t Gyro_DataHandle;
const osSemaphoreAttr_t Gyro_Data_attributes = {
  .name = "Gyro_Data"
};
/* Definitions for Radar_Data */
osSemaphoreId_t Radar_DataHandle;
const osSemaphoreAttr_t Radar_Data_attributes = {
  .name = "Radar_Data"
};
/* Definitions for DT35_Data */
osSemaphoreId_t DT35_DataHandle;
const osSemaphoreAttr_t DT35_Data_attributes = {
  .name = "DT35_Data"
};
/* Definitions for Chassis_Low_Speed */
osSemaphoreId_t Chassis_Low_SpeedHandle;
const osSemaphoreAttr_t Chassis_Low_Speed_attributes = {
  .name = "Chassis_Low_Speed"
};
/* Definitions for ready_connect */
osSemaphoreId_t ready_connectHandle;
const osSemaphoreAttr_t ready_connect_attributes = {
  .name = "ready_connect"
};
/* Definitions for transform_weapon */
osSemaphoreId_t transform_weaponHandle;
const osSemaphoreAttr_t transform_weapon_attributes = {
  .name = "transform_weapon"
};
/* Definitions for recycle_arm */
osSemaphoreId_t recycle_armHandle;
const osSemaphoreAttr_t recycle_arm_attributes = {
  .name = "recycle_arm"
};
/* Definitions for ready_capture */
osSemaphoreId_t ready_captureHandle;
const osSemaphoreAttr_t ready_capture_attributes = {
  .name = "ready_capture"
};
/* Definitions for ready_suction */
osSemaphoreId_t ready_suctionHandle;
const osSemaphoreAttr_t ready_suction_attributes = {
  .name = "ready_suction"
};
/* Definitions for start_capture */
osSemaphoreId_t start_captureHandle;
const osSemaphoreAttr_t start_capture_attributes = {
  .name = "start_capture"
};
/* Definitions for start_suction */
osSemaphoreId_t start_suctionHandle;
const osSemaphoreAttr_t start_suction_attributes = {
  .name = "start_suction"
};
/* Definitions for start_center */
osSemaphoreId_t start_centerHandle;
const osSemaphoreAttr_t start_center_attributes = {
  .name = "start_center"
};
/* Definitions for Take_block */
osSemaphoreId_t Take_blockHandle;
const osSemaphoreAttr_t Take_block_attributes = {
  .name = "Take_block"
};
/* Definitions for close_sucker */
osSemaphoreId_t close_suckerHandle;
const osSemaphoreAttr_t close_sucker_attributes = {
  .name = "close_sucker"
};
/* Definitions for transmit_block */
osSemaphoreId_t transmit_blockHandle;
const osSemaphoreAttr_t transmit_block_attributes = {
  .name = "transmit_block"
};
/* Definitions for place_block */
osSemaphoreId_t place_blockHandle;
const osSemaphoreAttr_t place_block_attributes = {
  .name = "place_block"
};
/* Definitions for Laser_L_Data */
osSemaphoreId_t Laser_L_DataHandle;
const osSemaphoreAttr_t Laser_L_Data_attributes = {
  .name = "Laser_L_Data"
};
/* Definitions for Laser_R_Data */
osSemaphoreId_t Laser_R_DataHandle;
const osSemaphoreAttr_t Laser_R_Data_attributes = {
  .name = "Laser_R_Data"
};
/* Definitions for Air_Data */
osSemaphoreId_t Air_DataHandle;
const osSemaphoreAttr_t Air_Data_attributes = {
  .name = "Air_Data"
};
/* Definitions for arm_init */
osSemaphoreId_t arm_initHandle;
const osSemaphoreAttr_t arm_init_attributes = {
  .name = "arm_init"
};
/* Definitions for Close_R_Sucker */
osSemaphoreId_t Close_R_SuckerHandle;
const osSemaphoreAttr_t Close_R_Sucker_attributes = {
  .name = "Close_R_Sucker"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void App_SBUS_Chassis(void *argument);
void DM_J4310_Task(void *argument);
void GO_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void App_print(void *argument);
extern void App_FDCAN(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Joy_Data */
  Joy_DataHandle = osSemaphoreNew(1, 0, &Joy_Data_attributes);

  /* creation of Gyro_Data */
  Gyro_DataHandle = osSemaphoreNew(1, 0, &Gyro_Data_attributes);

  /* creation of Radar_Data */
  Radar_DataHandle = osSemaphoreNew(1, 0, &Radar_Data_attributes);

  /* creation of DT35_Data */
  DT35_DataHandle = osSemaphoreNew(1, 0, &DT35_Data_attributes);

  /* creation of Chassis_Low_Speed */
  Chassis_Low_SpeedHandle = osSemaphoreNew(1, 0, &Chassis_Low_Speed_attributes);

  /* creation of ready_connect */
  ready_connectHandle = osSemaphoreNew(1, 0, &ready_connect_attributes);

  /* creation of transform_weapon */
  transform_weaponHandle = osSemaphoreNew(1, 0, &transform_weapon_attributes);

  /* creation of recycle_arm */
  recycle_armHandle = osSemaphoreNew(1, 0, &recycle_arm_attributes);

  /* creation of ready_capture */
  ready_captureHandle = osSemaphoreNew(1, 0, &ready_capture_attributes);

  /* creation of ready_suction */
  ready_suctionHandle = osSemaphoreNew(1, 0, &ready_suction_attributes);

  /* creation of start_capture */
  start_captureHandle = osSemaphoreNew(1, 0, &start_capture_attributes);

  /* creation of start_suction */
  start_suctionHandle = osSemaphoreNew(1, 0, &start_suction_attributes);

  /* creation of start_center */
  start_centerHandle = osSemaphoreNew(1, 0, &start_center_attributes);

  /* creation of Take_block */
  Take_blockHandle = osSemaphoreNew(1, 0, &Take_block_attributes);

  /* creation of close_sucker */
  close_suckerHandle = osSemaphoreNew(1, 0, &close_sucker_attributes);

  /* creation of transmit_block */
  transmit_blockHandle = osSemaphoreNew(1, 0, &transmit_block_attributes);

  /* creation of place_block */
  place_blockHandle = osSemaphoreNew(1, 0, &place_block_attributes);

  /* creation of Laser_L_Data */
  Laser_L_DataHandle = osSemaphoreNew(1, 0, &Laser_L_Data_attributes);

  /* creation of Laser_R_Data */
  Laser_R_DataHandle = osSemaphoreNew(1, 0, &Laser_R_Data_attributes);

  /* creation of Air_Data */
  Air_DataHandle = osSemaphoreNew(1, 0, &Air_Data_attributes);

  /* creation of arm_init */
  arm_initHandle = osSemaphoreNew(1, 0, &arm_init_attributes);

  /* creation of Close_R_Sucker */
  Close_R_SuckerHandle = osSemaphoreNew(1, 0, &Close_R_Sucker_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Handle_Data_ToChassis */
  Handle_Data_ToChassisHandle = osMessageQueueNew (1, sizeof(Handle_Data), &Handle_Data_ToChassis_attributes);

  /* creation of Handle_Data_ToUpper */
  Handle_Data_ToUpperHandle = osMessageQueueNew (1, sizeof(Handle_Data), &Handle_Data_ToUpper_attributes);

  /* creation of Chassis_To_Can */
  Chassis_To_CanHandle = osMessageQueueNew (1, sizeof(Chassis_Motor), &Chassis_To_Can_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of print */
  printHandle = osThreadNew(App_print, NULL, &print_attributes);

  /* creation of FDCAN */
  FDCANHandle = osThreadNew(App_FDCAN, NULL, &FDCAN_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadNew(App_SBUS_Chassis, NULL, &FDCAN_attributes);
  DM_J4310Handle = osThreadNew(DM_J4310_Task, NULL, &DM_J4310_attributes);
  GO_JointHandle = osThreadNew(GO_Task, NULL, &GO_Joint_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_App_print */
/**
  * @brief  Function implementing the print thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_App_print */
__weak void App_print(void *argument)
{
  /* USER CODE BEGIN App_print */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_print */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

