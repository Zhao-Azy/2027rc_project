#ifndef DRV_CAN_H
#define DRV_CAN_H

#include "main.h"

#ifdef STM32H7xx_HAL_H
	#include "fdcan.h"
#elif defined (__STM32F4xx_HAL_H) || defined(__STM32F1xx_HAL_H)
	#include "can.h"
#endif 



#define ALL_INIT 1			//H7系列fdcan是否全部初始化


#ifdef STM32H7xx_HAL_H
	void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
	
	#if ALL_INIT
		void FDCAN_All_Init(void);
	#endif

#elif defined (__STM32F4xx_HAL_H) || defined(__STM32F1xx_HAL_H)

	void Can1_Filter_Init(CAN_HandleTypeDef *hcan);

	#if defined (CAN2)		//检查单片机是否有can2外设
		void Can2_Filter_Init(CAN_HandleTypeDef *hcan);
		void Can_Filter_Init(void);
	#endif

#endif


#endif
