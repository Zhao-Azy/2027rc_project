#include "usart.h"
#include <string.h>
#include "my_main.h"
// lib

// app
#include "usart_it.h"
#include "uper_control.h"
#include "chassis_control.h"
#include "DT35.h"
#include "ZifoLib.h"
// bsp

static uint8_t Ccr_Check(uint8_t *message, uint8_t start_index, uint8_t len);

#define SBUS_SIGNAL_OK       0u
#define SBUS_SIGNAL_LOST     1u
#define SBUS_SIGNAL_FAILSAFE 2u

/*global variable-----------------------------------------------------------*/
uint8_t handle_rev_buff[HANDLE_LENGTH] = {0};							 // 手柄数据缓冲区
uint8_t gyro_rev_buff[GYRO_LENGTH] = {0};									 // 陀螺仪数据缓冲区
uint8_t radar_rev_buff[RADAR_LENGTH] = {0};								 // 小电脑
uint8_t DT35_rev_buff[DT35_LENGTH] = {0};									 // DT35数据缓冲区
uint8_t air_pump_rev_buff[AIR_PUMP_LENGTH] = {0};							//气泵数据缓冲区
uint8_t laser_l_rev_buff[LASER_LENGTH] = {0};							 // 激光测距L数据缓冲区
uint8_t laser_r_rev_buff[LASER_LENGTH] = {0};							 // 激光测距R数据缓冲区

volatile uint16_t CH[18] = {0};
volatile uint8_t sbus_failsafe_status = SBUS_SIGNAL_LOST;
volatile uint32_t sbus_last_tick = 0;
volatile uint32_t sbus_rx_start_status = HAL_ERROR;
volatile uint32_t sbus_rx_event_count = 0;
volatile uint32_t sbus_valid_frame_count = 0;
volatile uint32_t sbus_invalid_frame_count = 0;
volatile uint32_t sbus_error_count = 0;
volatile uint32_t sbus_last_error = HAL_UART_ERROR_NONE;
volatile uint16_t sbus_last_rx_size = 0;
volatile uint8_t sbus_footer = 0;

uint8_t gyro_cmd[] = {0x77, 0x05, 0x00, 0x0C, 0x08, 0x19}; // 陀螺仪命令
uint8_t nuc_cmd[] = {0xAA, 0x01, 0x01, 0xBB};							 // 小电脑显示图片初始化
uint8_t nuc_cmd1[] = {0xAA, 0x02, 0x00, 0xBB};						 // 对接完发这个
uint8_t nuc_cmd_start[] = {0xAA, 0x00, 0x01, 0xBB};				 // 让小电脑准备显示
uint8_t nuc_kfs[15] = {0};																 // 给小电脑发的KFS位置
uint8_t last_send_KFS[12] = {0};														 // 上一次发送给小电脑的KFS位置	

uint8_t infrared_cmd1[] = {0x01, 0x02, 0x03};					//红外放置第一列信号
uint8_t infrared_cmd2[] = {0xAA, 0x02, 0xBB};					//红放置第二列信号
uint8_t infrared_cmd3[] = {0xAA, 0x03, 0xBB};					//红外放置第三列信号

uint8_t inf_cmd[6] = {0};

volatile uint8_t is_send = 0;				//是否给小电脑发送了块的位置


/*fuction-------------------------------------------------------------------*/

static void SBUS_DecodeFrame(const uint8_t *frame)
{
	CH[0] = (uint16_t)(((uint16_t)frame[1] | ((uint16_t)frame[2] << 8)) & 0x07FFu);
	CH[1] = (uint16_t)((((uint16_t)frame[2] >> 3) | ((uint16_t)frame[3] << 5)) & 0x07FFu);
	CH[2] = (uint16_t)((((uint16_t)frame[3] >> 6) | ((uint16_t)frame[4] << 2) | ((uint16_t)frame[5] << 10)) & 0x07FFu);
	CH[3] = (uint16_t)((((uint16_t)frame[5] >> 1) | ((uint16_t)frame[6] << 7)) & 0x07FFu);
	CH[4] = (uint16_t)((((uint16_t)frame[6] >> 4) | ((uint16_t)frame[7] << 4)) & 0x07FFu);
	CH[5] = (uint16_t)((((uint16_t)frame[7] >> 7) | ((uint16_t)frame[8] << 1) | ((uint16_t)frame[9] << 9)) & 0x07FFu);
	CH[6] = (uint16_t)((((uint16_t)frame[9] >> 2) | ((uint16_t)frame[10] << 6)) & 0x07FFu);
	CH[7] = (uint16_t)((((uint16_t)frame[10] >> 5) | ((uint16_t)frame[11] << 3)) & 0x07FFu);
	CH[8] = (uint16_t)(((uint16_t)frame[12] | ((uint16_t)frame[13] << 8)) & 0x07FFu);
	CH[9] = (uint16_t)((((uint16_t)frame[13] >> 3) | ((uint16_t)frame[14] << 5)) & 0x07FFu);
	CH[10] = (uint16_t)((((uint16_t)frame[14] >> 6) | ((uint16_t)frame[15] << 2) | ((uint16_t)frame[16] << 10)) & 0x07FFu);
	CH[11] = (uint16_t)((((uint16_t)frame[16] >> 1) | ((uint16_t)frame[17] << 7)) & 0x07FFu);
	CH[12] = (uint16_t)((((uint16_t)frame[17] >> 4) | ((uint16_t)frame[18] << 4)) & 0x07FFu);
	CH[13] = (uint16_t)((((uint16_t)frame[18] >> 7) | ((uint16_t)frame[19] << 1) | ((uint16_t)frame[20] << 9)) & 0x07FFu);
	CH[14] = (uint16_t)((((uint16_t)frame[20] >> 2) | ((uint16_t)frame[21] << 6)) & 0x07FFu);
	CH[15] = (uint16_t)((((uint16_t)frame[21] >> 5) | ((uint16_t)frame[22] << 3)) & 0x07FFu);
	CH[16] = (uint16_t)(frame[23] & 0x01u);
	CH[17] = (uint16_t)((frame[23] >> 1) & 0x01u);

	sbus_failsafe_status = SBUS_SIGNAL_OK;
	if ((frame[23] & 0x04u) != 0u)
		sbus_failsafe_status = SBUS_SIGNAL_LOST;
	if ((frame[23] & 0x08u) != 0u)
		sbus_failsafe_status = SBUS_SIGNAL_FAILSAFE;
	sbus_footer = frame[24];
	sbus_last_tick = HAL_GetTick();
	sbus_valid_frame_count++;
}

static uint8_t SBUS_IsValidFrame(const uint8_t *frame)
{
	uint8_t footer_low_nibble = frame[24] & 0x0Fu;
	return (frame[0] == 0x0Fu) &&
		   ((frame[23] & 0xF0u) == 0u) &&
		   (footer_low_nibble == 0x00u || footer_low_nibble == 0x04u);
}

void SBUS_Process(const uint8_t *data, uint16_t size)
{
	static uint8_t frame[SBUS_FRAME_LENGTH];
	static uint8_t frame_index = 0;

	if (data == NULL)
		return;

	for (uint16_t data_index = 0; data_index < size; data_index++)
	{
		if (frame_index == 0u && data[data_index] != 0x0Fu)
			continue;

		frame[frame_index++] = data[data_index];
		if (frame_index < SBUS_FRAME_LENGTH)
			continue;

		if (SBUS_IsValidFrame(frame))
		{
			SBUS_DecodeFrame(frame);
			frame_index = 0;
		}
		else
		{
			uint8_t next_header = 1;
			sbus_invalid_frame_count++;
			while (next_header < SBUS_FRAME_LENGTH && frame[next_header] != 0x0Fu)
				next_header++;
			if (next_header < SBUS_FRAME_LENGTH)
			{
				frame_index = (uint8_t)(SBUS_FRAME_LENGTH - next_header);
				memmove(frame, &frame[next_header], frame_index);
			}
			else
			{
				frame_index = 0;
			}
		}
	}
}

void Uart_Init(void)
{
	// 手柄串口
	sbus_rx_start_status = HAL_UARTEx_ReceiveToIdle_DMA(&HANDLE_SERIAL, handle_rev_buff, HANDLE_LENGTH);
	if (sbus_rx_start_status == HAL_OK)
	{
		__HAL_DMA_DISABLE_IT(&HANDLE_SERIAL_RX, DMA_IT_HT);
	}

//	// 陀螺仪串口
//	HAL_UARTEx_ReceiveToIdle_DMA(&GYRO_SERIAL, gyro_rev_buff, GYRO_LENGTH); // 使能接收中断
//	__HAL_DMA_DISABLE_IT(&GYRO_SERIAL_RX, DMA_IT_HT);												// 关闭过半中断
//	HAL_UART_Transmit_DMA(&GYRO_SERIAL, gyro_rev_buff, 6);

//	// 小电脑串口
//	HAL_UARTEx_ReceiveToIdle_DMA(&RADAR_SERIAL, radar_rev_buff, RADAR_LENGTH); // 使能接收中断
//	__HAL_DMA_DISABLE_IT(&RADAR_SERIAL_RX, DMA_IT_HT);												 // 关闭过半中断
//	HAL_UART_Transmit_DMA(&RADAR_SERIAL, nuc_cmd, 4);

//	// DT35串口
//	HAL_UARTEx_ReceiveToIdle_DMA(&DT35_SERIAL, DT35_rev_buff, DT35_LENGTH); // 使能接收中断
//	__HAL_DMA_DISABLE_IT(&DT35_SERIAL_RX, DMA_IT_HT);												// 关闭过半中断

//	//气泵串口
//	HAL_UARTEx_ReceiveToIdle_DMA(&AIR_PUMP_SERIAL, air_pump_rev_buff, AIR_PUMP_LENGTH); // 使能接收中断
//	__HAL_DMA_DISABLE_IT(&AIRPUMP_SERIAL_RX, DMA_IT_HT);												// 关闭过半中断


//	// 激光测距L串口
//	HAL_UARTEx_ReceiveToIdle_DMA(&LASER_L_SERIAL, laser_l_rev_buff, LASER_LENGTH); // 使能接收中断
//	__HAL_DMA_DISABLE_IT(&LASER_L_SERIAL_RX, DMA_IT_HT);												 // 关闭过半中断

//	// 激光测距R串口
//	HAL_UARTEx_ReceiveToIdle_DMA(&LASER_R_SERIAL, laser_r_rev_buff, LASER_LENGTH); // 使能接收中断
//	__HAL_DMA_DISABLE_IT(&LASER_R_SERIAL_RX, DMA_IT_HT);												 // 关闭过半中断
}

	
/**
 * @brief:DMA中断回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//	// 陀螺仪串口
//	if (huart == &GYRO_SERIAL)
//	{
//		if (Gyro_DataHandle != NULL)
//		{
//			xSemaphoreGiveFromISR(Gyro_DataHandle, NULL);
//		}
//		MINS500_Data_Process(gyro_rev_buff, &MINS500_data); // 全局变量
//		HAL_UARTEx_ReceiveToIdle_DMA(&GYRO_SERIAL, gyro_rev_buff, GYRO_LENGTH); // 使能接收中断
//		__HAL_DMA_DISABLE_IT(&GYRO_SERIAL_RX, DMA_IT_HT);												// 关闭过半中断
//	}

	// 手柄串口
	if (huart == &HANDLE_SERIAL)
	{
		sbus_rx_event_count++;
		sbus_last_rx_size = Size;
		SBUS_Process(handle_rev_buff, Size);
		sbus_rx_start_status = HAL_UARTEx_ReceiveToIdle_DMA(&HANDLE_SERIAL, handle_rev_buff, HANDLE_LENGTH);
		if (sbus_rx_start_status == HAL_OK)
		{
			__HAL_DMA_DISABLE_IT(&HANDLE_SERIAL_RX, DMA_IT_HT);
		}
	}

//	// 小电脑串口
//	if (huart == &RADAR_SERIAL)
//	{
//		if (Radar_DataHandle != NULL)
//		{
//			xSemaphoreGiveFromISR(Radar_DataHandle, NULL);
//		}
//		Radar_Data_Process(&radar, radar_rev_buff);
//		HAL_UARTEx_ReceiveToIdle_DMA(&RADAR_SERIAL, radar_rev_buff, RADAR_LENGTH); // 使能接收中断
//		__HAL_DMA_DISABLE_IT(&RADAR_SERIAL_RX, DMA_IT_HT);												 // 关闭过半中断
//	}

//	// DT35串口
//	if (huart == &DT35_SERIAL)
//	{
//		if (DT35_DataHandle != NULL)
//		{
//			xSemaphoreGiveFromISR(DT35_DataHandle, NULL);
//		}
//		DT35_Data_Process(&DT35, DT35_rev_buff);
//		HAL_UARTEx_ReceiveToIdle_DMA(&DT35_SERIAL, DT35_rev_buff, DT35_LENGTH); // 使能接收中断
//		__HAL_DMA_DISABLE_IT(&DT35_SERIAL_RX, DMA_IT_HT);												// 关闭过半中断
//	}

//	
//	//气泵串口
//	if(huart == &AIR_PUMP_SERIAL)
//	{
//		if(Air_DataHandle != NULL)
//		{
//			xSemaphoreGiveFromISR(Air_DataHandle, NULL);
//		}
//		Air_pump_DataProcess(air_pum_state, air_pump_rev_buff);
//		HAL_UARTEx_ReceiveToIdle_DMA(&AIR_PUMP_SERIAL, air_pump_rev_buff, AIR_PUMP_LENGTH); // 使能接收中断
//		__HAL_DMA_DISABLE_IT(&AIRPUMP_SERIAL_RX, DMA_IT_HT);												// 关闭过半中断
//	}


//	// 激光测距L串口
//	if (huart == &LASER_L_SERIAL)
//	{
//		if(Laser_L_DataHandle != NULL)
//		{
//			xSemaphoreGiveFromISR(Laser_L_DataHandle, NULL);
//		}
//		STP23L_Distance_Process(laser_l_rev_buff, &stp23l_distance_left);
//		HAL_UARTEx_ReceiveToIdle_DMA(&LASER_L_SERIAL, laser_l_rev_buff, LASER_LENGTH); // 使能接收中断
//		__HAL_DMA_DISABLE_IT(&LASER_L_SERIAL_RX, DMA_IT_HT);							//关闭过半中断	
//	}

//	// 激光测距R串口
//	if (huart == &LASER_R_SERIAL)
//	{
//		if(Laser_R_DataHandle != NULL)
//		{
//			xSemaphoreGiveFromISR(Laser_R_DataHandle, NULL);
//		}
//		STP23L_Distance_Process(laser_r_rev_buff, &stp23l_distance_right);
//		HAL_UARTEx_ReceiveToIdle_DMA(&LASER_R_SERIAL, laser_r_rev_buff, LASER_LENGTH); // 使能接收中断
//		__HAL_DMA_DISABLE_IT(&LASER_R_SERIAL_RX, DMA_IT_HT);							//关闭过半中断	
//	}
}


////串口发送完成触发中断
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &RADAR_SERIAL)
//	{
//		is_send = 0;
//	}
//}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &HANDLE_SERIAL)
	{
		sbus_error_count++;
		sbus_last_error = huart->ErrorCode;
		sbus_rx_start_status = HAL_UARTEx_ReceiveToIdle_DMA(&HANDLE_SERIAL, handle_rev_buff, HANDLE_LENGTH);
		if (sbus_rx_start_status == HAL_OK)
		{
			__HAL_DMA_DISABLE_IT(&HANDLE_SERIAL_RX, DMA_IT_HT);
		}
	}
}

void App_Uart(void *argument)
{
//	static Handle_Data s_handle_data = {0}; // 局部变量
//	static uint32_t s_last_handletick = 0;
//	static uint32_t s_last_gyrotick = 0;
//	static uint32_t s_last_radartick = 0;
//	static uint32_t s_last_dt35tick = 0;
//	static uint32_t s_air_pump_tick = 0;
//	static uint32_t s_last_laser_l_tick = 0;
//	static uint32_t s_last_laser_r_tick = 0;


//	static uint8_t current_ccr = 0;			//块的位置校验值

//	static uint8_t irr_send = 0;		//红外通信

//	for (;;)
//	{
//		/* ====================================== 手柄 ====================================== */
//		if (xSemaphoreTake(Joy_DataHandle, 1) == pdTRUE)
//		{
//			taskENTER_CRITICAL();
//			Data_Process(handle_rev_buff, &s_handle_data);

//			//如果两次收到的数据包不同再触发解包，不要重复解包
//			if(memcmp(s_handle_data.KFS, last_send_KFS, 12) != 0)
//			{
//				Qt_Reset();
//				QT_Data_Process(&qt, s_handle_data.KFS);
//			}
//			taskEXIT_CRITICAL();
//			s_last_handletick = xTaskGetTickCount();
//		}

//		// 超时处理
//		if (xTaskGetTickCount() - s_last_handletick > 200)
//		{
//			MX_UART9_Init();
//			HAL_UARTEx_ReceiveToIdle_DMA(&HANDLE_SERIAL, handle_rev_buff, HANDLE_LENGTH);
//			__HAL_DMA_DISABLE_IT(&HANDLE_SERIAL_RX, DMA_IT_HT);
//		}

//		/* ====================================== 陀螺仪 ====================================== */
//		if (xSemaphoreTake(Gyro_DataHandle, 1) == pdTRUE)
//		{
//			taskENTER_CRITICAL();
//			MINS500_Data_Process(gyro_rev_buff, &MINS500_data); // 全局变量
//			taskEXIT_CRITICAL();
//			s_last_gyrotick = xTaskGetTickCount();
//		}

//		// 超时处理
//		if (xTaskGetTickCount() - s_last_gyrotick > 50)
//		{
//			MX_USART6_UART_Init();
//			HAL_UARTEx_ReceiveToIdle_DMA(&GYRO_SERIAL, gyro_rev_buff, GYRO_LENGTH); // 使能接收中断
//			__HAL_DMA_DISABLE_IT(&GYRO_SERIAL_RX, DMA_IT_HT);												// 关闭过半中断
//		}

//		/* ====================================== 小电脑 ====================================== */
//		if (xSemaphoreTake(Radar_DataHandle, 1) == pdTRUE)
//		{
//			taskENTER_CRITICAL();
//			Radar_Data_Process(&radar, radar_rev_buff);
//			QT_target_process(&qt, QT_State);
//			taskEXIT_CRITICAL();
//			s_last_radartick = xTaskGetTickCount();
//		}

//		// 超时处理
//		if (xTaskGetTickCount() - s_last_radartick > 200)
//		{
//			radar.data_flag = 1;
//			MX_UART5_Init();
//			HAL_UARTEx_ReceiveToIdle_DMA(&RADAR_SERIAL, radar_rev_buff, RADAR_LENGTH); // 使能接收中断
//			__HAL_DMA_DISABLE_IT(&RADAR_SERIAL_RX, DMA_IT_HT);												 // 关闭过半中断
//		}
//		else
//		{
//			radar.data_flag = 0;
//		}

//		/* ====================================== DT35 ====================================== */
//		if (xSemaphoreTake(DT35_DataHandle, 1) == pdTRUE)
//		{
//			taskENTER_CRITICAL();
//			DT35_Data_Process(&DT35, DT35_rev_buff);
//			taskEXIT_CRITICAL();
//			s_last_dt35tick = xTaskGetTickCount();
//		}

//		// 超时处理
//		if (xTaskGetTickCount() - s_last_dt35tick > 200)
//		{
//			MX_UART7_Init();
//			HAL_UARTEx_ReceiveToIdle_DMA(&DT35_SERIAL, DT35_rev_buff, DT35_LENGTH); // 使能接收中断
//			__HAL_DMA_DISABLE_IT(&DT35_SERIAL_RX, DMA_IT_HT);												// 关闭过半中断
//		}


//		/* ====================================== 气泵 ====================================== */
//		if (xSemaphoreTake(Air_DataHandle, 1) == pdTRUE)
//		{
//			taskENTER_CRITICAL();

//			taskEXIT_CRITICAL();
//			s_air_pump_tick = xTaskGetTickCount();
//		}

//		// 超时处理
//		if (xTaskGetTickCount() - s_air_pump_tick > 200)
//		{
//			MX_UART4_Init();
//			HAL_UARTEx_ReceiveToIdle_DMA(&AIR_PUMP_SERIAL, air_pump_rev_buff, AIR_PUMP_LENGTH); // 使能接收中断
//			__HAL_DMA_DISABLE_IT(&AIRPUMP_SERIAL_RX, DMA_IT_HT);												// 关闭过半中断
//		}

//		/* ====================================== 激光测距 ====================================== */
//		if(xSemaphoreTake(Laser_L_DataHandle, 1) == pdTRUE)
//		{
//			taskENTER_CRITICAL();
//			STP23L_Distance_Process(laser_l_rev_buff, &stp23l_distance_left);
//			taskEXIT_CRITICAL();
//			s_last_laser_l_tick = xTaskGetTickCount();
//		}

//		if(xSemaphoreTake(Laser_R_DataHandle, 1) == pdTRUE)
//		{
//			taskENTER_CRITICAL();
//			STP23L_Distance_Process(laser_r_rev_buff, &stp23l_distance_right);
//			taskEXIT_CRITICAL();
//			s_last_laser_r_tick = xTaskGetTickCount();
//		}

//		//超时处理
//		if(xTaskGetTickCount() - s_last_laser_l_tick > 200)
//		{
//			MX_USART1_UART_Init();
//			HAL_UARTEx_ReceiveToIdle_DMA(&LASER_L_SERIAL, laser_l_rev_buff, LASER_LENGTH); // 使能接收中断
//			__HAL_DMA_DISABLE_IT(&LASER_L_SERIAL_RX, DMA_IT_HT);							//关闭过半中断	
//		}

//		if(xTaskGetTickCount() - s_last_laser_r_tick > 200)
//		{
//			MX_USART2_UART_Init();
//			HAL_UARTEx_ReceiveToIdle_DMA(&LASER_R_SERIAL, laser_r_rev_buff, LASER_LENGTH); // 使能接收中断
//			__HAL_DMA_DISABLE_IT(&LASER_R_SERIAL_RX, DMA_IT_HT);							//关闭过半中断	
//		}


//		/* ====================================== 给小电脑发送块的位置 ====================================== */
//		current_ccr = Ccr_Check(s_handle_data.KFS, 0, 12) & 0x0F;
//		// 给小电脑发送KFS位置(在两次收到数据包不同的时候再发送)
//		if (current_ccr == 0x0E && is_send == 0 && memcmp(s_handle_data.KFS, last_send_KFS, 12) != 0)
//		{
//			nuc_kfs[0] = 0xEF;
//			for (uint8_t i = 0; i < 12; i++)
//			{
//				nuc_kfs[i + 1] = s_handle_data.KFS[i];
//			}
//			nuc_kfs[13] = Ccr_Check(nuc_kfs, 1, 12);
//			nuc_kfs[14] = 0xFE;
//			HAL_UART_Transmit_DMA(&RADAR_SERIAL, nuc_kfs, 15);
//			// HAL_UART_Transmit_DMA(&RADAR_SERIAL, nuc_cmd_start, 4);
//			is_send = 1;
//			radar.nuc_flag = 1;
//			memcpy(last_send_KFS, s_handle_data.KFS, 12);
//		}

//		if(memcmp(s_handle_data.KFS, last_send_KFS, 12) == 0)
//		{
//			radar.nuc_flag = 0;
//			if(is_send == 0)
//			{
//				//在发送完成之后再清空这个数组
//				memset(nuc_kfs, 0, sizeof(nuc_kfs));
//			}
//		}


//		if(is_send == 1)
//		{
//			inf_cmd[0] = 0xAA;
//			inf_cmd[1] = 0x01;
//			inf_cmd[2] = 0x02;
//			inf_cmd[3] = 0x03;
//			inf_cmd[4] = Ccr_Check(inf_cmd, 1, 3);
//			inf_cmd[5] = 0xBB;
//			HAL_UART_Transmit_DMA(&INFRARED_SERIAL, inf_cmd, 6);
//		}
//		

		/* ====================================== 存储按键 ====================================== */
//		for (int i = 0; i < 8; i++)
//		{
//			key_left[i].key = s_handle_data.key_data.key_left[i].key;
//			key_right[i].key = s_handle_data.key_data.key_right[i].key;
//			key_middle[i].key = s_handle_data.key_data.key_middle[i].key;
//		}

//		xQueueSend(Handle_Data_ToChassisHandle, &s_handle_data, 1); // 手柄数据发送给底盘
//		xQueueSend(Handle_Data_ToUpperHandle, &s_handle_data, 1);		// 手柄数据发送给上层

//		osDelay(1);
//	}
}

//static uint8_t Ccr_Check(uint8_t *message, uint8_t start_index, uint8_t len)
//{
//	uint8_t ccr = 0;
//	for (int i = start_index; i < start_index + len; i++)
//	{
//		ccr += message[i];
//	}
//	return ccr;
//}
