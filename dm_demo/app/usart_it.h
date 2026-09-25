#ifndef USART_IT_H
#define USART_IT_H

#include "main.h"

//手柄      5
#define SBUS_FRAME_LENGTH 25
#define HANDLE_LENGTH (SBUS_FRAME_LENGTH * 2)
#define HANDLE_SERIAL huart5
#define HANDLE_SERIAL_RX hdma_uart5_rx

//陀螺仪    3
#define GYRO_LENGTH 14
#define GYRO_SERIAL huart6
#define GYRO_SERIAL_RX hdma_usart6_rx

//小电脑    6
#define RADAR_LENGTH 30
#define RADAR_SERIAL huart2
#define RADAR_SERIAL_RX hdma_uart2_rx

//DT35  5
#define DT35_LENGTH 19
#define DT35_SERIAL huart7
#define DT35_SERIAL_RX hdma_uart7_rx

//激光测距L   1
#define LASER_LENGTH 195
#define LASER_L_SERIAL huart1
#define LASER_L_SERIAL_RX hdma_usart1_rx

//激光测距R    4
#define LASER_R_SERIAL huart2
#define LASER_R_SERIAL_RX hdma_usart2_rx

//气泵板   7
#define AIR_PUMP_LENGTH 7
#define AIR_PUMP_SERIAL huart4
#define AIRPUMP_SERIAL_RX hdma_uart4_rx

//一键启动 9
#define INFRARED_LENGTH 3
#define INFRARED_SERIAL huart10
#define INFRARED_SERIAL_RX hdma_uart5_rx
/*struct & enum & union-----------------------------------------------------*/

/*extern -------------------------------------------------------------------*/
//extern UART_HandleTypeDef huart1;
//extern DMA_HandleTypeDef hdma_usart1_rx;;        //激光测距L串口
//extern UART_HandleTypeDef huart2;;
//extern DMA_HandleTypeDef hdma_usart2_rx;         //激光测距R串口
//extern UART_HandleTypeDef huart6;
//extern DMA_HandleTypeDef hdma_usart6_rx;        //陀螺仪串口
//extern UART_HandleTypeDef huart9;               
//extern DMA_HandleTypeDef hdma_uart9_rx;         //手柄串口
//extern UART_HandleTypeDef huart4;
//extern DMA_HandleTypeDef hdma_uart4_rx;         //气泵板串口
//extern UART_HandleTypeDef huart7;
//extern DMA_HandleTypeDef hdma_uart7_rx;          //DT35串口
//extern UART_HandleTypeDef huart5;
//extern DMA_HandleTypeDef hdma_uart5_rx;;        //小电脑串口
//extern UART_HandleTypeDef huart10;
//extern DMA_HandleTypeDef hdma_usart10_rx;        //一键启动串口
//extern UART_HandleTypeDef huart8;               //Lora

extern uint8_t nuc_cmd1[];
extern uint8_t nuc_kfs[15];
extern uint8_t last_send_KFS[12];
extern uint8_t infrared_cmd1[];
extern uint8_t infrared_cmd2[];
extern uint8_t infrared_cmd3[];


/*fuction-------------------------------------------------------------------*/
void Uart_Init(void);

extern volatile uint16_t CH[18];
extern volatile uint8_t sbus_failsafe_status;
extern volatile uint32_t sbus_last_tick;
extern volatile uint32_t sbus_rx_start_status;
extern volatile uint32_t sbus_rx_event_count;
extern volatile uint32_t sbus_valid_frame_count;
extern volatile uint32_t sbus_invalid_frame_count;
extern volatile uint32_t sbus_error_count;
extern volatile uint32_t sbus_last_error;
extern volatile uint16_t sbus_last_rx_size;
extern volatile uint8_t sbus_footer;

void SBUS_Process(const uint8_t *data, uint16_t size);

#endif
