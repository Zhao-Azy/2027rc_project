#ifndef GYRO_MINS500_H
#define GYRO_MINS500_H

#include "main.h"


#define STDID 0x77      //包头

//#define GYRO_LENGTH 14
//#define GYRO_SERIAL huart3
//#define GYRO_SERIAL_RX hdma_usart3_rx

typedef struct
{
    float roll;
    float pitch;

    uint8_t first_flag;

    float yaw;
    float first_yaw;
    float delta_yaw;
    float last_yaw;
    float real_yaw;
	float real_yaw_1;

    uint8_t reset_yaw_cnt;          //重置yaw角次数
}Gyro;


void MINS500_Init(void);
void MINS500_Data_Process(uint8_t *data, Gyro *gyro);

#endif 

