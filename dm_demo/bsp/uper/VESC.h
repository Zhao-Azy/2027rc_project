#ifndef VESC_H
#define VESC_H

#include "main.h"
#include "drv_can.h"


typedef union 
{
	uint16_t u16;
	int16_t i16;
}U16_I16;


typedef union
{
	uint32_t u32;
	int32_t i32;
}U32_I32;




typedef enum
{
	CAN_PACKET_SET_DUTY  = 0,
	CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,           //电流刹车模式
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
    CAN_PACKET_SET_CURRENT_BRAKE_REL = 11       //相对电流刹车模式
} CAN_PACKET_ID;


typedef struct
{
	FDCAN_HandleTypeDef *hcan;


	uint8_t id;
	uint8_t poles;			//极对数
    float speed;
    float current_pos;
	float current;

	uint8_t first_flag;

	float last_pos;
	float real_pos;
	float delta_pos;

	float target_pos;
	float target_speed;

}VESC_HandleTypeDef;



void VESC_Init(VESC_HandleTypeDef *h_VESC, FDCAN_HandleTypeDef *hcan, uint8_t id, uint8_t poles);

void Vesc_Duty_mode(VESC_HandleTypeDef *h_VESC, float duty);
void Vesc_Current_mode(VESC_HandleTypeDef *h_VESC, float current);
void Vesc_Speed_mode(VESC_HandleTypeDef *h_VESC, float rpm);
void Vesc_Position_mode(VESC_HandleTypeDef *h_VESC, float pos);
void Vesc_Current_Break_mode(VESC_HandleTypeDef *h_VESC, float current);
void Vesc_Current_brake_rel_mode(VESC_HandleTypeDef *h_VESC, float current_rel);


#ifdef __FDCAN_H__
	void VESC_Data_Process(VESC_HandleTypeDef *h_VESC, FDCAN_RxHeaderTypeDef *p_msg, uint8_t *data);
#elif defined __CAN_H__
	void VESC_Data_Process(VESC_HandleTypeDef *h_VESC, CAN_RxHeaderTypeDef *p_msg, uint8_t *data);
#endif

static void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
static void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index); 


#endif

