#ifndef RS05_H
#define RS05_H


#include "main.h"
#include "drv_can.h"


typedef enum
{
    eRS05_MIT_MODE,
    eRS05_SPEED_MODE = 2,
    eRS05_CURRENT_MODE = 3,
    eRS05_POSITION_MODE = 5,
}RS05_Type;


typedef struct 
{
    uint16_t can_id;
    FDCAN_HandleTypeDef *hfdcan;

    uint16_t original_pos;
    uint16_t original_speed;
    uint16_t original_torque;


    float last_pos;
    float current_pos;
    float delta_pos;
    float real_pos;

    float pos;
    float speed;
    float torque;



}RS05_HandleTypeDef;

void RS05_Init(RS05_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, RS05_Type type, uint16_t can_id);
void RS05_MITMode(RS05_HandleTypeDef *motor, float target_pos, float target_speed, float Kp, float Kd, float torque);
void RS05_CurrentMode(RS05_HandleTypeDef *motor, float target_current);
void RS05_SpeedMode(RS05_HandleTypeDef *motor, float target_speed);
void RS05_PositionMode(RS05_HandleTypeDef *motor, float target_pos, float target_speed);

#ifdef __FDCAN_H__
	void RS05_Get_MotorData(RS05_HandleTypeDef *motor, FDCAN_RxHeaderTypeDef *p_msg, uint8_t *data_arry);
#elif defined __CAN_H__
	void RS05_Get_MotorData(CAN_RxHeaderTypeDef *p_msg, RS05_HandleTypeDef *motor, uint8_t *data_arry);
#endif

#endif
