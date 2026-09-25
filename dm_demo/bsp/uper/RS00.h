#ifndef RS00_H
#define RS00_H


#include "main.h"
#include "drv_can.h"


typedef enum
{
    eRS00_MIT_MODE,
    eRS00_SPEED_MODE = 2,
    eRS00_CURRENT_MODE = 3,
    eRS00_POSITION_MODE = 5,
}RS00_Type;


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



}RS00_HandleTypeDef;

void RS00_Init(RS00_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, RS00_Type type, uint16_t can_id);
void RS00_MITMode(RS00_HandleTypeDef *motor, float target_pos, float target_speed, float Kp, float Kd, float torque);
void RS00_CurrentMode(RS00_HandleTypeDef *motor, float target_current);
void RS00_SpeedMode(RS00_HandleTypeDef *motor, float target_speed);
void RS00_PositionMode(RS00_HandleTypeDef *motor, float target_pos, float target_speed);

#ifdef __FDCAN_H__
	void RS00_Get_MotorData(RS00_HandleTypeDef *motor, FDCAN_RxHeaderTypeDef *p_msg, uint8_t *data_arry);
#elif defined __CAN_H__
	void RS00_Get_MotorData(CAN_RxHeaderTypeDef *p_msg, RS00_HandleTypeDef *motor, uint8_t *data_arry);
#endif

#endif
