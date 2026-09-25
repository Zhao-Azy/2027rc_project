#ifndef MG5010E_H
#define MG5010E_H


#include "main.h"
#include "drv_can.h"


typedef struct 
{
    uint8_t can_id;
    FDCAN_HandleTypeDef *hfdcan;
    float ratio;        //减速比

    int64_t pos;

}MG5010E_HandleTypeDef;

void MG5010E_Init(MG5010E_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id, float ratio);
void MG5010E_SpeedMode(MG5010E_HandleTypeDef *motor, float target_speed, float current_limit);
void MG5010E_PositionMode(MG5010E_HandleTypeDef *motor, float target_pos);
void MG5010E_SpeedPosMode(MG5010E_HandleTypeDef *motor, uint16_t speed_limit, float target_pos);
void MG5010E_ReadPosition(MG5010E_HandleTypeDef *motor);
void MG5010E_ReadMotorState(MG5010E_HandleTypeDef *motor);

#ifdef __FDCAN_H__
    void MG5010E_GetPosition(MG5010E_HandleTypeDef *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);
#elif defined __CAN_H__
    void MG5010E_GetPosition(MG5010E_HandleTypeDef *motor, CAN_RxHeaderTypeDef *rx_header, uint8_t *data);
#endif

#endif

