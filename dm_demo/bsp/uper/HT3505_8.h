#ifndef HT3505_8_H
#define HT3505_8_H


#include "main.h"
#include "drv_can.h"

typedef struct 
{
    uint8_t can_id;
    FDCAN_HandleTypeDef *hfdcan;

    float position;

}HT3505_HandleTypeDef;


void HT3505_Init(HT3505_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id);
void HT3505_SpeedMode(HT3505_HandleTypeDef *motor, float target_speed);
void HT3505_PositionMode(HT3505_HandleTypeDef *motor, float target_pos, float target_speed);
void HT3505_ReadPosition(HT3505_HandleTypeDef *motor);

#ifdef __FDCAN_H__
    void HT3505Get_Position(HT3505_HandleTypeDef *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);
#elif defined __CAN_H__
    void HT3505Get_Position(HT3505_HandleTypeDef *motor, CAN_RxHeaderTypeDef *rx_header, uint8_t *data);
#endif


#endif
