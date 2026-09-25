#ifndef DJI_TRANSFORM_MASTER_H
#define DJI_TRANSFORM_MASTER_H

#include "main.h"
#include "drv_can.h"


typedef union 
{
    uint16_t u16;
    uint8_t u8[2];
}U16_U8;

typedef union 
{
    float f;
    uint8_t u8[4];
}F_U8;

typedef union 
{
    int16_t i16;
    uint8_t u8[2];
}I16_U8;

typedef enum 
{
    eTimeoutMode,  // timeout模式
    eHomingMode,    // 设置原点
    eCurrentMode,  // 电流模式
    eSpeedMode,    // 速度模式
    ePositionMode, // 位置模式
    eSetOrigin,
}DJI_cmd;


//电机类型
typedef enum
{
    M3508,
    M2006
}Motor_Type;


typedef struct 
{
    Motor_Type motor_type;
    DJI_cmd DJI_cmd;
    FDCAN_HandleTypeDef *hcan;

    uint16_t speed;
    float position;
    uint8_t id;         //电机id
    uint8_t slave_id;       //小板id
    uint16_t stdid; 
}DJI_HandleTypeDef;




void DJI_Init(DJI_HandleTypeDef *hdji, FDCAN_HandleTypeDef *hcan, Motor_Type motor_type, uint8_t slave_id, uint8_t id);
void DJI_Get_Meature( DJI_HandleTypeDef *hdji, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);

void DJI_CurrentMode(DJI_HandleTypeDef *hdji, uint16_t target_current);
void DJI_SpeedMode(DJI_HandleTypeDef *hdji, float target_speed, uint8_t current_limit);
void DJI_PositionMode(DJI_HandleTypeDef *hdji, float target_position, uint8_t speed_limit, uint8_t current_limit);
void DJI_HomingMode(DJI_HandleTypeDef *hdji, float target_speed);
void DJI_SetOrigin(DJI_HandleTypeDef *hdji);

#endif

