#ifndef DJI_C620_H
#define DJI_C620_H

#include "main.h"
#include "drv_can.h"
#include "pid.h"

// =====================================================================
// 大疆 C620 电调（配 M3508 电机）标准 CAN 协议（直连电调，无中继小板）
// 反馈帧 ID = 0x200 + 电机编号：1~4 号 => 0x201 ~ 0x204
// 控制帧 ID = 0x200，1~4 号电机打包成 8 字节，每电机 2 字节 int16 电流
// =====================================================================

#define C620_CURRENT_MAX  16384.0f   // 电流限幅（对应约 ±20A）

typedef struct
{
    FDCAN_HandleTypeDef *hcan;
    uint8_t id;              // 电机编号 1~4（反馈 ID = 0x200 + id）

    // 反馈量
    uint16_t angle;          // 转子机械角度 0~8191
    int16_t  speed;          // 转子转速 rpm（有符号）
    int16_t  torque;         // 实际转矩电流 -16384~16384
    uint8_t  temp;           // 电机温度 ℃

    // 控制量
    float   target_speed;    // 目标转速 rpm
    int16_t target_current;  // 速度环输出（目标电流 -16384~16384）
    PID_T   speed_pid;       // 速度环 PID
} DJI_C620_HandleTypeDef;

void DJI_C620_Init(DJI_C620_HandleTypeDef *h, FDCAN_HandleTypeDef *hcan, uint8_t id);
void DJI_C620_GetFeedback(DJI_C620_HandleTypeDef *h, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);
void DJI_C620_SpeedControl(DJI_C620_HandleTypeDef *h);
void DJI_C620_SendCurrent(DJI_C620_HandleTypeDef **motors);   // motors 为 4 个电机指针数组

#endif
