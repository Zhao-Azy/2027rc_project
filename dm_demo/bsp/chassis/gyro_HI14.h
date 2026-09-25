#ifndef GYRO_HI14_H
#define GYRO_HI14_H

#include "main.h"
#include "gyro_MINS500.h"   // 复用同一个 Gyro 结构，上层代码无需改动

/* ================= HI14 IMU（CANopen / J1939，挂 FDCAN3）=================
 * 数据源：
 *   CANopen（11 位标准帧，节点默认 8）
 *     0x180+ID  TPDO1 加速度 X/Y/Z  i16 × 0.001 G
 *     0x280+ID  TPDO2 角速度 X/Y/Z  i16 × 0.1 deg/s
 *     0x380+ID  TPDO3 欧拉角 R/P/Yaw i16 × 0.01 deg
 *     0x480+ID  TPDO4 四元数 W/X/Y/Z i16 × 0.0001
 *   J1939（29 位扩展帧，PF=0xFF，PGN = (ID>>8) & 0x3FFFF）
 *     0xFF34 加速度 i16 × 0.00048828 G     0xFF37 角速度 i16 × 0.061035 deg/s
 *     0xFF3A 磁场   i16 × 0.030517  μT     0xFF3D 横滚/俯仰 i32 × 0.001 deg
 *     0xFF41 航向（前 4 字节 0~360，后 4 字节 ±180）
 * 航向处理沿用原 MINS500 驱动：只累计传感器绝对航向的增量（展开 ±180），
 * 首次上电用固定初值让 real_yaw_1 从 180° 起算，最后乘刻度系数。
 * ====================================================================== */

typedef struct
{
    float acc[3];         // 三轴加速度 G
    float gyro[3];        // 三轴角速度 deg/s
    float mag[3];         // 三轴磁场 μT（仅 J1939 有）
    float roll;           // 横滚 deg
    float pitch;          // 俯仰 deg
    float yaw_360;        // 航向 0~360°
    float yaw_pm180;      // 航向 ±180°（当前航向源，喂给 Gyro 的那个）
    float temp;           // 温度 ℃（J1939 0xFF43）
    float quat[4];        // 四元数 W/X/Y/Z
    float quat_mag;       // 四元数模长（应≈1.0）
    float roll2;          // 0x388 那一路的前两个值（欧拉角）
    float pitch2;
    float yaw2;
    float yaw_min;        // 诊断：四元数航向的历史最小/最大
    float yaw_max;
    float yaw2_min;       // 诊断：0x388 航向的历史最小/最大
    float yaw2_max;
    uint32_t frame_count; // 已识别报文数
    uint32_t rx_any;      // 收到的任意 CAN 帧数
    uint32_t last_id;     // 最近一帧 CAN ID
    uint32_t stamp_ms;    // 最近一帧时间戳
    uint8_t  last_idtype; // 1=29位扩展 0=11位标准
    uint8_t  last_dlc;
    uint8_t  last_data[8];
} HI14_Data;

extern HI14_Data hi14;

void HI14_Init(void);
void HI14_CanRxProcess(FDCAN_RxHeaderTypeDef *hdr, uint8_t *data, Gyro *gyro);

#endif
