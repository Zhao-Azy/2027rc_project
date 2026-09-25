#include "DJI_C620.h"

// 速度环 PID 参数（需根据实际机械与负载调参）
// v6: Ki 0.03 → 0.02——v5 中超调主体是积分在 1.5s 平台期攒出的过冲；收尾轮
#define C620_SPEED_KP   5.0f
#define C620_SPEED_KI   0.02f
#define C620_SPEED_KD   20.0f

// 初始化
void DJI_C620_Init(DJI_C620_HandleTypeDef *h, FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    h->hcan = hcan;
    h->id   = id;

    h->angle  = 0;
    h->speed  = 0;
    h->torque = 0;
    h->temp   = 0;

    h->target_speed   = 0.0f;
    h->target_current = 0;

    // 速度环：目标转速 rpm -> 输出电流（限幅 ±16384）
    pid_param_init(&h->speed_pid,
                   PID_Position,
                   C620_CURRENT_MAX,   // 最大输出（电流）
                   3000.0f,            // 积分限幅
                   100.0f,             // 积分分离阈值（rpm；误差大于它时积分清零）
                   0.0f,               // 死区
                   10000.0f,           // 最大误差（rpm）
                   C620_SPEED_KP,
                   C620_SPEED_KI,
                   C620_SPEED_KD);
    h->speed_pid.d_on_measure = 1;     // C620 速度环：微分用测量值差分（无阶跃尖峰）
}

// 解析标准反馈帧（大端：角度/转速/转矩各 2 字节）
void DJI_C620_GetFeedback(DJI_C620_HandleTypeDef *h, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    if (rx_header->IdType != FDCAN_STANDARD_ID)
        return;
    if (rx_header->Identifier != (0x200u + h->id))
        return;

    h->angle  = (uint16_t)((data[0] << 8) | data[1]);   // 机械角度
    h->speed  = (int16_t)((data[2] << 8) | data[3]);    // 转速 rpm
    h->torque = (int16_t)((data[4] << 8) | data[5]);    // 转矩电流
    h->temp   = data[6];                                // 温度
}

// 速度环：目标转速 -> 目标电流
void DJI_C620_SpeedControl(DJI_C620_HandleTypeDef *h)
{
    float current = pid_calc(&h->speed_pid, h->target_speed, (float)h->speed);

    if (current >  C620_CURRENT_MAX) current =  C620_CURRENT_MAX;
    if (current < -C620_CURRENT_MAX) current = -C620_CURRENT_MAX;

    h->target_current = (int16_t)current;
}

// 将 4 个电机电流打包到 0x200 控制帧并发送
void DJI_C620_SendCurrent(DJI_C620_HandleTypeDef **motors)
{
    uint8_t data[8] = {0};

    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t idx = (motors[i]->id - 1) * 2;                            // id 1 => byte 0-1
        data[idx]     = (uint8_t)((motors[i]->target_current >> 8) & 0xFF);   // 高字节在前
        data[idx + 1] = (uint8_t)(motors[i]->target_current & 0xFF);          // 低字节
    }

    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier          = 0x200;                 // 控制帧（1~4 号电机）
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = 8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(motors[0]->hcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(motors[0]->hcan, &TxHeader, data);
    }
}
