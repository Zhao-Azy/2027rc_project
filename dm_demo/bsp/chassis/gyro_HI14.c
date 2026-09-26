#include "gyro_HI14.h"
#include <string.h>
#include <math.h>

/* ================= HI14 IMU 驱动实现 =================
 * 航向处理：
 *   1) 传感器给"绝对航向"，驱动只累计增量 delta（展开到 ±180）到 real_yaw；
 *   2) 首次收到数据时把 real_yaw 初始化为 HI14_YAW_INIT_DEG / HI14_YAW_SCALE，
 *      使 real_yaw_1 上电正好从 180° 开始（与底盘 target_yaw 的初值一致）；
 *   3) real_yaw_1 = real_yaw × HI14_YAW_SCALE（刻度修正）。
 * author: 
 * ==================================================== */

#define HI14_YAW_INIT_DEG   180.0f     // 上电初始航向（度）
#define HI14_YAW_SCALE      1.0f       // 刻度修正：原 MINS500 用 0.97297f；HI14 由四元数解算，取 1.0
#define HI14_RAD2DEG        57.29578f

HI14_Data hi14 = {0};

static uint8_t s_quat_ok = 0;   // 是否已收到过四元数（航向主源）

static int16_t HI14_ReadI16(const uint8_t *p)   // 小端
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t HI14_ReadI32(const uint8_t *p)   // 小端
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

void HI14_Init(void)
{
    s_quat_ok = 0;
    hi14.frame_count = 0;
    hi14.rx_any = 0;
    /* 总线本身的启动由 app 层 FDCAN_Init(&hfdcan3) 负责（与原驱动的分工一致） */
}

/* 航向累计：与原 MINS500_Data_Process 相同的写法 */
static void HI14_UpdateYaw(Gyro *gyro, float sensor_yaw)
{
    if (gyro->first_flag == 0)
    {
        gyro->last_yaw = sensor_yaw;                       // 记录第一次上电的偏航角
        gyro->real_yaw = HI14_YAW_INIT_DEG / HI14_YAW_SCALE;
        gyro->reset_yaw_cnt++;
        gyro->first_flag = 1;
    }

    gyro->yaw = sensor_yaw;

    gyro->delta_yaw = gyro->yaw - gyro->last_yaw;
    if (gyro->delta_yaw > 180.0f)
    {
        gyro->delta_yaw -= 360.0f;
    }
    else if (gyro->delta_yaw < -180.0f)
    {
        gyro->delta_yaw += 360.0f;
    }

    gyro->real_yaw += gyro->delta_yaw;
    gyro->last_yaw  = gyro->yaw;

    gyro->real_yaw_1 = gyro->real_yaw * HI14_YAW_SCALE;
}

/**
 * @brief 处理一帧 HI14 报文（在 FDCAN3 的接收回调里调用）
 * @param hdr  接收头
 * @param data 数据域
 * @param gyro 目标陀螺仪结构（通常是 &MINS500_data）
 */
void HI14_CanRxProcess(FDCAN_RxHeaderTypeDef *hdr, uint8_t *data, Gyro *gyro)
{
    uint8_t have_yaw = 0;

    hi14.rx_any++;
    hi14.last_id     = hdr->Identifier;
    hi14.last_idtype = (hdr->IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
    hi14.last_dlc    = (uint8_t)hdr->DataLength;
    hi14.stamp_ms    = HAL_GetTick();
    memcpy(hi14.last_data, data, 8);

    if (hdr->IdType != FDCAN_EXTENDED_ID)
    {
        /* ---------- CANopen（11 位标准帧，节点默认 8）---------- */
        switch (hdr->Identifier & 0x780u)
        {
        case 0x180u:   /* 加速度 X/Y/Z，i16 × 0.001 G */
            hi14.acc[0] = (float)HI14_ReadI16(&data[0]) * 0.001f;
            hi14.acc[1] = (float)HI14_ReadI16(&data[2]) * 0.001f;
            hi14.acc[2] = (float)HI14_ReadI16(&data[4]) * 0.001f;
            hi14.frame_count++;
            break;

        case 0x280u:   /* 角速度 X/Y/Z，i16 × 0.1 deg/s */
            hi14.gyro[0] = (float)HI14_ReadI16(&data[0]) * 0.1f;
            hi14.gyro[1] = (float)HI14_ReadI16(&data[2]) * 0.1f;
            hi14.gyro[2] = (float)HI14_ReadI16(&data[4]) * 0.1f;
            hi14.frame_count++;
            break;

        case 0x380u:   /* 欧拉角 横滚/俯仰/航向，i16 × 0.01 deg */
            hi14.roll2  = (float)HI14_ReadI16(&data[0]) * 0.01f;
            hi14.pitch2 = (float)HI14_ReadI16(&data[2]) * 0.01f;
            hi14.yaw2   = -(float)HI14_ReadI16(&data[4]) * 0.01f;   /* 与四元数同一符号约定 */
            if (hi14.yaw2 < hi14.yaw2_min) { hi14.yaw2_min = hi14.yaw2; }
            if (hi14.yaw2 > hi14.yaw2_max) { hi14.yaw2_max = hi14.yaw2; }
            if (s_quat_ok == 0u)   /* 还没有四元数时，先用这一路当航向源 */
            {
                hi14.yaw_pm180 = hi14.yaw2;
                have_yaw = 1;
            }
            hi14.frame_count++;
            break;

        case 0x480u:   /* 四元数 W/X/Y/Z，i16 × 0.0001 → 解算横滚/俯仰/航向 */
        {
            float w = (float)HI14_ReadI16(&data[0]) * 0.0001f;
            float x = (float)HI14_ReadI16(&data[2]) * 0.0001f;
            float y = (float)HI14_ReadI16(&data[4]) * 0.0001f;
            float z = (float)HI14_ReadI16(&data[6]) * 0.0001f;
            float t;

            hi14.quat[0] = w;
            hi14.quat[1] = x;
            hi14.quat[2] = y;
            hi14.quat[3] = z;
            hi14.quat_mag = sqrtf(w * w + x * x + y * y + z * z);

            hi14.roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * HI14_RAD2DEG;
            t = 2.0f * (w * y - z * x);
            if (t > 1.0f)  { t = 1.0f; }
            if (t < -1.0f) { t = -1.0f; }
            hi14.pitch = asinf(t) * HI14_RAD2DEG;

            /* 符号统一：模块四元数解出的航向与"逆时针为正"相反（也与原 MINS500 的约定相反），
             * 且与 gyro[2] 的变化率互为镜像。这里统一翻正，上层（yaw 环 / 坐标变换）无需再管符号。*/
            hi14.yaw_pm180 = -(atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * HI14_RAD2DEG);
            hi14.yaw_360   = (hi14.yaw_pm180 < 0.0f) ? (hi14.yaw_pm180 + 360.0f) : hi14.yaw_pm180;

            if (hi14.yaw_pm180 < hi14.yaw_min) { hi14.yaw_min = hi14.yaw_pm180; }
            if (hi14.yaw_pm180 > hi14.yaw_max) { hi14.yaw_max = hi14.yaw_pm180; }

            s_quat_ok = 1u;
            have_yaw  = 1u;
            hi14.frame_count++;
            break;
        }

        default:
            break;
        }
    }
    else
    {
        /* ---------- J1939（29 位扩展帧，PF 恒为 0xFF）---------- */
        uint32_t pgn = (hdr->Identifier >> 8) & 0x3FFFFu;

        switch (pgn)
        {
        case 0xFF34:   /* 加速度 i16 × 0.00048828 G */
            hi14.acc[0] = (float)HI14_ReadI16(&data[0]) * 0.00048828f;
            hi14.acc[1] = (float)HI14_ReadI16(&data[2]) * 0.00048828f;
            hi14.acc[2] = (float)HI14_ReadI16(&data[4]) * 0.00048828f;
            break;

        case 0xFF37:   /* 角速度 i16 × 0.061035 deg/s */
            hi14.gyro[0] = (float)HI14_ReadI16(&data[0]) * 0.061035f;
            hi14.gyro[1] = (float)HI14_ReadI16(&data[2]) * 0.061035f;
            hi14.gyro[2] = (float)HI14_ReadI16(&data[4]) * 0.061035f;
            break;

        case 0xFF3A:   /* 磁场 i16 × 0.030517 μT */
            hi14.mag[0] = (float)HI14_ReadI16(&data[0]) * 0.030517f;
            hi14.mag[1] = (float)HI14_ReadI16(&data[2]) * 0.030517f;
            hi14.mag[2] = (float)HI14_ReadI16(&data[4]) * 0.030517f;
            break;

        case 0xFF3D:   /* 横滚/俯仰 i32 × 0.001 deg */
            hi14.roll  = (float)HI14_ReadI32(&data[0]) * 0.001f;
            hi14.pitch = (float)HI14_ReadI32(&data[4]) * 0.001f;
            break;

        case 0xFF41:   /* 航向：前 4 字节 0~360 顺时针，后 4 字节 ±180 逆时针为正
                        * 厂商约定"顺时针为正"，与本地"逆时针为正"相反，故取负统一 */
            hi14.yaw_pm180 = -(float)HI14_ReadI32(&data[4]) * 0.001f;
            hi14.yaw_360   = (hi14.yaw_pm180 < 0.0f) ? (hi14.yaw_pm180 + 360.0f) : hi14.yaw_pm180;
            have_yaw = 1u;
            break;

        case 0xFF43:   /* 温度 i16 × 0.01 ℃ */
            hi14.temp = (float)HI14_ReadI16(&data[0]) * 0.01f;
            break;

        default:
            break;
        }

        hi14.frame_count++;
    }

    if (have_yaw != 0u && gyro != NULL)
    {
        HI14_UpdateYaw(gyro, hi14.yaw_pm180);
        gyro->roll  = hi14.roll;
        gyro->pitch = hi14.pitch;
    }
}
