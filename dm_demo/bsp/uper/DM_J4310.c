#include "DM_J4310.h"
#include <string.h>

/*==============================================================================
 * 达妙 DM-J4310-2EC 驱动实现
 *-------------------------------------------------------------------------------
 * 报文格式、定点映射、单位口径都写在 DM_J4310.h 顶部的说明里，这里只说实现要点：
 *   1) 所有多字节字段按手册"低字节在前"拆装，不用 #pragma pack 的结构体和位域
 *      （位域排布各家编译器不一样，这里统一移位 + 掩码，一眼能对上手冊表格）；
 *   2) 控制函数内部就把帧发出去了（和 RS00 / MG5010E / DJI_C620 的用法一致），
 *      调用一次 = 发一帧，不需要再手动 Send；Send 只用于"按上一次的目标重发"；
 *   3) 组帧前先裁剪，句柄里留下的指令就是真正发出去的指令，调试时看的值可信；
 *   4) 没使能就不发控制帧（上电零输出），使能/失能帧本身不受此限制；
 *   5) 发送 FIFO 满时直接丢这一帧、不等待，避免把控制周期拖爆；
 *   6) 反馈在 CAN 接收中断里解析：只做移位、乘除和累加，不打印、不申请资源。
 *============================================================================*/

/*------------------------------------------------------------------------------
 * 内部函数声明
 *----------------------------------------------------------------------------*/
static float    DM_J4310_Clamp(float value, float min_value, float max_value);
static uint16_t DM_J4310_FloatToUint(float x, float x_min, float x_max, uint8_t bits);
static float    DM_J4310_UintToFloat(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void     DM_J4310_PutFloatLE(uint8_t *dst, float value);
static uint8_t  DM_J4310_Transmit(DM_J4310_HandleTypeDef *motor,
                                  uint16_t id, const uint8_t *data, uint8_t len);
static uint8_t  DM_J4310_SendCommand(DM_J4310_HandleTypeDef *motor, uint8_t cmd);
static void     DM_J4310_ResetCommand(DM_J4310_HandleTypeDef *motor);

/*==============================================================================
 * 初始化与参数
 *============================================================================*/
/**
 * @brief  初始化句柄：只建立本地状态，不使能、不发帧
 * @note   memset 清零是有意的：上电初始状态必须是"零输出"，不能带着栈上的垃圾值跑
 */
void DM_J4310_Init(DM_J4310_HandleTypeDef *motor,
                   DM_CAN_HandleTypeDef *hcan,
                   uint16_t can_id, uint16_t master_id, float ratio)
{
    if (motor == NULL)
    {
        return;
    }

    memset(motor, 0, sizeof(*motor));

    motor->hcan      = hcan;
    motor->can_id    = can_id;
    motor->master_id = master_id;
    motor->ratio     = (ratio > 0.01f) ? ratio : DM_J4310_DEFAULT_RATIO;

    /* 幅值先按 J4310-2EC 出厂值填；驱动器里改过必须再调 DM_J4310_SetLimit()，
     * 否则命令会被静默等比缩放（见头文件第二节的警告） */
    motor->p_max = DM_J4310_DEFAULT_PMAX;
    motor->v_max = DM_J4310_DEFAULT_VMAX;
    motor->t_max = DM_J4310_DEFAULT_TMAX;

    motor->mode    = (uint8_t)eDM_J4310_MODE_MIT;
    motor->enabled = 0u;                 /* 上电失能，等上层显式 Enable */

    DM_J4310_ResetCommand(motor);
}

/**
 * @brief  改控制幅值：必须和驱动器"参数设置 → 控制幅值"里的 PMAX / VMAX / TMAX 一致
 * @note   传 0 或负数会被当成"没改"，退回该型号的出厂默认值，避免除零和全零幅值
 */
void DM_J4310_SetLimit(DM_J4310_HandleTypeDef *motor,
                       float p_max, float v_max, float t_max)
{
    if (motor == NULL)
    {
        return;
    }

    motor->p_max = (p_max > 0.1f) ? p_max : DM_J4310_DEFAULT_PMAX;
    motor->v_max = (v_max > 0.1f) ? v_max : DM_J4310_DEFAULT_VMAX;
    motor->t_max = (t_max > 0.1f) ? t_max : DM_J4310_DEFAULT_TMAX;
}

/*==============================================================================
 * 使能 / 失能 / 清错 / 存零点（四个都是"立即发一帧"，返回 1 表示已放进 FIFO）
 *============================================================================*/
uint8_t DM_J4310_Enable(DM_J4310_HandleTypeDef *motor)
{
    if (DM_J4310_SendCommand(motor, DM_J4310_CMD_ENABLE) == 0u)
    {
        return 0u;
    }

    /* 使能帧只负责"通电"，不动已经写好的指令：
     *   上电时指令是 0（Init 清过），所以第一次使能输出 0；
     *   想一使能就停在某处，先 PositionMode(当前位置, kp, kd) 再 Enable。 */
    motor->enabled = 1u;

    return 1u;
}

uint8_t DM_J4310_Disable(DM_J4310_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return 0u;
    }

    /* 先清本地使能标记：即使这一帧没发出去（FIFO 满），也绝不再发控制帧 */
    motor->enabled = 0u;
    DM_J4310_ResetCommand(motor);

    return DM_J4310_SendCommand(motor, DM_J4310_CMD_DISABLE);
}

uint8_t DM_J4310_ClearError(DM_J4310_HandleTypeDef *motor)
{
    return DM_J4310_SendCommand(motor, DM_J4310_CMD_CLEAR_ERROR);
}

/**
 * @brief  把当前位置存成零点（写驱动器 Flash，掉电不丢）
 * @note   电机必须静止时调用；存完驱动器反馈的位置会变成 0，
 *         所以置 zero_pending，下一帧反馈重新对齐多圈累加值，避免 real_pos 跳一下
 */
uint8_t DM_J4310_SaveZeroPosition(DM_J4310_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return 0u;
    }

    if (DM_J4310_SendCommand(motor, DM_J4310_CMD_SAVE_ZERO) == 0u)
    {
        return 0u;
    }

    motor->zero_pending = 1u;

    return 1u;
}

/*==============================================================================
 * 控制目标（每个函数 = 写好指令 + 立刻发一帧，返回 1 表示已放进 FIFO）
 *============================================================================*/
/**
 * @brief  MIT 力位混合模式
 * @param  pos    目标位置 rad（关节侧，±PMAX 内）
 * @param  vel    目标速度 rad/s（一般给 0，做速度前馈时才给）
 * @param  kp     位置刚度 N·m/rad（0~500）
 * @param  kd     速度阻尼 N·m·s/rad（0~5，纯位置控制时别给 0）
 * @param  torque 前馈力矩 N·m
 */
uint8_t DM_J4310_MITMode(DM_J4310_HandleTypeDef *motor,
                         float pos, float vel, float kp, float kd, float torque)
{
    if (motor == NULL)
    {
        return 0u;
    }

    motor->mode          = (uint8_t)eDM_J4310_MODE_MIT;
    motor->target_pos    = pos;
    motor->target_vel    = vel;
    motor->kp            = kp;
    motor->kd            = kd;
    motor->target_torque = torque;

    return DM_J4310_Send(motor);
}

/** @brief 位置模式：纯位置 PD（速度目标、前馈力矩都清 0） */
uint8_t DM_J4310_PositionMode(DM_J4310_HandleTypeDef *motor, float pos, float kp, float kd)
{
    return DM_J4310_MITMode(motor, pos, 0.0f, kp, kd, 0.0f);
}

/**
 * @brief 速度模式（走 MIT 帧：Kp = 0，只留速度阻尼）
 * @note  位置项无效，但目标位置仍写成当前角度：万一以后 kp 被改大，
 *        不会因为目标停在 0 而猛地往零点甩
 */
uint8_t DM_J4310_SpeedMode(DM_J4310_HandleTypeDef *motor, float vel, float kd)
{
    if (motor == NULL)
    {
        return 0u;
    }

    motor->mode          = (uint8_t)eDM_J4310_MODE_MIT;
    motor->target_pos    = motor->pos;
    motor->target_vel    = vel;
    motor->kp            = 0.0f;
    motor->kd            = kd;
    motor->target_torque = 0.0f;

    return DM_J4310_Send(motor);
}

/** @brief 力矩模式：只有前馈力矩，Kp / Kd 都为 0 */
uint8_t DM_J4310_TorqueMode(DM_J4310_HandleTypeDef *motor, float torque)
{
    if (motor == NULL)
    {
        return 0u;
    }

    motor->mode          = (uint8_t)eDM_J4310_MODE_MIT;
    motor->target_pos    = motor->pos;
    motor->target_vel    = 0.0f;
    motor->kp            = 0.0f;
    motor->kd            = 0.0f;
    motor->target_torque = torque;

    return DM_J4310_Send(motor);
}

/** @brief 零力矩模式：什么都不出，可以手拖动检查通讯 / 装配是否顺畅 */
uint8_t DM_J4310_ZeroTorqueMode(DM_J4310_HandleTypeDef *motor)
{
    return DM_J4310_TorqueMode(motor, 0.0f);
}

/**
 * @brief  位置速度模式（PV，帧 ID = 0x100 + CAN_ID，两个小端 float）
 * @param  pos 目标位置 rad（float，可超过 ±PMAX 做多圈移动）
 * @param  vel 匀速段速度 rad/s（加/减速度由驱动器参数决定，这里只给上限速度）
 * @note   切这个模式前，驱动器里的"控制模式"必须已经写成速度位置模式，
 *         否则它按 MIT 解释这一帧、ID 对不上，直接不理
 */
uint8_t DM_J4310_PVMode(DM_J4310_HandleTypeDef *motor, float pos, float vel)
{
    if (motor == NULL)
    {
        return 0u;
    }

    motor->mode       = (uint8_t)eDM_J4310_MODE_PV;
    motor->target_pos = pos;
    motor->target_vel = vel;

    return DM_J4310_Send(motor);
}

/** @brief 纯速度模式（帧 ID = 0x200 + CAN_ID，一个小端 float），同样要求驱动器先切成速度模式 */
uint8_t DM_J4310_VelMode(DM_J4310_HandleTypeDef *motor, float vel)
{
    if (motor == NULL)
    {
        return 0u;
    }

    motor->mode       = (uint8_t)eDM_J4310_MODE_VEL;
    motor->target_vel = vel;

    return DM_J4310_Send(motor);
}

/*==============================================================================
 * 发送
 *============================================================================*/
/**
 * @brief  按句柄里当前的 mode 组帧发出去
 * @return 1 = 已放进发送 FIFO；0 = 没发（未使能 / 句柄空 / FIFO 满）
 * @note   控制函数内部已经调过它了，一般不用自己再调；
 *         需要"按上一次的目标重发一帧"（心跳、刚使能完补一帧）时才用
 */
uint8_t DM_J4310_Send(DM_J4310_HandleTypeDef *motor)
{
    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return 0u;
    }

    /* 没使能就一帧都不发：既是"上电零输出"的兜底，也能让"忘了使能"这件事
     * 通过 skip_cnt 一眼看出来（比电机悄悄不动好查） */
    if (motor->enabled == 0u)
    {
        motor->skip_cnt++;
        return 0u;
    }

    switch (motor->mode)
    {
        case eDM_J4310_MODE_PV:
            return DM_J4310_SendPV(motor);

        case eDM_J4310_MODE_VEL:
            return DM_J4310_SendVel(motor);

        case eDM_J4310_MODE_MIT:
        default:
            return DM_J4310_SendMIT(motor);
    }
}

/**
 * @brief  MIT 控制帧（帧 ID = CAN_ID）
 * @note   Position 16 位 / Velocity 12 位 / Kp 12 位 / Kd 12 位 / Torque 12 位
 *         按位拼进 8 字节，拼法见头文件第一节
 */
uint8_t DM_J4310_SendMIT(DM_J4310_HandleTypeDef *motor)
{
    uint8_t  data[DM_J4310_FRAME_LEN];
    uint16_t pos_u;
    uint16_t vel_u;
    uint16_t kp_u;
    uint16_t kd_u;
    uint16_t tau_u;

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return 0u;
    }

    /* 先裁剪再组帧：句柄里留下的就是真正发出去的值，
     * 调试时看到的 target_* / kp / kd 可信，不用再猜"是不是被限幅了" */
    motor->target_pos    = DM_J4310_Clamp(motor->target_pos,    -motor->p_max, motor->p_max);
    motor->target_vel    = DM_J4310_Clamp(motor->target_vel,    -motor->v_max, motor->v_max);
    motor->target_torque = DM_J4310_Clamp(motor->target_torque, -motor->t_max, motor->t_max);
    motor->kp            = DM_J4310_Clamp(motor->kp, 0.0f, DM_J4310_KP_MAX);
    motor->kd            = DM_J4310_Clamp(motor->kd, 0.0f, DM_J4310_KD_MAX);

    pos_u = DM_J4310_FloatToUint(motor->target_pos, -motor->p_max, motor->p_max, 16u);
    vel_u = DM_J4310_FloatToUint(motor->target_vel, -motor->v_max, motor->v_max, 12u);
    kp_u  = DM_J4310_FloatToUint(motor->kp, 0.0f, DM_J4310_KP_MAX, 12u);
    kd_u  = DM_J4310_FloatToUint(motor->kd, 0.0f, DM_J4310_KD_MAX, 12u);
    tau_u = DM_J4310_FloatToUint(motor->target_torque, -motor->t_max, motor->t_max, 12u);

    data[0] = (uint8_t)((pos_u >> 8) & 0xFFu);
    data[1] = (uint8_t)(pos_u & 0xFFu);
    data[2] = (uint8_t)((vel_u >> 4) & 0xFFu);
    data[3] = (uint8_t)((((vel_u & 0x000Fu) << 4) | ((kp_u >> 8) & 0x000Fu)) & 0xFFu);
    data[4] = (uint8_t)(kp_u & 0xFFu);
    data[5] = (uint8_t)((kd_u >> 4) & 0xFFu);
    data[6] = (uint8_t)((((kd_u & 0x000Fu) << 4) | ((tau_u >> 8) & 0x000Fu)) & 0xFFu);
    data[7] = (uint8_t)(tau_u & 0xFFu);

    return DM_J4310_Transmit(motor, motor->can_id, data, (uint8_t)DM_J4310_FRAME_LEN);
}

/** @brief 位置速度模式帧（帧 ID = 0x100 + CAN_ID）：p_des、v_des 各一个 float */
uint8_t DM_J4310_SendPV(DM_J4310_HandleTypeDef *motor)
{
    uint8_t data[DM_J4310_FRAME_LEN];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return 0u;
    }

    /* PV 的位置是 float，不按 PMAX 映射：多圈移动直接给大数就行，
     * 只做一层兜底限幅（防止 NaN / 天文数字），速度仍按 VMAX 收口 */
    motor->target_pos = DM_J4310_Clamp(motor->target_pos, -DM_J4310_PV_POS_LIMIT, DM_J4310_PV_POS_LIMIT);
    motor->target_vel = DM_J4310_Clamp(motor->target_vel, -motor->v_max, motor->v_max);

    DM_J4310_PutFloatLE(&data[0], motor->target_pos);
    DM_J4310_PutFloatLE(&data[4], motor->target_vel);

    return DM_J4310_Transmit(motor,
                             (uint16_t)(DM_J4310_ID_PV_OFFSET + motor->can_id),
                             data, (uint8_t)DM_J4310_FRAME_LEN);
}

/** @brief 纯速度模式帧（帧 ID = 0x200 + CAN_ID）：只有一个 float，DLC = 4 */
uint8_t DM_J4310_SendVel(DM_J4310_HandleTypeDef *motor)
{
    uint8_t data[4];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return 0u;
    }

    motor->target_vel = DM_J4310_Clamp(motor->target_vel, -motor->v_max, motor->v_max);

    DM_J4310_PutFloatLE(&data[0], motor->target_vel);

    return DM_J4310_Transmit(motor,
                             (uint16_t)(DM_J4310_ID_VEL_OFFSET + motor->can_id),
                             data, 4u);
}

/*==============================================================================
 * 接收解析（在 CAN 接收中断里调用）
 *============================================================================*/
/**
 * @brief  核对"这一帧是不是本电机的"后解析
 * @return 1 = 是且解析成功；0 = 不是本机 / 长度不够 / 内容对不上
 * @note   同一路 CAN 上挂多台电机时，在回调里对每个句柄都调一次，谁也不影响谁
 */
uint8_t DM_J4310_GetFeedback(DM_J4310_HandleTypeDef *motor,
                             DM_CAN_RxHeaderTypeDef *rx_header, const uint8_t *data)
{
    if ((motor == NULL) || (rx_header == NULL) || (data == NULL))
    {
        return 0u;
    }

#ifdef __FDCAN_H__
    if (rx_header->IdType != FDCAN_STANDARD_ID)
    {
        return 0u;                                  /* 达妙用标准帧，扩展帧不可能是它 */
    }
    if (rx_header->Identifier != (uint32_t)motor->master_id)
    {
        return 0u;
    }
    if (rx_header->DataLength < DM_J4310_FRAME_LEN)
    {
        motor->bad_cnt++;
        return 0u;
    }
#elif defined __CAN_H__
    if (rx_header->IDE != CAN_ID_STD)
    {
        return 0u;
    }
    if (rx_header->StdId != (uint32_t)motor->master_id)
    {
        return 0u;
    }
    if (rx_header->DLC < DM_J4310_FRAME_LEN)
    {
        motor->bad_cnt++;
        return 0u;
    }
#endif

    return DM_J4310_ParseFeedback(motor, data);
}

/**
 * @brief  解析 8 字节反馈数据（三种控制模式的反馈格式完全相同）
 * @note   反馈里的位置在 ±PMAX 处环回（比如从 +12.5 一下跳到 -12.5），
 *         所以这里做增量累加，real_pos 才是连续角度；pos 保持驱动器口径
 */
uint8_t DM_J4310_ParseFeedback(DM_J4310_HandleTypeDef *motor, const uint8_t *data)
{
    uint8_t id;
    uint8_t err;

    if ((motor == NULL) || (data == NULL))
    {
        return 0u;
    }

    motor->correct = 0u;

    id  = (uint8_t)(data[0] & 0x0Fu);
    err = (uint8_t)((data[0] >> 4) & 0x0Fu);

    /* 再核一次 ID：MST_ID 出厂是 0x00，同一路 CAN 上几台电机的反馈帧 ID
     * 完全一样，只能靠 D[0] 低 4 位区分（所以 MST_ID 建议改成 CAN_ID + 0x10） */
    if (id != (uint8_t)(motor->can_id & 0x0Fu))
    {
        motor->bad_cnt++;
        return 0u;
    }

    motor->id  = id;
    motor->err = err;

    motor->pos_raw = (uint16_t)((((uint16_t)data[1]) << 8) | (uint16_t)data[2]);
    motor->vel_raw = (uint16_t)((((uint16_t)data[3]) << 4) | ((uint16_t)data[4] >> 4));
    motor->tau_raw = (uint16_t)(((((uint16_t)data[4]) & 0x000Fu) << 8) | (uint16_t)data[5]);

    motor->t_mos   = data[6];
    motor->t_rotor = data[7];

    motor->pos    = DM_J4310_UintToFloat(motor->pos_raw, -motor->p_max, motor->p_max, 16u);
    motor->vel    = DM_J4310_UintToFloat(motor->vel_raw, -motor->v_max, motor->v_max, 12u);
    motor->torque = DM_J4310_UintToFloat(motor->tau_raw, -motor->t_max, motor->t_max, 12u);

    if ((motor->zero_pending != 0u) || (motor->ok_cnt == 0u))
    {
        /* 刚存过零点 / 上电第一帧：不累加，直接把 real_pos 对齐到驱动器口径 */
        motor->real_pos     = motor->pos;
        motor->delta_pos    = 0.0f;
        motor->zero_pending = 0u;
    }
    else
    {
        motor->delta_pos = motor->pos - motor->last_pos;
        if (motor->delta_pos > motor->p_max)
        {
            motor->delta_pos -= 2.0f * motor->p_max;      /* 跨 +PMAX 环回 */
        }
        else if (motor->delta_pos < -motor->p_max)
        {
            motor->delta_pos += 2.0f * motor->p_max;      /* 跨 -PMAX 环回 */
        }
        motor->real_pos += motor->delta_pos;
    }

    motor->last_pos = motor->pos;

    motor->correct      = 1u;
    motor->online       = 1u;
    motor->ok_cnt++;
    motor->last_rx_tick = HAL_GetTick();

    return 1u;
}

/** @brief 在线判定：DM_J4310_ONLINE_TIMEOUT 内收到过有效反馈就算在线 */
uint8_t DM_J4310_IsOnline(DM_J4310_HandleTypeDef *motor)
{
    if ((motor == NULL) || (motor->ok_cnt == 0u))
    {
        return 0u;
    }

    if ((HAL_GetTick() - motor->last_rx_tick) > (uint32_t)DM_J4310_ONLINE_TIMEOUT)
    {
        motor->online = 0u;
        return 0u;
    }

    motor->online = 1u;

    return 1u;
}

/** @brief 状态码转文字（串口打印用；返回常量字符串，不占运行期内存） */
const char *DM_J4310_ErrName(uint8_t err)
{
    switch (err)
    {
        case eDM_J4310_ERR_DISABLED:       return "disabled";
        case eDM_J4310_ERR_ENABLED:        return "enabled";
        case eDM_J4310_ERR_OVER_VOLTAGE:   return "over-voltage";
        case eDM_J4310_ERR_UNDER_VOLTAGE:  return "under-voltage";
        case eDM_J4310_ERR_OVER_CURRENT:   return "over-current";
        case eDM_J4310_ERR_MOS_OVER_TEMP:  return "mos-over-temp";
        case eDM_J4310_ERR_COIL_OVER_TEMP: return "coil-over-temp";
        case eDM_J4310_ERR_LOST_COMM:      return "lost-comm";
        case eDM_J4310_ERR_OVERLOAD:       return "overload";
        default:                           return "unknown";
    }
}

/*==============================================================================
 * 协议底层
 *============================================================================*/
/**
 * @brief 发一帧标准数据帧
 * @note  FIFO 满时丢帧返回 0，不 while 死等：控制任务里死等会让整个周期抖动，
 *        也可能因为总线被堵而死在这里（手册 8.2 / 11.2 都强调过）
 */
static uint8_t DM_J4310_Transmit(DM_J4310_HandleTypeDef *motor,
                                 uint16_t id, const uint8_t *data, uint8_t len)
{
    uint8_t i;

    if ((motor == NULL) || (motor->hcan == NULL) || (data == NULL))
    {
        return 0u;
    }

    if (len > (uint8_t)DM_J4310_FRAME_LEN)
    {
        len = (uint8_t)DM_J4310_FRAME_LEN;
    }

    /* 留一份"最近发出去的帧"，配合 CAN 分析仪对波形用（Live Watch 看 last_tx_buf） */
    for (i = 0u; i < (uint8_t)DM_J4310_FRAME_LEN; i++)
    {
        motor->last_tx_buf[i] = (i < len) ? data[i] : 0xFFu;
    }

#ifdef __FDCAN_H__
    {
        FDCAN_TxHeaderTypeDef tx_header;

        tx_header.Identifier          = id;
        tx_header.IdType              = FDCAN_STANDARD_ID;      /* 达妙固定标准帧 */
        tx_header.TxFrameType         = FDCAN_DATA_FRAME;
        tx_header.DataLength          = len;                   /* 经典帧：字节数就是 DLC */
        tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
        tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
        tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        tx_header.MessageMarker       = 0u;

        if (HAL_FDCAN_GetTxFifoFreeLevel(motor->hcan) == 0u)
        {
            motor->tx_full_cnt++;
            return 0u;
        }

        /* HAL 的形参不是 const，这里只读，强转掉 const 是安全的 */
        if (HAL_FDCAN_AddMessageToTxFifoQ(motor->hcan, &tx_header, (uint8_t *)data) != HAL_OK)
        {
            motor->tx_full_cnt++;
            return 0u;
        }
    }
#elif defined __CAN_H__
    {
        CAN_TxHeaderTypeDef tx_header;
        uint32_t            mailbox;

        tx_header.StdId              = id;
        tx_header.ExtId              = 0u;
        tx_header.IDE                = CAN_ID_STD;              /* 达妙固定标准帧 */
        tx_header.RTR                = CAN_RTR_DATA;
        tx_header.DLC                = len;
        tx_header.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_GetTxMailboxesFreeLevel(motor->hcan) == 0u)
        {
            motor->tx_full_cnt++;
            return 0u;
        }

        if (HAL_CAN_AddTxMessage(motor->hcan, &tx_header, (uint8_t *)data, &mailbox) != HAL_OK)
        {
            motor->tx_full_cnt++;
            return 0u;
        }
    }
#endif

    motor->last_tx_tick = HAL_GetTick();

    return 1u;
}

/** @brief 当前控制模式下，控制帧 / 特殊命令帧使用的 CAN ID
 *  @note  MIT: CAN_ID；PV: 0x100 + CAN_ID；纯速度: 0x200 + CAN_ID（手册 4.2~4.4）。
 *         使能/失能/清错/存零点也是“控制帧”，ID 要跟当前模式一致，
 *         否则 PV 模式下电机根本不认（现场踩过：1 号机被设成 PV，MIT 帧全被忽略）。 */
static uint16_t DM_J4310_CmdId(const DM_J4310_HandleTypeDef *motor)
{
    switch (motor->mode)
    {
        case eDM_J4310_MODE_PV:
            return (uint16_t)(DM_J4310_ID_PV_OFFSET + motor->can_id);

        case eDM_J4310_MODE_VEL:
            return (uint16_t)(DM_J4310_ID_VEL_OFFSET + motor->can_id);

        case eDM_J4310_MODE_MIT:
        default:
            return motor->can_id;
    }
}

/** @brief 四种特殊命令帧：前 7 字节 0xFF，D[7] 是命令码 */
static uint8_t DM_J4310_SendCommand(DM_J4310_HandleTypeDef *motor, uint8_t cmd)
{
    uint8_t data[DM_J4310_FRAME_LEN];
    uint8_t i;

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return 0u;
    }

    for (i = 0u; i < ((uint8_t)DM_J4310_FRAME_LEN - 1u); i++)
    {
        data[i] = 0xFFu;
    }
    data[DM_J4310_FRAME_LEN - 1u] = cmd;

    return DM_J4310_Transmit(motor, DM_J4310_CmdId(motor), data,
                             (uint8_t)DM_J4310_FRAME_LEN);
}

/** @brief 指令清零（初始化/失能时用，保证"下一次输出从 0 开始"） */
static void DM_J4310_ResetCommand(DM_J4310_HandleTypeDef *motor)
{
    motor->target_pos    = 0.0f;
    motor->target_vel    = 0.0f;
    motor->target_torque = 0.0f;
    motor->kp            = 0.0f;
    motor->kd            = 0.0f;
}

/*==============================================================================
 * 小工具
 *============================================================================*/
/** @brief 限幅；顺带把 NaN 变成 0（NaN 参与比较全为假，必须先单独判掉） */
static float DM_J4310_Clamp(float value, float min_value, float max_value)
{
    if (!(value == value))
    {
        return 0.0f;
    }
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/** @brief 浮点 -> 无符号定点（手册 4.2 的 float_to_uint，超范围时饱和，不做回绕） */
static uint16_t DM_J4310_FloatToUint(float x, float x_min, float x_max, uint8_t bits)
{
    float    span = x_max - x_min;
    uint32_t full;

    if ((span <= 0.0f) || (bits == 0u) || (bits > 16u))
    {
        return 0u;
    }

    if (!(x == x))
    {
        x = 0.0f;
    }

    full = (uint32_t)((1ul << bits) - 1ul);

    if (x <= x_min)
    {
        return 0u;
    }
    if (x >= x_max)
    {
        return (uint16_t)full;
    }

    return (uint16_t)((x - x_min) * ((float)full) / span);
}

/** @brief 无符号定点 -> 浮点（手册 4.2 的 uint_to_float） */
static float DM_J4310_UintToFloat(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float full;

    if ((bits == 0u) || (bits > 16u))
    {
        return 0.0f;
    }

    full = (float)((1ul << bits) - 1ul);

    return ((float)x_int) * span / full + x_min;
}

/** @brief 按手册"低位在前"把 float 拆成 4 字节（不依赖编译器/CPU 端序） */
static void DM_J4310_PutFloatLE(uint8_t *dst, float value)
{
    union
    {
        float    f;
        uint32_t u;
    } conv;

    conv.f = value;

    dst[0] = (uint8_t)(conv.u & 0xFFu);
    dst[1] = (uint8_t)((conv.u >> 8) & 0xFFu);
    dst[2] = (uint8_t)((conv.u >> 16) & 0xFFu);
    dst[3] = (uint8_t)((conv.u >> 24) & 0xFFu);
}

    