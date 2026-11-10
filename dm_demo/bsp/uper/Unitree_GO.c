#include "Unitree_GO.h"
#include <string.h>

/*==============================================================================
 * 宇树 GO-M8010-6 驱动（RS485 半双工）
 *-------------------------------------------------------------------------------
 * 报文格式（手册 8.2 节，小端）：
 *   主机 -> 电机（17 字节）
 *     [0..1]  0xFE 0xEE          包头
 *     [2]     id(bit0~3) | status(bit4~6) | 保留(bit7)
 *     [3..4]  tor_des  int16  实际力矩     = T*256
 *     [5..6]  spd_des  int16  实际角速度   = W/2π*256
 *     [7..10] pos_des  int32  实际角度     = Pos/2π*32768
 *     [11..12] k_pos   int16  位置刚度     = K_P*1280（即 K_P/25.6*32768）
 *     [13..14] k_spd   int16  速度阻尼     = K_W*1280
 *     [15..16] CRC16   int16  前 15 字节的 CRC16-CCITT
 *
 *   电机 -> 主机（16 字节）
 *     [0..1]  0xFD 0xEE          包头
 *     [2]     id(bit0~3) | status(bit4~6)
 *     [3..4]  torque   int16  实际力矩      = τ*256
 *     [5..6]  speed    int16  实际角速度    = ω/(2π)*256
 *     [7..10] pos      int32  实际角度      = θ/(2π)*32768
 *     [11]    temp     int8   温度 ℃
 *     [12..13] 错误码 bit0~2 + 足端力 bit3~14 + 保留 bit15
 *     [14..15] CRC16          前 14 字节的 CRC16-CCITT
 *
 * 注：定点换算里的 2π 用 6.2831853（R2 的代码用的是 6.28318），换算误差 1e-5 量级，
 *     比协议本身的定点量化（位置 1LSB≈0.0002rad）还小，不影响使用。
 *============================================================================*/

/* 包头与定点缩放系数 */
#define UNITREE_GO_HEAD_TX0         0xFEu
#define UNITREE_GO_HEAD_TX1         0xEEu
#define UNITREE_GO_HEAD_RX0         0xFDu
#define UNITREE_GO_HEAD_RX1         0xEEu

#define UNITREE_GO_T_SCALE          256.0f                  // N·m   -> 定点
#define UNITREE_GO_W_SCALE          (256.0f / UNITREE_GO_TWO_PI)
#define UNITREE_GO_P_SCALE          (32768.0f / UNITREE_GO_TWO_PI)
#define UNITREE_GO_K_SCALE          (32768.0f / 25.6f)      // = 1280

/* 协议允许的指令范围（手册表 1） */
#define UNITREE_GO_T_LIMIT          127.99f
#define UNITREE_GO_W_LIMIT          804.0f
#define UNITREE_GO_P_LIMIT          411774.0f
#define UNITREE_GO_K_LIMIT          25.599f

/* 485 方向脚电平：实际取值由句柄里的 de_active_high 决定（Init 时用宏 UNITREE_GO_DE_ACTIVE_HIGH
 * 作为默认，运行中可以改，方便现场试极性） */

/*------------------------------------------------------------------------------
 * 内部函数
 *----------------------------------------------------------------------------*/
static float   Unitree_GO_Clamp(float value, float min_value, float max_value);
static void    Unitree_GO_SaturateCommand(Unitree_GO_HandleTypeDef *motor);
static void    Unitree_GO_BuildFrame(Unitree_GO_HandleTypeDef *motor);
static void    Unitree_GO_ClearRx(Unitree_GO_HandleTypeDef *motor);
static void    Unitree_GO_DeSet(Unitree_GO_HandleTypeDef *motor, uint8_t transmit);
static void    Unitree_GO_DeTx(Unitree_GO_HandleTypeDef *motor);
static void    Unitree_GO_DeRx(Unitree_GO_HandleTypeDef *motor);

/*==============================================================================
 * 初始化
 *============================================================================*/
void Unitree_GO_Init(Unitree_GO_HandleTypeDef *motor,
                     UART_HandleTypeDef *huart,
                     GPIO_TypeDef *de_port, uint16_t de_pin,
                     uint8_t id, float ratio)
{
    GPIO_InitTypeDef gpio_init = {0};

    if (motor == NULL)
    {
        return;
    }

    motor->huart   = huart;
    motor->de_port = de_port;
    motor->de_pin  = de_pin;

#if (UNITREE_GO_DE_ACTIVE_HIGH != 0)
    motor->de_active_high = 1u;
#else
    motor->de_active_high = 0u;
#endif

    motor->id    = (id > UNITREE_GO_ID_BROADCAST) ? UNITREE_GO_ID_BROADCAST : id;
    motor->ratio = (ratio > 0.01f) ? ratio : UNITREE_GO_DEFAULT_RATIO;

    /* 指令清零：初始锁定(0)，等上层下发控制目标时再自动切到 FOC(1) */
    motor->mode = UNITREE_GO_MODE_LOCK;
    motor->T    = 0.0f;
    motor->W    = 0.0f;
    motor->Pos  = 0.0f;
    motor->K_P  = 0.0f;
    motor->K_W  = 0.0f;

    motor->pos_offset = 0.0f;

    motor->temperature = 0;
    motor->error_code  = 0u;
    motor->foot_force  = 0u;
    motor->torque      = 0.0f;
    motor->speed       = 0.0f;
    motor->pos         = 0.0f;
    motor->joint_torque = 0.0f;
    motor->joint_speed  = 0.0f;
    motor->joint_pos    = 0.0f;

    motor->correct     = 0u;
    motor->ok_cnt      = 0u;
    motor->timeout_cnt = 0u;
    motor->crc_err_cnt = 0u;
    motor->last_tick   = 0u;

    memset(motor->tx_buf, 0, sizeof(motor->tx_buf));
    memset(motor->rx_buf, 0, sizeof(motor->rx_buf));

    /* 485 收发方向脚：推挽输出，初始切到接收方向 */
    if (de_port != NULL)
    {
        if (de_port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
        else if (de_port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
        else if (de_port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
        else if (de_port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
        else if (de_port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
        else { }

        gpio_init.Pin   = de_pin;
        gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
        gpio_init.Pull  = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(de_port, &gpio_init);

        Unitree_GO_DeSet(motor, 0u);    /* 0 = 接收方向 */
    }
}

/*==============================================================================
 * 控制目标
 *============================================================================*/
void Unitree_GO_SetMode(Unitree_GO_HandleTypeDef *motor, uint8_t mode)
{
    if (motor == NULL)
    {
        return;
    }
    motor->mode = (uint8_t)(mode & 0x07u);
}

void Unitree_GO_Enable(Unitree_GO_HandleTypeDef *motor)
{
    Unitree_GO_SetMode(motor, UNITREE_GO_MODE_FOC);
}

void Unitree_GO_Lock(Unitree_GO_HandleTypeDef *motor)
{
    Unitree_GO_SetMode(motor, UNITREE_GO_MODE_LOCK);
}

void Unitree_GO_Calibrate(Unitree_GO_HandleTypeDef *motor)
{
    Unitree_GO_SetMode(motor, UNITREE_GO_MODE_CALIBRATE);
}

/* 力位混合模式：目标位置/速度/力矩按关节（输出轴）侧单位，
 * 内部换算： Pos = 零点 + 关节角*减速比
 *           W   = 关节角速度*减速比
 *           T   = 关节力矩/减速比
 *           K_P/K_W 的口径由 UNITREE_GO_GAIN_IN_JOINT_SPACE 决定（关节侧时除以减速比²） */
void Unitree_GO_MITMode(Unitree_GO_HandleTypeDef *motor,
                        float joint_pos, float joint_speed,
                        float kp, float kd, float joint_torque)
{
    float gain_scale;

    if (motor == NULL)
    {
        return;
    }

#if (UNITREE_GO_GAIN_IN_JOINT_SPACE != 0)
    gain_scale = motor->ratio * motor->ratio;
#else
    gain_scale = 1.0f;
#endif

    motor->mode = UNITREE_GO_MODE_FOC;
    motor->Pos  = motor->pos_offset + joint_pos * motor->ratio;
    motor->W    = joint_speed * motor->ratio;
    motor->T    = joint_torque / motor->ratio;
    motor->K_P  = kp / gain_scale;
    motor->K_W  = kd / gain_scale;
}

/* 位置模式：纯位置 PD（速度目标、前馈力矩都清 0） */
void Unitree_GO_PositionMode(Unitree_GO_HandleTypeDef *motor,
                             float joint_pos, float kp, float kd)
{
    Unitree_GO_MITMode(motor, joint_pos, 0.0f, kp, kd, 0.0f);
}

/* 速度模式：T=0、K_P=0，只有速度阻尼 K_W 起作用（阻尼模式就是目标速度给 0） */
void Unitree_GO_SpeedMode(Unitree_GO_HandleTypeDef *motor,
                          float joint_speed, float kd)
{
    float gain_scale;

    if (motor == NULL)
    {
        return;
    }

#if (UNITREE_GO_GAIN_IN_JOINT_SPACE != 0)
    gain_scale = motor->ratio * motor->ratio;
#else
    gain_scale = 1.0f;
#endif

    motor->mode = UNITREE_GO_MODE_FOC;
    motor->Pos  = motor->pos_offset;
    motor->W    = joint_speed * motor->ratio;
    motor->T    = 0.0f;
    motor->K_P  = 0.0f;
    motor->K_W  = kd / gain_scale;
}

/* 力矩模式：只有前馈力矩 */
void Unitree_GO_TorqueMode(Unitree_GO_HandleTypeDef *motor, float joint_torque)
{
    if (motor == NULL)
    {
        return;
    }

    motor->mode = UNITREE_GO_MODE_FOC;
    motor->Pos  = motor->pos_offset;
    motor->W    = 0.0f;
    motor->T    = joint_torque / motor->ratio;
    motor->K_P  = 0.0f;
    motor->K_W  = 0.0f;
}

/* 零力矩模式：电机主动抵消自身摩擦，可以用手拖动 */
void Unitree_GO_ZeroTorqueMode(Unitree_GO_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->mode = UNITREE_GO_MODE_FOC;
    motor->Pos  = motor->pos_offset;
    motor->W    = 0.0f;
    motor->T    = 0.0f;
    motor->K_P  = 0.0f;
    motor->K_W  = 0.0f;
}

/* 转子侧原始接口：参数直接对应手册表 1，不做减速比折算 */
void Unitree_GO_MITModeRaw(Unitree_GO_HandleTypeDef *motor,
                           float pos, float speed, float kp, float kd, float torque)
{
    if (motor == NULL)
    {
        return;
    }

    motor->mode = UNITREE_GO_MODE_FOC;
    motor->Pos  = pos;
    motor->W    = speed;
    motor->T    = torque;
    motor->K_P  = kp;
    motor->K_W  = kd;
}

/* 把当前转子位置记成关节 0 点 */
void Unitree_GO_SetZeroPosition(Unitree_GO_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->pos_offset = motor->pos;
    motor->joint_pos  = 0.0f;
}

/*==============================================================================
 * 通讯
 *============================================================================*/
uint8_t Unitree_GO_SendRecv(Unitree_GO_HandleTypeDef *motor)
{
    if ((motor == NULL) || (motor->huart == NULL))
    {
        return 0u;
    }

    Unitree_GO_BuildFrame(motor);
    Unitree_GO_ClearRx(motor);

    /* 485 切发送，发完（HAL_UART_Transmit 返回时 TC 已置位）再切回接收 */
    Unitree_GO_DeTx(motor);
    if (HAL_UART_Transmit(motor->huart, motor->tx_buf, UNITREE_GO_TX_LEN,
                          UNITREE_GO_TX_TIMEOUT_MS) != HAL_OK)
    {
        Unitree_GO_DeRx(motor);
        motor->correct = 0u;
        motor->timeout_cnt++;
        return 0u;
    }
    Unitree_GO_DeRx(motor);

    /* 广播(id=15)时总线没有回包，直接返回发送结果，别在这里白等超时 */
    if (motor->id == UNITREE_GO_ID_BROADCAST)
    {
        return 1u;
    }

    /* 有些 485 收发芯片的接收使能没跟着 DE 关，发送期间会把自己发的字节又收回来，
     * 这里在正式等回包前再清一次，避免把自己的包当成电机回包（电机的回包至少
     * 要等它收完+校验+处理，远晚于这几个指令周期，清不坏真回包）。 */
    Unitree_GO_ClearRx(motor);

    if (HAL_UART_Receive(motor->huart, motor->rx_buf, UNITREE_GO_RX_LEN,
                         UNITREE_GO_RX_TIMEOUT_MS) != HAL_OK)
    {
        motor->correct = 0u;
        motor->timeout_cnt++;
        return 0u;
    }

    return Unitree_GO_ParseData(motor, motor->rx_buf);
}

/* 只发不收：广播(id=15)、编码器标定命令用 */
uint8_t Unitree_GO_SendOnly(Unitree_GO_HandleTypeDef *motor)
{
    if ((motor == NULL) || (motor->huart == NULL))
    {
        return 0u;
    }

    Unitree_GO_BuildFrame(motor);
    Unitree_GO_ClearRx(motor);

    Unitree_GO_DeTx(motor);
    if (HAL_UART_Transmit(motor->huart, motor->tx_buf, UNITREE_GO_TX_LEN,
                          UNITREE_GO_TX_TIMEOUT_MS) != HAL_OK)
    {
        Unitree_GO_DeRx(motor);
        motor->timeout_cnt++;
        return 0u;
    }
    Unitree_GO_DeRx(motor);

    return 1u;
}

/* 解析一帧 16 字节反馈：1 = 有效帧，0 = 帧头/CRC 错 */
uint8_t Unitree_GO_ParseData(Unitree_GO_HandleTypeDef *motor, const uint8_t *data)
{
    uint16_t crc_recv;
    uint16_t crc_calc;
    int16_t  torque_raw;
    int16_t  speed_raw;
    int32_t  pos_raw;
    uint16_t error_force;

    if ((motor == NULL) || (data == NULL))
    {
        return 0u;
    }

    motor->correct = 0u;

    if ((data[0] != UNITREE_GO_HEAD_RX0) || (data[1] != UNITREE_GO_HEAD_RX1))
    {
        motor->crc_err_cnt++;
        return 0u;
    }

    crc_recv = (uint16_t)((uint16_t)data[14] | ((uint16_t)data[15] << 8));
    crc_calc = Unitree_GO_Crc16(data, UNITREE_GO_RX_LEN - 2u);
    if (crc_recv != crc_calc)
    {
        motor->crc_err_cnt++;
        return 0u;
    }

    torque_raw = (int16_t)((uint16_t)data[3] | ((uint16_t)data[4] << 8));
    speed_raw  = (int16_t)((uint16_t)data[5] | ((uint16_t)data[6] << 8));
    pos_raw    = (int32_t)((uint32_t)data[7] | ((uint32_t)data[8] << 8) |
                           ((uint32_t)data[9] << 16) | ((uint32_t)data[10] << 24));
    error_force = (uint16_t)((uint16_t)data[12] | ((uint16_t)data[13] << 8));

    motor->temperature = (int8_t)data[11];
    motor->error_code  = (uint8_t)(error_force & 0x0007u);          // bit0~2
    motor->foot_force  = (uint16_t)((error_force >> 3) & 0x0FFFu);  // bit3~14

    motor->torque = (float)torque_raw / 256.0f;
    motor->speed  = (float)speed_raw / 256.0f * UNITREE_GO_TWO_PI;
    motor->pos    = (float)pos_raw / 32768.0f * UNITREE_GO_TWO_PI;

    motor->joint_torque = motor->torque * motor->ratio;
    motor->joint_speed  = motor->speed / motor->ratio;
    motor->joint_pos    = (motor->pos - motor->pos_offset) / motor->ratio;

    motor->correct   = 1u;
    motor->ok_cnt++;
    motor->last_tick = HAL_GetTick();

    return 1u;
}

uint8_t Unitree_GO_IsOnline(const Unitree_GO_HandleTypeDef *motor)
{
    if ((motor == NULL) || (motor->ok_cnt == 0u))
    {
        return 0u;
    }

    return (uint8_t)((HAL_GetTick() - motor->last_tick) <= UNITREE_GO_ONLINE_TIMEOUT);
}

/*==============================================================================
 * 协议底层
 *============================================================================*/
/* CRC16-CCITT 反射实现：init = 0，多项式 0x1021（反射值 0x8408），
 * 与 Linux 内核 crc_ccitt() 的结果一致，也就是 R2_Code 里那张 256 项表的结果 */
uint16_t Unitree_GO_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0u;
    uint16_t i;
    uint8_t  bit;

    if (data == NULL)
    {
        return 0u;
    }

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 0x0001u) != 0u)
            {
                crc = (uint16_t)((crc >> 1) ^ 0x8408u);
            }
            else
            {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }

    return crc;
}

static void Unitree_GO_BuildFrame(Unitree_GO_HandleTypeDef *motor)
{
    int16_t  torque_raw;
    int16_t  speed_raw;
    int32_t  pos_raw;
    int16_t  kp_raw;
    int16_t  kw_raw;
    uint16_t crc;

    Unitree_GO_SaturateCommand(motor);

    torque_raw = (int16_t)(motor->T * UNITREE_GO_T_SCALE);
    speed_raw  = (int16_t)(motor->W * UNITREE_GO_W_SCALE);
    pos_raw    = (int32_t)(motor->Pos * UNITREE_GO_P_SCALE);
    kp_raw     = (int16_t)(motor->K_P * UNITREE_GO_K_SCALE);
    kw_raw     = (int16_t)(motor->K_W * UNITREE_GO_K_SCALE);

    motor->tx_buf[0]  = UNITREE_GO_HEAD_TX0;
    motor->tx_buf[1]  = UNITREE_GO_HEAD_TX1;
    motor->tx_buf[2]  = (uint8_t)(((motor->id & 0x0Fu)) | ((motor->mode & 0x07u) << 4));
    motor->tx_buf[3]  = (uint8_t)((uint16_t)torque_raw & 0xFFu);
    motor->tx_buf[4]  = (uint8_t)(((uint16_t)torque_raw >> 8) & 0xFFu);
    motor->tx_buf[5]  = (uint8_t)((uint16_t)speed_raw & 0xFFu);
    motor->tx_buf[6]  = (uint8_t)(((uint16_t)speed_raw >> 8) & 0xFFu);
    motor->tx_buf[7]  = (uint8_t)((uint32_t)pos_raw & 0xFFu);
    motor->tx_buf[8]  = (uint8_t)(((uint32_t)pos_raw >> 8) & 0xFFu);
    motor->tx_buf[9]  = (uint8_t)(((uint32_t)pos_raw >> 16) & 0xFFu);
    motor->tx_buf[10] = (uint8_t)(((uint32_t)pos_raw >> 24) & 0xFFu);
    motor->tx_buf[11] = (uint8_t)((uint16_t)kp_raw & 0xFFu);
    motor->tx_buf[12] = (uint8_t)(((uint16_t)kp_raw >> 8) & 0xFFu);
    motor->tx_buf[13] = (uint8_t)((uint16_t)kw_raw & 0xFFu);
    motor->tx_buf[14] = (uint8_t)(((uint16_t)kw_raw >> 8) & 0xFFu);

    crc = Unitree_GO_Crc16(motor->tx_buf, UNITREE_GO_TX_LEN - 2u);
    motor->tx_buf[15] = (uint8_t)(crc & 0xFFu);
    motor->tx_buf[16] = (uint8_t)((crc >> 8) & 0xFFu);
}

/*==============================================================================
 * 内部小工具
 *============================================================================*/
static float Unitree_GO_Clamp(float value, float min_value, float max_value)
{
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

/* 按协议范围裁剪下发的指令，保证发出去的就是记录下来的 */
static void Unitree_GO_SaturateCommand(Unitree_GO_HandleTypeDef *motor)
{
    motor->T   = Unitree_GO_Clamp(motor->T,   -UNITREE_GO_T_LIMIT, UNITREE_GO_T_LIMIT);
    motor->W   = Unitree_GO_Clamp(motor->W,   -UNITREE_GO_W_LIMIT, UNITREE_GO_W_LIMIT);
    motor->Pos = Unitree_GO_Clamp(motor->Pos, -UNITREE_GO_P_LIMIT, UNITREE_GO_P_LIMIT);
    motor->K_P = Unitree_GO_Clamp(motor->K_P, 0.0f, UNITREE_GO_K_LIMIT);
    motor->K_W = Unitree_GO_Clamp(motor->K_W, 0.0f, UNITREE_GO_K_LIMIT);
    motor->mode = (uint8_t)(motor->mode & 0x07u);
}

/* 清掉接收侧可能残留的错误标志和半个字节 */
static void Unitree_GO_ClearRx(Unitree_GO_HandleTypeDef *motor)
{
    __HAL_UART_CLEAR_OREFLAG(motor->huart);
    __HAL_UART_CLEAR_FEFLAG(motor->huart);
    __HAL_UART_CLEAR_NEFLAG(motor->huart);
    __HAL_UART_CLEAR_PEFLAG(motor->huart);
    (void)motor->huart->Instance->RDR;
}

/* 485 方向脚：transmit=1 切发送、0 切接收。极性由句柄的 de_active_high 决定（可运行时改） */
static void Unitree_GO_DeSet(Unitree_GO_HandleTypeDef *motor, uint8_t transmit)
{
    uint8_t level;

    if (motor->de_port == NULL)
    {
        return;
    }

    /* de_active_high = 1 时：发送=高，接收=低；= 0 时反过来 */
    if (motor->de_active_high != 0u)
    {
        level = transmit;
    }
    else
    {
        level = (uint8_t)((transmit != 0u) ? 0u : 1u);
    }

    HAL_GPIO_WritePin(motor->de_port, motor->de_pin,
                      (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 收发芯片使能时间是百 ns 级，这里等几个指令周期（多等无副作用） */
#if (UNITREE_GO_DE_SETTLE_LOOPS > 0u)
    {
        volatile uint32_t i;
        for (i = 0u; i < UNITREE_GO_DE_SETTLE_LOOPS; i++)
        {
            __NOP();
        }
    }
#endif
}

/* 485 切发送 */
static void Unitree_GO_DeTx(Unitree_GO_HandleTypeDef *motor)
{
    Unitree_GO_DeSet(motor, 1u);
}

/* 485 切接收：回到接收电平后才能读到电机回的报文 */
static void Unitree_GO_DeRx(Unitree_GO_HandleTypeDef *motor)
{
    Unitree_GO_DeSet(motor, 0u);
}
