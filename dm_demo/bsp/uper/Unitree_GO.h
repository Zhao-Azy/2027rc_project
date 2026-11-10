#ifndef UNITREE_GO_H
#define UNITREE_GO_H

#include "main.h"
#include "usart.h"

/*==============================================================================
 * 宇树 GO-M8010-6 关节电机驱动（RS485 半双工，单文件封装）
 *-------------------------------------------------------------------------------
 * 移植自 R2_Code/Application/Unitree（gom_protocol.c/h + Unitree_control.c/h
 * + crc_ccitt.c/h，三对文件），合并成这一个模块，对接顺序和用法与项目里
 * RS00 / RS05 / MG5010E 等关节电机封装保持一致。
 *
 * 与原 R2 代码的区别：
 *   1) 原来是"串口 + USB 转 485 模块"（PC 端转换器自己控制收发方向），
 *      现在直接用开发板板载的 485：USART + GPIO 方向脚(DE) 手动切收发，
 *      所以发送前拉高 DE、发完最后一个字节(TC)后拉低 DE 再收。
 *   2) 协议收发改成按字节拆装 + 位运算，不再用 #pragma pack 的位域结构体，
 *      避免不同编译器对位域排布的处理差异（字节序与手册表 1/表 2 完全一致）。
 *   3) 对外接口统一用"关节（减速器输出轴）侧"单位，内部按减速比换算。
 *============================================================================*/

/*==============================================================================
 * 一、板载 485 通道选择
 *-------------------------------------------------------------------------------
 * DM-MC-Board02 板载两路高速 RS485（收发芯片最高 10Mbps，丝印 RS485-U2 / RS485-U3），
 * 宇树 GO-M8010-6 固定 4Mbps / 8 数据位 / 无校验 / 1 停止位：
 *
 *   通道 1：USART2：PD5=TX / PD6=RX，方向脚 DE = PD4
 *   通道 2：USART3：PD8=TX / PD9=RX，方向脚 DE = PB14
 *
 * 两路串口都在 usart.c 里按 4Mbps 配好了。电机插在哪个 485 口上就选哪一路，
 * 拿不准就先用 1，收不到回包（timeout_cnt 一直涨）再改成 2 试。
 * DE 方向脚由 Unitree_GO_Init() 自己初始化，不用在 CubeMX / gpio.c 里再配。
 *============================================================================*/
#define UNITREE_GO_485_CHANNEL      1

#if (UNITREE_GO_485_CHANNEL == 1)
    #define UNITREE_GO_485_UART     huart2
    #define UNITREE_GO_DE_PORT      GPIOD
    #define UNITREE_GO_DE_PIN       GPIO_PIN_4
#elif (UNITREE_GO_485_CHANNEL == 2)
    #define UNITREE_GO_485_UART     huart3
    #define UNITREE_GO_DE_PORT      GPIOB
    #define UNITREE_GO_DE_PIN       GPIO_PIN_14
#else
    #error "UNITREE_GO_485_CHANNEL 只能是 1 或 2"
#endif

/*==============================================================================
 * 二、协议常量
 *============================================================================*/
#define UNITREE_GO_TX_LEN           17u         // 主机 -> 电机 17 字节
#define UNITREE_GO_RX_LEN           16u         // 电机 -> 主机 16 字节
#define UNITREE_GO_ID_BROADCAST     15u         // id=15 广播（总线无返回）
#define UNITREE_GO_DEFAULT_RATIO    6.33f       // GO-M8010-6 减速比
#define UNITREE_GO_TWO_PI           6.2831853f

/* 电机工作模式（协议里的 status，3bit） */
#define UNITREE_GO_MODE_LOCK        0u          // 锁定（默认）
#define UNITREE_GO_MODE_FOC         1u          // FOC 闭环控制
#define UNITREE_GO_MODE_CALIBRATE   2u          // 编码器标定（总线上只能挂一个电机）

/* 单次通讯超时（ms）：4Mbps 下 17 字节约 34us，给 1ms 已经很宽松 */
#define UNITREE_GO_TX_TIMEOUT_MS    2u
#define UNITREE_GO_RX_TIMEOUT_MS    1u
/* 判定电机在线的时间窗（ms） */
#define UNITREE_GO_ONLINE_TIMEOUT   50u
/* DE 切方向后等待收发芯片稳定的空循环次数（约 8 个循环/次 @480MHz，64 次约 0.5us）。
 * 收发芯片的使能时间在百 ns 级；DE 拉高后总线本来就是空闲电平，多等一会儿没有副作用，
 * 只有"太短"会让第一个字节(0xFE)被吃掉，所以这里给足余量。 */
#define UNITREE_GO_DE_SETTLE_LOOPS  64u
/* 485 方向脚极性：1 = DE 高电平发送（板载收发芯片默认），
 * 万一实测收发方向相反，把这里改成 0 即可，不用改驱动代码 */
#define UNITREE_GO_DE_ACTIVE_HIGH   1

/* K_P / K_W（刚度、阻尼）的换算口径，两种口径差 减速比²≈40 倍：
 *   1 = 关节侧（默认）：内部除以 ratio² 再下发。关节刚度 1 就是 1 N·m/rad，
 *       常用值 kp = 2~20、kd = 0.5~2。
 *   0 = 转子侧：和手册表 1、R2 里 motor_mit_control() 的用法一样，原样下发；
 *       手册/R2 示例里的 0.02~0.2 属于这种口径（换算到关节侧要 ×40）。
 * 口径对不上时的现象是"电机完全不转"（系数被定点量化到只有 1 个 LSB）或"猛地一甩"，
 * 所以这里必须显式写清楚用哪种。 */
#define UNITREE_GO_GAIN_IN_JOINT_SPACE  1

/*==============================================================================
 * 三、句柄
 *-------------------------------------------------------------------------------
 * 指令字段（mode/T/W/Pos/K_P/K_W）是"电机转子侧"，与协议、手册完全对应；
 * 反馈同时保留转子侧的 torque/speed/pos 和换算后的 joint_* 两套，
 * 用哪套方便就用哪套，不要重复换算。
 *============================================================================*/
typedef struct
{
    UART_HandleTypeDef *huart;          // 485 串口（huart2 / huart3）
    GPIO_TypeDef       *de_port;        // 485 方向脚端口（NULL 表示不用方向脚）
    uint16_t            de_pin;         // 485 方向脚引脚
    uint8_t             de_active_high; // 方向脚极性：1=DE 高电平发送（Init 时取宏，可运行时改）

    uint8_t             id;             // 电机 ID（0~14，15 为广播）
    float               ratio;          // 减速比，GO-M8010-6 = 6.33

    /* ---- 下发给电机的指令（转子侧，与协议单位一致） ---- */
    uint8_t             mode;           // 0锁定 1FOC 2标定
    float               T;              // 转子力矩 N·m
    float               W;              // 转子角速度 rad/s
    float               Pos;            // 转子角度 rad（多圈累加，已含零点偏移）
    float               K_P;            // 位置刚度系数 0~25.599
    float               K_W;            // 速度刚度（阻尼）系数 0~25.599

    float               pos_offset;     // 位置上电零点（转子侧 rad）

    /* ---- 反馈（转子侧，由协议原始定点数换算） ---- */
    int8_t              temperature;    // 温度 ℃
    uint8_t             error_code;     // 0正常 1过热 2过流 3过压 4编码器 5母线欠压 6绕组过热
    uint16_t            foot_force;     // 足端力 12bit 原始值
    float               torque;         // N·m
    float               speed;          // rad/s
    float               pos;            // rad

    /* ---- 反馈（关节侧 = 转子侧 ÷/× 减速比） ---- */
    float               joint_torque;   // N·m
    float               joint_speed;    // rad/s
    float               joint_pos;      // rad

    /* ---- 通讯状态 ---- */
    uint8_t             correct;        // 最近一帧是否有效（1 有效）
    uint32_t            ok_cnt;         // 成功帧数
    uint32_t            timeout_cnt;    // 超时（电机没回）次数
    uint32_t            crc_err_cnt;    // 帧头/CRC 错误次数
    uint32_t            last_tick;      // 最近一次收到有效帧的 tick

    uint8_t             tx_buf[UNITREE_GO_TX_LEN];
    uint8_t             rx_buf[UNITREE_GO_RX_LEN];
} Unitree_GO_HandleTypeDef;

/*==============================================================================
 * 四、接口
 *============================================================================*/
void    Unitree_GO_Init(Unitree_GO_HandleTypeDef *motor,
                        UART_HandleTypeDef *huart,
                        GPIO_TypeDef *de_port, uint16_t de_pin,
                        uint8_t id, float ratio);

/* 模式控制（含使能/失能） */
void    Unitree_GO_SetMode(Unitree_GO_HandleTypeDef *motor, uint8_t mode);
void    Unitree_GO_Enable(Unitree_GO_HandleTypeDef *motor);         // 切到 FOC(1)
void    Unitree_GO_Lock(Unitree_GO_HandleTypeDef *motor);           // 切到 锁定(0)
void    Unitree_GO_Calibrate(Unitree_GO_HandleTypeDef *motor);      // 编码器标定(2)，总线只能挂一个

/* 控制目标（关节侧单位，内部按减速比折算到转子侧） */
void    Unitree_GO_MITMode(Unitree_GO_HandleTypeDef *motor,
                           float joint_pos, float joint_speed,
                           float kp, float kd, float joint_torque);
void    Unitree_GO_PositionMode(Unitree_GO_HandleTypeDef *motor,
                                float joint_pos, float kp, float kd);
void    Unitree_GO_SpeedMode(Unitree_GO_HandleTypeDef *motor,
                             float joint_speed, float kd);
void    Unitree_GO_TorqueMode(Unitree_GO_HandleTypeDef *motor, float joint_torque);
void    Unitree_GO_ZeroTorqueMode(Unitree_GO_HandleTypeDef *motor); // 零力矩（可自由拖动）

/* 控制目标（转子侧单位，与手册/协议一一对应，调试或标定时用） */
void    Unitree_GO_MITModeRaw(Unitree_GO_HandleTypeDef *motor,
                              float pos, float speed, float kp, float kd, float torque);

/* 位置零点：把当前转子位置记成关节 0 点（之后 joint_pos 从 0 开始） */
void    Unitree_GO_SetZeroPosition(Unitree_GO_HandleTypeDef *motor);

/* 通讯：发一帧命令并等一帧反馈，返回 1 表示这一轮收发+校验成功 */
uint8_t Unitree_GO_SendRecv(Unitree_GO_HandleTypeDef *motor);
/* 只发不收（广播、标定命令用），返回 1 表示发送成功 */
uint8_t Unitree_GO_SendOnly(Unitree_GO_HandleTypeDef *motor);
/* 解析一帧 16 字节反馈（如果以后改成 DMA/中断接收，在回调里调这个） */
uint8_t Unitree_GO_ParseData(Unitree_GO_HandleTypeDef *motor, const uint8_t *data);

uint8_t Unitree_GO_IsOnline(const Unitree_GO_HandleTypeDef *motor);

/* CRC16-CCITT（反射，init=0，多项式 0x1021），协议收发都用它 */
uint16_t Unitree_GO_Crc16(const uint8_t *data, uint16_t len);

/*==============================================================================
 * 五、用法示例
 *-------------------------------------------------------------------------------
 *  // main.c 里 usart.c 已经跑过 MX_USART2_UART_Init()（4Mbps 8N1），
 *  // 任务/初始化里只要建句柄（第 5 个参数是电机 ID，必须和电机里烧的一致！
 *  // 用宇树 MotorTools 的 swboot 可以看，出厂未设置时是 15，本项目当前那台是 1）：
 *  Unitree_GO_HandleTypeDef go_joint;
 *  Unitree_GO_Init(&go_joint, &UNITREE_GO_485_UART,
 *                  UNITREE_GO_DE_PORT, UNITREE_GO_DE_PIN, 1, 6.33f);
 *
 *  // 控制循环里（建议周期 >= 2ms，一轮收发 2 个电机约 0.6ms）：
 *  Unitree_GO_MITMode(&go_joint, 1.57f, 0.0f, 20.0f, 0.6f, 0.0f);  // 关节 1.57rad, kp=20, kd=0.6
 *  Unitree_GO_SendRecv(&go_joint);
 *
 *  // 注意 kp/kd 的口径：默认是"关节侧"刚度/阻尼（1 = 1 N·m/rad），常用 2~20；
 *  // 手册/R2 示例里 0.02~0.2 那种是转子侧系数，要么乘 40 再传，要么把
 *  // UNITREE_GO_GAIN_IN_JOINT_SPACE 改成 0 直接原样下发。
 *  // 口径对不上时电机表现为"完全不转"（系数太小被定点量化掉了）。
 *
 *  // 同一路 485 上挂多个电机时，同一个任务里按 ID 依次 MITMode+SendRecv 即可，
 *  // 不要在两个任务里同时发（半双工总线，会互相踩包）。
 *
 *  // 读结果：
 *  //   go_joint.joint_pos / joint_speed / joint_torque  关节侧反馈
 *  //   go_joint.temperature / error_code / correct      状态
 *  //   go_joint.timeout_cnt / crc_err_cnt               通讯质量
 *============================================================================*/

#endif
