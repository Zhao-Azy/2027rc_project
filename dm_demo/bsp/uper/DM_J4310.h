#ifndef DM_J4310_H
#define DM_J4310_H

#include "main.h"
#include "drv_can.h"

/*==============================================================================
 * 达妙 DM-J4310-2EC 关节电机驱动（CAN 1Mbps，标准帧，单文件封装）
 *-------------------------------------------------------------------------------
 * 协议来源：《调试助手使用说明书（达妙驱动控制协议）V1.4》第四章 CAN 通信。
 * 写法对齐 bsp/uper/Unitree_GO.c/.h（同样的"句柄 + 模式函数 + 反馈解析 + 在线判定"），
 * 唯一的区别在物理层：
 *   宇树 GO 是 485 半双工，一问一答（发一帧、等一帧）；
 *   达妙是 CAN 总线，控制帧主动发、反馈帧在 CAN 接收中断里按 Master ID 解析
 *   （和本工程 RS00 / DJI_C620 的用法一样）。
 *
 * ---------------------------------------------------------------------------
 * 一、报文速查（每帧 8 字节，标准帧；数据段里的多字节一律"低字节在前"）
 * ---------------------------------------------------------------------------
 * ① 控制帧（主机 -> 电机），MIT 模式，帧 ID = CAN_ID
 *      D[0] = p_des[15:8]                  D[1] = p_des[7:0]
 *      D[2] = v_des[11:4]                  D[3] = (v_des[3:0] << 4) | Kp[11:8]
 *      D[4] = Kp[7:0]                      D[5] = Kd[11:4]
 *      D[6] = (Kd[3:0] << 4) | t_ff[11:8]  D[7] = t_ff[7:0]
 *
 * ② 特殊命令帧（帧 ID 同样是 CAN_ID，前 7 字节全 0xFF，只有 D[7] 不同）
 *      FF FF FF FF FF FF FF FC   使能
 *      FF FF FF FF FF FF FF FD   失能（上电默认状态）
 *      FF FF FF FF FF FF FF FE   保存位置零点（会写驱动器 Flash）
 *      FF FF FF FF FF FF FF FB   清除错误
 *
 * ③ 位置速度模式（PV），帧 ID = 0x100 + CAN_ID
 *      D[0..3] = p_des (float)             D[4..7] = v_des (float，梯形加减速的匀速段)
 *      注意：这两个是真正的 float，不走 PMAX/VMAX 的定点映射，
 *            所以 PV 的目标位置可以超过 ±PMAX（多圈移动），只有反馈会被环回
 *
 * ④ 纯速度模式，帧 ID = 0x200 + CAN_ID，DLC = 4
 *      D[0..3] = v_des (float)
 *
 * ⑤ 反馈帧（电机 -> 主机），三种模式格式相同，帧 ID = Master ID
 *      D[0] = ID(bit0~3) | ERR(bit4~7)
 *      D[1] = POS[15:8]    D[2] = POS[7:0]
 *      D[3] = VEL[11:4]    D[4] = (VEL[3:0] << 4) | T[11:8]    D[5] = T[7:0]
 *      D[6] = T_MOS（MOS 平均温度 ℃）     D[7] = T_Rotor（线圈平均温度 ℃）
 *
 * ---------------------------------------------------------------------------
 * 二、定点映射（手册 4.2 的 float_to_uint / uint_to_float）
 * ---------------------------------------------------------------------------
 *      POS  16 位：[-PMAX, +PMAX]      VEL 12 位：[-VMAX, +VMAX]
 *      T    12 位：[-TMAX, +TMAX]      Kp  12 位：[0, 500]      Kd 12 位：[0, 5]
 *
 *  ⚠ PMAX / VMAX / TMAX 是驱动器"参数设置 → 控制幅值"里的三个数，必须和本驱动一致，
 *    否则命令被静默等比缩放（现象：走不到位 / 转速对不上 / 力矩偏小，但一切"正常"、
 *    不报任何错，最难查）。J4310-2EC 出厂值：PMAX = 12.5 rad，VMAX = 30 rad/s，
 *    TMAX = 10 N·m；换电机、刷固件、改过幅值之后请先"读参数"再改下面的宏。
 *    另外 KT_OUT 应为 0、齿轮系数应为 1，否则力矩反馈口径也会变。
 *
 * ---------------------------------------------------------------------------
 * 三、单位口径（和宇树 GO 驱动最大的区别，别照抄那边的换算）
 * ---------------------------------------------------------------------------
 *  达妙驱动器参数里带"减速比 GR"，位置 / 速度 / 力矩的收发都是
 *  【减速器输出轴 = 关节侧】的量：
 *      pos    关节角      rad
 *      vel    关节角速度  rad/s
 *      torque 关节力矩    N·m
 *  所以本驱动内部【不做】ratio 换算。宇树 GO 那套 "Pos ×ratio、W ×ratio、T ÷ratio"
 *  在这里是错的：转发值会大 10 倍，力矩会小 10 倍，一上电就飞。
 *  ratio 字段只是记下来，方便上层自己算转子侧的量（轮速、反电动势之类）。
 *
 * ---------------------------------------------------------------------------
 * 四、CAN 通道（DM 固定 1Mbps 经典帧，不能用 CANFD 的波特率切换）
 * ---------------------------------------------------------------------------
 *      hfdcan1：C620 四轮底盘（1Mbps）—— 要和 DM 共用总线时自己保证 ID 不打架
 *      hfdcan2：空（1Mbps）—— 建议 DM 挂这里，本工程示例都用 hfdcan2
 *      hfdcan3：HI14 IMU（500kbps）—— 绝对不能挂 DM
 *
 *  初始化顺序（少一步都不通）：
 *      MX_FDCANx_Init()  →  FDCAN_Init(&hfdcanx)   [drv_can.c：全局滤波 + 开 RX 中断 + Start]
 *      →  DM_J4310_Init()  →  DM_J4310_Enable()
 *============================================================================*/

/*==============================================================================
 * 0、CAN 类型抽象
 *-------------------------------------------------------------------------------
 * H7 是 FDCAN（HAL_FDCAN_xxx），F4 是 CAN（HAL_CAN_xxx），两套 API 完全不兼容
 * （手册 8.2 节里专门点过这个坑）。这里用两个别名把差异收在头文件里，
 * C 文件再用 #ifdef __FDCAN_H__ 分开写"发一帧 / 收一帧"那几行，
 * 业务代码（模式函数、解析、裁剪）两边共用一份。
 *============================================================================*/
#ifdef __FDCAN_H__
typedef FDCAN_HandleTypeDef    DM_CAN_HandleTypeDef;
typedef FDCAN_RxHeaderTypeDef  DM_CAN_RxHeaderTypeDef;
#elif defined __CAN_H__
typedef CAN_HandleTypeDef      DM_CAN_HandleTypeDef;
typedef CAN_RxHeaderTypeDef    DM_CAN_RxHeaderTypeDef;
#else
#error "DM_J4310.h 需要先包含 fdcan.h(FDCAN) 或 can.h(CAN)，请通过 drv_can.h 引入"
#endif

/*==============================================================================
 * 一、协议常量
 *============================================================================*/
#define DM_J4310_FRAME_LEN          8u      // 一帧固定 8 字节（纯速度模式发 4 字节）
#define DM_J4310_ID_PV_OFFSET       0x100u  // 位置速度模式：帧 ID = 0x100 + CAN_ID
#define DM_J4310_ID_VEL_OFFSET      0x200u  // 纯速度模式：  帧 ID = 0x200 + CAN_ID

/* 特殊命令码，放在 D[7]，前 7 字节全 0xFF
 * （手册 4.8“清除错误”那张表排版漏了一列 D[2]，实际和另外三条一样是 7 个 0xFF + 命令字） */
#define DM_J4310_CMD_ENABLE         0xFCu   // 使能
#define DM_J4310_CMD_DISABLE        0xFDu   // 失能（上电默认）
#define DM_J4310_CMD_SAVE_ZERO      0xFEu   // 保存位置零点（写 Flash）
#define DM_J4310_CMD_CLEAR_ERROR    0xFBu   // 清除错误

/* 控制幅值默认值（J4310-2EC 出厂值，必须和驱动器里读出来的一致） */
#define DM_J4310_DEFAULT_PMAX       12.5f   // rad
#define DM_J4310_DEFAULT_VMAX       30.0f   // rad/s
#define DM_J4310_DEFAULT_TMAX       10.0f   // N·m

/* Kp / Kd 幅值（手册固定值，改不了） */
#define DM_J4310_KP_MAX             500.0f  // N·m/rad
#define DM_J4310_KD_MAX             5.0f    // N·m·s/rad

/* 机械参数与默认通讯参数 */
#define DM_J4310_DEFAULT_RATIO      10.0f   // J4310-2EC 减速比 10:1（仅记录，协议已折算）
#define DM_J4310_DEFAULT_CAN_ID     0x01u   // 驱动器里一般烧成 0x01（以实际读出为准）
#define DM_J4310_DEFAULT_MASTER_ID  0x00u   // 出厂值；建议在调试助手里改成 CAN_ID + 0x10

/* PV 模式位置给定是 float，不受 PMAX 映射限制（多圈也能给），
 * 这里只留一层兜底限幅，防止上位机算飞了给出一个天文数字（1000 rad ≈ 159 圈） */
#define DM_J4310_PV_POS_LIMIT       1000.0f

/* 判定掉线的时间窗（ms）：驱动器默认 CAN Timeout 是 50us × N 个计数，
 * 正常控制周期 2~10ms，超过 100ms 没有一帧反馈基本就是线掉了 / 电机没上电。 */
#define DM_J4310_ONLINE_TIMEOUT     100u

/*==============================================================================
 * 二、工作模式（本驱动本地状态，决定 DM_J4310_Send() 组哪一帧）
 *============================================================================*/
typedef enum
{
    eDM_J4310_MODE_MIT = 0,             // MIT 模式：帧 ID = CAN_ID（8 字节定点）
    eDM_J4310_MODE_PV  = 1,             // 位置速度模式：帧 ID = 0x100 + CAN_ID（2 个 float）
    eDM_J4310_MODE_VEL = 2,             // 纯速度模式：  帧 ID = 0x200 + CAN_ID（1 个 float）
} DM_J4310_Mode_e;

/*==============================================================================
 * 三、反馈帧状态码 ERR（手册 4.1）
 *============================================================================*/
typedef enum
{
    eDM_J4310_ERR_DISABLED      = 0x0,  // 失能
    eDM_J4310_ERR_ENABLED       = 0x1,  // 使能（正常工作的唯一取值）
    eDM_J4310_ERR_OVER_VOLTAGE  = 0x8,  // 超压
    eDM_J4310_ERR_UNDER_VOLTAGE = 0x9,  // 欠压
    eDM_J4310_ERR_OVER_CURRENT  = 0xA,  // 过电流
    eDM_J4310_ERR_MOS_OVER_TEMP = 0xB,  // MOS 过温
    eDM_J4310_ERR_COIL_OVER_TEMP= 0xC,  // 电机线圈过温
    eDM_J4310_ERR_LOST_COMM     = 0xD,  // 通讯丢失（CAN Timeout 触发）
    eDM_J4310_ERR_OVERLOAD      = 0xE,  // 过载
} DM_J4310_Err_e;

/*==============================================================================
 * 四、句柄
 *-------------------------------------------------------------------------------
 * 指令字段（target_pos / target_vel / target_torque / kp / kd）是关节侧单位，
 * 与协议一一对应；反馈字段 pos / vel / torque 也是关节侧，
 * real_pos 是过零修正后多圈累加的连续角度（MIT 的位置在 ±PMAX 处会翻转）。
 *============================================================================*/
typedef struct
{
    DM_CAN_HandleTypeDef *hcan;         // 挂在哪路 CAN（hfdcan1 / hfdcan2）
    uint16_t              can_id;       // 控制帧 ID = 驱动器参数 CAN_ID
    uint16_t              master_id;    // 反馈帧 ID = 驱动器参数 Master ID
    float                 ratio;        // 减速比（仅记录，协议收发已是关节侧，不参与换算）

    /* ---- 控制幅值：必须与驱动器 PMAX / VMAX / TMAX 一致 ---- */
    float                 p_max;        // rad
    float                 v_max;        // rad/s
    float                 t_max;        // N·m

    /* ---- 下发给电机的指令（关节侧） ---- */
    uint8_t               mode;         // DM_J4310_Mode_e
    uint8_t               enabled;      // 本地使能标记：0 时 DM_J4310_Send() 不发控制帧
    float                 target_pos;   // rad
    float                 target_vel;   // rad/s
    float                 target_torque;// N·m
    float                 kp;           // 0 ~ 500 N·m/rad
    float                 kd;           // 0 ~ 5   N·m·s/rad

    /* ---- 反馈（关节侧） ---- */
    uint8_t               id;           // 反馈帧里的 ID（= CAN_ID 低 4 位）
    uint8_t               err;          // DM_J4310_Err_e
    uint16_t              pos_raw;      // 原始定点 16 位
    uint16_t              vel_raw;      // 原始定点 12 位
    uint16_t              tau_raw;      // 原始定点 12 位
    uint8_t               t_mos;        // MOS 平均温度 ℃（0~255，超 127 也不会变负）
    uint8_t               t_rotor;      // 线圈平均温度 ℃
    float                 pos;          // 当前角度 rad（±p_max 内，过零会翻转）
    float                 vel;          // 角速度 rad/s
    float                 torque;       // 力矩 N·m
    float                 last_pos;     // 上一帧 pos，用于多圈累加
    float                 delta_pos;    // 本帧增量（已做过零修正）
    float                 real_pos;     // 多圈累加后的连续角度 rad

    /* ---- 状态 / 诊断 ---- */
    uint8_t               correct;      // 最近一帧反馈是否有效（1 有效）
    uint8_t               online;       // 在线标志（随时间窗自动更新）
    uint8_t               zero_pending; // 刚发过"保存零点"，下一帧反馈重新对齐 real_pos
    uint32_t              ok_cnt;       // 收到的有效反馈帧数
    uint32_t              bad_cnt;      // ID / 长度 / 内容不对，被丢掉的帧数
    uint32_t              skip_cnt;     // 未使能时被跳过的控制帧数
    uint32_t              tx_full_cnt;  // 发送 FIFO 满、这一帧没发出去
    uint32_t              last_rx_tick; // 最近一次收到有效反馈的 tick
    uint32_t              last_tx_tick; // 最近一次发送成功的 tick

    uint8_t               last_tx_buf[DM_J4310_FRAME_LEN];  // 最近一帧发出去的内容（调试用）
} DM_J4310_HandleTypeDef;

/*==============================================================================
 * 五、接口
 *============================================================================*/

/**
 * @brief  初始化一个 DM-J4310-2EC 句柄
 * @param  motor      句柄
 * @param  hcan       该电机挂的 CAN（必须已经 FDCAN_Init/CAN 启动、1Mbps）
 * @param  can_id     驱动器里的 CAN_ID（控制帧用）
 * @param  master_id  驱动器里的 Master ID（反馈帧用；出厂 0x00，建议改成 can_id + 0x10）
 * @param  ratio      减速比，J4310-2EC = 10.0f（仅记录，协议已是关节侧）
 * @note   只做本地初始化，不使能、不发任何帧（安全：上电零输出）。
 *         幅值先按 J4310-2EC 出厂值填，驱动器里改过请再调 DM_J4310_SetLimit()。
 */
void    DM_J4310_Init(DM_J4310_HandleTypeDef *motor,
                      DM_CAN_HandleTypeDef *hcan,
                      uint16_t can_id, uint16_t master_id, float ratio);

/**
 * @brief  改控制幅值（必须与驱动器"控制幅值"页里的 PMAX/VMAX/TMAX 一致）
 * @param  p_max 位置幅值 rad（如 12.5）
 * @param  v_max 速度幅值 rad/s（如 30.0）
 * @param  t_max 力矩幅值 N·m（如 10.0）
 */
void    DM_J4310_SetLimit(DM_J4310_HandleTypeDef *motor,
                          float p_max, float v_max, float t_max);

/* ---- 使能 / 失能 / 清错 / 存零点：这四个是"立即发一帧"，返回 1 表示发出去了 ---- */
/**
 * @brief  使能：发一帧使能命令，之后控制函数才会真的往总线上发控制帧
 * @note   只负责"通电"，不改已写好的指令。上电时指令是 0，所以第一次使能输出 0；
 *         若要一使能就停在某个位置，先 DM_J4310_PositionMode(当前位置, kp, kd) 再使能。
 */
uint8_t DM_J4310_Enable(DM_J4310_HandleTypeDef *motor);            // 使能（绿灯亮）
uint8_t DM_J4310_Disable(DM_J4310_HandleTypeDef *motor);           // 失能（指令清零，默认状态）
uint8_t DM_J4310_ClearError(DM_J4310_HandleTypeDef *motor);        // 清除过温等错误
uint8_t DM_J4310_SaveZeroPosition(DM_J4310_HandleTypeDef *motor);  // 把当前位置存成零点（写 Flash）

/* ---- 控制目标（关节侧单位；每个函数内部就把这一帧发出去了，不用再手动 Send） ---- */
/**
 * @brief  MIT 力位混合模式
 * @param  pos    目标位置 rad（关节侧，±PMAX 内）
 * @param  vel    目标速度 rad/s（一般给 0，做速度前馈时才给）
 * @param  kp     位置刚度 N·m/rad（0~500）
 * @param  kd     速度阻尼 N·m·s/rad（0~5，纯位置控制时别给 0）
 * @param  torque 前馈力矩 N·m
 * @return 1 = 已放进发送 FIFO；0 = 没发（未使能 / 句柄空 / FIFO 满）
 */
uint8_t DM_J4310_MITMode(DM_J4310_HandleTypeDef *motor,
                         float pos, float vel, float kp, float kd, float torque);
uint8_t DM_J4310_PositionMode(DM_J4310_HandleTypeDef *motor, float pos, float kp, float kd);
uint8_t DM_J4310_SpeedMode(DM_J4310_HandleTypeDef *motor, float vel, float kd);
uint8_t DM_J4310_TorqueMode(DM_J4310_HandleTypeDef *motor, float torque);
uint8_t DM_J4310_ZeroTorqueMode(DM_J4310_HandleTypeDef *motor);    // 零力矩（可自由拖动）
uint8_t DM_J4310_PVMode(DM_J4310_HandleTypeDef *motor, float pos, float vel);   // 位置速度模式
uint8_t DM_J4310_VelMode(DM_J4310_HandleTypeDef *motor, float vel);             // 纯速度模式

/* ---- 通讯 ---- */
/**
 * @brief  按句柄里当前的目标和 mode 重发一帧
 * @return 1 = 已放进发送 FIFO；0 = 没发（未使能 / 句柄空 / FIFO 满）
 * @note   控制函数内部已经调用过它了，平时不用自己调；
 *         用于"按上次目标补一帧"（心跳保持、刚使能完补一帧）。
 *         控制周期建议 2~10ms，间隔超过驱动器 CAN Timeout 会报"通讯丢失"。
 */
uint8_t DM_J4310_Send(DM_J4310_HandleTypeDef *motor);

/* 只发某一帧（一般不直接用，调试 / 混用模式时用） */
uint8_t DM_J4310_SendMIT(DM_J4310_HandleTypeDef *motor);
uint8_t DM_J4310_SendPV( DM_J4310_HandleTypeDef *motor);
uint8_t DM_J4310_SendVel(DM_J4310_HandleTypeDef *motor);

/**
 * @brief  在 CAN 接收中断里解析一帧反馈（HAL_FDCAN_RxFifo0Callback 里调用）
 * @param  motor     句柄
 * @param  rx_header 接收头（用来核对标准帧 + Master ID）
 * @param  data      8 字节数据
 * @return 1 = 是本电机的帧且解析成功；0 = 不是 / 被丢弃
 * @note   中断里只做解析，不要 printf / osDelay / 等锁。
 */
uint8_t DM_J4310_GetFeedback(DM_J4310_HandleTypeDef *motor,
                             DM_CAN_RxHeaderTypeDef *rx_header, const uint8_t *data);

/* 只解析 8 字节数据（已经确认是本机帧时用；也方便离线拿日志回放） */
uint8_t DM_J4310_ParseFeedback(DM_J4310_HandleTypeDef *motor, const uint8_t *data);

/* 在线判定：DM_J4310_ONLINE_TIMEOUT 内收到过有效反馈就是在线 */
uint8_t DM_J4310_IsOnline(DM_J4310_HandleTypeDef *motor);

/* 状态码转文字，串口打印用（返回的是常量字符串，不占运行期内存） */
const char *DM_J4310_ErrName(uint8_t err);

/*==============================================================================
 * 六、用法（照抄即可跑）
 *-------------------------------------------------------------------------------
 *  // 1) 初始化：main.c 的 USER CODE 2 段，或者任务开头（建一次就够）
 *  DM_J4310_HandleTypeDef dm_joint;
 *  FDCAN_Init(&hfdcan2);                                        // 全局滤波+开中断+Start
 *  DM_J4310_Init(&dm_joint, &hfdcan2, 0x01u, 0x00u, 10.0f);     // CAN_ID / MST_ID / 减速比
 *  //  MST_ID 出厂是 0x00（此时靠反馈数据里的 ID 区分电机）；
 *  //  在调试助手里改成 CAN_ID + 0x10 之后，这里跟着改，多机同总线更稳。
 *
 *  // 同一路 CAN 上挂多台：每台一个句柄，初始化各来一次即可
 *  //   DM_J4310_Init(&dm_joint2, &hfdcan2, 0x02u, 0x00u, 10.0f);
 *
 *  // 2) 使能（上电默认失能；没使能时控制函数一帧都不会发，只在 skip_cnt 上记一笔）
 *  DM_J4310_Enable(&dm_joint);
 *  osDelay(20);                                                 // 等驱动器切到使能态
 *
 *  // 3) 控制循环（2~10ms 一次；一个函数 = 一帧，不需要再 Send）
 *  DM_J4310_MITMode(&dm_joint, 1.57f, 0.0f, 5.0f, 0.3f, 0.0f);  // 关节 1.57rad，Kp=5，Kd=0.3
 *
 *  // 4) 反馈：app/fdcan_it.c 的 HAL_FDCAN_RxFifo0Callback 里
 *  //      if (hfdcan == &hfdcan2)
 *  //      {
 *  //          DM_J4310_GetFeedback(&dm_joint,  &rx_header, rx_data);
 *  //          DM_J4310_GetFeedback(&dm_joint2, &rx_header, rx_data);
 *  //      }
 *
 *  // 5) 读结果（Live Watch 直接看句柄）
 *  //    dm_joint.real_pos / vel / torque      关节角(多圈连续) / 角速度 / 力矩
 *  //    dm_joint.err                          0 失能 1 使能 8 超压 9 欠压 A 过流
 *  //                                          B MOS 过温 C 线圈过温 D 通讯丢失 E 过载
 *  //    dm_joint.t_mos / t_rotor              MOS 温度 / 线圈温度（℃）
 *  //    DM_J4310_IsOnline(&dm_joint)          掉线判定
 *  //    dm_joint.ok_cnt / bad_cnt / skip_cnt / tx_full_cnt   通讯质量
 *
 *  // 排错表
 *  //   ok_cnt 不涨          -> CAN 线 / 没共地 / 不是 1Mbps / 忘了 FDCAN_Init(&hfdcan2) /
 *  //                            MST_ID 和驱动器里的对不上
 *  //   ok_cnt 涨但电机不动  -> err 不是 1（没使能或有错误）；PMAX/VMAX/TMAX 对不上；
 *  //                            MIT 里 Kp、Kd 都太小（Kd=0 时位置环几乎不响应）
 *  //   skip_cnt 一直涨      -> 忘了 DM_J4310_Enable()，或使能帧没发出去（看 tx_full_cnt）
 *  //   tx_full_cnt 涨       -> 发的太密（<1ms 一帧没必要），或者总线被别的设备堵住
 *  //   一使能就"飞"         -> 使能前把目标写成当前位置（或先 ZeroTorqueMode 再使能）
 *  //   位置在 ±12.5 处跳变  -> 正常，MIT 的位置是环回的，用 real_pos 才是连续角度
 *  //   想上电时这个位置算 0 -> 装配好后调一次 DM_J4310_SaveZeroPosition()（写 Flash）
 *============================================================================*/

#endif /* DM_J4310_H */

    