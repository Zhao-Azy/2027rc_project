#include "my_main.h"
#include "uper_control.h"
#include "chassis_control.h"
#include "chassis.h"
#include "fdcan_it.h"
#include "STP_23L.h"
#include "usart_it.h"
#include "iwdg.h"
#include <math.h>


static void MIT_Planning(MechanicalArm_HandleTypeDef *arm);
static void Steer_Homing_FSM(void);

//定时获取MG5010E位置
uint32_t MG5010E_tick = 0;
uint32_t HT3505_tick = 0;

uint8_t ready_index = 0;
uint32_t delay_tick = 0;
uint32_t wait_tick = 0;

// 单 3508 测试句柄（方案A：C620 电调直连，挂 hfdcan2）
DJI_C620_HandleTypeDef test_3508;

/* HI14 IMU 的数据结构与解析已下沉到 bsp 驱动：bsp/chassis/gyro_HI14.c
 * 这里只保留"按 CAN ID 统计"这一项 app 层诊断用的表。 */

/* 收到的 CAN 帧按 ID 统计（诊断用） */
HI14_IdStat hi14_ids[HI14_ID_MAX] = {0};
uint8_t hi14_id_num = 0;

static void HI14_RecordId(FDCAN_RxHeaderTypeDef *hdr, const uint8_t *d)
{
	for (uint8_t i = 0; i < hi14_id_num; i++)
	{
		if (hi14_ids[i].id == hdr->Identifier && hi14_ids[i].idtype == (uint8_t)hdr->IdType)
		{
			hi14_ids[i].cnt++;
			return;
		}
	}
	if (hi14_id_num < HI14_ID_MAX)
	{
		hi14_ids[hi14_id_num].id     = hdr->Identifier;
		hi14_ids[hi14_id_num].idtype = (uint8_t)hdr->IdType;
		hi14_ids[hi14_id_num].dlc    = (uint8_t)hdr->DataLength;
		hi14_ids[hi14_id_num].cnt    = 1u;
		for (uint8_t k = 0; k < 8u; k++)
		{
			hi14_ids[hi14_id_num].d[k] = d[k];
		}
		hi14_id_num++;
	}
}


/* ---------- HI14 波特率自动扫描 ----------
 * PSR 显示 Form Error、错误计数器打满 = 总线有活动但解不出帧，最常见原因就是波特率不匹配。
 * HI14 的 CAN 速率可配（1000/800/500/250/125 kbps，出厂默认 500k），这里每 2 秒收不到帧
 * 就切到下一档，一旦收到帧就自动锁定不再切换。FDCAN 内核时钟 = 120MHz。
 *   kbps = 120MHz / (Prescaler × (1 + TimeSeg1 + TimeSeg2))                        */
typedef struct
{
	uint16_t kbps;
	uint8_t  psc;
	uint8_t  seg1;
	uint8_t  seg2;
} HI14_RateTypeDef;

static const HI14_RateTypeDef hi14_rates[] =
{
	{1000, 3,  29, 10},   // 120M/(3×40)
	{ 800, 3,  39, 10},   // 120M/(3×50)
	{ 500, 3,  69, 10},   // 120M/(3×80)
	{ 250, 3, 139, 20},   // 120M/(3×160)
	{ 125, 6, 139, 20},   // 120M/(6×160)
};

#define HI14_RATE_SCAN   0   // 1=波特率自动扫描；0=固定用 MX_FDCAN3_Init 设的速率（实测 500k 正确）

/* 是否用 HI14 IMU 航向给底盘做"锁头"（yaw 环）：
 *   0 = 不启用 —— 完全保持原来的驾驶手感（遥控器直接给平移+旋转）
 *   1 = 启用   —— 上电时锁定当前朝向，之后 pid_w 维持航向，遥控器只管平移 */
#define CHASSIS_YAW_HOLD   1
/* 符号：HI14 驱动的航向已经在 bsp 层统一成"逆时针为正"（与原 MINS500 一致），
 * 所以这里用 +1，yaw 环和坐标变换同时正确。若以后换传感器导致方向反了再改。 */
#define CHASSIS_YAW_SIGN   (+1.0f)

/* yaw 环整定开关：1 = 目标航向在 ±YAW_TUNE_STEP_DEG 之间方波切换（阶跃测试），
 *                 同时把 target/actual/err/Vw/gyroZ 以 50Hz 从串口输出，供采数拟合；
 *                 0 = 正常锁头（恒定目标）。整定已完成，置 0。 */
#define YAW_TUNE            0
#define YAW_TUNE_STEP_DEG   20.0f    // 方波幅值（±20°，车会来回转，注意留空间）
#define YAW_TUNE_HALF_MS    3000u    // 半周期 3s（留出足够时间看完整阶跃响应）

/* 方向选择：用遥控器旋钮 CH[4] 选 4 个方向（基准 = 上电时锁定的朝向）
 *   CH[4]: 200~600 → 初始方向   600~1000 → 顺时针 90°
 *         1000~1400 → 180°     1400~1800 → 顺时针 270°
 * 边界带滞回，旋钮停在边界附近时不会来回跳档。 */
#define YAW_DIR_SEL         1        // 1=启用旋钮选方向；0=只锁上电方向
/* 驱动已统一成"逆时针为正"，所以"顺时针转 90°"= 目标航向减 90° ⇒ 取 -1 */
#define YAW_DIR_CW_SIGN     (-1.0f)  // 若旋钮档实际转向相反，改成 (+1.0f)
#define YAW_DIR_HYST        20u      // 档位边界滞回（SBUS 原始值）

/* yaw 环遥测（类型声明在 fdcan_it.h，供 app/chassis_control.c 打印） */
YawTel_TypeDef yaw_tel = {0};

#define HI14_RATE_NUM   (sizeof(hi14_rates) / sizeof(hi14_rates[0]))

uint32_t hi14_bitrate_kbps = 500;   // 当前正在尝试/已锁定的速率（供 Vofa 观察）
static uint32_t hi14_rate_idx      = 2;   // 从 500k 开始（手册出厂默认）
static uint32_t hi14_scan_tick     = 0;
static uint32_t hi14_scan_frames   = 0;

static void HI14_ApplyRate(uint32_t idx)
{
	HAL_FDCAN_DeInit(&hfdcan3);   // 回到 RESET，使下次 Init 重走 MspInit（时钟/引脚/中断）
	hfdcan3.Init.NominalPrescaler     = hi14_rates[idx].psc;
	hfdcan3.Init.NominalSyncJumpWidth = hi14_rates[idx].seg2;   // SJW ≤ TimeSeg2
	hfdcan3.Init.NominalTimeSeg1      = hi14_rates[idx].seg1;
	hfdcan3.Init.NominalTimeSeg2      = hi14_rates[idx].seg2;
	if (HAL_FDCAN_Init(&hfdcan3) == HAL_OK)
	{
		FDCAN_Init(&hfdcan3);   // 全局滤波 + RX FIFO0 中断 + Start
	}
	hi14_bitrate_kbps = hi14_rates[idx].kbps;
}

/**
 * @brief 波特率自动扫描：2 秒内没有新帧就切下一档；收到帧后自动锁定
 */
void HI14_BitrateScan(void)
{
	if (hi14.frame_count != hi14_scan_frames)   // 有新帧 → 当前速率正确，锁定
	{
		hi14_scan_frames = hi14.frame_count;
		hi14_scan_tick   = HAL_GetTick();
		return;
	}

	if ((HAL_GetTick() - hi14_scan_tick) < 2000u)
		return;

	hi14_rate_idx = (hi14_rate_idx + 1u) % HI14_RATE_NUM;
	HI14_ApplyRate(hi14_rate_idx);
	hi14_scan_tick = HAL_GetTick();
}

/**
 * @brief:FDCAN中断回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

	if(hfdcan == &hfdcan1)
	{
//		VESC_Data_Process(&wheelLF, &rx_header, rx_data);
//		VESC_Data_Process(&wheelRF, &rx_header, rx_data);
//		VESC_Data_Process(&wheelRB, &rx_header, rx_data);
//		VESC_Data_Process(&wheelLB, &rx_header, rx_data);

//		DJI_Get_Meature(&LF_steer, &rx_header, rx_data);
//		DJI_Get_Meature(&RF_steer, &rx_header, rx_data);
 //		DJI_Get_Meature(&RB_steer, &rx_header, rx_data);
//		DJI_Get_Meature(&LB_steer, &rx_header, rx_data);
		
		DJI_C620_GetFeedback(&wheelLF, &rx_header, rx_data);
		DJI_C620_GetFeedback(&wheelRF, &rx_header, rx_data);
		DJI_C620_GetFeedback(&wheelRB, &rx_header, rx_data);
		DJI_C620_GetFeedback(&wheelLB, &rx_header, rx_data);

	}else if(hfdcan == &hfdcan2)
	{
		DJI_C620_GetFeedback(&test_3508, &rx_header, rx_data);   // 单 3508 反馈解析

//		//左臂
//		MG5010E_GetPosition(&Left_arm.Down_motor, &rx_header, rx_data);
//		RS00_Get_MotorData(&Left_arm.High_motor, &rx_header, rx_data);
//		RS05_Get_MotorData(&Left_arm.Sucker_motor, &rx_header, rx_data);

//		//右臂
//		MG5010E_GetPosition(&Right_arm.Down_motor, &rx_header, rx_data);
//		RS00_Get_MotorData(&Right_arm.High_motor, &rx_header, rx_data);
//		RS05_Get_Mo torData(&Right_arm.Sucker_motor, &rx_header, rx_data);
	}else if(hfdcan == &hfdcan3)
	{
		HI14_RecordId(&rx_header, rx_data);                        // 诊断：按 CAN ID 统计
		HI14_CanRxProcess(&rx_header, rx_data, &MINS500_data);     // bsp 驱动：解析 + 航向累计（原 MINS500 逻辑）

//		RS00_Get_MotorData(&Spin_clow, &rx_header, rx_data);
//		RS05_Get_MotorData(&Translation_motor, &rx_header, rx_data);
//		DJI_Get_Meature(&Weapon_flex, &rx_header, rx_data);
//		DJI_Get_Meature(&Storage_motor, &rx_header, rx_data);
//		DJI_Get_Meature(&Left_flex, &rx_header, rx_data);
//		DJI_Get_Meature(&Right_flex, &rx_header, rx_data);
    }
}


//用于定时跑电机规划(and 定时给AK发指令 )
//void my_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim == &htim4)
//	{
//		//当检测到FDCAN总线因发送错误过多自动关闭时重启
//		if (hfdcan1.Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EP))
//		{
//			HAL_FDCAN_Stop(&hfdcan1);
//			FDCAN_Init(&hfdcan1);
//		}
//		if (hfdcan2.Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EP))
//		{
//			HAL_FDCAN_Stop(&hfdcan2);
//			FDCAN_Init(&hfdcan2);
//		}
//		if (hfdcan3.Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EP))
//		{
//			HAL_FDCAN_Stop(&hfdcan3);
//			FDCAN_Init(&hfdcan3);
//		}

//		//补充电机规划
//		/**left_arm**/
//		MIT_Planning(&Left_arm);
//		/**right_arm**/
//		MIT_Planning(&Right_arm);


//		//爪子旋转
//		if(Spin_T.start_flag == 1)
//		{
//			uper_motor_pos.spin_clow_pos = Pos_Planning(&Spin_T);
//			RS00_MITMode(&Spin_clow, uper_motor_pos.spin_clow_pos, 0.0f, 130.0f, 5.0f, 0.0f);

//			if(Spin_T.end_flag == 1)
//			{
//				Spin_T.start_flag = 0;
//			}
//		}

//		//定时获取左右臂大臂电机位置
//		if(Left_arm.Down_motor.hfdcan != NULL && Right_arm.Down_motor.hfdcan != NULL)
//		{
//			MG5010E_tick ++;
//			if(MG5010E_tick >= 100)
//			{
//				MG5010E_ReadPosition(&Left_arm.Down_motor);
//				MG5010E_ReadPosition(&Right_arm.Down_motor);
//				MG5010E_tick = 0;
//			}
//		}

//		//读取吸盘状态
//		if(stp23l_distance_left < L_DISTANCW_MIN)
//		{
//			Left_arm.sucker_state = 1;
//		}else
//		{
//			Left_arm.sucker_state = 0;
//		}

//		if(stp23l_distance_right < R_DISTANCE_MIN)
//		{
//			Right_arm.sucker_state = 1;
//		}else
//		{
//			Right_arm.sucker_state = 0;
//		}


//		//舵轮回零
//		if(steer_homing[0].steer->hcan != NULL && steer_homing[1].steer->hcan != NULL && steer_homing[2].steer->hcan != NULL && steer_homing[3].steer->hcan != NULL)
//		{
//			Steer_Homing_FSM();
//		}
//	}
//}


////电机MIT规划
//void MIT_Planning(MechanicalArm_HandleTypeDef *arm)
//{
//	//小臂
//	if(arm->High_T.start_flag == 1)
//	{
//		arm->high_pos = Pos_Planning(&arm->High_T);
//		RS00_MITMode(&arm->High_motor, arm->high_pos, 0.0f, 130.0f, 5.0f, 0.0f);

//		if(arm->High_T.end_flag == 1)
//		{
//			arm->High_T.start_flag = 0;
//		}
//	}


//	//吸盘
//	if(arm->Sucker_T.start_flag == 1)
//	{
//		arm->sucker_pos = Pos_Planning(&arm->Sucker_T);
//		RS05_MITMode(&arm->Sucker_motor, arm->sucker_pos, 0.0f, 100.0f, 5.0f, 0.0f);

//		if(arm->Sucker_T.end_flag == 1)
//		{
//			arm->Sucker_T.start_flag = 0;
//		}
//	}
//}


////舵轮回零
//void Steer_Homing_FSM(void)
//{
//	switch(steer_homing_state)
//	{
//		case eHOMING_IDLE:
//		{
//			//等待回零指令
//			ready_index = 0;
//			delay_tick = 0;
//			wait_tick = 0;
//			break;
//		}
//		case eHOMING_READY:
//		{
//			delay_tick ++;
//			if(delay_tick > 100)
//			{
//				if(ready_index < 4)
//				{
//					DJI_SpeedMode(steer_homing[ready_index].steer, 20.0f, 0);
//					ready_index++;
//					delay_tick = 0;
//				}
//				else
//				{
//					// 全部发送完毕，清空索引并跳转
//					ready_index = 0;
//					delay_tick = 0;
//					steer_homing_state = eHOMING_MOVING;
//				}
//			}
//			break;
//		}
//		case eHOMING_MOVING:
//		{		
//			for(uint8_t i = 0; i < 4; i++)
//			{
//				if(steer_homing[i].is_homed == 0)
//				{
//					if(HAL_GPIO_ReadPin(steer_homing[i].port, steer_homing[i].pin) == GPIO_PIN_SET)
//					{
//						DJI_SetOrigin(steer_homing[i].steer);
//						if(fabsf(steer_homing[i].steer->position) < 0.1f) // 位置接近零点
//						{
//							steer_homing[i].is_homed = 1;
//						}
//					}
//				}
//			}
//			if(steer_homing[0].is_homed && steer_homing[1].is_homed && steer_homing[2].is_homed && steer_homing[3].is_homed)
//			{
//				wait_tick ++;
//				if(wait_tick > 300)
//				{
//					steer_homing_state = eHOMING_DONE;
//					wait_tick = 0;
//				}
//			}
//			break;
//		}
//		case eHOMING_DONE:
//		{
//			break;
//		}
//	}

//}

/**
 * @brief FDCAN发送任务：底盘全向轮速度环控制（标准C620协议，直连电调）
 * @note  需在 freertos.c 中取消 osThreadNew(App_FDCAN, ...) 的注释才会运行
 */
void App_FDCAN(void *argument)
{
	static Chassis_Motor s_chassis_motor = {0};
	static uint32_t watchdog_tick = 0;
	DJI_C620_HandleTypeDef *wheel_motors[4] = {&wheelLF, &wheelRF, &wheelRB, &wheelLB};

	for(;;)
	{
		#if HI14_RATE_SCAN
		HI14_BitrateScan();   // HI14 IMU：波特率自动扫描（收到帧后自动停住）
		#endif

		if (xQueueReceive(Chassis_To_CanHandle, &s_chassis_motor, 1) != pdPASS)
		{
			osDelay(1);
			continue;
		}

		if (wheel_motors[0]->hcan == NULL || wheel_motors[1]->hcan == NULL ||
			wheel_motors[2]->hcan == NULL || wheel_motors[3]->hcan == NULL)
		{
			osDelay(1);
			continue;
		}

		// 更新四轮目标转速（rpm）
		wheelLF.target_speed = s_chassis_motor.wheel_speed[0];
		wheelRF.target_speed = s_chassis_motor.wheel_speed[1];
		wheelRB.target_speed = s_chassis_motor.wheel_speed[2];
		wheelLB.target_speed = s_chassis_motor.wheel_speed[3];

		// 速度环：目标转速 -> 目标电流
		for(uint8_t i = 0; i < 4; i++)
		{
			DJI_C620_SpeedControl(wheel_motors[i]);
		}

		// 打包发送 0x200 控制帧
		DJI_C620_SendCurrent(wheel_motors);
		if ((HAL_GetTick() - watchdog_tick) >= 100u)
		{
			HAL_IWDG_Refresh(&hiwdg1);
			watchdog_tick = HAL_GetTick();
		}

		osDelay(1);
	}
}


/* ================= 单 3508 测试（方案A：C620 直连） ================= */

// 单电机发送（现有 DJI_C620_SendCurrent 是一次打包 4 个电机，单测别直接用）
void C620_Single_Send(DJI_C620_HandleTypeDef *h)
{
    uint8_t data[8] = {0};
    uint8_t idx = (h->id - 1) * 2;                      // id=1 => 字节 0-1
    data[idx]     = (uint8_t)((h->target_current >> 8) & 0xFF);
    data[idx + 1] = (uint8_t)(h->target_current & 0xFF);

    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier          = 0x200;               // 控制帧固定 0x200
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = 8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;

    // 非阻塞：FIFO 满就跳过，避免废帧塞满 FIFO 后任务卡死、TEC 冻住看不清真实状态
    if (HAL_FDCAN_GetTxFifoFreeLevel(h->hcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(h->hcan, &TxHeader, data);
    }
}

// 单 3508 初始化：启动 FDCAN2 + 初始化电机
void Test_3508_Init(void)
{
    FDCAN_Init(&hfdcan2);                     // Start FDCAN2 + 全局滤波 + 使能 RX 中断
    DJI_C620_Init(&test_3508, &hfdcan2, 1);   // id=1 => 反馈帧 0x201
}

// 单 3508 测试任务
void Test_3508_Task(void *argument)
{
    for(;;)
    {
        test_3508.target_speed = 200.0f;      // 目标转速 rpm，先小值试转
        DJI_C620_SpeedControl(&test_3508);    // 速度环 -> 目标电流
        C620_Single_Send(&test_3508);
        osDelay(1);
    }
}

void Chassis_C620_Init(void)
{
	FDCAN_Init(&hfdcan1);
	DJI_C620_Init(&wheelLF, &hfdcan1, 1);
	DJI_C620_Init(&wheelRF, &hfdcan1, 2);
	DJI_C620_Init(&wheelRB, &hfdcan1, 3);
	DJI_C620_Init(&wheelLB, &hfdcan1, 4);

	FDCAN_Init(&hfdcan3);   // 启动 HI14 IMU 所在总线（CANopen/J1939 接收；原来只在未调用的 Upper_Init 里启动）
	HI14_Init();            // bsp 驱动：清零 IMU 状态（解析在 bsp/chassis/gyro_HI14.c）

	/* 整定 v7：RB 轮摩擦特别低，闭环轻阻尼、超调 ~20%——单独降 Kp（不影响其他三轮） */
	wheelRB.speed_pid.kp = 4.0f;
}

#define SBUS_AXIS_MIN       200
#define SBUS_AXIS_MAX       1800
#define SBUS_AXIS_CENTER    1000
#define SBUS_AXIS_DEADBAND  200
#define CHASSIS_INPUT_MAX   150.0f
/* 轮速比例：逆解算给出的是"轮子输出轴 rpm"，而 C620 用的是"电机转子 rpm"。
 * M3508 是 19:1 减速，故理论上应 ×19；实测手感按 5.65 偏慢（实际车速约为理论值的 30%）。   *
 * 换算：转子rpm = 轮速rpm × 本系数；轮子线速度 = (转子rpm / 19) / 60 × 2π × 0.075 m      *
 *   5.65 → 约 1.1 m/s（原值）    10.0 → 约 2.0 m/s（当前）    再往上加就接近电机上限(≈8900rpm) */
#define CHASSIS_SPEED_SCALE 10.0f

/* 快慢档：遥控器 CH[8] 两段开关
 *   200  → 慢档 GEAR_SLOW_SCALE（对接、九宫格放块等精细动作）
 *   1800 → 快档 GEAR_FAST_SCALE（跑位）                                           */
#define GEAR_FAST_SCALE     12.0f
#define GEAR_SLOW_SCALE     6.0f
#define GEAR_HYST           50u

static float SBUS_Axis(uint16_t value)
{
	int32_t offset = (int32_t)value - SBUS_AXIS_CENTER;
	if (offset > -SBUS_AXIS_DEADBAND && offset < SBUS_AXIS_DEADBAND)
		return 0.0f;
	if (value < SBUS_AXIS_MIN)
		value = SBUS_AXIS_MIN;
	if (value > SBUS_AXIS_MAX)
		value = SBUS_AXIS_MAX;
	return ((float)((int32_t)value - SBUS_AXIS_CENTER) / 800.0f) * CHASSIS_INPUT_MAX;
}

/**
 * @brief 旋钮 CH[4] → 方向档位偏移（度）
 * @note  档位：200~600 → 0°（初始方向）、600~1000 → 90°、1000~1400 → 180°、1400~1800 → 270°
 *        带滞回（YAW_DIR_HYST），旋钮停在边界附近不会来回跳档；上电时档位从 0 开始，
 *        若旋钮已停在别的档，会在几个控制周期内自动跳到对应档位。
 */
static float YAW_DirOffset(uint16_t ch)
{
	static uint8_t s_dir = 0;                         // 当前档位 0~3
	uint16_t up_th   = (uint16_t)(600u + 400u * s_dir);   // 升档阈值 600/1000/1400
	uint16_t down_th = (uint16_t)(200u + 400u * s_dir);   // 降档阈值 200/600/1000/1400

	if (s_dir < 3u && ch > (uint16_t)(up_th + YAW_DIR_HYST))
	{
		s_dir++;
	}
	else if (s_dir > 0u && ch < (uint16_t)(down_th - YAW_DIR_HYST))
	{
		s_dir--;
	}

	return (float)s_dir * 90.0f;
}

void App_SBUS_Chassis(void *argument)
{
	Chassis_Data sbus_chassis = {stop};
	Chassis_Motor motor = {0};
	static uint8_t s_gear_fast = 1u;   // 快慢档状态（CH[8]）
#if CHASSIS_YAW_HOLD
	static uint8_t s_yaw_ready = 0;   // IMU 航向可用后才开 yaw 环
	static float   s_yaw_base  = 0.0f;   // 上电锁定的基准朝向
#if YAW_TUNE
	static uint32_t s_tune_tick = 0;
	static uint8_t  s_tune_sign = 0;
#endif
#endif

	Chassis_Init(&sbus_chassis, 0.075f, 0.15f);
	Chassis_Control(&sbus_chassis, mid_speed, 0);

	for (;;)
	{
		if (sbus_failsafe_status != 0u || (HAL_GetTick() - sbus_last_tick) > 100u)
		{
			sbus_chassis.input_world_velocity.Vx = 0.0f;
			sbus_chassis.input_world_velocity.Vy = 0.0f;
			sbus_chassis.input_world_velocity.Vw = 0.0f;
			sbus_chassis.planning_velocity.Vx = 0.0f;
			sbus_chassis.planning_velocity.Vy = 0.0f;
			sbus_chassis.stop_flag = 1;
		}
		else
		{
			sbus_chassis.input_world_velocity.Vx = SBUS_Axis(CH[0]);
			sbus_chassis.input_world_velocity.Vy = SBUS_Axis(CH[1]);
			sbus_chassis.input_world_velocity.Vw = SBUS_Axis(CH[3]);
			sbus_chassis.stop_flag = 0;
		}

		Velocity_Planning(&sbus_chassis);
#if CHASSIS_YAW_HOLD
		/* 用驱动算好的航向锁头：MINS500_data 由 bsp/chassis/gyro_HI14.c 按原 MINS500 逻辑填充
		 *  —— real_yaw_1 是"上电从 180° 起算"的连续航向，first_flag=1 表示数据有效 */
		if (MINS500_data.first_flag != 0u)
		{
			if (s_yaw_ready == 0u)
			{
				s_yaw_base = MINS500_data.real_yaw_1 * CHASSIS_YAW_SIGN;   // 记录上电朝向
				sbus_chassis.target_yaw = s_yaw_base;
				s_yaw_ready = 1u;
			}
#if YAW_TUNE
			/* 整定用：目标航向在 基准±STEP 之间方波切换，采完数据后把 YAW_TUNE 改回 0 */
			if ((HAL_GetTick() - s_tune_tick) >= YAW_TUNE_HALF_MS)
			{
				s_tune_tick = HAL_GetTick();
				s_tune_sign = (s_tune_sign == 0u) ? 1u : 0u;
			}
			sbus_chassis.target_yaw = s_yaw_base +
			                          ((s_tune_sign != 0u) ? YAW_TUNE_STEP_DEG : (-YAW_TUNE_STEP_DEG));
#elif YAW_DIR_SEL
			/* 旋钮 CH[4] 选方向：基准(上电朝向) + 0/90/180/270° */
			sbus_chassis.target_yaw = s_yaw_base + YAW_DirOffset(CH[4]) * YAW_DIR_CW_SIGN;
#else
			sbus_chassis.target_yaw = s_yaw_base;
#endif
			sbus_chassis.current_yaw = MINS500_data.real_yaw_1 * CHASSIS_YAW_SIGN;
		}
		else
		{
			s_yaw_ready = 0u;   // 没有陀螺仪数据就不锁头，保持原来的行为
		}
		Chassis_Control(&sbus_chassis, mid_speed, s_yaw_ready);

		/* 遥测：供串口采数（yaw 环整定） */
		yaw_tel.target = sbus_chassis.target_yaw;
		yaw_tel.actual = sbus_chassis.current_yaw;
		yaw_tel.err    = sbus_chassis.target_yaw - sbus_chassis.current_yaw;
		yaw_tel.vw     = sbus_chassis.target_world_velocity.Vw;
		yaw_tel.gz     = hi14.gyro[2];
#else
		Chassis_Control(&sbus_chassis, mid_speed, 0);
#endif
		/* 快慢档：遥控器 CH[8] 两段开关（200=慢档 / 1800=快档），带滞回 */
		if (CH[8] > (1000u + GEAR_HYST))      { s_gear_fast = 1u; }
		else if (CH[8] < (1000u - GEAR_HYST)) { s_gear_fast = 0u; }

		float wheel_scale = (s_gear_fast != 0u) ? GEAR_FAST_SCALE : GEAR_SLOW_SCALE;

		for (uint8_t i = 0; i < 4; i++)
		{
			motor.wheel_speed[i] = sbus_chassis.wheel_speed[i] * wheel_scale;
			motor.steer_angle[i] = sbus_chassis.steer_angle[i];
		}

		(void)osMessageQueuePut(Chassis_To_CanHandle, &motor, 0, 0);
		osDelay(2);
	}
}

//void App_FDCAN(void *argument)
//{
//	static Chassis_Motor s_chassis_motor = {0};
//	for(;;)
//	{
//		xQueueReceive(Chassis_To_CanHandle,&s_chassis_motor,1);


//		//舵向电机 在电机回零完成之后在进行发指令
//		if(steer_homing_state == eHOMING_DONE)
//		{
//			DJI_PositionMode(&LF_steer, s_chassis_motor.steer_angle[0], 50.0f, 0.0f);
//			osDelay(1);
//			DJI_PositionMode(&RF_steer, s_chassis_motor.steer_angle[1], 50.0f, 0.0f);
//			osDelay(1);
//			DJI_PositionMode(&RB_steer, s_chassis_motor.steer_angle[2], 50.0f, 0.0f);
//			osDelay(1);
//			DJI_PositionMode(&LB_steer, s_chassis_motor.steer_angle[3], 50.0f, 0.0f);

//			Vesc_Speed_mode(&wheelLF, s_chassis_motor.wheel_speed[0]);              //轮向电机
//			osDelay(1);
//			Vesc_Speed_mode(&wheelRF, s_chassis_motor.wheel_speed[1]);              //轮向电机
//			osDelay(1);
//			Vesc_Speed_mode(&wheelRB, s_chassis_motor.wheel_speed[2]);              //轮向电机
//			osDelay(1);
//			Vesc_Speed_mode(&wheelLB, s_chassis_motor.wheel_speed[3]);              //轮向电机
//			osDelay(1);
//		}



//		/*fdcan重启*/
//        while ( (FDCAN1->PSR & FDCAN_PSR_BO) ) 
//        {
//            HAL_FDCAN_Stop(&hfdcan1);
//            HAL_FDCAN_DeInit(&hfdcan1);
//            HAL_FDCAN_Stop(&hfdcan2);
//            HAL_FDCAN_DeInit(&hfdcan2);
//            HAL_FDCAN_Stop(&hfdcan3);
//            HAL_FDCAN_DeInit(&hfdcan3);
//            __HAL_RCC_FDCAN_CLK_DISABLE();
//            __HAL_RCC_FDCAN_FORCE_RESET();
//            __HAL_RCC_FDCAN_RELEASE_RESET();
//            __HAL_RCC_FDCAN_CLK_ENABLE();
//            
//            osDelay(10);        
//            HAL_FDCAN_Init(&hfdcan1);
//            HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
//            HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//            HAL_FDCAN_Start(&hfdcan1);
//            
//            HAL_FDCAN_Init(&hfdcan2);
//            HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
//            HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//            HAL_FDCAN_Start(&hfdcan2);
//            
//            HAL_FDCAN_Init(&hfdcan3);
//            HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
//            HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//            HAL_FDCAN_Start(&hfdcan3);
//            osDelay(10);	
//        }		
//		
//		osDelay(1);
//	}
//}


