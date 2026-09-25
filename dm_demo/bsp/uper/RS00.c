#include "RS00.h"
#include "cmsis_os.h"

static void Enable_Motor(RS00_HandleTypeDef *motor);
static void Disable_Motor(RS00_HandleTypeDef *motor);
static void Set_Homing(RS00_HandleTypeDef *motor);
static void Active_Feedback(RS00_HandleTypeDef *motor);
static void Select_MotorType(RS00_HandleTypeDef *motor, RS00_Type type);
static void Parameter_Writing(RS00_HandleTypeDef *motor, uint16_t index, uint8_t *data);
static float Uint_To_Float(int x_int, float x_min, float x_max, int bits);
static int Float_To_Uint(float x, float x_min, float x_max, int bits);



#ifdef __FDCAN_H__
	static void Can_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t length);
#elif defined __CAN_H__
	static void Can_Transmit(CAN_HandleTypeDef *hcan, int32_t id, uint8_t *tx_Data, uint8_t length);
#endif

typedef union
{
    float f;
    uint8_t u8[4];
} F_U8;

enum
{
    eMITControl = 1,
    eMotorFeedback = 2,         // 电机反馈帧
    eEnableMotor = 3,           // 使能电机
    eDisableMotor = 4,          // 失能电机
    eSetOrigin = 6,             // 设置原点
    eVoluntaryReporting = 0x18, // 主动上报
    eParameterWriting = 0x12,   // 单个参数写入
};

void RS00_Init(RS00_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, RS00_Type type, uint16_t can_id)
{
    motor->hfdcan = hfdcan;
    motor->can_id = can_id;

    Active_Feedback(motor);
    HAL_Delay(20);
    Select_MotorType(motor, type);
    HAL_Delay(20);
    // Set_Homing(motor);
    HAL_Delay(20);
    Enable_Motor(motor);
    HAL_Delay(20);
}

// 电流模式 target_current A
void RS00_CurrentMode(RS00_HandleTypeDef *motor, float target_current)
{
    F_U8 current;
    uint8_t data[4] = {0};

    if (target_current > 16.0f)
    {
        target_current = 16.0f;
    }
    else if (target_current < -16.0f)
    {
        target_current = -16.0f;
    }

    current.f = target_current;
    data[0] = current.u8[0];
    data[1] = current.u8[1];
    data[2] = current.u8[2];
    data[3] = current.u8[3];

    Parameter_Writing(motor, 0x7006, data);
}

// 速度模式 target_speed rad/s
void RS00_SpeedMode(RS00_HandleTypeDef *motor, float target_speed)
{
    F_U8 speed;
    uint8_t data[4] = {0};

    if (target_speed > 33.0f)
    {
        target_speed = 33.0f;
    }
    else if (target_speed < -33.0f)
    {
        target_speed = -33.0f;
    }

    speed.f = target_speed;
    data[0] = speed.u8[0];
    data[1] = speed.u8[1];
    data[2] = speed.u8[2];
    data[3] = speed.u8[3];

    Parameter_Writing(motor, 0x700A, data);
}

// 位置模式  target_pos rad  speed_limit rad
void RS00_PositionMode(RS00_HandleTypeDef *motor, float target_pos, float target_speed)
{
    F_U8 pos;
    F_U8 max_speed;
    F_U8 max_current;

    uint8_t pos_data[4] = {0};
    uint8_t speed[4] = {0};

    if (target_speed > 33.0f)
    {
        target_speed = 33.0f;
    }

    pos.f = target_pos;
    max_speed.f = target_speed;

    for (int i = 0; i < 4; i++)
    {
        pos_data[i] = pos.u8[i];
        speed[i] = max_speed.u8[i];
    }

    Parameter_Writing(motor, 0x7016, pos_data);
    HAL_Delay(10);
    Parameter_Writing(motor, 0x7017, speed);
}

// MIT模式 如果想要实现位置控制，位置给定，扭矩给0，目标速度给0（位置控制的时候kd一定不能为0）！
void RS00_MITMode(RS00_HandleTypeDef *motor, float target_pos, float target_speed, float Kp, float Kd, float torque)
{
    uint16_t torque_data = 0;
    uint16_t pos_data = 0;
    uint16_t speed_data = 0;
    uint16_t kp_data = 0;
    uint16_t kd_data = 0;
    uint8_t tx_data[8] = {0};

    // 边界处理
    if (target_pos < -12.57f)
    {
        target_pos = -12.57f;
    }
    else if (target_pos > 12.57f)
    {
        target_pos = 12.57f;
    }

    if (target_speed < -33.0f)
    {
        target_speed = -33.0f;
    }
    else if (target_speed > 33.0f)
    {
        target_speed = 33.0f;
    }

    if (Kp < 0)
    {
        Kp = 0;
    }
    else if (Kp > 500.0f)
    {
        Kp = 500.0f;
    }

    if (Kd < 0)
    {
        Kd = 0;
    }
    else if (Kd > 5.0f)
    {
        Kd = 5.0f;
    }

    torque_data = (uint16_t)(((torque + 14.0f) / 28.0f) * 65535);
    pos_data = (uint16_t)(((target_pos + 12.57f) / 25.14f) * 65535);
    speed_data = (uint16_t)(((target_speed + 33.0f) / 66.0f) * 65535);
    kp_data = (uint16_t)((Kp / 500.0f) * 65535);
    kd_data = (uint16_t)((Kd / 5.0f) * 65535);

    tx_data[0] = pos_data >> 8;
    tx_data[1] = pos_data;
    tx_data[2] = speed_data >> 8;
    tx_data[3] = speed_data;
    tx_data[4] = kp_data >> 8;
    tx_data[5] = kp_data;
    tx_data[6] = kd_data >> 8;
    tx_data[7] = kd_data;

    uint32_t id = (eMITControl << 24) | (torque_data << 8) | motor->can_id;

    Can_Transmit(motor->hfdcan, id, tx_data, 8);
}


// 电机使能
static void Enable_Motor(RS00_HandleTypeDef *motor)
{
    uint8_t tx_data[8] = {0};
    uint32_t id = (eEnableMotor << 24) | motor->can_id;

    Can_Transmit(motor->hfdcan, id, tx_data, 8);
}

// 电机失能
static void Disable_Motor(RS00_HandleTypeDef *motor)
{
    uint8_t tx_data[8] = {0};
    uint32_t id = (eDisableMotor << 24) | motor->can_id;

    Can_Transmit(motor->hfdcan, id, tx_data, 8);
}

// 设置原点
static void Set_Homing(RS00_HandleTypeDef *motor)
{
    uint8_t tx_data[8] = {0};
    uint32_t id = (eSetOrigin << 24) | motor->can_id;

    tx_data[0] = 1;

    Can_Transmit(motor->hfdcan, id, tx_data, 8);
}

// 主动反馈
static void Active_Feedback(RS00_HandleTypeDef *motor)
{
    uint8_t tx_data[8] = {0};
    uint32_t id = (eVoluntaryReporting << 24) | motor->can_id;

    for (int i = 0; i < 6; i++)
    {
        tx_data[i] = i + 1;
    }
    tx_data[6] = 1;
    tx_data[7] = 0;

    Can_Transmit(motor->hfdcan, id, tx_data, 8);
}

// 电机模式选择
static void Select_MotorType(RS00_HandleTypeDef *motor, RS00_Type type)
{
    uint8_t data[4] = {0};
    data[0] = type;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;

    Parameter_Writing(motor, 0x7005, data);
}


// 单个参数写入   低字节在前，高字节在后
static void Parameter_Writing(RS00_HandleTypeDef *motor, uint16_t index, uint8_t *data)
{
    uint8_t tx_data[8] = {0};
    uint32_t id = (eParameterWriting << 24) | motor->can_id;

    tx_data[0] = index;
    tx_data[1] = index >> 8;
    tx_data[2] = 0;
    tx_data[3] = 0;
    tx_data[4] = data[0];
    tx_data[5] = data[1];
    tx_data[6] = data[2];
    tx_data[7] = data[3];

    Can_Transmit(motor->hfdcan, id, tx_data, 8);
}


#ifdef __FDCAN_H__

	// 获取电机数据
	void RS00_Get_MotorData(RS00_HandleTypeDef *motor, FDCAN_RxHeaderTypeDef *p_msg, uint8_t *data_arry)
	{
		if (p_msg->IdType == FDCAN_EXTENDED_ID)
		{
			if (((p_msg->Identifier >> 8) & 0xFF) == motor->can_id)
			{
				motor->original_pos = (data_arry[0] << 8) | data_arry[1];
				motor->original_speed = (data_arry[2] << 8) | data_arry[3];
				motor->original_torque = (data_arry[4] << 8) | data_arry[5];
				motor->pos = Uint_To_Float(motor->original_pos, -12.57f, 12.57f, 16);
				motor->speed = Uint_To_Float(motor->original_speed, -33.0f, 33.0f, 16);
				motor->torque = Uint_To_Float(motor->original_torque, -14.0f, 14.0f, 16);

                //防止其在12.57跳变
                motor->delta_pos = motor->pos - motor->last_pos;
                if(motor->delta_pos > 12.57f)
                {
                    motor->delta_pos -= 25.14f;
                }else if(motor->delta_pos < -12.57f)
                {
                    motor->delta_pos += 25.14f;
                }
                motor->real_pos += motor->delta_pos;
                motor->last_pos = motor->pos;
			}
		}
	}
	
	
    // FDCAN发送扩展帧
    static void Can_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t length)
    {
        if (length > 8)
            length = 8;

        FDCAN_TxHeaderTypeDef TxHeader;

        TxHeader.Identifier = id;                         // 就是F4系列的StdId即发送的标识符
        TxHeader.IdType = FDCAN_EXTENDED_ID;              // 扩展id
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
        TxHeader.DataLength = length;                     // 数据长度
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // CAN发送错误指示
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 波特率切换模式关闭
        TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 经典can模式
        TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不储存发送事件
        TxHeader.MessageMarker = 0;                       // 消息标记0

        while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
        {
        }
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
    }

#elif defined __CAN_H__

	// 获取电机数据
	void RS00_Get_MotorData(CAN_RxHeaderTypeDef *p_msg, RS00_HandleTypeDef *motor, uint8_t *data_arry)
	{
		if (p_msg->IDE == CAN_ID_EXT)
		{
			if (((p_msg->ExtId >> 8) & 0xFF) == motor->can_id)
			{
				motor->original_pos = (data_arry[0] << 8) | data_arry[1];
				motor->original_speed = (data_arry[2] << 8) | data_arry[3];
				motor->original_torque = (data_arry[4] << 8) | data_arry[5];
				motor->pos = Uint_To_Float(motor->original_pos, -12.57f, 12.57f, 16);
				motor->speed = Uint_To_Float(motor->original_speed, -44.0f, 44.0f, 16);
				motor->torque = Uint_To_Float(motor->original_torque, -17.0f, 17.0f, 16);
			}
		}
	}
	
    // can发送
    static void Can_Transmit(CAN_HandleTypeDef *hcan, int32_t id, uint8_t *tx_Data, uint8_t length)
    {
        CAN_TxHeaderTypeDef TxHeader;
        uint32_t can_SendMailBox;
        if (length > 8)
        {
            length = 8;
        }

        TxHeader.IDE = CAN_ID_EXT; // 扩展帧            //如果使用扩展帧的话改为CAN_ID_EXT 下面的stdid也改一下
        TxHeader.StdId = 0;
        TxHeader.ExtId = id;

        TxHeader.RTR = CAN_RTR_DATA; // 数据帧
        TxHeader.DLC = length;       // 数据长度
        TxHeader.TransmitGlobalTime = DISABLE;

        HAL_CAN_AddTxMessage(hcan, &TxHeader, tx_Data, &can_SendMailBox);
    }

#endif


// 无符号转浮点数
static float Uint_To_Float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offest = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offest;
}

// 浮点数转无符号
static int Float_To_Uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offest = x_min;
    return (int)((x - offest) * ((float)((1 << bits) - 1)) / span);
}