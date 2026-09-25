#include "MG5010E.h"


#define RPM_TO_DPS 6.0f

static void Set_Homing(MG5010E_HandleTypeDef *motor);

#ifdef __FDCAN_H__
    static void Can_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint8_t length);
#elif defined __CAN_H__
	static void Can_Transmit(CAN_HandleTypeDef *hcan, int32_t id, uint8_t *tx_Data, uint8_t length);
#endif


enum
{
    eSpeedCommand = 0xA2,
    ePositionCommand = 0xA3,
    eSpeedPosCommand = 0xA4,
    eSetOriginCommand = 0x19,
    eReadMotorStateCommand = 0x9C,
    eReadPositionCommand = 0x92
};

typedef union
{
    int32_t I32;
    uint8_t u8[4];
}I32_U8;

typedef union 
{
    int16_t I16;
    uint8_t u8[2];
}I16_U8;

typedef union 
{
    int64_t I64;
    uint8_t u8[8];
}I64_U8;


void MG5010E_Init(MG5010E_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id, float ratio)
{
    motor->hfdcan = hfdcan;
    motor->can_id = can_id;
    motor->ratio = ratio;

//   Set_Homing(motor);
}


//target_speed rpm target_current A
void MG5010E_SpeedMode(MG5010E_HandleTypeDef *motor, float target_speed, float current_limit)
{
    I32_U8 speed;
    I16_U8 current;

    if(current_limit >= 33.0f)
    {
        current_limit = 33.0f;
    }
    else if(current_limit < -33.0f)
    {
        current_limit = -33.0f;
    }

    target_speed = target_speed * motor->ratio * RPM_TO_DPS;     //由电机输出轴转速转换为电机内部转速  rpm - > 度/s

    speed.I32 = (int32_t)(target_speed * 100.0f);
    current.I16 = (int16_t)((current_limit / 33.0f) * 2048);

    uint8_t tx_data[8] = {0};
    tx_data[0] = eSpeedCommand;
    tx_data[1] = 0X00;
    tx_data[2] = current.u8[0];
    tx_data[3] = current.u8[1];
    tx_data[4] = speed.u8[0];
    tx_data[5] = speed.u8[1];
    tx_data[6] = speed.u8[2];
    tx_data[7] = speed.u8[3];

    Can_Transmit(motor->hfdcan, (0x140 | motor->can_id), tx_data, 8);
}


//多圈模式  target_pos 度
void MG5010E_PositionMode(MG5010E_HandleTypeDef *motor, float target_pos)
{
    I32_U8 position;
    uint8_t tx_data[8] = {0};

    //将电机输出端角度转化为电机内部角度
    position.I32 = (int32_t)(target_pos * motor->ratio * 100.0f);

    tx_data[0] = ePositionCommand;
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = position.u8[0];
    tx_data[5] = position.u8[1];
    tx_data[6] = position.u8[2];
    tx_data[7] = position.u8[3];    

    Can_Transmit(motor->hfdcan, (0x140 | motor->can_id), tx_data, 8);
}


//多圈模式  target_pos 度  target_speed rpm
void MG5010E_SpeedPosMode(MG5010E_HandleTypeDef *motor, uint16_t speed_limit, float target_pos)
{
    I32_U8 pos;
    uint8_t tx_data[8] = {0};

    //由电机输出端转换到电机内部转速
    speed_limit = speed_limit * motor->ratio * RPM_TO_DPS;

    //将电机输出端位置转化为电机内部位置
    pos.I32 = (int32_t)(target_pos * motor->ratio * 100.0f);
    tx_data[0] = eSpeedPosCommand;
    tx_data[1] = 0x00;
    tx_data[2] = speed_limit & 0xFF;
    tx_data[3] = (speed_limit >> 8) & 0xFF;
    tx_data[4] = pos.u8[0];
    tx_data[5] = pos.u8[1];
    tx_data[6] = pos.u8[2];
    tx_data[7] = pos.u8[3];

    Can_Transmit(motor->hfdcan, (0x140 | motor->can_id), tx_data, 8);
}


//读取电机角度（一收一发制） 在定时器里发
void MG5010E_ReadPosition(MG5010E_HandleTypeDef *motor)
{
    uint8_t tx_data[8] = {0};
    tx_data[0] = eReadPositionCommand;

    for(uint8_t i = 1; i < 8; i++)
    {
        tx_data[i] = 0x00;
    }

    Can_Transmit(motor->hfdcan, (0x140 | motor->can_id), tx_data, 8);

}


//读取电机状态(一发一收制)
void MG5010E_ReadMotorState(MG5010E_HandleTypeDef *motor)
{
    uint8_t tx_data[8] = {0};
    tx_data[0] = eReadMotorStateCommand;

    for(uint8_t i = 1; i < 8; i++)
    {
        tx_data[i] = 0x00;
    }

    Can_Transmit(motor->hfdcan, (0x140 | motor->can_id), tx_data, 8);

}


//需要重新上电之后生效！！！！
static void Set_Homing(MG5010E_HandleTypeDef *motor)
{
    uint8_t tx_data[8] = {0};
    tx_data[0] = eSetOriginCommand;

    for(uint8_t i = 1; i < 8; i++)
    {
        tx_data[i] = 0x00;
    }

    Can_Transmit(motor->hfdcan, (0x140 | motor->can_id), tx_data, 8);
}


#ifdef __FDCAN_H__
    void MG5010E_GetPosition(MG5010E_HandleTypeDef *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
    {
        I64_U8 position;

        if(rx_header->IdType == FDCAN_STANDARD_ID)
        {
            if(rx_header->Identifier == (0x140 | motor->can_id))
            {   
                if(data[0] == 0x92)
                {
                    for(uint8_t i = 0; i < 7; i++)
                    {
                        position.u8[i] = data[i + 1];
                    }
    
                    //符号处理
                    if (data[7] & 0x80) 
                    {
                        //如果高位是1，说明是负数
                        position.u8[7] = 0xFF; 
                    }
                    else 
                    {
                        //如果高位是0，说明是正数
                        position.u8[7] = 0x00; 
                    }
    
                    motor->pos = position.I64 / motor->ratio / 100.0f;
                }
            }
        }
    }


	//can发送标准帧
	static void Can_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint8_t length)
	{
		if(length > 8) length = 8;

		FDCAN_TxHeaderTypeDef TxHeader;

		TxHeader.Identifier = id;       //就是F4系列的StdId即发送的标识符
		TxHeader.IdType = FDCAN_STANDARD_ID;        //标准id
		TxHeader.TxFrameType = FDCAN_DATA_FRAME;    //数据帧
		TxHeader.DataLength = length;               //数据长度
		TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;    //CAN发送错误指示
		TxHeader.BitRateSwitch = FDCAN_BRS_OFF;         //波特率切换模式关闭
		TxHeader.FDFormat = FDCAN_CLASSIC_CAN;          //经典can模式
		TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;       //不储存发送事件
		TxHeader.MessageMarker = 0;        //消息标记0

		while(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan)==0){}
		HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);  
	}
#elif defined __CAN_H__
	
    void MG5010E_GetPosition(MG5010E_HandleTypeDef *motor, CAN_RxHeaderTypeDef *rx_header, uint8_t *data)
    {
        I64_U8 position;

        if(rx_header->IDE == CAN_ID_STD)
        {
            if(rx_header->StdId == (0x140 | motor->can_id))
            {   
                for(uint8_t i = 0; i < 7; i++)
                {
                    position.u8[i] = data[i + 1];
                }

                //符号处理
                if (data[7] & 0x80) 
                {
                    //如果高位是1，说明是负数
                    position.u8[7] = 0xFF; 
                }
                else 
                {
                    //如果高位是0，说明是正数
                    position.u8[7] = 0x00; 
                }
                motor->pos = (float)(position.I64) / 100.0f;
            }
        }
    }
    // can发送标准帧
    static void Can_Transmit(CAN_HandleTypeDef *hcan, int32_t id, uint8_t *tx_Data, uint8_t length)
    {
        CAN_TxHeaderTypeDef TxHeader;
        uint32_t can_SendMailBox;
        if (length > 8)
        {
            length = 8;
        }

        TxHeader.IDE = CAN_ID_STD; // 标准帧            //如果使用扩展帧的话改为CAN_ID_EXT 下面的stdid也改一下
        TxHeader.StdId = id;
        TxHeader.ExtId = 0;

        TxHeader.RTR = CAN_RTR_DATA; // 数据帧
        TxHeader.DLC = length;       // 数据长度
        TxHeader.TransmitGlobalTime = DISABLE;

        HAL_CAN_AddTxMessage(hcan, &TxHeader, tx_Data, &can_SendMailBox);
    }
#endif
