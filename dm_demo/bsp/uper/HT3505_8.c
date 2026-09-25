#include "HT3505_8.h"


static void Set_Homing(HT3505_HandleTypeDef *motor);

#ifdef __FDCAN_H__
    static void Can_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint8_t length);
#elif defined __CAN_H__
	static void Can_Transmit(CAN_HandleTypeDef *hcan, int32_t id, uint8_t *tx_Data, uint8_t length);
#endif


enum
{
    eReadPosCommand = 0xA3,
    eSetHomingCommand = 0xB1,
    eSpeedCommand = 0xC1,
    ePositionCommand = 0xC2,
    eSpeedLimitCommand = 0xB2,
};

typedef union
{
    int32_t I32;
    uint8_t u8[4];
}I32_U8;

typedef union
{
    uint32_t U32;
    uint8_t u8[4];
}U32_U8;


//初始化
void HT3505_Init(HT3505_HandleTypeDef *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id)
{
    motor->can_id = can_id;
    motor->hfdcan = hfdcan;

    Set_Homing(motor);
}


//target_speed rpm
void HT3505_SpeedMode(HT3505_HandleTypeDef *motor, float target_speed)
{
    I32_U8 speed;
    uint8_t tx_data[5] = {0};

    speed.I32 = (int32_t)(target_speed * 100.0f);
    tx_data[0] = eSpeedCommand;
    tx_data[1] = speed.u8[0];
    tx_data[2] = speed.u8[1];
    tx_data[3] = speed.u8[2];
    tx_data[4] = speed.u8[3];

    Can_Transmit(motor->hfdcan, motor->can_id, tx_data, 5);
}


//target_pos 度  target_speed rpm(最大可以达到400  100转速一下会比较震)
void HT3505_PositionMode(HT3505_HandleTypeDef *motor, float target_pos, float target_speed)
{
    I32_U8 pos;
    U32_U8 speed;
    uint8_t tx_data1[5] = {0};
    uint8_t tx_data2[5] = {0};

    pos.I32 = (int32_t)(target_pos * (16384.0f / 360.0f));
    speed.U32 = (int32_t)(target_speed * 100.0f);

    tx_data1[0] = ePositionCommand;
    tx_data2[0] = eSpeedLimitCommand;
    for(uint8_t i = 0; i < 4; i++)
    {
        tx_data1[i + 1] = pos.u8[i];
        tx_data2[i + 1] = speed.u8[i];
    }

    Can_Transmit(motor->hfdcan, motor->can_id, tx_data2, 5);
    HAL_Delay(1);
    Can_Transmit(motor->hfdcan, motor->can_id, tx_data1, 5);
}


//读取角度指令
void HT3505_ReadPosition(HT3505_HandleTypeDef *motor)
{
    uint8_t tx_data = eReadPosCommand;
    Can_Transmit(motor->hfdcan, motor->can_id, &tx_data, 1);
}


//设置原点
static void Set_Homing(HT3505_HandleTypeDef *motor)
{
    uint8_t tx_data = eSetHomingCommand;
    Can_Transmit(motor->hfdcan, motor->can_id, &tx_data, 1);
}


#ifdef __FDCAN_H__
    //解析电机角度
    void HT3505Get_Position(HT3505_HandleTypeDef *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
    {
        if(rx_header->IdType == FDCAN_STANDARD_ID)
        {
            if(rx_header->Identifier == motor->can_id)
            {   
                I32_U8 pos;

                if(data[0] == eReadPosCommand)
                {
                    pos.u8[0] = data[3];
                    pos.u8[1] = data[4];
                    pos.u8[2] = data[5];
                    pos.u8[3] = data[6];

                    motor->position = (float)pos.I32 * (360.0f / 16384.0f);
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
	
    //解析电机角度
    void HT3505Get_Position(HT3505_HandleTypeDef *motor, CAN_RxHeaderTypeDef *rx_header, uint8_t *data)
    {
        if(rx_header->IDE == CAN_ID_STD)
        {
            if(rx_header->StdId == motor->can_id)
            {   
                I32_U8 pos;

                if(data[0] == eReadPosCommand)
                {
                    pos.u8[0] = data[3];
                    pos.u8[1] = data[4];
                    pos.u8[2] = data[5];
                    pos.u8[3] = data[6];

                    motor->position = (float)pos.I32 * (360.0f / 16384.0f);
                }
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
