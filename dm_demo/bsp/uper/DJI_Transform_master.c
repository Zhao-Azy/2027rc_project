#include "DJI_Transform_master.h"

static void Can_Transmit(DJI_HandleTypeDef *hdji, uint8_t *data, uint8_t length);


//初始化
void DJI_Init(DJI_HandleTypeDef *hdji, FDCAN_HandleTypeDef *hcan, Motor_Type motor_type, uint8_t slave_id, uint8_t id)
{
    hdji->hcan = hcan;
    hdji->motor_type = motor_type;
    hdji->id = id;             // 电机id
    hdji->slave_id = slave_id; // 小板id    

    if(hdji->motor_type == M3508)
    {
        hdji->stdid = 0x400;            //自定义的can发送标示符
    }else if(hdji->motor_type == M2006)
    {
        hdji->stdid = 0x300;
    }

}


/**
   *  @brief 电流模式 
   *  @param 大疆电机结构体
   *  @param 目标电流
   *  @retval 无
   */
void DJI_CurrentMode(DJI_HandleTypeDef *hdji, uint16_t target_current)
{
    U16_U8 u8_u16;
    uint8_t tx_data[8] = {0};

    hdji->DJI_cmd = eCurrentMode;

    u8_u16.u16 = target_current;
    tx_data[0] = hdji->id;
    tx_data[1] = u8_u16.u8[0];
    tx_data[2] = u8_u16.u8[1];
    tx_data[3] = 0;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;
    
    Can_Transmit(hdji, tx_data, 8);
}


/**
   *  @brief 速度模式 
   *  @param 大疆电机结构体
   *  @param 目标速度 这里给的是输出轴转速 3508最大482rpm   2006最大500rpm
   *  @param 电流限制（0~100） 0为不限制
   *  @retval 无
   */
void DJI_SpeedMode(DJI_HandleTypeDef *hdji, float target_speed, uint8_t current_limit)
{
    F_U8 f_u8;
    uint8_t tx_data[8];

    hdji->DJI_cmd = eSpeedMode;

    f_u8.f = target_speed;

    tx_data[0] = hdji->id;
    tx_data[1] = f_u8.u8[0];
    tx_data[2] = f_u8.u8[1];
    tx_data[3] = f_u8.u8[2];
    tx_data[4] = f_u8.u8[3];
    
    tx_data[5] = current_limit;     //电流限制给0为不限制
    tx_data[6] = 0;
    tx_data[7] = 0;

    Can_Transmit(hdji, tx_data, 8);
}


/**
   *  @brief 位置模式 
   *  @param 大疆电机结构体
   *  @param 目标位置 这里给的是度数
   *  @param 速度限制（0~100） 0为不限制
   *  @param 电流限制（0~100） 0为不限制
   *  @retval 无
   */
void DJI_PositionMode(DJI_HandleTypeDef *hdji, float target_position, uint8_t speed_limit, uint8_t current_limit)
{
    F_U8 f_u8;
    uint8_t tx_data[8];

    hdji->DJI_cmd = ePositionMode;

    f_u8.f = target_position;
    tx_data[0] = hdji->id;
    tx_data[1] = f_u8.u8[0];
    tx_data[2] = f_u8.u8[1];
    tx_data[3] = f_u8.u8[2];
    tx_data[4] = f_u8.u8[3];

    tx_data[5] = speed_limit;
    tx_data[6] = current_limit;
    tx_data[7] = 0;


    Can_Transmit(hdji, tx_data, 8);
}


/**
   *  @brief 回零模式 
   *  @param 大疆电机结构体
   *  @param 目标速度 这里的目标速度尽量小一点 不然可能会撞坏机构
   *  @retval 无
   */
void DJI_HomingMode(DJI_HandleTypeDef *hdji, float target_speed)
{
    F_U8 f_u8;
    uint8_t tx_data[8];

    hdji->DJI_cmd = eHomingMode;

    f_u8.f = target_speed;
    tx_data[0] = hdji->id;
    tx_data[1] = f_u8.u8[0];
    tx_data[2] = f_u8.u8[1];
    tx_data[3] = f_u8.u8[2];
    tx_data[4] = f_u8.u8[3];
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;
    

    Can_Transmit(hdji, tx_data, 8);
}


//设置原点
void DJI_SetOrigin(DJI_HandleTypeDef *hdji)
{
    uint8_t tx_data[8] = {0};

    hdji->DJI_cmd = eSetOrigin;

    tx_data[0] = hdji->id;
    tx_data[1] = 0;
    tx_data[2] = 0;
    tx_data[3] = 0;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    Can_Transmit(hdji, tx_data, 8);
}

//获取数据
void DJI_Get_Meature(DJI_HandleTypeDef *hdji, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    F_U8 f_u8;
    U16_U8 u16_u8;


    if(rx_header->IdType == FDCAN_STANDARD_ID)
    {
        if((rx_header->Identifier & 0x0F) == hdji->slave_id)
        {
            if(data[0] == hdji->id)
            {
                f_u8.u8[0] = data[1];
                f_u8.u8[1] = data[2];
                f_u8.u8[2] = data[3];
                f_u8.u8[3] = data[4];       //位置
        
                u16_u8.u8[0] = data[5];
                u16_u8.u8[1] = data[6];     //速度
        
                hdji->position = f_u8.f;
                hdji->speed = u16_u8.u16;
            
            }
        }

    }
    
}

static void Can_Transmit(DJI_HandleTypeDef *hdji, uint8_t *data, uint8_t length)
{
    if(length > 8) length = 8;

    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = hdji->stdid | (hdji->DJI_cmd << 4) | hdji->slave_id;

    TxHeader.IdType = FDCAN_STANDARD_ID;        //标准id
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;    //数据帧
    TxHeader.DataLength = length;               //数据长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;    //CAN发送错误指示
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;         //波特率切换模式关闭
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;          //经典can模式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;       //不储存发送事件
    TxHeader.MessageMarker = 0;        //消息标记0

    while(HAL_FDCAN_GetTxFifoFreeLevel(hdji->hcan)==0){}
    HAL_FDCAN_AddMessageToTxFifoQ(hdji->hcan, &TxHeader, data);  
}
