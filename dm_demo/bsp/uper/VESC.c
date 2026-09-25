#include "VESC.h"

static void Can_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint8_t length);


enum
{
   SPEED_CURRENT_STATUS = 9,
   POSTION_STATUS = 16,
};



//初始化
void VESC_Init(VESC_HandleTypeDef *h_VESC, FDCAN_HandleTypeDef *hcan, uint8_t id, uint8_t poles)
{
   h_VESC->hcan = hcan;
   h_VESC->id = id;
   h_VESC->poles = poles;
}


/**
 *  @brief  占空比模式
 *  @param  can句柄
 *  @param  控制器ID
 *  @param  占空比
 *  @retval 无
 */
void Vesc_Duty_mode(VESC_HandleTypeDef *h_VESC, float duty)
{
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
   Can_Transmit(h_VESC->hcan, h_VESC->id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

/**
 *  @brief  电流模式
 *  @param  can句柄
 *  @param  控制器ID
 *  @param  电流
 *  @retval 无
 */
void Vesc_Current_mode(VESC_HandleTypeDef *h_VESC, float current)
{
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
   Can_Transmit(h_VESC->hcan, h_VESC->id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

/**
 *  @brief  转速模式
 *  @param  can句柄
 *  @param  控制器ID
 *  @param  转速
 *  @retval 无
 */
void Vesc_Speed_mode(VESC_HandleTypeDef *h_VESC, float rpm)
{
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_int32(buffer, (int32_t)rpm, &send_index);
   Can_Transmit(h_VESC->hcan, h_VESC->id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

/**
 *  @brief  位置模式
 *  @param  can句柄
 *  @param  控制器ID
 *  @param  位置
 *  @retval 无
 */
void Vesc_Position_mode(VESC_HandleTypeDef *h_VESC, float pos)
{
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
   Can_Transmit(h_VESC->hcan, h_VESC->id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

/**
 *  @brief  电流刹车模式
 *  @param  can句柄
 *  @param  控制器ID
 *  @param  电流
 *  @retval 无
 */
void Vesc_Current_Break_mode(VESC_HandleTypeDef *h_VESC, float current)
{
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
   Can_Transmit(h_VESC->hcan, h_VESC->id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

/**
 *  @brief  相对电流刹车模式
 *  @param  can句柄
 *  @param  控制器ID
 *  @param  相对电流
 *  @retval 无
 */
void Vesc_Current_brake_rel_mode(VESC_HandleTypeDef *h_VESC, float current_rel)
{
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_float32(buffer, current_rel, 1e5, &send_index);
   Can_Transmit(h_VESC->hcan, h_VESC->id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
}



#ifdef __FDCAN_H__
   //获取电机数据
   void VESC_Data_Process(VESC_HandleTypeDef *h_VESC, FDCAN_RxHeaderTypeDef *p_msg, uint8_t *data)
   {
      U16_I16 u16_i16;
      U32_I32 u32_i32;

      if (p_msg->IdType != FDCAN_EXTENDED_ID)
      {
         return;
      }

     if(((p_msg->Identifier >> 8) & 0xFF) == SPEED_CURRENT_STATUS)      //对应的反馈模式(相当于命令id)
     {
        if((p_msg->Identifier & 0xFF) == h_VESC->id) 
        {
           u32_i32.u32 = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
           u16_i16.u16 = (data[4] << 8) | data[5];

           h_VESC->speed = (float)(u32_i32.i32 / h_VESC->poles);       //由erpm转化为rpm  
           h_VESC->current = (float)(u16_i16.i16 / 10.0f);

        }
     }
//	 if(((p_msg->Identifier >> 8) & 0xFF) == POSTION_STATUS)        
//      {
//         if((p_msg->Identifier & 0xFF) == h_VESC->id)
//         {
//            u16_i16.u16  = (data[6] << 8) | data[7];
//            h_VESC->current_pos = (float)(u16_i16.i16 / 50.0f);

//            if(h_VESC->first_flag == 0)
//            {
//               h_VESC->last_pos = h_VESC->current_pos;
//               h_VESC->first_flag = 1;
//            }

//            h_VESC->delta_pos = h_VESC->current_pos -  h_VESC->last_pos;
//            if(h_VESC->delta_pos > 300.0f)
//            {
//               h_VESC->delta_pos -= 360.0f;
//            }else if(h_VESC->delta_pos < -300.0f)
//            {
//               h_VESC->delta_pos += 360.0f;
//            }
//            h_VESC->real_pos += h_VESC->delta_pos;
//            h_VESC->last_pos = h_VESC->current_pos;

//         }

//      }

   }

   static void Can_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint8_t length)
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
   
#elif define __CAN_H__
   
   //获取电机数据
   void VESC_Data_Process(VESC_HandleTypeDef *h_VESC, CAN_HandleTypeDef *p_msg, uint8_t *data)
   {
      U16_I16 u16_i16;
      U32_I32 u32_i32;

      if (p_msg->IDE != CAN_ID_EXT)
      {
         return;
      }

      if(((p_msg->ExtId >> 8) & 0xFF) == 9)      //对应的反馈模式(相当于命令id)
      {
         if((p_msg->ExtId & 0xFF) == h_VESC->id) 
         {
            u32_i32.u32 = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
            u16_i16.u16 = (data[4] << 8) | data[5];

            h_VESC->speed = (float)(u32_i32.i32 / h_VESC->poles);       //由erpm转化为rpm  这个后续在修改，可能不太对
            h_VESC->current = (float)(u16_i16.i16 / 10.0f);

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



//以大端的方式把一个32位数据转换为4个8位数据
static void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
   buffer[(*index)++] = number >> 24;
   buffer[(*index)++] = number >> 16;
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
}

static void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index)
{
   buffer_append_int32(buffer, (int32_t)(number * scale), index);
}
