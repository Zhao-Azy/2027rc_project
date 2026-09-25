#include "drv_can.h"

#ifdef STM32H7xx_HAL_H

	void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan)
	{
        //接收标准帧和扩展帧 拒绝远程帧
		HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        //启用RX FIFO0新消息到达中断
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		//启动FDCAN控制器
        HAL_FDCAN_Start(hfdcan);   
	}

    #if ALL_INIT
        void FDCAN_All_Init(void)
        {
            FDCAN_Init(&hfdcan1);
            FDCAN_Init(&hfdcan2);
            FDCAN_Init(&hfdcan3);
        }
    #endif


#elif defined (__STM32F4xx_HAL_H) || defined(__STM32F1xx_HAL_H)

    void Can1_Filter_Init(CAN_HandleTypeDef *hcan)
    {
        CAN_FilterTypeDef can_filter_st; 

        can_filter_st.FilterActivation = ENABLE;
        can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
        can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
        can_filter_st.FilterIdHigh = 0x0000;
        can_filter_st.FilterIdLow = 0x0000;
        can_filter_st.FilterMaskIdHigh = 0x0000;
        can_filter_st.FilterMaskIdLow = 0x0000;
        can_filter_st.FilterBank = 0;      //can1滤波器组从0开始使用
        can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

        #if defined(__STM32F4xx_HAL_H)
            can_filter_st.SlaveStartFilterBank = 14;       //滤波器组分配（对半分）
        #endif

        HAL_CAN_ConfigFilter(hcan, &can_filter_st);                      // 初始化滤波器
        HAL_CAN_Start(hcan);                                             // 开启can
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 滤波器中断    
    }

    #if defined(CAN2)       //检查单片机是否有can2外设

        void Can2_Filter_Init(CAN_HandleTypeDef *hcan)
        {
            CAN_FilterTypeDef can_filter_st;

            can_filter_st.FilterActivation = ENABLE;
            can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
            can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
            can_filter_st.FilterIdHigh = 0x0000;
            can_filter_st.FilterIdLow = 0x0000;
            can_filter_st.FilterMaskIdHigh = 0x0000;
            can_filter_st.FilterMaskIdLow = 0x0000;
            can_filter_st.FilterBank = 14;         //can2滤波器组从14开始使用
            can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

            HAL_CAN_ConfigFilter(hcan, &can_filter_st);                      // 初始化滤波器
            HAL_CAN_Start(hcan);                                             // 开启can
            HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 滤波器中断   
        }


        void Can_Filter_Init(void)
        {
            Can1_Filter_Init(&hcan1);
            Can2_Filter_Init(&hcan2);
        }
    #endif
    
#endif


//FDCAN发送标准帧
// void Can_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint8_t length)
// {
//     if(length > 8) length = 8;

//     FDCAN_TxHeaderTypeDef TxHeader;

//     TxHeader.Identifier = id;       //就是F4系列的StdId即发送的标识符
//     TxHeader.IdType = FDCAN_STANDARD_ID;        //标准id
//     TxHeader.TxFrameType = FDCAN_DATA_FRAME;    //数据帧
//     TxHeader.DataLength = length;               //数据长度
//     TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;    //CAN发送错误指示
//     TxHeader.BitRateSwitch = FDCAN_BRS_OFF;         //波特率切换模式关闭
//     TxHeader.FDFormat = FDCAN_CLASSIC_CAN;          //经典can模式
//     TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;       //不储存发送事件
//     TxHeader.MessageMarker = 0;        //消息标记0

//     while(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan)==0){}
//     HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);  
// }


//FDCAN发送扩展帧
// static void Can_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint8_t length)
// {
//     if(length > 8) length = 8;

//     FDCAN_TxHeaderTypeDef TxHeader;

//     TxHeader.Identifier = id;       //就是F4系列的StdId即发送的标识符
//     TxHeader.IdType = FDCAN_EXTENDED_ID;        //扩展id
//     TxHeader.TxFrameType = FDCAN_DATA_FRAME;    //数据帧
//     TxHeader.DataLength = length;               //数据长度
//     TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;    //CAN发送错误指示
//     TxHeader.BitRateSwitch = FDCAN_BRS_OFF;         //波特率切换模式关闭
//     TxHeader.FDFormat = FDCAN_CLASSIC_CAN;          //经典can模式
//     TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;       //不储存发送事件
//     TxHeader.MessageMarker = 0;        //消息标记0

//     while(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan)==0){}
//     HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);  
// }
