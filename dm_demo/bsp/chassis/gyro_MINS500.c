#include "gyro_MINS500.h"

static uint8_t Ccr_Check(uint8_t *message, uint8_t start_index, uint8_t len);

// MINS500初始化 使用DMA串口接收
void MINS500_Init(void)
{
    // HAL_UARTEx_ReceiveToIdle_DMA(&GYRO_SERIAL, handle_rev_buff, GYRO_LENGTH);       //使能接收中断
    // __HAL_DMA_DISABLE_IT(&GYRO_SERIAL_RX, DMA_IT_HT);             //关闭过半中断
}

// 陀螺仪数据处理
void MINS500_Data_Process(uint8_t *pdata, Gyro *gyro)
{
    if (pdata[0] == STDID && ((Ccr_Check(pdata, 1, 12) & 0xFF) == pdata[13]))
    {
        gyro->pitch = (float)((pdata[4] & 0x0F) * 100.0f + (pdata[5] >> 4) * 10.0f + (pdata[5] & 0x0F) +
                              (pdata[6] >> 4) * 0.1f + (pdata[6] & 0x0F) * 0.01f);

        gyro->roll = (float)((pdata[7] & 0x0F) * 100.0f + (pdata[8] >> 4) * 10.0f + (pdata[8] & 0x0F) +
                             (pdata[9] >> 4) * 0.1f + (pdata[9] & 0x0F) * 0.01f);

        gyro->yaw = (float)((pdata[10] & 0x0F) * 100.0f + (pdata[11] >> 4) * 10.0f + (pdata[11] & 0x0F) +
                            (pdata[12] >> 4) * 0.1f + (pdata[12] & 0x0F) * 0.01f);

        // 符号处理
        if (pdata[4] >> 4 == 0x01)
        {
            gyro->pitch = -gyro->pitch;
        }
        if (pdata[7] >> 4 == 0x01)
        {
            gyro->roll = -gyro->roll;
        }
        if (pdata[10] >> 4 == 0x01)
        {
            gyro->yaw = -gyro->yaw;
        }
		
    }

    //记录第一次上电的偏航角
    if (gyro->first_flag == 0)
    {
        gyro->last_yaw = gyro->yaw;
        gyro->real_yaw = 180.0f / 0.97297f;
        gyro->reset_yaw_cnt ++;
        gyro->first_flag = 1;
    }

//	gyro->yaw = gyro->yaw - gyro->first_yaw;
	
    // yaw角处理
    gyro->delta_yaw = gyro->yaw - gyro->last_yaw;
    if (gyro->delta_yaw > 180.0f)
    {
        gyro->delta_yaw -= 360.0f;
    }
    else if (gyro->delta_yaw < -180.0f)
    {
        gyro->delta_yaw += 360.0f;
    }

    gyro->real_yaw += gyro->delta_yaw;
    gyro->last_yaw = gyro->yaw;
	
	gyro->real_yaw_1 = gyro->real_yaw * 0.97297f;
}

// 校验计算 para：开始索引 数据长度
static uint8_t Ccr_Check(uint8_t *message, uint8_t start_index, uint8_t len)
{
    uint8_t ccr = 0;
    for (int i = start_index; i < start_index + len; i++)
    {
        ccr += message[i];
    }
    return ccr;
}
