#include "handle_rev.h"

static float Rock_Process(uint16_t rock);
static uint8_t Ccr_Check(uint8_t *message, uint8_t start_index, uint8_t len);
static uint8_t Thumb_Process(uint16_t data);

Handle_Data handle_data;


// 手柄数据处理
//void Data_Process(uint8_t *pdata, Handle_Data *handle_data)
//{
//    Handle_Init_Data handle_init_data;
////    if (pdata[0] == FRAME_HEADER && pdata[17] == FRAME_END)
////    {
//	
//	for(uint8_t i = 0; i < 100; i++)
//	{
//		if(pdata[i] == FRAME_HEADER  && pdata[i+17] == FRAME_END)
//        {
//            if (Ccr_Check(pdata, (i + 1), (i + 15)) == pdata[i+16])
//            {
//                // 初步解析数据
//                handle_init_data.rock[0].u16 = (uint16_t)(pdata[i + 1] | (pdata[i + 2] << 8)); // 左摇杆X
//                handle_init_data.rock[1].u16 = (uint16_t)(pdata[i + 3] | (pdata[i + 4] << 8)); // 左摇杆Y
//                handle_init_data.rock[2].u16 = (uint16_t)(pdata[i + 5] | (pdata[i + 6] << 8)); // 右摇杆X
//                handle_init_data.rock[3].u16 = (uint16_t)(pdata[i + 7] | (pdata[i + 8] << 8)); // 右摇杆Y
//                handle_init_data.rock[4].u16 = (uint16_t)(pdata[i + 9] | (pdata[i + 10] << 8)); //左拨轮 
//                handle_init_data.rock[5].u16 = (uint16_t)(pdata[i + 11] | pdata[i + 12] << 8);  //右波轮

//                handle_init_data.key[0].key_state = pdata[i + 13];                          // 左边按键
//                handle_init_data.key[1].key_state = pdata[i + 14];                         // 右边按键
//                handle_init_data.key[2].key_state = pdata[i + 15];                          //中间按键

//                // 左摇杆
//                handle_data->rock_data.rock_left_x = -Rock_Process(handle_init_data.rock[0].u16);
//                handle_data->rock_data.rock_left_y = -Rock_Process(handle_init_data.rock[1].u16);

//                // 右摇杆
//                handle_data->rock_data.rock_right_x = -Rock_Process(handle_init_data.rock[2].u16);
//                handle_data->rock_data.rock_right_y = -Rock_Process(handle_init_data.rock[3].u16);

//                //左拨轮        ！！！需处理
//                handle_data->rock_data.left_thumb = Thumb_Process(handle_init_data.rock[4].u16);
//                //右拨轮
//                handle_data->rock_data.right_thumb = Thumb_Process(handle_init_data.rock[5].u16);

//                // 左半区按键
//                handle_data->key_data.key_left[0].key = handle_init_data.key[0].key0;
//                handle_data->key_data.key_left[1].key = handle_init_data.key[0].key1;
//                handle_data->key_data.key_left[2].key = handle_init_data.key[0].key2;
//                handle_data->key_data.key_left[3].key = handle_init_data.key[0].key3;
//                handle_data->key_data.key_left[4].key = handle_init_data.key[0].key4;
//                handle_data->key_data.key_left[5].key = handle_init_data.key[0].key5;

//                // 右半区按键
//                handle_data->key_data.key_right[0].key = handle_init_data.key[1].key0;
//                handle_data->key_data.key_right[1].key = handle_init_data.key[1].key1;
//                handle_data->key_data.key_right[2].key = handle_init_data.key[1].key2;
//                handle_data->key_data.key_right[3].key = handle_init_data.key[1].key3;
//                handle_data->key_data.key_right[4].key = handle_init_data.key[1].key4;
//                handle_data->key_data.key_right[5].key = handle_init_data.key[1].key5;

//                //中间按键
//                handle_data->key_data.key_middle[0].key = handle_init_data.key[2].key0;
//                handle_data->key_data.key_middle[1].key = handle_init_data.key[2].key1;
//                handle_data->key_data.key_middle[2].key = handle_init_data.key[2].key2;
//                handle_data->key_data.key_middle[3].key = handle_init_data.key[2].key3;
//                handle_data->key_data.key_middle[4].key = handle_init_data.key[2].key4;
//                handle_data->key_data.key_middle[5].key = handle_init_data.key[2].key5;
//                handle_data->key_data.key_middle[6].key = handle_init_data.key[2].key6;
//                handle_data->key_data.key_middle[7].key = handle_init_data.key[2].key7;
//    //        }
//            break;
//        }
//	}
//    }
//}
void Data_Process(uint8_t *pdata, Handle_Data *handle_data)
{
    Handle_Init_Data handle_init_data;
    if (pdata[0] == FRAME_HEADER && pdata[29] == FRAME_END)
    {
        if (Ccr_Check(pdata, 1, 27) == pdata[28])
        {
            // 初步解析数据
            handle_init_data.rock[0].u16 = (uint16_t)(pdata[1] | (pdata[2] << 8)); // 左摇杆X
            handle_init_data.rock[1].u16 = (uint16_t)(pdata[3] | (pdata[4] << 8)); // 左摇杆Y
            handle_init_data.rock[2].u16 = (uint16_t)(pdata[5] | (pdata[6] << 8)); // 右摇杆X
            handle_init_data.rock[3].u16 = (uint16_t)(pdata[7] | (pdata[8] << 8)); // 右摇杆Y
            handle_init_data.rock[4].u16 = (uint16_t)(pdata[9] | (pdata[10] << 8)); //左拨轮 
            handle_init_data.rock[5].u16 = (uint16_t)(pdata[11] | pdata[12] << 8);  //右波轮

            handle_init_data.key[0].key_state = pdata[13];                          // left
            handle_init_data.key[1].key_state = pdata[14];                         // right
            handle_init_data.key[2].key_state = pdata[15];                          // middle 

            for(uint8_t i =0 ;i < 12; i++)
            {
                handle_data->KFS[i] = pdata[16 + i];
            }

            // 左摇杆
            handle_data->rock_data.rock_left_x = -Rock_Process(handle_init_data.rock[0].u16);
            handle_data->rock_data.rock_left_y = -Rock_Process(handle_init_data.rock[1].u16);

            // 右摇杆
            handle_data->rock_data.rock_right_x = -Rock_Process(handle_init_data.rock[2].u16);
            handle_data->rock_data.rock_right_y = -Rock_Process(handle_init_data.rock[3].u16);

            //左拨轮        ！！！需处理
            handle_data->rock_data.left_thumb = Thumb_Process(handle_init_data.rock[4].u16);
            //右拨轮
            handle_data->rock_data.right_thumb = Thumb_Process(handle_init_data.rock[5].u16);

            // 左半区按键
            handle_data->key_data.key_left[0].key = handle_init_data.key[0].key0;
            handle_data->key_data.key_left[1].key = handle_init_data.key[0].key1;
            handle_data->key_data.key_left[2].key = handle_init_data.key[0].key2;
            handle_data->key_data.key_left[3].key = handle_init_data.key[0].key3;
            handle_data->key_data.key_left[4].key = handle_init_data.key[0].key4;
            handle_data->key_data.key_left[5].key = handle_init_data.key[0].key5;
            handle_data->key_data.key_left[6].key = handle_init_data.key[0].key6;
            handle_data->key_data.key_left[7].key = handle_init_data.key[0].key7;

            // 右半区按键
            handle_data->key_data.key_right[0].key = handle_init_data.key[1].key0;
            handle_data->key_data.key_right[1].key = handle_init_data.key[1].key1;
            handle_data->key_data.key_right[2].key = handle_init_data.key[1].key2;
            handle_data->key_data.key_right[3].key = handle_init_data.key[1].key3;
            handle_data->key_data.key_right[4].key = handle_init_data.key[1].key4;
            handle_data->key_data.key_right[5].key = handle_init_data.key[1].key5;
            handle_data->key_data.key_right[6].key = handle_init_data.key[1].key6;
            handle_data->key_data.key_right[7].key = handle_init_data.key[1].key7;

            //中间按键
            handle_data->key_data.key_middle[0].key = handle_init_data.key[2].key0;
            handle_data->key_data.key_middle[1].key = handle_init_data.key[2].key1;
            handle_data->key_data.key_middle[2].key = handle_init_data.key[2].key2;
            handle_data->key_data.key_middle[3].key = handle_init_data.key[2].key3;
            handle_data->key_data.key_middle[4].key = handle_init_data.key[2].key4;
            handle_data->key_data.key_middle[5].key = handle_init_data.key[2].key5;
            handle_data->key_data.key_middle[6].key = handle_init_data.key[2].key6;
            handle_data->key_data.key_middle[7].key = handle_init_data.key[2].key7;
        }
    }
}


//按键检测函数（检测上升沿）
bool Key_Process(Key *pkey)
{
    if (pkey->key == 1 && pkey->key_state == 0)
    {
        pkey->key_state = 1;
		return 1;
    }
    else if (pkey->key == 0)
    {
        pkey->key_state = 0;
        return 0;
    }
    return 0;
}




//num为按下次数
uint8_t Key_Detect(Key *pkey, uint8_t num)
{
	if(Key_Process(pkey))
	{
		pkey->press_num ++;
		if(pkey->press_num > num)
		{
			pkey->press_num = 1;
		}
		return pkey->press_num;
	}else
	{
		return 0;
	}
}



// 遥感数据处理
static float Rock_Process(uint16_t rock)
{
    float percent = 0.0f;
    if (rock >= 2295)
    {
        // 正向范围：2195-4095 映射到 0-100
        percent = (rock - 2195) / 19.0f;		//这个5.0f是补偿ADC采集到达不了4095以至于到达不了-100
    }
    else if (rock <= 1700 && rock > 0)
    {
        // 负向范围：0-1900 映射到 -100-0
        percent = ((rock - 1800) / 19.0f) - 6.0f;			
    }
    else
    {
        percent = 0.0f;
    }
    // 1900-2195之间的值保持为0（死区）
    return percent;
}


//拨轮数据处理
static uint8_t Thumb_Process(uint16_t data)
{
    if(data >= 450 && data <= 550)
    {
        return 1;               //左拨
    }else if(data >= 1900 && data <= 2100)
    {
        return 2;               //中按
    }else if(data >= 3680 && data <= 3780)
    {
        return 3;               //右拨
    }else
    {
        return 0;
    }
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
