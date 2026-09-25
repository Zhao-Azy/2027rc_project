#ifndef HANDLE_REV_H
#define HANDLE_REV_H

#include "main.h"
#include "stdbool.h"


#define FRAME_HEADER 0xAA
#define FRAME_END 0xBB


//按键联合体
typedef union 
{
    struct
    {
        bool key0 : 1;
        bool key1 : 1;
        bool key2 : 1; 
        bool key3 : 1;
        bool key4 : 1;
        bool key5 : 1;
        bool key6 : 1;
        bool key7 : 1;
    };
	
    uint8_t key_state;   
}Key_u;


//ADC联合体
typedef union 
{
    uint8_t u8[2];
    uint16_t u16;
}Rock_u;


//手柄接收数据结构体
typedef struct 
{
    //手柄摇杆,拨轮
    Rock_u rock[6];
    //按键
    Key_u key[4];
}Handle_Init_Data;


//摇杆数据结构体  采用前x左y的定义方式
typedef struct 
{
    float rock_left_x;
    float rock_left_y;
    float rock_right_x;
    float rock_right_y;
    
    uint8_t left_thumb;         //左拨轮
    uint8_t right_thumb;        //右拨轮
}Rock_Data;


//单个按键结构体 
typedef struct 
{
    uint8_t key;
    uint8_t key_state;
    uint8_t press_num;
}Key;


//按键数据结构体
typedef struct 
{
    Key key_left[8];
    Key key_right[8];
    Key key_middle[8];
}Key_Data;



//最终摇杆数据key_DR
typedef struct 
{
    Rock_Data rock_data;
    Key_Data key_data;
    uint8_t KFS[12];
}Handle_Data;

extern Handle_Data handle_data;

// extern uint8_t handle_rev_buff[HANDLE_LENGTH];           //手柄数据缓冲区



void Data_Process(uint8_t *pdata, Handle_Data *handle_data);
bool Key_Process(Key *key);
uint8_t Key_Detect(Key *pkey, uint8_t num);

#endif
