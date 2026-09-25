#ifndef __STP_23L_H_
#define __STP_23L_H_

#include "main.h"

// #define L_DISTANCW_MIN 0.026f
#define L_DISTANCW_MIN 0.026f
#define R_DISTANCE_MIN 0.017f

typedef struct 
{
    int16_t   distance;          //测量距离
    uint16_t  noise;             //环境噪声
    uint32_t  peak;              //接收强度信息
    uint8_t   confidence;        //置信度
    uint32_t  intg;              //积分次数
    int16_t   reftof;            //温度表征值
}STP_DATA;  

extern float stp23l_distance_left;
extern float stp23l_distance_right;

void STP23L_Distance_Process(uint8_t *data,float *stp23l_distance);

#endif
