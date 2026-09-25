#ifndef RADAR_H
#define RADAR_H

#include "main.h"



typedef struct
{
    float x;
    float y;
    float z;

    float translation_distance;                     //当前对接差的距离 单位m
    float high_err;                                 //对接高度误差
    float front_err;                                //对接前后误差

    uint8_t error_flag;                                //错误标志位
    uint8_t route_flag;                                 //路径标志位
    uint8_t transmit_block_flag;                      //递块标志位

    uint8_t data_flag;                              //数据是否丢失标志位  串口一直收不到则置1
    uint8_t nuc_flag;                   //是否给小电脑发送数据标志位

}Radar_HandleTypeDef;





void Radar_Data_Process(Radar_HandleTypeDef *radar, uint8_t *pdata);



#endif

