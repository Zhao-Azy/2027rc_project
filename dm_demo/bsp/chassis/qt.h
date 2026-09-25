#ifndef QT_H
#define QT_H

#include "main.h"



typedef struct
{
    uint8_t R1_route[3];
    uint8_t step;
    uint8_t R1_route_state[4];
    uint8_t index;      //解小电脑路径的索引
    uint8_t temp;       //外部用来更新点位的索引

    uint8_t calc_flag;      //是否允许解算块的位置，防止重复进入解算
}QT_HandleTypeDef;


void QT_Data_Process(QT_HandleTypeDef *qt, uint8_t *data);
void Qt_Reset(void);

#endif

