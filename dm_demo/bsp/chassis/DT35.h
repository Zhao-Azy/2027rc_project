#ifndef DT35_H
#define DT35_H

#include "main.h"


typedef union 
{
    float value[4];
    uint8_t data[16];
}DT35_U;


void DT35_Data_Process(DT35_U *DT35, uint8_t *pdata);
void Air_pump_DataProcess(uint8_t *state, uint8_t *pdata);
#endif
