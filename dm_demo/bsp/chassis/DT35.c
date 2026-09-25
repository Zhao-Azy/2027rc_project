#include "DT35.h"
#include "path.h"
#include "chassis_control.h"

static uint8_t Ccr_Check(uint8_t *message,uint8_t start_index,uint8_t len);


//DT35数据处理
void DT35_Data_Process(DT35_U *DT35, uint8_t *pdata)
{
    if(pdata[0] == 0xAF && pdata[18] == 0xFA)
    {
		if(pdata[17] == Ccr_Check(pdata, 1, 16))
        {
            for(uint8_t i=0; i < 16; i++)
            {
                DT35->data[i] = pdata[i + 1];
            }
        }
    }
}


void Air_pump_DataProcess(uint8_t *state, uint8_t *pdata)
{
    if(pdata[0] == 0xEF && pdata[6] == 0xFE)
    {
        if(pdata[5] == Ccr_Check(pdata, 1, 4))
        {
            for(uint8_t i = 0; i < 4; i++)
            {
                state[i] = pdata[i + 1];
            }
        }
    }

}


///计算从索引 start_index 开始，长度为 len 的数据的校验和
static uint8_t Ccr_Check(uint8_t *message,uint8_t start_index,uint8_t len)
{
  uint8_t ccr = 0;
  for(int i = start_index; i < start_index + len; i++)
  {
    ccr += message[i];
  }
  return ccr;
}




