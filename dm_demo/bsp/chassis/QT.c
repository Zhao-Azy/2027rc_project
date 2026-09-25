#include "QT.h"
#include "chassis_control.h"
#include "path.h"
#include <string.h>

static uint8_t Ccr_Check(uint8_t *message,uint8_t start_index,uint8_t len);


void QT_Data_Process(QT_HandleTypeDef *qt, uint8_t *data)
{
    //解手机的包
	if((Ccr_Check(data, 0, 12) & 0x0F) == 0x0E)
	{
        //这里因为手机传包的原因，只能倒着解包
		for(int i = 11; i >= 0; i --)
        {
			if(data[i] == 0x01)             //如果是R1的块
			{
                if(qt->step < 3)
                {
                    qt->R1_route[qt->step] = i + 1;
                    qt->step ++;
                }                                    
			}
		}
    }
}


//清空函数
void Qt_Reset(void)
{
    qt.step = 0;
    qt.index = 0;
    qt.calc_flag = 0;
    memset(qt.R1_route, 0, sizeof(qt.R1_route));                  //解出手机的块的位置
    memset(QT_State, 0, sizeof(QT_State));                          //R1块的位置
    memset(qt.R1_route_state, 0, sizeof(qt.R1_route_state));      //R1需要取得块的位置
}


static uint8_t Ccr_Check(uint8_t *message,uint8_t start_index,uint8_t len)
{
  uint8_t ccr = 0;
  for(int i = start_index; i < start_index + len; i++)
  {
    ccr += message[i];
  }
  return ccr;
}
