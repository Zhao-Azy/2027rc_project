#pragma once

#include "main.h"
#include "core_cm7.h"
#include <assert.h>
#include "usart.h"
/*Keil的math库和vscode里面的math库不一样*/
#define M_E		     2.71828182845904523540
#define M_LOG2E		 1.44269504088896340740
#define M_LOG10E	 0.43429448190325182765
#define M_LN2		   0.69314718055994530942
#define M_LN10		 2.30258509299404568402
//#define M_PI		   3.14159265358979323846
#define M_PI_2		 1.57079632679489661923
#define M_PI_4		 0.78539816339744830962
#define M_1_PI		 0.31830988618379067154
#define M_2_PI		 0.63661977236758134308
#define M_2_SQRTPI 1.12837916709551257390
#define M_SQRT2		 1.41421356237309504880
#define M_SQRT1_2	 0.70710678118654752440
#define MY_INFINITY 0x7F800000U

#define CURRENT_TIME  (((float)HAL_GetTick()) / 1000.0f)

#define  TIME_START()  uint32_t _t_start = DWT->CYCCNT
#define  TIME_END()    uint32_t _t_cost = DWT->CYCCNT - _t_start

typedef struct
{
  union
  {
    HAL_StatusTypeDef uart[4];
    HAL_StatusTypeDef fdcan[3];
    HAL_StatusTypeDef general[4];
  };//匿名结构体,gnu99扩展

	struct
	{
		uint16_t chassis;
		uint16_t encoder;
		uint16_t area;
		uint16_t gyro;
	}r;
	uint32_t error_num;
	uint32_t r_num[8];
	float temp_float[16];
	uint8_t temp_u8[16];
}Debug_Zifo;

#define CH_count 16  // 通道数
#pragma pack(push, 1)  // 确保结构体无填充字节（压栈对齐方式）
typedef struct {
    float fdata[CH_count];
    uint8_t tail[4];   // 固定帧尾
} JustFloat_Frame;
#pragma pack(pop)      // 恢复原有对齐方式


extern Debug_Zifo Zifo;
double deg_to_rad(float degrees);
double rad_to_deg(float radians);
void fprint(UART_HandleTypeDef *huart,
                    float mes1, float mes2, float mes3,
                    float mes4, float mes5, float mes6,float mes7,float mes8,float mes9,float mes10,float mes11, float mes12, float mes13,float mes14, float mes15,float mes16);
