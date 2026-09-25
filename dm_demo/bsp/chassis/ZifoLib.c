#include "ZifoLib.h"

/*---------------调试---------------*/
Debug_Zifo Zifo;
// BUG: xxxx —— 这里确定是个 bug，待修复
// FIXME: xxxx —— 这段代码逻辑有错或需要修改
// TODO: xxxx —— 功能未实现、待补充
// XXX: xxxx —— 实现太糙，以后优化
// BUG:
// FIXME:
// TODO:
// XXX:
/*---------------串口---------------*/
JustFloat_Frame vofa_frame = {.tail = {0x00, 0x00, 0x80, 0x7f}}; // justfloat协议要求帧尾
void fprint(UART_HandleTypeDef *huart,
                    float mes1, float mes2, float mes3,
                    float mes4, float mes5, float mes6,float mes7,float mes8,float mes9,float mes10,float mes11, float mes12, float mes13,float mes14, float mes15,float mes16)
{
    vofa_frame.fdata[0] = mes1;
    vofa_frame.fdata[1] = mes2;
    vofa_frame.fdata[2] = mes3;
    vofa_frame.fdata[3] = mes4;
    vofa_frame.fdata[4] = mes5;
    vofa_frame.fdata[5] = mes6;
		vofa_frame.fdata[6] = mes7;
		vofa_frame.fdata[7] = mes8;
		vofa_frame.fdata[8] = mes9;
		vofa_frame.fdata[9] = mes10;
		vofa_frame.fdata[10] = mes11;
		vofa_frame.fdata[11] = mes12;
		vofa_frame.fdata[12] = mes13;
		vofa_frame.fdata[13] = mes14;
		vofa_frame.fdata[14] = mes15;
		vofa_frame.fdata[15] = mes16;

    //Zifo.uart[1] = HAL_UART_Transmit_DMA(huart, (uint8_t*)&vofa_frame, sizeof(JustFloat_Frame));
    // PID 自动整定调试期：改用阻塞发送，排除 DMA 状态机/中断的一切变数（调完可换回 DMA）
    Zifo.uart[1] = HAL_UART_Transmit(huart, (uint8_t*)&vofa_frame, sizeof(JustFloat_Frame), HAL_MAX_DELAY);
}
/*---------------代数---------------*/
#include <math.h>
// 使用快速近似方法的角度转弧度
double deg_to_rad(float degrees)
{
    // 使用预计算的转换系数
    static const double DEG_TO_RAD = 0.0174532925199432957692369076848861271344287188854172545609719144f;
    return degrees * DEG_TO_RAD;
}

// 使用快速近似方法的弧度转角度
double rad_to_deg(float radians)
{
    // 使用预计算的转换系数
    static const double RAD_TO_DEG = 57.295779513082320876798154814105170332405472466564321549160243861f;
    return radians * RAD_TO_DEG;
}
/*---------------时间---------------*/
//#define CURRENT_TIME  (((float)HAL_GetTick()) / 1000.0f)

void DWT_Init(void)
{
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;  // 使能 DWT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;               // 使能周期计数器
    DWT->CYCCNT = 0;
}
/*---------------次数---------------*/
