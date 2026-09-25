#include "WS2811.h"
#include "cmsis_os.h"
#include <stdlib.h>

#define LED_HIGH 50
#define LED_LOW 25


void WS2811_setColor(WS2811_HandleTypeDef* hsw2811, uint8_t startNum, uint8_t endNum, uint8_t R, uint8_t G, uint8_t B);
void WS2811_flicker(WS2811_HandleTypeDef *strip,
                           uint8_t start,
                           uint8_t end,
                           uint8_t on_r,
                           uint8_t on_g,
                           uint8_t on_b,
                           uint32_t period_ms);
void WS2811_ErrHander();


struct WS2811_pf sWS2811_pf = 
{
	.setColor = WS2811_setColor,
	.flicker = WS2811_flicker,
};


void WS2811_init(WS2811_HandleTypeDef *hsw2811, TIM_HandleTypeDef *htim, uint32_t pwmChannel, uint8_t ledNum, uint16_t* sTxData, uint16_t sTxDataLen)
{
	if (hsw2811->status.initBool == false)
	{
		if (sTxDataLen != (ledNum * 24 + WS2812_REFRESH_DATA_NUM))
		{
			WS2811_ErrHander();
		}
		
		hsw2811->pf = &sWS2811_pf;
		hsw2811->status.txData = sTxData;
		hsw2811->status.htim = htim;
		hsw2811->status.ledNum = ledNum;
		hsw2811->status.initBool = true;
		hsw2811->status.pwmChannel = pwmChannel;
	}
}


void WS2811_setColor(WS2811_HandleTypeDef* hsw2811, uint8_t startNum, uint8_t endNum, uint8_t R, uint8_t G, uint8_t B)
{
	if (startNum == 0 || endNum > hsw2811->status.ledNum || startNum > endNum) 
	{
		WS2811_ErrHander();
		return;
	}
	
	uint8_t color[3] = {R, G, B}; 
	for (uint8_t i = startNum - 1; i < endNum; i++) //startNum从1开始
	{
		for (uint8_t c = 0; c < 3; c++) 
		{
			for (int8_t bit = 7; bit >= 0; bit--) 
			{
				if (color[c] & (1 << bit)) 
				{
					hsw2811->status.txData[i * 24 + c * 8 + (7 - bit) + WS2812_REFRESH_DATA_NUM] = LED_HIGH;
				} else {
					hsw2811->status.txData[i * 24 + c * 8 + (7 - bit) + WS2812_REFRESH_DATA_NUM] = LED_LOW;
				}
			}
		}
	}
	HAL_TIM_PWM_Start_DMA(hsw2811->status.htim, hsw2811->status.pwmChannel, (uint32_t *)hsw2811->status.txData, hsw2811->status.ledNum * 24 + WS2812_REFRESH_DATA_NUM);
}

void WS2811_flicker(WS2811_HandleTypeDef *strip,
                           uint8_t start,
                           uint8_t end,
                           uint8_t on_r,
                           uint8_t on_g,
                           uint8_t on_b,
                           uint32_t period_ms)
{
    static uint32_t last_toggle_tick = 0;
    static uint8_t blink_on = 0;

    if (xTaskGetTickCount() - last_toggle_tick >= period_ms) {
        last_toggle_tick = xTaskGetTickCount();
        blink_on = !blink_on;
    }

    if (blink_on) {
        strip->pf->setColor(strip, start, end, on_r, on_g, on_b);
    } else {
        strip->pf->setColor(strip, start, end, 0, 0, 0);
    }
}


void WS2811_ErrHander()
{
	__disable_irq();
    while (1)
    {
    }
}
