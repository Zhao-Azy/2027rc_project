#ifndef WS2812_H__
#define WS2812_H__

#include <stdbool.h>
#include "dma.h"
#include "tim.h"

#define LED_COLOR_WHITE     0xff, 0xff, 0xff  // white
#define LED_COLOR_BLACK     0x00, 0x00, 0x00  // off
#define LED_COLOR_RED       0xff, 0x00, 0x00  // red
#define LED_COLOR_GREEN     0x00, 0x00, 0x52  // green, board wiring swaps G/B
#define LED_COLOR_BLUE      0x00, 0x80, 0x00  // blue, board wiring swaps G/B
#define LED_COLOR_GRAY   0x80, 0x80, 0x80  // red + blue
#define LED_COLOR_PURPLE    0x80, 0x80, 0x00  // purple
#define LED_COLOR_GREEN     0x00, 0x00, 0x52  // green
#define LED_COLOR_YELLOW    0xd0, 0x00, 0xd0
#define LED_COLOR_PURPLE    0x80, 0x80, 0x00  // red + blue
#define LED_COLOR_ORANGE    0xff, 0x00, 0x30  // red + a little green


typedef struct WS2811_Handle WS2811_HandleTypeDef;

struct WS2811_pf
{
	void (*setColor)(WS2811_HandleTypeDef* hsw2811, uint8_t startNum, uint8_t endNum, uint8_t R, uint8_t G, uint8_t B);
	void (*flicker)(WS2811_HandleTypeDef *strip,
                           uint8_t start,
                           uint8_t end,
                           uint8_t on_r,
                           uint8_t on_g,
                           uint8_t on_b,
                           uint32_t period_ms);
};

struct WS2811_Status
{
	TIM_HandleTypeDef 	*htim;
	bool 				initBool;
	uint8_t 			ledNum;
	uint16_t 			*txData;
	uint32_t			pwmChannel;
};

struct WS2811_Handle
{
	struct WS2811_pf		*pf;
	struct WS2811_Status 	status;
};

void WS2811_init(WS2811_HandleTypeDef *hsw2811, TIM_HandleTypeDef *htim, uint32_t pwmChannel, uint8_t ledNum, uint16_t* sTxData, uint16_t sTxDataLen);

#define WS2812_REFRESH_DATA_NUM (10 * 24)

#define WS2811_Init(WS2811, htim, pwmChannel, ledNum)	\
static uint16_t WS2811##_RxData[ledNum * 24 + WS2812_REFRESH_DATA_NUM] = {0};\
WS2811_init(&WS2811, &htim, pwmChannel, ledNum, WS2811##_RxData, ledNum * 24 + WS2812_REFRESH_DATA_NUM)

#endif
