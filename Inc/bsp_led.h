#ifndef _BSP_LED_H_
#define _BSP_LED_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#define LED_NUM	3

typedef	union
{
	struct{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t blank;
	}color;
	uint8_t ledrgb[4];
	uint32_t rgbcolor;
}rgbLEDduty;

typedef struct
{
	rgbLEDduty leds[LED_NUM];
	uint8_t lednum;
}LED_ConfigTypeDef;

void BSP_LED_INIT(LED_ConfigTypeDef *config);
void BSP_LED_REFRESH(LED_ConfigTypeDef *config);

#endif

