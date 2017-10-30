#include "bsp_led.h"

extern TIM_HandleTypeDef htim3;
void BSP_LED_INIT(LED_ConfigTypeDef *config)
{
   HAL_GPIO_WritePin(RGB0_GPIO_Port,RGB0_Pin,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(RGB1_GPIO_Port,RGB1_Pin,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(RGB2_GPIO_Port,RGB2_Pin,GPIO_PIN_RESET);
	
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,config->leds[0].color.blue);
   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,config->leds[0].color.red);
   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,config->leds[0].color.green);
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}

void BSP_LED_REFRESH(LED_ConfigTypeDef *config)
{

	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,config->leds[0].color.blue);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,config->leds[0].color.red);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,config->leds[0].color.green);
		
}
