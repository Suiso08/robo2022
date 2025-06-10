#include "Motor.hpp"
#include "main.h"
#include "stdlib.h"

Motor::Motor(TIM_HandleTypeDef *htim, uint16_t TIM_CHANNEL, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin):
        tim_name(htim), tim_channel(TIM_CHANNEL), gpio_name(GPIOx), gpio_pin(GPIO_Pin){;}

void Motor::init()
{
	if (HAL_TIM_PWM_Start(tim_name, tim_channel) != HAL_OK) { Error_Handler(); }

	__HAL_TIM_SET_COMPARE(tim_name, tim_channel, 0);
	HAL_GPIO_WritePin(gpio_name, gpio_pin, GPIO_PIN_RESET);//初期化の後にピンの設定をする
}

void Motor::speed(float speed)
{
	if(speed == 0)
	{
		__HAL_TIM_SET_COMPARE(tim_name, tim_channel, 0);
	}
	else
	{
		if(speed < 0.0)
			HAL_GPIO_WritePin(gpio_name, gpio_pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(gpio_name, gpio_pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(tim_name, tim_channel, abs((int)(speed * 19999)));
	}
}
