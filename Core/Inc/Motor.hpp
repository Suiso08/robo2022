#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_
#include "main.h"

class Motor{
public:
    Motor(TIM_HandleTypeDef *htim, uint16_t TIM_CHANNEL, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
    void init();
    void speed(float speed);

private:
    TIM_HandleTypeDef *tim_name;
    uint16_t tim_channel;
    GPIO_TypeDef *gpio_name;
    uint16_t gpio_pin;

};

#endif //_MOTOR_HPP_
