#include "Encoder.hpp"
#include "main.h"

Encoder::Encoder(TIM_HandleTypeDef *htim, TIM_TypeDef *ttim): phtim(htim), pttim(ttim){;}

void Encoder::init()
{
	HAL_TIM_Encoder_Start(phtim, TIM_CHANNEL_ALL);
	//HAL_TIM_Base_Start_IT(tim_name);
}

void Encoder::reset()
{
	pttim->CNT = 0;
	count = 0;
}

void Encoder::update()
{
	buffer = pttim->CNT;
	pttim->CNT = 0;
	count += buffer;
}

int16_t Encoder::getdiff()
{
	return buffer;
}

int16_t Encoder::getcount()
{
	return count;
}
