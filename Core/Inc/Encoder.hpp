#ifndef _ENCODER_HPP_
#define _ENCODER_HPP_
#include "main.h"

class Encoder{
public:
	Encoder(TIM_HandleTypeDef *htim, TIM_TypeDef *ttim);
	void init();
	void reset();
	void update();
	int16_t getdiff();
	int16_t getcount();
private:
	TIM_HandleTypeDef *phtim;
	TIM_TypeDef *pttim;
	uint16_t buffer;
	int16_t count;
};

#endif //_ENCODER_HPP_
