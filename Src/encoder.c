/*
 * encoder.c
 *
 *  Created on: 4 Nov 2019
 *      Author: antal
 */

#include "encoder.h"
#include "stm32f4xx_hal_tim.h"

static dtEncoder EncoderList[ENCODER_NR];

uint8_t initEncoder(uint8_t ID, TIM_HandleTypeDef* TIM, uint32_t maxValue)
{
	static uint8_t EncoderIdx = 0;
	uint8_t retVal = 0u;
	if (EncoderIdx != ENCODER_NR)
	{
		EncoderList[EncoderIdx++] = (dtEncoder){
			.ID = ID,
			.counterValue = 0u,
			.currentRotation = 0u,
			._pMaxValue = maxValue,
			._pTimer = TIM
		};
	}

	HAL_TIM_Encoder_Start_IT(EncoderList[ID]._pTimer, TIM_CHANNEL_ALL);
	return retVal;
}

volatile uint32_t getEncoderValue(uint8_t ID)
{
	EncoderList[ID].counterValue = (uint32_t)__HAL_TIM_GET_COUNTER(EncoderList[ID]._pTimer);
	return EncoderList[ID].counterValue;
}

volatile uint32_t getEncoderRotation(uint8_t ID)
{
	return EncoderList[ID].currentRotation;
}

uint8_t resetEncoderValue(uint8_t ID)
{
	uint8_t retVal = 0u;
	EncoderList[ID].counterValue = 0u;
	__HAL_TIM_SetCounter(EncoderList[ID]._pTimer, 0u);
	EncoderList[ID].currentRotation = 0u;
	return retVal;
}

uint8_t setEncoderValue(uint8_t ID, uint32_t value)
{
	uint8_t retVal = 0u;
	EncoderList[ID].counterValue = value;
	__HAL_TIM_SetCounter(EncoderList[ID]._pTimer, value);
	//EncoderList[ID].currentRotation = 0u;
	return retVal;
}

uint8_t incrementRotation(uint8_t ID)
{
	uint8_t retVal = 0u;
	EncoderList[ID].currentRotation++;
	return retVal;
}

uint8_t setRotation(uint8_t ID, uint32_t value)
{
	uint8_t retVal = 0u;
	EncoderList[ID].currentRotation = value;
	return retVal;
}

uint8_t resetAllEncoders()
{
	uint8_t retVal = 0;
	for(uint8_t idx = 0u; idx != ENCODER_NR; idx++)
	{
		resetEncoderValue(idx);
		setRotation(idx, 0u);
	}
	return retVal;
}


