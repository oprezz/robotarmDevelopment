/*
 * encoder.h
 *
 *  Created on: 4 Nov 2019
 *      Author: antal
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f4xx_hal.h"

#define ENCODER_NR (uint8_t)3u
#define ENCODER_NR0_ZR20 (uint8_t)0u
#define ENCODER_NR0_MAXVALUE (uint32_t)100000u

typedef struct {
	uint8_t ID;
	volatile uint32_t counterValue;
	volatile uint32_t currentRotation; //whole rotation
	uint32_t _pMaxValue;
	TIM_HandleTypeDef* _pTimer;	//=TIM
} dtEncoder;

uint8_t initEncoder(uint8_t ID, TIM_HandleTypeDef* TIM, uint32_t maxValue);
volatile uint32_t getEncoderValue(uint8_t ID);
volatile uint32_t getEncoderRotation(uint8_t ID);
uint8_t resetEncoderValue(uint8_t ID);
uint8_t setRotation(uint8_t ID, uint32_t value);
uint8_t setEncoderValue(uint8_t ID, uint32_t value);
uint8_t incrementRotation(uint8_t ID);
uint8_t resetAllEncoders();



#endif /* ENCODER_H_ */
