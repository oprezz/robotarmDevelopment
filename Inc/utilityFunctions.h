/*
 * utilityFunctions.h
 *
 *  Created on: 30 Sep 2019
 *      Author: antal
 */

#ifndef UTILITYFUNCTIONS_H_
#define UTILITYFUNCTIONS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "mainStateMachine.h"

typedef struct
{
	uint32_t x;
	uint32_t y;
	uint32_t z;
	uint8_t grabPos; // 0 - open, 1 - grabbed, rest is reserved for special grabbing positions
} position;

void initPositions(const uint8_t pieceNumber);
uint8_t maxOfThree(uint8_t value1, uint8_t value2, uint8_t value3);
void setDesiredPos(const uint8_t posIdx);
uint8_t calcRcrValue();
void ReInitMotorTimer(const uint8_t RCRValue);
void StartMotorPWMs();
bool PosReached();
void StopMotor();
void setDirections();

#endif /* UTILITYFUNCTIONS_H_ */
