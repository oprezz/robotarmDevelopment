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
#include "motors.h"
#include "stm32f4xx_hal_tim.h"

#define UINT8T_MAXV (uint8_t)255u

/* global variable definitions */


/* function definitions */
void LedLD3OFF();
void LedLD3ON();
void LedLD3Toggle();
void LedLD4OFF();
void LedLD4ON();
void LedLD4Toggle();

#endif /* UTILITYFUNCTIONS_H_ */
