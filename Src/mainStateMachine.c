/*
 * mainStateMachine.c
 *
 *  Created on: 23 Sep 2019
 *      Author: antal
 */

#include "mainStateMachine.h"


/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern dtMotor MotorR1;
extern dtMotor MotorPHorizontal;
extern dtMotor MotorPVertical;



/* STATES */
MSM_state_fn msmStateReset, msmStateHoming, msmStateReady, msmStateLearning, msmStateRunning, msmStateError;



/* @Utility function
 * TODO: ABS value is not yet implemented
 */
uint8_t maxOfThree(uint8_t value1, uint8_t value2, uint8_t value3)
{
	uint8_t max = 0;
	max = (value1 > value2) ? value1 : value2;
	max = (max > value3) ? max : value3;

	return max;
}


/* Reset state of the main state machine.
 * to be implemented
 * */
uint8_t msmStateReset(struct MSM_state* state)
{
	/* CONDITION TO BE ADDED */
	state->next = msmStateHoming;
	state->stateName = MSM_STATE_HOMING;

	return state->stateName;
}

/* Homing state of the main state machine.
 * to be implemented
 * */
uint8_t msmStateHoming(struct MSM_state* state)
{
	state->next = msmStateReady;
	state->stateName = MSM_STATE_READY;

	return state->stateName;
}

/* Ready state of the main state machine.
 * to be implemented
 * */
uint8_t msmStateReady(struct MSM_state* state)
{
	state->next = msmStateLearning;
	state->stateName = MSM_STATE_LEARNING;

	return state->stateName;
}

/* Learning state of the main state machine.
 * to be implemented
 * */
uint8_t msmStateLearning(struct MSM_state* state)
{
	state->next = msmStateRunning;
	state->stateName = MSM_STATE_RUNNING;

	return state->stateName;
}

/* Running state of the main state machine.
 * to be implemented
 * */
uint8_t msmStateRunning(struct MSM_state* state)
{
	uint8_t RCRValue = 0;
	if ((MotorR1.motorState == 0) && (MotorPHorizontal.motorState == 0))
	{
		/* set des pos */


		MotorR1.desiredPos = 15;
		MotorPHorizontal.desiredPos = 6;

		/* calc rcr and init timer */
		RCRValue = maxOfThree(MotorR1.desiredPos-MotorR1.currPos, MotorPHorizontal.desiredPos-MotorPHorizontal.currPos, MotorPVertical.desiredPos - MotorPVertical.currPos);
		htim1.Init.RepetitionCounter = RCRValue;
		if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
		{
			Error_Handler();
		}

		/* set other motor params and start pwm */
		MotorR1.motorState = 1;
		MotorR1.dir = 2;
		MotorPHorizontal.motorState = 1;
		MotorPHorizontal.dir = 2;
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	}

	if (MotorPHorizontal.currPos == MotorPHorizontal.desiredPos)
	{

		MotorPHorizontal.motorState = 2;
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
	}

	if (MotorR1.currPos == MotorR1.desiredPos)
	{
		MotorR1.motorState = 2;
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
	}

	return state->stateName;
}

/* Error state of the main state machine.
 * to be implemented
 * */
uint8_t msmStateError(struct MSM_state* state)
{
	state->next = msmStateError;
	state->stateName = MSM_STATE_ERROR;

	return state->stateName;
}
/* */
