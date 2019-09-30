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
 * to be implemented: grabbing
 * */
uint8_t msmStateRunning(struct MSM_state* state)
{
	/* keep track of the current position */
	static uint8_t posIdx = 0u;
	uint8_t RCRValue = 0u;

	/* motors are not running OR the Timer1 is ready, this case the RCR has reached zero,
	 * but the arm is not yet at the desired position */
	if (((MotorR1.motorState == MOTORSTATE_STOPPED)			&&
		(MotorPHorizontal.motorState == MOTORSTATE_STOPPED) &&
		(MotorPVertical.motorState == MOTORSTATE_STOPPED)) ||
		(HAL_TIM_STATE_READY == &htim1.State))
	{
		/* if desired position is reached, set the next position */
		if (PosReached())
		{
			/* set desired position */
			setDesiredPos(posIdx);

			/* set the required directions of the motors */
			setDirections();
		}

		/* calc RCR - repetition counter register */
		RCRValue = calcRcrValue();

		/* Start the timer with the new RCRValue */
		ReInitMotorTimer(RCRValue);

		/* set other motorstate params and start pwm */
		StartMotorPWMs();
	}

	/* stop motor if reached desired position */
	StopMotor();

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




