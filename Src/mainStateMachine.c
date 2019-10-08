/*
 * mainStateMachine.c
 *
 *  Created on: 23 Sep 2019
 *      Author: antal
 */

#include "mainStateMachine.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern dtMotor MotorR1;
extern dtMotor MotorPHorizontal;
extern dtMotor MotorPVertical;
extern dtPosition desiredPositions[];


/* STATES */
MSM_state_fn msmStateReset, msmStateHoming, msmStateReady, msmStateLearning, msmStateRunning, msmStateError;



/* Reset state of the main state machine.
 * Loads the map, currently hard-coded map
 * to be implemented
 * */
uint8_t msmStateReset(struct MSM_state* state)
{
	/* load some positions - yet hard coded */
	char msg[] = {"load pos"};
	HAL_UART_Transmit(&huart1, (uint8_t*)(&msg), sizeof(msg)/sizeof(msg[0]), 1000);

	/* #0: initial position */
	desiredPositions[0] = (dtPosition){ .x = 0u, .y = 0u, .z = 0u, .grabPos = 0u};

	/* #1: 1st position */
	desiredPositions[1] = (dtPosition){ .x = 10u, .y = 12u, .z = 6u, .grabPos = 0u};

	/* #2: 2nd position */
	desiredPositions[2] = (dtPosition){ .x = 20u, .y = 12u, .z = 6u, .grabPos = 0u};

	/* #3: 3rd position */
	desiredPositions[3] = (dtPosition){ .x = 420u, .y = 12u, .z = 6u, .grabPos = 1u};

	/* #4: 4th position */
	desiredPositions[4] = (dtPosition){ .x = 15u, .y = 12u, .z = 6u, .grabPos = 1u};

	/* #5: 5th position */
	desiredPositions[5] = (dtPosition){ .x = 0u, .y = 0u, .z = 0u, .grabPos = 1u};

	/* #3: 6th position */
	desiredPositions[6] = (dtPosition){ .x = 0u, .y = 0u, .z = 0u, .grabPos = 0u};

	/* CONDITION TO BE ADDED */
	state->next = msmStateHoming;
	state->stateName = MSM_STATE_HOMING;

	return state->stateName;
}

/* Homing state of the main state machine.
 * homing statemachine is used
 * */
uint8_t msmStateHoming(struct MSM_state* state)
{
	static uint8_t HomingStates = HSM_STATE_RESETHOMEDBITS;

	switch (HomingStates)
	{
		case HSM_STATE_RESETHOMEDBITS:
			MotorR1.homed = 0u;
			MotorPHorizontal.homed = 0u;
			MotorPVertical.homed = 0;
			HomingStates = HSM_STATE_MOTORPH_NEG;
			break;
		case HSM_STATE_MOTORPH_NEG:
			break;

	}


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
	/* flag, is set, if the next position is further then the possible RCR value at once */
	static uint8_t RCRoverflow = 0u;
	uint8_t RCRValue = 0u;
	uint32_t setDirRetVal = 0u;

	/* DEBUG ONLY */
	char msg_01[4] = {""};
	char DEBUGMSG_u32[14] = {""};

	/* motors are not running OR the Timer1 is ready, this case the RCR has reached zero,
	 * but the arm is not yet at the desired position
	 * explanation the last statement after the OR:
	 * 	- RCRoverflow occured, because the next position is further then 255 steps (RCR is uint8)
	 * 	- RCRRemainingValue is the total nubmer of steps that the 3 channels has to step in one RCR cycle.
	 * 		if it has reached zero, then further steps are needed with the reinitialization of the timer
	 * */
	if (((MotorR1.motorState == MOTORSTATE_STOPPED)			&&
		(MotorPHorizontal.motorState == MOTORSTATE_STOPPED) &&
		(MotorPVertical.motorState == MOTORSTATE_STOPPED)) ||
		((1u == RCRoverflow) && (0u == RCRRemainingValue)))
	{

		/* if desired position is reached, set the next position */
		if (PosReached())
		{
			/* set desired position */
			setDesiredPos(posIdx);

			/* set the required directions of the motors */
			setDirRetVal = setAllDirectionsTowardsDesiredPos();

			posIdx = (posIdx == MAXPOSITIONS-1) ? (0u) : (posIdx + 1);

			/* debug only
			sprintf(msg_01, "%3d", posIdx);
			HAL_UART_Transmit(&huart1, (uint8_t*)(&msg_01), sizeof(msg_01)/sizeof(msg_01[0]), 1000);
			*/
		}

		/* no error */
		if (0u == setDirRetVal)
		{
			/* calc RCR - repetition counter register , set RCRoverflow if needed*/
			RCRValue = calcRcrValue(&RCRoverflow);

			/* Start the timer with the new RCRValue */
			ReInitMotorTimer(RCRValue);

			/* set other motorstate params and start pwm */
			StartMotorPWMs();
		}
		/* error in setting the direction */
		else
		{
			/* TODO: error data shall be sent later */
			state->next = msmStateError;
			state->stateName = MSM_STATE_ERROR;
			return state->stateName;
		}
	}

	/* stopping the motors are handled in IT */
	/* stop motor if reached desired position */
	//StopMotor();

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




