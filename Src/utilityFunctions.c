/*
 * utilityFunctions.c
 *
 *  Created on: 30 Sep 2019
 *      Author: antal
 */

#include "utilityFunctions.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern dtMotor MotorR1;
extern dtMotor MotorPHorizontal;
extern dtMotor MotorPVertical;
extern position desiredPositions[];

/* @Utility function
 *
 */
uint8_t maxOfThree(uint8_t value1, uint8_t value2, uint8_t value3)
{
	uint8_t max = 0;
	max = (value1 > value2) ? value1 : value2;
	max = (max > value3) ? max : value3;

	return max;
}



/* Initialize positions to default value
 * ~ constructor
 */
void initPositions(const uint8_t pieceNumber)
{
	for(uint8_t idx = 0; idx < pieceNumber; idx++)
	{
		desiredPositions[idx].x = 0u;
		desiredPositions[idx].y = 0u;
		desiredPositions[idx].z = 0u;
		desiredPositions[idx].grabPos = 0u;
	}
}

/* Sets the motor desired positions to the desired positions
 * TODO: implement kinematics for proper work
 */
void setDesiredPos(const uint8_t posIdx)
{

	MotorR1.desiredPos = desiredPositions[posIdx].x;
	MotorPHorizontal.desiredPos = desiredPositions[posIdx].y;
	MotorPVertical.desiredPos = desiredPositions[posIdx].z;

	/* TODO: add the grabbing */
}


/* Calculates RCR value
 * RCR value could be bigger then uint8 but then it should be handled in other place
 */
uint8_t calcRcrValue()
{
	uint8_t RCRValue;
	/* distance between current pos and desired pos - abs value */
	uint32_t xTempu32 = (MotorR1.desiredPos > MotorR1.currPos) ? (MotorR1.desiredPos - MotorR1.currPos) : (MotorR1.currPos - MotorR1.desiredPos);
	uint32_t yTempu32 = (MotorPHorizontal.desiredPos > MotorPHorizontal.currPos) ? (MotorPHorizontal.desiredPos - MotorPHorizontal.currPos) : (MotorPHorizontal.currPos - MotorPHorizontal.desiredPos);
	uint32_t zTempu32 = (MotorPVertical.desiredPos > MotorPVertical.currPos) ? (MotorPVertical.desiredPos - MotorPVertical.currPos) : (MotorPVertical.currPos - MotorPVertical.desiredPos);

	RCRValue = maxOfThree((uint8_t)xTempu32, (uint8_t)yTempu32, (uint8_t)zTempu32);
	return RCRValue;
}


/*
 * Start the 3 motor's timer with the new RCR value
 */
void ReInitMotorTimer(const uint8_t RCRValue)
{
	htim1.Init.RepetitionCounter = RCRValue;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
}

/*
 * This function starts the 3 PWM signal generation for the 3 main motors
 */
void StartMotorPWMs()
{
	if (MOTORSTATE_STOPPED == MotorR1.motorState)
	{
		MotorR1.motorState = MOTORSTATE_RUNNING;
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	}

	if (MOTORSTATE_STOPPED == MotorPHorizontal.motorState)
	{
		MotorPHorizontal.motorState = MOTORSTATE_RUNNING;
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	}

	if (MOTORSTATE_STOPPED == MotorPVertical.motorState)
	{
		MotorPVertical.motorState = MOTORSTATE_RUNNING;
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
	}
}

/*
 * This function checks if any of the motors have to be stopped
 */
void StopMotor()
{
	if ((MOTORSTATE_RUNNING == MotorR1.motorState) && (MotorR1.currPos == MotorR1.desiredPos))
	{
		MotorR1.motorState = MOTORSTATE_STOPPED;
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
	}

	if ((MOTORSTATE_RUNNING == MotorPHorizontal.motorState) && (MotorPHorizontal.currPos == MotorPHorizontal.desiredPos))
	{
		MotorPHorizontal.motorState = MOTORSTATE_STOPPED;
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
	}

	if ((MOTORSTATE_RUNNING == MotorPVertical.motorState) && (MotorPVertical.currPos == MotorPVertical.desiredPos))
	{
		MotorPVertical.motorState = MOTORSTATE_STOPPED;
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_3);
	}
}


/*
 * Set the motor directions according to the needs
 */
void setDirections()
{
	MotorR1.dir = (MotorR1.desiredPos > MotorR1.currPos) ? MOTORDIR_POSITIVE : MOTORDIR_NEGATIVE;
	MotorPHorizontal.dir = (MotorPHorizontal.desiredPos > MotorPHorizontal.currPos) ? MOTORDIR_POSITIVE : MOTORDIR_NEGATIVE;
	MotorPVertical.dir = (MotorPVertical.desiredPos > MotorPVertical.currPos) ? MOTORDIR_POSITIVE : MOTORDIR_NEGATIVE;
}

/*
 * Check if the current position is the desired position
 */
bool PosReached()
{
	bool retVal = true;
	if (MotorR1.currPos != MotorR1.desiredPos)
	{
		retVal = false;
	}

	if (MotorPHorizontal.currPos != MotorPHorizontal.desiredPos)
	{
		retVal = false;
	}

	if (MotorPVertical.currPos != MotorPVertical.desiredPos)
	{
		retVal = false;
	}


	return retVal;
}
