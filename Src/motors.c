/*
 * motors.c
 *
 *  Created on: 8 Oct 2019
 *      Author: antal
 *      This file handles the THREE stepper motor of the system.
 *      Some functions have guards for the THREE motor. Don't attach more without rewriting the code!
 */

#include "motors.h"
#include "main.h"
#include "mainStateMachine.h"

/* Global variables */
extern volatile uint16_t RCRRemainingValue;
extern volatile uint16_t testRCRRemainingValue;

/* extern global variables */
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

/* Private variables */

static dtStepperMotor StepperMotors[STMOTOR_NR]; //MotorR1, MotorPHorizontal, MotorPVertical

/* positions - 12 piece , init in main*/
dtPosition desiredPositions[MAXPOSITIONS];

/* BEGIN static functions */
static uint8_t maxOfThree(uint8_t value1, uint8_t value2, uint8_t value3);
/* END static functions */

/*
 * @brief initializes the stepper motors, call it once around startupcode
 */
bool initStepperMotors(dtStepperMotorInit motor)
{
	static uint8_t MotorIdx = 0;
	dtStepperMotor tempMotor;
	if (MotorIdx != STMOTOR_NR)
	{

		StepperMotors[MotorIdx++] = (dtStepperMotor){
				.ID = motor.ID,
				.currPos = 0u,
				.desiredPos = 0u,
				.homed = false,
				.dir = MOTORDIR_POSITIVE,
				.allowedDir = MOTORALLOW_BOTHDIR,
				.motorState = MOTORSTATE_STOPPED,
				.TIM_CH = motor.TIM_CH,
				.TIM = motor.TIM,
				.enablePIN = motor.enablePIN,
				.enablePORT = motor.enablePORT,
				.dirPIN = motor.dirPIN,
				.dirPORT = motor.dirPORT
		};
		return true;
	}
	else
	{
		return false;
	}

	HAL_GPIO_WritePin(MotorR1_ENABLE_GPIO_Port, MotorR1_ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MotorPHorizontal_ENABLE_GPIO_Port, MotorPHorizontal_ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MotorPVertical_ENABLE_GPIO_Port, MotorPVertical_ENABLE_Pin, GPIO_PIN_SET);

}

/* getter functions */
volatile uint32_t getSTMotorCurrPos(uint8_t ID)
{
	return StepperMotors[ID].currPos;
}

volatile uint32_t getSTMotorDesiredPos(uint8_t ID)
{
	return StepperMotors[ID].desiredPos;
}

bool getSTMotorHomed(uint8_t ID)
{
	return StepperMotors[ID].homed;
}

volatile uint8_t getSTMotorDir(uint8_t ID)
{
	return StepperMotors[ID].dir;
}

uint8_t getSTMotorAllowedDir(uint8_t ID)
{
	return StepperMotors[ID].allowedDir;
}

uint8_t getSTMotorMotorState(uint8_t ID)
{
	return StepperMotors[ID].motorState;
}

uint32_t getSTMotorTIMCH(uint8_t ID)
{
	return StepperMotors[ID].TIM_CH;
}

TIM_HandleTypeDef* getSTMotorTIM(uint8_t ID)
{
	return StepperMotors[ID].TIM;
}

uint16_t getSTMotorEnablePIN(uint8_t ID)
{
	return StepperMotors[ID].enablePIN;
}

GPIO_TypeDef* getSTMotorEnablePORT(uint8_t ID)
{
	return StepperMotors[ID].enablePORT;
}

uint16_t getSTMotorDirPIN(uint8_t ID)
{
	return StepperMotors[ID].dirPIN;
}

GPIO_TypeDef* getSTMotorDirPORT(uint8_t ID)
{
	return StepperMotors[ID].dirPORT;
}


/* setter functions */
void setSTMotorCurrPos(uint8_t ID, uint32_t value)
{
	StepperMotors[ID].currPos = value;
}

void setSTMotorDesiredPos(uint8_t ID, uint32_t value)
{
	StepperMotors[ID].desiredPos = value;
}

void setSTMotorHomed(uint8_t ID, bool value)
{
	StepperMotors[ID].homed = value;
}

void setSTMotorAllowedDir(uint8_t ID, uint8_t value)
{
	StepperMotors[ID].allowedDir = value;
}

void setSTMotorMotorState(uint8_t ID, uint8_t value)
{
	StepperMotors[ID].motorState = value;
}

void setSTMotorTIMCH(uint8_t ID, uint32_t value)
{
	StepperMotors[ID].TIM_CH = value;
}

/*
 * @brief Sets the direction of the motor received in @param1
 * to the desiredDirection received in @param2
 * if @param2 is MOTORDIR_TODESIREDPOS that means, we want to set the direction
 * so that the motor goes into the desired position (preset on the motorProperty)
 * before setting the desired direction the function checks if that direction is enabled
 */
uint8_t setMotorDirection(const uint8_t ID, const uint8_t desiredDirection)
{
	uint8_t retVal = 0u;
	if(MOTORDIR_TODESIREDPOS == desiredDirection)
	{
		if (StepperMotors[ID].desiredPos > StepperMotors[ID].currPos)
		{
			/* reinvoke the function with new paramters */
			return retVal = setMotorDirection(ID, MOTORDIR_POSITIVE);
		}
		else
		{
			/* reinvoke the function with new paramters */
			return retVal = setMotorDirection(ID, MOTORDIR_NEGATIVE);
		}
	}
	else if (MOTORDIR_POSITIVE == desiredDirection)
	{
		if((StepperMotors[ID].allowedDir & MOTORALLOW_POSDIR) >> (MOTORALLOW_POSDIR-1u))
		{
			StepperMotors[ID].dir = MOTORDIR_POSITIVE;
		}
		else
		{
			StepperMotors[ID].dir = MOTORDIR_UNDEFINED;
			retVal |= (uint8_t)0x01;
		}
	}
	else if (MOTORDIR_NEGATIVE == desiredDirection)
	{
		if((StepperMotors[ID].allowedDir & MOTORALLOW_NEGDIR) >> (MOTORALLOW_NEGDIR-1u))
		{
			StepperMotors[ID].dir = MOTORDIR_NEGATIVE;
		}
		else
		{
			LedLD4ON();
			StepperMotors[ID].dir = MOTORDIR_UNDEFINED;
			retVal |= (uint8_t)0x02;
		}
	}
	else if (MOTORDIR_UNDEFINED == desiredDirection)
	{
		/* this one is probably not used */
		StepperMotors[ID].dir = MOTORDIR_UNDEFINED;
		retVal |= (uint8_t)0x03;
	}
	else
	{
		StepperMotors[ID].dir = MOTORDIR_UNDEFINED;
		retVal |= (uint8_t)0x04;
	}

	/* Write the necessary output */
	if (0u == retVal)
	{
		HAL_GPIO_WritePin(StepperMotors[ID].dirPORT, StepperMotors[ID].dirPIN, desiredDirection);
	}

	return retVal;
}

/*
 * @brief returns the number of motors
 */
uint8_t getMotorNumbers()
{
	return STMOTOR_NR;
}

/*
 * Set the motor directions according to the needs
 *	@retVal |=
 *		0b0000 0000(0x00): no error in setting the direction
 *		0b0000 0001(0x01): error in setting the direction @MotorR1
 *		0b0000 0010(0x02): error in setting the direction @MotorPH
 *		0b0000 0100(0x04): error in setting the direction @MotorPV
 */
uint32_t setAllDirectionsTowardsDesiredPos()
{
	uint32_t retVal = 0u;
	char DEBUGMSG_10[10] = {""};

	if( getMotorNumbers() == 3u)
	{
		retVal = (setMotorDirection(STMOTOR_R1_ID, MOTORDIR_TODESIREDPOS) << 16);
		retVal = (setMotorDirection(STMOTOR_PH_ID, MOTORDIR_TODESIREDPOS) << 8);
		retVal = setMotorDirection(STMOTOR_PV_ID, MOTORDIR_TODESIREDPOS);
	}
	else { retVal = 0xFFFFFFFF; }
	return retVal;
}

/*
 * @brief Check if the current position is the desired position
 */
bool posReached(const uint8_t ID)
{
	bool retVal = true;
	if (StepperMotors[ID].currPos != StepperMotors[ID].desiredPos)
	{
		retVal = false;
	}

	return retVal;
}
/*
 * @brief checks if each motor has reached its desired position
 */
bool posAllReached()
{
	bool retVal = true;
	uint8_t tempMotorNumbers = getMotorNumbers();

	if (tempMotorNumbers == 3u)
	{
		for(uint8_t idx = 0; idx < tempMotorNumbers; idx++)
		{
			retVal &= posReached(idx);
		}
	}
	else { retVal = false; }
	return retVal;
}

/*
 * @brief  Sets ONE motor's desired position to the desired position
 */
void setDesiredPos(const uint8_t ID, const uint32_t desiredPosition)
{
	StepperMotors[ID].desiredPos = desiredPosition;
}

/*
 * @brief Sets the motor desired positions to the desired positions
 * TODO: implement kinematics for proper work
 */
void setAllDesiredPos(const uint8_t posIdx)
{
	setDesiredPos(STMOTOR_R1_ID, desiredPositions[posIdx].x);
	setDesiredPos(STMOTOR_PH_ID, desiredPositions[posIdx].y);
	setDesiredPos(STMOTOR_PV_ID, desiredPositions[posIdx].z);

	/* TODO: add the grabbing */
}


/*
 * @brief starts the PWM of the motor
 */
void startMotorPWM(const uint8_t ID)
{
	StepperMotors[ID].motorState = MOTORSTATE_RUNNING;
	HAL_TIM_PWM_Start_IT(&htim1, StepperMotors[ID].TIM_CH);
}

/*
 * This function starts the 3 PWM signal generation for the 3 main motors
 */
void startAllMotorPWMs()
{
	uint8_t tempMotorNumber = getMotorNumbers();

	for (uint8_t idx = 0; idx < tempMotorNumber; idx++)
	{
		if (!posReached(idx))
		{
			startMotorPWM(idx);
		}
	}
}

/*
 * @brief Start the 3 motor's timer with the new RCR value
 */
void reInitMotorTimer(const uint8_t RCRValue)
{
	/* this could be more general, but the 3 motor is never likely to change timer-base
	 * and now they all shave timer1, this function works well like this */
	htim1.Init.RepetitionCounter = RCRValue;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
}



/*
 * @brief This function stops the motor received in the parameter
 */
void stopMotorPWM(const uint8_t ID)
{
	StepperMotors[ID].motorState = MOTORSTATE_STOPPED;
	HAL_TIM_PWM_Stop_IT(&htim1, StepperMotors[ID].TIM_CH);
}


/*
 * @brief This function checks if any of the motors have to be stopped based on the current and desired position
 */
void stopAllMotorBasedPos()
{
	uint8_t tempMotorNumber = getMotorNumbers();

	for (uint8_t idx = 0; idx < tempMotorNumber; idx++)
	{
		if ((MOTORSTATE_RUNNING == getSTMotorMotorState(idx)) && posReached(idx))
		{
			stopMotorPWM(idx);
		}
	}
}

/* TODO */
uint8_t FreeRunMotorInDesiredDir(const uint8_t ID, const uint8_t desDir)
{
	uint8_t retVal = 0u;
	return retVal;
}



/*
 * @Utility function
 */
static uint8_t maxOfThree(uint8_t value1, uint8_t value2, uint8_t value3)
{
	uint8_t max = 0;
	max = (value1 > value2) ? value1 : value2;
	max = (max > value3) ? max : value3;

	return max;
}



/**************************/
/* these are being under consideration to be transferred to another .h.c file */

/*
 * @brief Calculates RCR value
 * RCR value could be bigger then uint8 but then it should be handled in other place
 */
uint8_t calcRcrValue(uint8_t *RCRoverflow)
{
	uint8_t RCRValue;
	/* distance between current pos and desired pos - abs value */
	uint32_t xTempu32 = (getSTMotorDesiredPos(STMOTOR_R1_ID) > getSTMotorCurrPos(STMOTOR_R1_ID)) ? (getSTMotorDesiredPos(STMOTOR_R1_ID) - getSTMotorCurrPos(STMOTOR_R1_ID)) : (getSTMotorCurrPos(STMOTOR_R1_ID) - getSTMotorDesiredPos(STMOTOR_R1_ID));
	uint32_t yTempu32 = (getSTMotorDesiredPos(STMOTOR_PH_ID) > getSTMotorCurrPos(STMOTOR_PH_ID)) ? (getSTMotorDesiredPos(STMOTOR_PH_ID) - getSTMotorCurrPos(STMOTOR_PH_ID)) : (getSTMotorCurrPos(STMOTOR_PH_ID) - getSTMotorDesiredPos(STMOTOR_PH_ID));
	uint32_t zTempu32 = (getSTMotorDesiredPos(STMOTOR_PV_ID) > getSTMotorCurrPos(STMOTOR_PV_ID)) ? (getSTMotorDesiredPos(STMOTOR_PV_ID) - getSTMotorCurrPos(STMOTOR_PV_ID)) : (getSTMotorCurrPos(STMOTOR_PV_ID) - getSTMotorDesiredPos(STMOTOR_PV_ID));

	/* count the total steps of the 3 CH in one RCR cycle */
	RCRRemainingValue = (uint8_t)xTempu32 + (uint8_t)yTempu32 + (uint8_t)zTempu32;
	//testRCRRemainingValue = (uint8_t)xTempu32 + (uint8_t)yTempu32 + (uint8_t)zTempu32;
	if ((xTempu32 > UINT8T_MAXV) || (yTempu32 > UINT8T_MAXV) || (zTempu32 > UINT8T_MAXV))
	{
		*RCRoverflow = 1;
		RCRValue = 255;
	}
	else
	{
		*RCRoverflow = 0;
		RCRValue = maxOfThree((uint8_t)xTempu32, (uint8_t)yTempu32, (uint8_t)zTempu32);
	}

	return RCRValue;
}

/*
 * @brief increments the motor's current position to the corresponding direction
 */
void incrementSTMotorPos(const uint8_t ID)
{
	uint32_t tCurrPos = getSTMotorCurrPos(ID);
	uint8_t tDir = getSTMotorDir(ID);
	tCurrPos = (tDir == MOTORDIR_POSITIVE) ? (tCurrPos + 1) : (tCurrPos - 1);
	setSTMotorCurrPos(ID, tCurrPos);
}



/* Initialize positions to default value
 * ~ constructor
 */
void initPositions()
{
	for(uint8_t idx = 0; idx < MAXPOSITIONS; idx++)
	{
		desiredPositions[idx] = (dtPosition){
			.x = 0u,
			.y = 0u,
			.z = 0u,
			.grabPos = 0u
		};
	}
}
