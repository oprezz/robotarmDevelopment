/*
 * motors.h
 *
 *  Created on: 8 Oct 2019
 *      Author: antal
 */

#ifndef MOTORS_H_
#define MOTORS_H_


#include "main.h"
#include "utilityFunctions.h"
#include <stdio.h>
#include "encoder.h"

/* BEGIN defines */
/* motor defines */
#define MOTORSTATE_RUNNING (uint8_t)1u
#define MOTORSTATE_STOPPED (uint8_t)0u
#define MOTORDIR_TODESIREDPOS	(uint8_t)3u
#define MOTORDIR_UNDEFINED	(uint8_t)2u
#define MOTORDIR_POSITIVE 	(uint8_t)1u
#define MOTORDIR_NEGATIVE 	(uint8_t)0u
#define MOTORALLOW_BOTHDIR	(uint8_t)3u //0b0000_0011
#define MOTORALLOW_POSDIR	(uint8_t)2u	//0b0000_0010
#define MOTORALLOW_NEGDIR	(uint8_t)1u	//0b0000_0001
#define MOTORALLOW_NODIR	(uint8_t)0u	//0b0000_0000
#define MOTORALLOW_MASK		(uint8_t)0b00000011 //3u
#define STMOTOR_R1_ID		(uint8_t)0u
#define STMOTOR_PH_ID		(uint8_t)1u
#define STMOTOR_PV_ID		(uint8_t)2u
#define STMOTOR_NR			(uint8_t)3u

/* Positions */
#define MAXPOSITIONS (uint8_t)12u

/* general */
#define UINT8T_MAXV (uint8_t)255u

/* END defines */

/* BEGIN type definitions */
typedef struct
{
	uint32_t x;
	uint32_t y;
	uint32_t z;
	uint8_t grabPos; // 0 - open, 1 - grabbed, rest is reserved for special grabbing positions
} dtPosition;


/* motor typedef */
typedef struct{
	uint8_t ID;
	volatile uint32_t currPos;
	volatile uint32_t desiredPos;
	bool homed;
	volatile uint8_t dir; 		// 0: negative direction, 1: positive direction, 2: undefined
	/* 3 (0b11): both directions are allowed,
	 * 2 (0b10): only Positive direction is allowed
	 * 1 (0b01): only Negative direction is allowed
	 * 0 (0b01): No direction is allowed
	 */
	uint8_t allowedDir;
	uint8_t motorState; // 0: stopped, 1: running, 2: wanted to set wrong direction
	/* these are needed, so more general functions can be implemented
	 * COM is not listed here, as that shall always be 0 */
	uint32_t TIM_CH;	//0:0x00000000, 1:0x..04, 2:0x..08, :0x0..0C
	TIM_HandleTypeDef* TIM; //address of the used timer
	uint16_t enablePIN;
	GPIO_TypeDef* enablePORT;
	uint16_t dirPIN;
	GPIO_TypeDef* dirPORT;
} dtStepperMotor;


typedef struct{
	uint8_t ID;
	uint32_t TIM_CH;	//0:0x00000000, 1:0x..04, 2:0x..08, :0x0..0C
	TIM_HandleTypeDef* TIM; //address of the used timer
	uint16_t enablePIN;
	GPIO_TypeDef* enablePORT;
	uint16_t dirPIN;
	GPIO_TypeDef* dirPORT;
} dtStepperMotorInit;
/* END type definitions */

/* BEGIN getter functions */
volatile uint32_t getSTMotorCurrPos(uint8_t ID);
volatile uint32_t getSTMotorDesiredPos(uint8_t ID);
bool getSTMotorHomed(uint8_t ID);
volatile uint8_t getSTMotorDir(uint8_t ID);
uint8_t getSTMotorAllowedDir(uint8_t ID);
uint8_t getSTMotorMotorState(uint8_t ID);
uint32_t getSTMotorTIMCH(uint8_t ID);
TIM_HandleTypeDef* getSTMotorTIM(uint8_t ID);
uint16_t getSTMotorEnablePIN(uint8_t ID);
GPIO_TypeDef* getSTMotorEnablePORT(uint8_t ID);
uint16_t getSTMotorDirPIN(uint8_t ID);
GPIO_TypeDef* getSTMotorDirPORT(uint8_t ID);
/* END getter functions */

/* BEGIN setter functions */
/* THESE functions are very basic, most needs update */
void setSTMotorCurrPos(uint8_t ID, uint32_t value);
void setSTMotorDesiredPos(uint8_t ID, uint32_t value);
void setSTMotorHomed(uint8_t ID, bool value);
void setSTMotorAllowedDir(uint8_t ID, uint8_t value);
void setSTMotorMotorState(uint8_t ID, uint8_t value);
void setSTMotorTIMCH(uint8_t ID, uint32_t value);
/* END setter functions */


/* BEGIN API function definitions */
bool initStepperMotors();
uint8_t setMotorDirection(const uint8_t ID, const uint8_t desiredDirection);
uint8_t getMotorNumbers();
uint32_t setAllDirectionsTowardsDesiredPos();
bool posReached(const uint8_t ID);
bool posAllReached();
void setDesiredPos(const uint8_t ID, const uint32_t desiredPosition);
void setAllDesiredPos(const uint8_t posIdx);
void startMotorPWM(const uint8_t ID);
void startAllMotorPWMs();
void reInitMotorTimer(const uint8_t RCRValue);
void stopMotorPWM(const uint8_t ID);
void stopAllMotorBasedPos();
uint8_t FreeRunMotorInDesiredDir(const uint8_t MotorID, const uint8_t desDir);
void incrementSTMotorPos(const uint8_t ID);


/* END API function definitions */


/* might be moved */
uint8_t calcRcrValue(uint8_t *RCRoverflow);
void initPositions();

#endif /* MOTORS_H_ */
