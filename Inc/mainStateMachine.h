/*
 * mainStateMachine.h
 *
 *  Created on: 23 Sep 2019
 *      Author: antal
 */

#ifndef MAINSTATEMACHINE_H_
#define MAINSTATEMACHINE_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "utilityFunctions.h"
#include <stdio.h>


/* main state machine structure declaration */
typedef enum{
	MSM_STATE_RESET = (uint8_t)0u,
	MSM_STATE_HOMING = (uint8_t)10u,
	MSM_STATE_READY = (uint8_t)20u,
	MSM_STATE_LEARNING = (uint8_t)30u,
	MSM_STATE_RUNNING = (uint8_t)40u,
	MSM_STATE_ERROR = (uint8_t)50u
} FSM_mainStateMachine;

struct MSM_state;

/* MSM function type definition */
typedef uint8_t MSM_state_fn(struct MSM_state *);

struct MSM_state
{
	MSM_state_fn* next;
	FSM_mainStateMachine stateName;

	/* data for the states */
};


/* main state machine states: */
uint8_t msmStateReset(struct MSM_state*);
uint8_t msmStateHoming(struct MSM_state*);
uint8_t msmStateReady(struct MSM_state*);
uint8_t msmStateLearning(struct MSM_state*);
uint8_t msmStateRunning(struct MSM_state*);
uint8_t msmStateError(struct MSM_state*);
/* */


#endif /* MAINSTATEMACHINE_H_ */
