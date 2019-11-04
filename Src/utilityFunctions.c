/*
 * utilityFunctions.c
 *
 *  Created on: 30 Sep 2019
 *      Author: antal
 */

#include "utilityFunctions.h"

/* External variables --------------------------------------------------------*/

/* Reset in-board LD3 pin for debugging info */
void LedLD3OFF()
{
	HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, GPIO_PIN_RESET);
}

/* Set in-board LD3 pin for debugging info */
void LedLD3ON()
{
	HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, GPIO_PIN_SET);
}

/* Toggle in-board LD4 pin for debugging info */
void LedLD3Toggle()
{
	HAL_GPIO_TogglePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin);
}



/* Reset in-board LD4 pin for debugging info */
void LedLD4OFF()
{
	HAL_GPIO_WritePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin, GPIO_PIN_RESET);
}

/* Set in-board LD4 pin for debugging info */
void LedLD4ON()
{
	HAL_GPIO_WritePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin, GPIO_PIN_SET);
}

/* Toggle in-board LD4 pin for debugging info */
void LedLD4Toggle()
{
	HAL_GPIO_TogglePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin);
}


