/*
 * utilityFunctions.c
 *
 *  Created on: 30 Sep 2019
 *      Author: antal
 */

#include "utilityFunctions.h"

/* External variables --------------------------------------------------------*/


/*
 * Toggle in-board LD4 pin for debugging info
 */
void LedToggle()
{
	HAL_GPIO_TogglePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin);
}
