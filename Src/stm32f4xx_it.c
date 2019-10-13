/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utilityFunctions.h"
#include "mainStateMachine.h"
#include "motors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
extern volatile uint16_t RCRRemainingValue;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */


/*
 * @brief This function handles the external interrupts
 * mostly the Limit switches: if the LS is SET then the motor reached MAX/MIN position - one direction is disabled.
 * 		if the LS is RESET (falling edge IT) -> the motor just left the MAX/MIN -> both directions are allowed again
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t temp = 0u;
	switch(GPIO_Pin){
		case GPIO_PIN_2:
			if(HAL_GPIO_ReadPin(LSPos_MotorR1_GPIO_Port, LSPos_MotorR1_Pin) == 1)
			{
				/* In this case, the Limit Switch is ON, pressed. The SW does not allow the motor to go further in this direction */
				stopMotorPWM(STMOTOR_R1_ID);
				temp = (getSTMotorAllowedDir(STMOTOR_R1_ID) & ~(MOTORALLOW_POSDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_R1_ID, temp);
			}
			else
			{
				temp = (getSTMotorAllowedDir(STMOTOR_R1_ID) | (MOTORALLOW_POSDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_R1_ID, temp);
			}
		break;

		case GPIO_PIN_3:
			if(HAL_GPIO_ReadPin(LSNeg_MotorR1_GPIO_Port, LSNeg_MotorR1_Pin) == 1)
			{
				/* In this case, the Limit Switch is ON, pressed. The SW does not allow the motor to go further in this direction */
				stopMotorPWM(STMOTOR_R1_ID);
				temp = (getSTMotorAllowedDir(STMOTOR_R1_ID) & ~(MOTORALLOW_NEGDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_R1_ID, temp);
			}
			else
			{
				temp = (getSTMotorAllowedDir(STMOTOR_R1_ID) | (MOTORALLOW_NEGDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_R1_ID, temp);
				}
		break;

		case GPIO_PIN_4:
			if(HAL_GPIO_ReadPin(LSPos_MotorPHorizontal_GPIO_Port, LSPos_MotorPHorizontal_Pin) == 1)
			{
				/* In this case, the Limit Switch is ON, pressed. The SW does not allow the motor to go further in this direction */
				stopMotorPWM(STMOTOR_PH_ID);
				temp = (getSTMotorAllowedDir(STMOTOR_PH_ID) & ~(MOTORALLOW_POSDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_PH_ID, temp);
			}
			else
			{
				temp = (getSTMotorAllowedDir(STMOTOR_PH_ID) | (MOTORALLOW_POSDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_PH_ID, temp);
			}
		break;

		case GPIO_PIN_5:
			if(HAL_GPIO_ReadPin(LSNeg_MotorPHorizontal_Pin, LSNeg_MotorPHorizontal_Pin) == 1)
			{
				/* In this case, the Limit Switch is ON, pressed. The SW does not allow the motor to go further in this direction */
				stopMotorPWM(STMOTOR_PH_ID);
				temp = (getSTMotorAllowedDir(STMOTOR_PH_ID) & ~(MOTORALLOW_NEGDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_PH_ID, temp);
			}
			else
			{
				temp = (getSTMotorAllowedDir(STMOTOR_PH_ID) | (MOTORALLOW_NEGDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_PH_ID, temp);
			}
		break;

		case GPIO_PIN_6:
			if(HAL_GPIO_ReadPin(LSPos_MotorPVertical_GPIO_Port, LSPos_MotorPVertical_Pin) == 1)
			{
				/* In this case, the Limit Switch is ON, pressed. The SW does not allow the motor to go further in this direction */
				stopMotorPWM(STMOTOR_PV_ID);
				temp = (getSTMotorAllowedDir(STMOTOR_PV_ID) & ~(MOTORALLOW_POSDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_PV_ID, temp);
			}
			else
			{
			temp = (getSTMotorAllowedDir(STMOTOR_PV_ID) | (MOTORALLOW_POSDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_PV_ID, temp);
			}
		break;

		case GPIO_PIN_7:
			if(HAL_GPIO_ReadPin(LSNeg_MotorPVertical_GPIO_Port, LSNeg_MotorPVertical_Pin) == 1)
			{
				/* In this case, the Limit Switch is ON, pressed. The SW does not allow the motor to go further in this direction */
				stopMotorPWM(STMOTOR_PV_ID);
				temp = (getSTMotorAllowedDir(STMOTOR_PV_ID) & ~(MOTORALLOW_NEGDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_PV_ID, temp);
			}
			else
			{
				temp = (getSTMotorAllowedDir(STMOTOR_PV_ID) | (MOTORALLOW_NEGDIR)) & MOTORALLOW_MASK;
				setSTMotorAllowedDir(STMOTOR_PV_ID, temp);
			}
		break;


		default:
			break;
		}
}



/**
 * This function handles TIM1 PWM callback
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		/* a pulse on any TIM1 channel is generated -> one step, lower RCR */
		RCRRemainingValue--;

		switch (htim->Channel)
		{
			/* count current movement */
			case HAL_TIM_ACTIVE_CHANNEL_1:
				incrementSTMotorPos(STMOTOR_R1_ID);
				if(posReached(STMOTOR_R1_ID))
				{
					stopMotorPWM(STMOTOR_R1_ID);
				}
				break;

			case HAL_TIM_ACTIVE_CHANNEL_2:
				incrementSTMotorPos(STMOTOR_PH_ID);
				if(posReached(STMOTOR_PH_ID))
				{
					stopMotorPWM(STMOTOR_PH_ID);
				}
				break;

			case HAL_TIM_ACTIVE_CHANNEL_3:
				incrementSTMotorPos(STMOTOR_PV_ID);
				if(posReached(STMOTOR_PV_ID))
				{
					stopMotorPWM(STMOTOR_PV_ID);
				}
				break;

			default:
				break;

		}
	}
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
