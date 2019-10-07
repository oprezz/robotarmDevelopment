/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* motor typedef */
typedef struct{
	const uint8_t ID;
	volatile uint32_t currPos;
	volatile uint32_t desiredPos;
	bool homed;
	volatile uint8_t dir; 		// 0: negative direction, 1: positive direction
	uint8_t allowedDir;
	uint8_t motorState; // 0: stopped, 1: running
} dtMotor;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ButtonPos_MotorPVertical_Pin GPIO_PIN_3
#define ButtonPos_MotorPVertical_GPIO_Port GPIOF
#define ButtonNeg_MotorPVertical_Pin GPIO_PIN_4
#define ButtonNeg_MotorPVertical_GPIO_Port GPIOF
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define UART2_TX_STM_Pin GPIO_PIN_2
#define UART2_TX_STM_GPIO_Port GPIOA
#define UART2_RX_STM_Pin GPIO_PIN_3
#define UART2_RX_STM_GPIO_Port GPIOA
#define ADC1_IN4_reservedforfutureuse_Pin GPIO_PIN_4
#define ADC1_IN4_reservedforfutureuse_GPIO_Port GPIOA
#define SPI1_SCK_GPIOEXTENDER_Pin GPIO_PIN_5
#define SPI1_SCK_GPIOEXTENDER_GPIO_Port GPIOA
#define Encoder1MotorR1_Pin GPIO_PIN_6
#define Encoder1MotorR1_GPIO_Port GPIOA
#define SPI1_MOSI_GPIOEXTENDER_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIOEXTENDER_GPIO_Port GPIOA
#define ButtonPos_MotorR1_Pin GPIO_PIN_0
#define ButtonPos_MotorR1_GPIO_Port GPIOG
#define ButtonPos_MotorR1G1_Pin GPIO_PIN_1
#define ButtonPos_MotorR1G1_GPIO_Port GPIOG
#define STEP_MotorR1_Pin GPIO_PIN_9
#define STEP_MotorR1_GPIO_Port GPIOE
#define STEP_MotorPHorizontal_Pin GPIO_PIN_11
#define STEP_MotorPHorizontal_GPIO_Port GPIOE
#define STEP_MotorPVertical_Pin GPIO_PIN_13
#define STEP_MotorPVertical_GPIO_Port GPIOE
#define UART3_TX_reservedforfutureuse_Pin GPIO_PIN_10
#define UART3_TX_reservedforfutureuse_GPIO_Port GPIOB
#define UART3_RX_reservedforfutureuse_Pin GPIO_PIN_11
#define UART3_RX_reservedforfutureuse_GPIO_Port GPIOB
#define STEP_gripperMotorThumb_Pin GPIO_PIN_12
#define STEP_gripperMotorThumb_GPIO_Port GPIOD
#define STEP_gripperMotorIndexMiddle_Pin GPIO_PIN_13
#define STEP_gripperMotorIndexMiddle_GPIO_Port GPIOD
#define STEP_gripperMotorRingPinky_Pin GPIO_PIN_14
#define STEP_gripperMotorRingPinky_GPIO_Port GPIOD
#define LSPos_MotorR1_Pin GPIO_PIN_2
#define LSPos_MotorR1_GPIO_Port GPIOG
#define LSNeg_MotorPHorizontal_Pin GPIO_PIN_5
#define LSNeg_MotorPHorizontal_GPIO_Port GPIOG
#define LSPos_MotorPVertical_Pin GPIO_PIN_6
#define LSPos_MotorPVertical_GPIO_Port GPIOG
#define LSNeg_MotorPVertical_Pin GPIO_PIN_7
#define LSNeg_MotorPVertical_GPIO_Port GPIOG
#define GPIO_PG8_Interrupt_reservedforfutureuse_Pin GPIO_PIN_8
#define GPIO_PG8_Interrupt_reservedforfutureuse_GPIO_Port GPIOG
#define Encoder2MotorR1_Pin GPIO_PIN_7
#define Encoder2MotorR1_GPIO_Port GPIOC
#define UART1_TX_STLINK_PC_Pin GPIO_PIN_9
#define UART1_TX_STLINK_PC_GPIO_Port GPIOA
#define UART1_RX_STLINK_PC_Pin GPIO_PIN_10
#define UART1_RX_STLINK_PC_GPIO_Port GPIOA
#define ButtonPos_MotorPHorizontal_Pin GPIO_PIN_13
#define ButtonPos_MotorPHorizontal_GPIO_Port GPIOA
#define ButtonNeg_MotorPHorizontal_Pin GPIO_PIN_14
#define ButtonNeg_MotorPHorizontal_GPIO_Port GPIOA
#define ButtonGrip_Actuator_Pin GPIO_PIN_9
#define ButtonGrip_Actuator_GPIO_Port GPIOG
#define ButtonRelease_Actuator_Pin GPIO_PIN_10
#define ButtonRelease_Actuator_GPIO_Port GPIOG
#define Button_EMERGENCY_Pin GPIO_PIN_11
#define Button_EMERGENCY_GPIO_Port GPIOG
#define GreenLed_LD3_Pin GPIO_PIN_13
#define GreenLed_LD3_GPIO_Port GPIOG
#define RedLed_LD4_Pin GPIO_PIN_14
#define RedLed_LD4_GPIO_Port GPIOG
#define SPI1_MISO_GPIOEXTENDER_Pin GPIO_PIN_4
#define SPI1_MISO_GPIOEXTENDER_GPIO_Port GPIOB
#define I2C1_SCL_reservedforfutureuse_Pin GPIO_PIN_6
#define I2C1_SCL_reservedforfutureuse_GPIO_Port GPIOB
#define I2C1_SDA_reservedforfutureuse_Pin GPIO_PIN_7
#define I2C1_SDA_reservedforfutureuse_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* motor defines */
#define MOTORSTATE_RUNNING (uint8_t)1u
#define MOTORSTATE_STOPPED (uint8_t)0u
#define MOTORDIR_POSITIVE (uint8_t)1u
#define MOTORDIR_NEGATIVE (uint8_t)0u

/* Positions */
#define MAXPOSITIONS (uint8_t)12u

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
