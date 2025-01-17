/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	GPIO_TypeDef* Group;
	uint16_t Pin;
}GPIO_FUNCTION_STRUCT;

typedef struct
{
	GPIO_FUNCTION_STRUCT* enable;
	GPIO_FUNCTION_STRUCT* direction;
	TIM_TypeDef* pulse;
	int motor_index;
	float current_degree;
	uint32_t motor_motion_start_cnt;
	uint32_t motor_motion_start_tick;
	int motor_motion_flag;
	float target_speed;
	float last_speed;
	float current_speed;
}STEPPER_MOTOR;

typedef struct
{
	GPIO_FUNCTION_STRUCT* enable;
	GPIO_FUNCTION_STRUCT* direction;
	TIM_TypeDef* timer;
	int motor_index;
	float current_degree;
	uint32_t motor_motion_start_cnt;
	uint32_t motor_motion_start_tick;
	int motor_motion_flag;
	float target_speed;
	float last_speed;
	float current_speed;
}DC_MOTOR;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define MOTOR_1 1

#define MOTOR_IDLE 0
#define MOTOR_READY_MOVING 1
#define MOTOR_MOVING 2
#define MOTOR_STOPING 3

#define MSG_HEAD_SIZE 4
#define MAX_BUFFER_SIZE 36
#define HMI_PARAMETERS_LENGTH 2

#define PAGE_INIT 0
#define PAGE_MAIN 1
#define PAGE_AUTO 2

#define EVENT_INCREASE_RPM 0xA1
#define EVENT_DECREASE_RPM 0xA2
#define EVENT_TURN_LEFT 0xA3
#define EVENT_TURN_RIGHT 0xA4
#define EVENT_STOP 0xA5
#define EVENT_START 0xA6
#define EVENT_AUTO 0xA7
#define EVENT_AUTO_HR_INC 0xB0
#define EVENT_AUTO_HR_DEC 0xB1
#define EVENT_AUTO_MIN_INC 0xB2
#define EVENT_AUTO_MIN_DEC 0xB3
#define EVENT_AUTO_SEC_INC 0xB4
#define EVENT_AUTO_SEC_DEC 0xB5
#define EVENT_AUTO_CANCEL 0xB6
#define EVENT_AUTO_SET 0xB7

#define PARAM_INCH_PER_SECOND 0
#define PARAM_DEGREE 1
#define PARAM_AUTOHOUR 2
#define PARAM_AUTOMIN 3
#define PARAM_AUTOSEC 4
#define VALUE_AUTOMODE_TIME 5
#define VALUE_MOVEMENT 6
#define VALUE_EXECUTION_TIME 7
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
/* USER CODE BEGIN Private defines */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern int page_index;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
