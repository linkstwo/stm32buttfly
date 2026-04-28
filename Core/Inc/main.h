/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

void Error_Handler(void);

#define PWM_M2_1_Pin GPIO_PIN_2
#define PWM_M2_1_GPIO_Port GPIOA
#define PWM_M2_2_Pin GPIO_PIN_3
#define PWM_M2_2_GPIO_Port GPIOA
#define PWM_M1_2_Pin GPIO_PIN_4
#define PWM_M1_2_GPIO_Port GPIOA
#define PWM_M1_1_Pin GPIO_PIN_3
#define PWM_M1_1_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
