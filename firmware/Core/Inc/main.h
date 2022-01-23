/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include <stdio.h>


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);
#define INDICATOR_PIN           GPIO_PIN_7
#define INDICATOR_PORT          GPIOB

#define MEAS_CTRL_PIN           GPIO_PIN_3
#define MEAS_CTRL_PORT          GPIOB


#define KX022_I2C_ADDRESS     (0x1F << 1)
#define KX022_WAI_VAL         (0x14)
 
#define KX022_XOUT_L          (0x06)
#define KX022_XOUT_H          (0x07)
#define KX022_YOUT_L          (0x08)
#define KX022_YOUT_H          (0x09)
#define KX022_ZOUT_L          (0x0A)
#define KX022_ZOUT_H          (0x0B)
#define KX022_WHO_AM_I        (0x0F)
#define KX022_CNTL1           (0x18)
#define KX022_CNTL3           (0x1A)
#define KX022_ODCNTL          (0x1B)
#define KX022_TILT_TIMER      (0x22)


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
