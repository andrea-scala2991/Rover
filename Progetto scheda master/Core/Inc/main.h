/*
    Copyright (C) 2025

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

TRACCIA: 
Progettare e implementare un rover in grado di eseguire i comandi ricevuti tramite un controller il rover può trovarsi in tre diversi stati operativi: OK quando il sistema funziona correttamente senza anomalie, DEGRADATO in caso di malfunzionamento parziale che riduce alcune funzionalità ma mantiene il rover operativo, EMERGENZA quando il rover entra in uno stato di sicurezza per prevenire danni limitando o sospendendo le operazioni.
*/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define echoS2_Pin GPIO_PIN_8
#define echoS2_GPIO_Port GPIOC
#define triggerS2_Pin GPIO_PIN_9
#define triggerS2_GPIO_Port GPIOC
#define echoS3_Pin GPIO_PIN_8
#define echoS3_GPIO_Port GPIOA
#define triggerS3_Pin GPIO_PIN_9
#define triggerS3_GPIO_Port GPIOA
#define echoS1_Pin GPIO_PIN_10
#define echoS1_GPIO_Port GPIOC
#define triggerS1_Pin GPIO_PIN_11
#define triggerS1_GPIO_Port GPIOC
#define rele1_Pin GPIO_PIN_12
#define rele1_GPIO_Port GPIOC
#define rele2_Pin GPIO_PIN_2
#define rele2_GPIO_Port GPIOD
#define B1_pin_Pin GPIO_PIN_4
#define B1_pin_GPIO_Port GPIOB
#define B2_pin_Pin GPIO_PIN_5
#define B2_pin_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
