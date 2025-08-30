/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "com_master.h"
#include "controller.h"
#include "hcsr04.h"
#include "MPU-6050.h"
#include "temperature.h"
#include "spi.h"
#include "tim.h"


void dataCheck(uint16_t start_temp,HAL_StatusTypeDef statusController,partial_master_t* data_master);
void degraded(partial_master_t* global_data);
void emergency();

partial_master_t data_master;
global_t global_data;
char usart_buffer2[70];
HAL_StatusTypeDef status;

uint8_t buffer_recive[16];
uint8_t buffer_send[33];

uint32_t tick_iniziale;
uint32_t tick_finale;
uint8_t status_boards=-1;

uint8_t final_decision;

UltrasonicSensor sensor1, sensor2, sensor3;


char u[50];
uint16_t start_temp=0;
HAL_StatusTypeDef statusController;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for lettura */
osThreadId_t letturaHandle;
const osThreadAttr_t lettura_attributes = {
  .name = "lettura",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for comunicazione */
osThreadId_t comunicazioneHandle;
const osThreadAttr_t comunicazione_attributes = {
  .name = "comunicazione",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for degradato */
osThreadId_t degradatoHandle;
const osThreadAttr_t degradato_attributes = {
  .name = "degradato",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for emergenza */
osThreadId_t emergenzaHandle;
const osThreadAttr_t emergenza_attributes = {
  .name = "emergenza",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void letturaTask(void *argument);
void comunicazioneTask(void *argument);
void degradatoTask(void *argument);
void emergenzaTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	Ultrasonic_Init(&sensor1, triggerS1_GPIO_Port, triggerS1_Pin, echoS1_GPIO_Port, echoS1_Pin, &htim1);
	Ultrasonic_Init(&sensor2, triggerS2_GPIO_Port, triggerS2_Pin, echoS2_GPIO_Port, echoS2_Pin, &htim2);
	Ultrasonic_Init(&sensor3, triggerS3_GPIO_Port, triggerS3_Pin, echoS3_GPIO_Port, echoS3_Pin,&htim7);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of lettura */
  letturaHandle = osThreadNew(letturaTask, NULL, &lettura_attributes);

  /* creation of comunicazione */
  comunicazioneHandle = osThreadNew(comunicazioneTask, NULL, &comunicazione_attributes);

  /* creation of degradato */
  degradatoHandle = osThreadNew(degradatoTask, NULL, &degradato_attributes);

  /* creation of emergenza */
  emergenzaHandle = osThreadNew(emergenzaTask, NULL, &emergenza_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_letturaTask */
/**
  * @brief  Function implementing the lettura thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_letturaTask */
void letturaTask(void *argument)
{
  /* USER CODE BEGIN letturaTask */
  /* Infinite loop */
  for(;;)
  {
	    //LETTURA
		if(data_master.temperatura2>90 || data_master.temperatura2==-1){start_temp++;}
		else{start_temp=0;}

		data_master.sonar1=(uint16_t)Ultrasonic_GetDistance(&sensor1);
		data_master.sonar2=(uint16_t)Ultrasonic_GetDistance(&sensor2);
		data_master.sonar3=(uint16_t)Ultrasonic_GetDistance(&sensor3);

		statusController=HAL_I2C_Master_Receive(&hi2c1, 0x55 << 1, (uint8_t *)&data_master.controller, sizeof(controller_t), 2000);

		dataCheck(start_temp, statusController, &data_master);

    osDelay(1);
  }
  /* USER CODE END letturaTask */
}

/* USER CODE BEGIN Header_comunicazioneTask */
/**
* @brief Function implementing the comunicazione thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_comunicazioneTask */
void comunicazioneTask(void *argument)
{
  /* USER CODE BEGIN comunicazioneTask */
  /* Infinite loop */
  for(;;)
  {

		//reset variabili di stato e decisione
		status_boards=3;
		data_master.B2_decision=4;
		final_decision = 3;

		//Invio dati allo slave
		serialize_partial(&data_master, buffer_send);

		HAL_GPIO_WritePin(GPIOB, B2_pin_Pin, GPIO_PIN_SET);

		tick_iniziale=osKernelGetTickCount();

		while (HAL_GPIO_ReadPin(GPIOB, B1_pin_Pin) == GPIO_PIN_RESET){
			if(osKernelGetTickCount()-tick_iniziale > 10){
				status_boards=0;
			    goto fine;
			}
		}

		HAL_SPI_Transmit(&hspi1, (uint8_t*)buffer_send, sizeof(buffer_send), HAL_MAX_DELAY);

		HAL_GPIO_WritePin(GPIOB, B2_pin_Pin, GPIO_PIN_RESET);


		//RICEZIONE DALLO SLAVE
		tick_iniziale=osKernelGetTickCount();

		while (HAL_GPIO_ReadPin(GPIOB, B1_pin_Pin) == GPIO_PIN_RESET){
			if(osKernelGetTickCount()-tick_iniziale > 10){
				status_boards=0;
			    goto fine;
			}
		}

		HAL_GPIO_WritePin(GPIOB, B2_pin_Pin, GPIO_PIN_SET);

		HAL_SPI_Receive(&hspi1, buffer_recive, sizeof(buffer_recive), HAL_MAX_DELAY);

		HAL_GPIO_WritePin(GPIOB, B2_pin_Pin, GPIO_PIN_RESET);

		deserialize_partial(&global_data,&data_master,buffer_recive);

		//decisiono 0 emergenza sullo slave
		//decisione 1 attuazione sulo slave
		//decisione 2 degradato sul master
		//decisione 3 emergenza sul master

		if(global_data.B2_state==0 && global_data.B1_state==1){
			data_master.B2_decision = 0;
		}
		else if(global_data.B2_state==1 && global_data.B1_state==1){
			data_master.B2_decision = 1;
		}
		else if(global_data.B2_state==1 && global_data.B1_state==0){
			status_boards=1;
			data_master.B2_decision = 2;
		}
		else if(global_data.B2_state==0 && global_data.B1_state==0){
			status_boards=0;
			data_master.B2_decision=3;
		}

		//final decision 1 decisioni b1 e b2 uguali
		//final decision 0 decisioni b1 e b2 diverse
		if(data_master.B2_decision==global_data.B1_decision){
			final_decision=1;
		}
		else{
			final_decision=0;
			if(data_master.B2_state==0){
				status_boards=0;
			}
			else{
				status_boards=1;
			}
		}

		//SCAMBIO DECISIONE
		HAL_GPIO_WritePin(GPIOB, B2_pin_Pin, GPIO_PIN_SET);

		tick_iniziale=osKernelGetTickCount();
		while (HAL_GPIO_ReadPin(GPIOB, B1_pin_Pin) == GPIO_PIN_RESET){
			if(osKernelGetTickCount()-tick_iniziale > 10 ){
				status_boards=0;
			    goto fine;
			}
		}

		HAL_SPI_Transmit(&hspi1,&final_decision, sizeof(final_decision), HAL_MAX_DELAY);

		HAL_GPIO_WritePin(GPIOB, B2_pin_Pin, GPIO_PIN_RESET);

		fine:
		if(final_decision==1){
			if(data_master.B2_decision==2 || data_master.B2_decision==3){
				HAL_GPIO_WritePin(rele1_GPIO_Port, rele1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(rele2_GPIO_Port, rele2_Pin, GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(rele1_GPIO_Port, rele1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(rele2_GPIO_Port, rele2_Pin, GPIO_PIN_RESET);
			}
		}
		else{
			if(data_master.B2_state == 1){
				status_boards = 1;
			}else{
				status_boards = 0;
			}
			HAL_GPIO_WritePin(rele1_GPIO_Port, rele1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(rele2_GPIO_Port, rele2_Pin, GPIO_PIN_SET);
		}

    osDelay(1);
  }
  /* USER CODE END comunicazioneTask */
}

/* USER CODE BEGIN Header_degradatoTask */
/**
* @brief Function implementing the degradato thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_degradatoTask */
void degradatoTask(void *argument)
{
  /* USER CODE BEGIN degradatoTask */
  /* Infinite loop */
  for(;;)
  {

	if(status_boards==1){
		degraded(&data_master);

	}

    osDelay(1);
  }
  /* USER CODE END degradatoTask */
}

/* USER CODE BEGIN Header_emergenzaTask */
/**
* @brief Function implementing the emergenza thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_emergenzaTask */
void emergenzaTask(void *argument)
{
  /* USER CODE BEGIN emergenzaTask */
  /* Infinite loop */
  for(;;)
  {

	if(status_boards==0){
		emergency();
	}

    osDelay(1);
  }
  /* USER CODE END emergenzaTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief Verifica la validità dei dati di input e aggiorna lo stato di B2.
 *
 * Questa funzione controlla se i dati forniti sono validi in base a diversi criteri:
 * - `start_temp` non deve superare il valore di 200.
 * - `statusController` deve essere uguale a `HAL_OK`.
 * - I valori `ax`, `ay`, `bx` e `by` del controller non devono essere tutti zero.
 * - I valori `ax`, `ay`, `bx` e `by` devono rientrare nell'intervallo valido [0, 511].
 *
 * Se uno qualsiasi di questi controlli fallisce, il campo `B2_state` di `data_master` viene impostato a 0,
 * altrimenti viene impostato a 1.
 *
 * @param start_temp Valore iniziale della temperatura.
 * @param statusController Stato restituito dal controller, deve essere `HAL_OK`.
 * @param data_master Puntatore alla struttura `partial_master_t` contenente i dati da verificare.
 */
void dataCheck(uint16_t start_temp,HAL_StatusTypeDef statusController,partial_master_t* data_master){


	  if(start_temp>200 ||
			  statusController!=HAL_OK ||
			  (are_values_zero(data_master->controller.ax, data_master->controller.ay, data_master->controller.bx, data_master->controller.by) ||
			  is_invalid_value(data_master->controller.ax, 0, 511) ||
			  is_invalid_value(data_master->controller.ay, 0, 511) ||
			  is_invalid_value(data_master->controller.bx, 0, 511) ||
			  is_invalid_value(data_master->controller.by, 0, 511)))
	  {
		  data_master->B2_state=0;
	  }else{
		  data_master->B2_state=1;
	 }
}




void degraded(partial_master_t* global_data){
	wheel_speeds_t speeds;
	uint8_t INpower2=0;
	uint8_t INpower4=0;

	if(global_data->controller.ay > PAD1_y_value_min_su && global_data->controller.ay <= PAD1_y_value_max_su){
		if(global_data->sonar2>70){
			speeds.left_speed=80;
			speeds.right_speed=80;
		}
		else{
			speeds.left_speed=64;
			speeds.right_speed=64;
		}
	}
	else if(global_data->controller.ay >= PAD1_y_value_min_giu && global_data->controller.ay < PAD1_y_value_max_giu){
		speeds.left_speed=80;
		speeds.right_speed=50;
	}
	else if(global_data->controller.ay >= PAD1_y_value_max_giu && global_data->controller.ay <= PAD1_y_value_min_su){

    	if (global_data->controller.bx >= 350 && global_data->controller.bx <= 511) { //fermo gira verso sinitra
        	speeds.left_speed=50;
        	speeds.right_speed=80;
        }
        else if(global_data->controller.bx >= 0 && global_data->controller.bx <= 150){ //fermo gira verso destra
        	speeds.left_speed=80;
          	speeds.right_speed=50;
        }
        else if(global_data->controller.bx>150 && global_data->controller.bx<350){//fermo sia nalogico pad1_y e fermo analogico pad2_x allora il rover sta fermo
        	speeds.left_speed=64;
        	speeds.right_speed=64;
        }
    }

	INpower2=speeds.left_speed+127;
	INpower4=speeds.left_speed+127;

	HAL_UART_Transmit(&huart1, &speeds.right_speed, 1,0);
	HAL_UART_Transmit(&huart1, &INpower2, 1,0);
	HAL_UART_Transmit(&huart3, &speeds.right_speed, 1,0);
	HAL_UART_Transmit(&huart3, &INpower4, 1,0);
}





void emergency(){
	uint8_t power=64;
	uint8_t power2=64+127;

	HAL_UART_Transmit(&huart1, &power, 1,0);
	HAL_UART_Transmit(&huart1, &power2, 1,0);
	HAL_UART_Transmit(&huart3, &power, 1,0);
	HAL_UART_Transmit(&huart3, &power2,1, 0);
}

/* USER CODE END Application */

