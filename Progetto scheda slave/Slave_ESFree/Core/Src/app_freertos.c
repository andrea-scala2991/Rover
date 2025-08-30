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
#include <string.h>
#include "controller.h"
#include "temperature.h"
#include "hc-sr04.h"
#include "MPU-6050.h"
#include "tim.h"
#include "com.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "pid.h"
#include "scheduler.h"
#include "ssd1306_conf_template.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void dataCheck(uint16_t start_temp, partial_slave_t* data_salve);
void motori(global_t* global_data,PID_Controller *pid1,PID_Controller *pid2,PID_Controller *pid3,PID_Controller *pid4);
void led(global_t* global_data);
void emergency();
void ledEmergency();
void ledActuation();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
PID_Controller pid1,pid2,pid3,pid4;
uint16_t start_temp;
uint32_t tick, timeout;
uint8_t buffer_receive[33];
uint8_t buffer_send[16];
partial_slave_t data_slave;
global_t global_data;
uint8_t final_decision;
uint8_t state_boards=3;
uint8_t ledOn = 0;
uint8_t lastBtn = 0;
char usart_buffer2[100];

/* USER CODE END Variables */
/* Definitions for ReadingTask */
osThreadId_t ReadingTaskHandle;
const osThreadAttr_t ReadingTask_attributes = {
  .name = "ReadingTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for CommunicationTa */
osThreadId_t CommunicationTaHandle;
const osThreadAttr_t CommunicationTa_attributes = {
  .name = "CommunicationTa",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for ActuationTask */
osThreadId_t ActuationTaskHandle;
const osThreadAttr_t ActuationTask_attributes = {
  .name = "ActuationTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for EmergencyTask */
osThreadId_t EmergencyTaskHandle;
const osThreadAttr_t EmergencyTask_attributes = {
  .name = "EmergencyTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartReadingTask(void *argument);
void StartCommunicationTask(void *argument);
void StartActuationTask(void *argument);
void StartEmergencyTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */



	PID_Init(&pid1,0.001, 0.5,0);
	PID_Init(&pid2,0.001, 0.5,0);
	PID_Init(&pid3,0.001, 0.5,0);
	PID_Init(&pid4,0.001, 0.5,0);
	HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, SET);
	HAL_GPIO_WritePin(LED2_R_GPIO_Port, LED2_R_Pin, SET);




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
  /* creation of ReadingTask */
  ReadingTaskHandle = osThreadNew(StartReadingTask, NULL, &ReadingTask_attributes);

  /* creation of CommunicationTa */
  CommunicationTaHandle = osThreadNew(StartCommunicationTask, NULL, &CommunicationTa_attributes);

  /* creation of ActuationTask */
  ActuationTaskHandle = osThreadNew(StartActuationTask, NULL, &ActuationTask_attributes);

  /* creation of EmergencyTask */
  EmergencyTaskHandle = osThreadNew(StartEmergencyTask, NULL, &EmergencyTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartReadingTask */
/**
  * @brief  Function implementing the ReadingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartReadingTask */
void StartReadingTask(void *argument)
{
  /* USER CODE BEGIN StartReadingTask */
  /* Infinite loop */
  for(;;)
  {

		data_slave.temperatura1=TemperatureSensor_Read();

	  	if(data_slave.temperatura1>90 || data_slave.temperatura1==-1){start_temp++;}
	  	else{start_temp=0;}

	  	data_slave.encoder1=TIM2->CNT;
	  	data_slave.encoder2=TIM3->CNT;
	  	data_slave.encoder3=TIM4->CNT;
	  	data_slave.encoder4=TIM5->CNT;

	  	//controllo validità dei dati rilevati dai sensori
	  	dataCheck(start_temp, &data_slave);
	  	osDelay(1);
  }
  /* USER CODE END StartReadingTask */
}

/* USER CODE BEGIN Header_StartCommunicationTask */
/**
* @brief Function implementing the CommunicationTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommunicationTask */
void StartCommunicationTask(void *argument)
{
  /* USER CODE BEGIN StartCommunicationTask */
  /* Infinite loop */
  for(;;)
  {
	  // Reset delle variabili di stato
	  state_boards=3;
	  final_decision = 3;
	  data_slave.B1_decision = 4;



	  //RICEZIONE PARTIAL STATE DAL MASTER

	  timeout=osKernelGetTickCount();
	  while (HAL_GPIO_ReadPin(B2_pin_GPIO_Port, B2_pin_Pin) == GPIO_PIN_RESET){
		  if(osKernelGetTickCount()-timeout > 80){
				state_boards = 0;
				goto fine;
		  }
	  }

	  HAL_GPIO_WritePin(GPIOB, B1_pin_Pin, GPIO_PIN_SET);

	  HAL_SPI_Receive(&hspi1, buffer_receive, sizeof(buffer_receive), HAL_MAX_DELAY);

	  HAL_GPIO_WritePin(GPIOB, B1_pin_Pin, GPIO_PIN_RESET);

	  deserialize_partial(&global_data,&data_slave,buffer_receive);


	  //DECISIONE DI B1 IN BASE AI DATI

		if(global_data.B2_state==0 && global_data.B1_state==1){ //Emergenza slave
			state_boards=0;
			data_slave.B1_decision = 0;
		}
		else if(global_data.B2_state==1 && global_data.B1_state==1){ //Attuazione slave
			state_boards=1;
			data_slave.B1_decision = 1;
		}
		else if(global_data.B2_state==1 && global_data.B1_state==0){ //Degradato master
			data_slave.B1_decision = 2;
		}
		else if(global_data.B2_state==0 && global_data.B1_state==0){ // Emergenza master
			data_slave.B1_decision = 3;
		}



		//INVIO PARTIAL STATE AL MASTER PIù DECISIONE SLAVE

		serialize_partial(&data_slave, buffer_send);


		HAL_GPIO_WritePin(GPIOB, B1_pin_Pin, GPIO_PIN_SET);
	    timeout = osKernelGetTickCount();
	    while (HAL_GPIO_ReadPin(GPIOB, B2_pin_Pin) == GPIO_PIN_RESET){
	  		if(osKernelGetTickCount()-timeout > 10){
	  			/*snprintf(buf, sizeof(buf),"timeout2: %d\r\n", osKernelGetTickCount()-timeout);
	  			HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);*/
	  				state_boards = 0;
	  				goto fine;
	  		}
	     }


		  HAL_SPI_Transmit(&hspi1, (uint8_t*)buffer_send, sizeof(buffer_send), HAL_MAX_DELAY);

		  HAL_GPIO_WritePin(GPIOB, B1_pin_Pin, GPIO_PIN_RESET);




	  //RICEZIONE DECISIONE FINALE MASTER PIù CONTROLLO FINALE
			timeout=osKernelGetTickCount();
			while (HAL_GPIO_ReadPin(B2_pin_GPIO_Port, B2_pin_Pin) == GPIO_PIN_RESET){
				if(osKernelGetTickCount()-timeout > 10){
					state_boards = 0;
					goto fine;
	  	  		}
	  	  	}

	  	  HAL_GPIO_WritePin(GPIOB, B1_pin_Pin, GPIO_PIN_SET);

	  	  HAL_SPI_Receive(&hspi1, &final_decision, sizeof(final_decision), HAL_MAX_DELAY);

	  	  HAL_GPIO_WritePin(GPIOB, B1_pin_Pin, GPIO_PIN_RESET);

	  	  //Se la decisione finale è uguale a 0 significa che le due schede
	  	  //hanno preso decisioni differenti e quindi il master prende il comando
	  	  if(final_decision == 0){
	  		  state_boards = 3;
	  	  }

	fine:
	osDelay(1);
  }
  /* USER CODE END StartCommunicationTask */
}

/* USER CODE BEGIN Header_StartActuationTask */
/**
* @brief Function implementing the ActuationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartActuationTask */
void StartActuationTask(void *argument)
{
  /* USER CODE BEGIN StartActuationTask */
  /* Infinite loop */
  for(;;)
  {
	if(state_boards==1){
		//funzione per il comando dei motori
		motori(&global_data,&pid1,&pid2,&pid3,&pid4);

		//Suono clacson
		if(global_data.controller.b_btn==1){
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
		// la variabile globale ledOn viene utilizzata per capire se i led devono essere accesi o spenti
		if(ledOn == 0 && global_data.controller.btn2 == 1 && lastBtn != 1){
			ledOn = 1;
			lastBtn = 1;
		}else if(global_data.controller.btn2 == 1 && lastBtn != 1){
			ledOn = 0;
			lastBtn = 1;
		}else if(global_data.controller.btn2 == 0){
			lastBtn = 0;
		}
		ledActuation();

	}
    osDelay(1);
  }
  /* USER CODE END StartActuationTask */
}

/* USER CODE BEGIN Header_StartEmergencyTask */
/**
* @brief Function implementing the EmergencyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEmergencyTask */
void StartEmergencyTask(void *argument)
{
  /* USER CODE BEGIN StartEmergencyTask */
  /* Infinite loop */
  for(;;)
  {
	if(state_boards==0){
		emergency();
	}
    osDelay(1);
  }
  /* USER CODE END StartEmergencyTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void dataCheck(uint16_t start_temp, partial_slave_t* data_slave){

	if(data_slave->encoder1 < 0 || data_slave->encoder2< 0 || data_slave->encoder3< 0 || data_slave->encoder4< 0 || start_temp>200){
		data_slave->B1_state=0;
	}
	else{
		data_slave->B1_state=1;
	}

}

void ledActuation(){
	if(ledOn == 1){

	HAL_GPIO_WritePin(LED1_W_GPIO_Port, LED1_W_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_W_GPIO_Port, LED2_W_Pin, GPIO_PIN_RESET);
	}else{
	HAL_GPIO_WritePin(LED1_W_GPIO_Port, LED1_W_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_W_GPIO_Port, LED2_W_Pin, GPIO_PIN_SET);
	}
	return;
}

void motori(global_t* global_data,PID_Controller *pid1,PID_Controller *pid2,PID_Controller *pid3,PID_Controller *pid4){

	wheel_speeds_t speed;
	uint32_t tick1,tick2,tick3,tick4;
	int counterValue1,counterValue2,counterValue3,counterValue4;
	uint8_t power1,power2,power3,power4,INpower2,INpower4;

	speed=compute(&global_data->controller, global_data->sonar1,global_data->sonar2,global_data->sonar3);


	tick1 = HAL_GetTick();
	counterValue1 = TIM2->CNT;
	power1=PID_Compute(pid1, counterValue1, tick1, speed.right_speed, 2448.0,1);


	tick2 = HAL_GetTick();
	counterValue2 = TIM3->CNT;
	power2=PID_Compute(pid2, counterValue2, tick2, speed.left_speed, 2448.0,2);
	INpower2=power2+127;


	tick3 = HAL_GetTick();
	counterValue3 = TIM4->CNT;
	power3=PID_Compute(pid3, counterValue3, tick3, speed.right_speed, 2448.0,3);


	tick4 = HAL_GetTick();
	counterValue4 = TIM5->CNT;
	power4=PID_Compute(pid4, counterValue4, tick4, speed.left_speed, 2448.0,4);
	INpower4=power4+127;



	HAL_UART_Transmit(&huart1, &power1,1,0);
	HAL_UART_Transmit(&huart1, &INpower2, 1,0);
	HAL_UART_Transmit(&huart3, &power3, 1,0);
	HAL_UART_Transmit(&huart3, &INpower4, 1,0);


}


void emergency(){
    uint8_t power=64;
    uint8_t power2=64+127;

    HAL_UART_Transmit(&huart1, &power, 1,0);
    HAL_UART_Transmit(&huart1, &power2, 1,0);
    HAL_UART_Transmit(&huart3, &power, 1,0);
    HAL_UART_Transmit(&huart3, &power2, 1,0);

}
/* USER CODE END Application */

