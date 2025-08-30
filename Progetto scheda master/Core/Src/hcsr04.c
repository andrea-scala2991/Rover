/*
    Copyright (C) 2025  Andrea

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

/* USER LIBRARY: Ultrasonic Sensor Management */
#include "hcsr04.h"
#include "tim.h"
#include "gpio.h"



void Ultrasonic_Init(UltrasonicSensor *sensor, GPIO_TypeDef *trig_port, uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin, TIM_HandleTypeDef *timer)
{
    sensor->trig_port = trig_port;
    sensor->trig_pin = trig_pin;
    sensor->echo_port = echo_port;
    sensor->echo_pin = echo_pin;
    sensor->timer = timer;
    sensor->error_count = 0; // Inizializza il contatore errori
}

int16_t Ultrasonic_GetDistance(UltrasonicSensor *sensor)
{
    uint32_t start_tick;
    uint32_t value1, value2;

    // Trigger del sensore
    HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(sensor->timer, 0);
    while (__HAL_TIM_GET_COUNTER(sensor->timer) < 10);  // Attendi 10 us
    HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);

    // Attende che il segnale ECHO si alzi
    start_tick = HAL_GetTick();
    while (!HAL_GPIO_ReadPin(sensor->echo_port, sensor->echo_pin)) {
        if (HAL_GetTick() - start_tick > 30) { // Timeout 30 ms
            sensor->error_count++;  // Incrementa il conteggio degli errori


            return sensor->last_valid_distance; // Ritorna l'ultima distanza valida
        }
    }
    value1 = __HAL_TIM_GET_COUNTER(sensor->timer);

    // Attende che il segnale ECHO si abbassi
    start_tick = HAL_GetTick();
    while (HAL_GPIO_ReadPin(sensor->echo_port, sensor->echo_pin)) {
        if (HAL_GetTick() - start_tick > 30) { // Timeout 30 ms
            sensor->error_count++;  // Incrementa il conteggio degli errori


            return sensor->last_valid_distance; // Ritorna l'ultima distanza valida
        }
    }
    value2 = __HAL_TIM_GET_COUNTER(sensor->timer);

    // Calcola la distanza in cm
    int16_t distance = (int16_t)((value2 - value1) * 0.034 / 2);

    // Se la misura è valida, resetta il contatore errori e aggiorna l'ultima distanza valida
    if (distance > 0 && distance <= 400) {
        sensor->error_count = 0;
        sensor->last_valid_distance = distance;
    }

    return sensor->last_valid_distance; // Ritorna sempre un valore affidabile
}






