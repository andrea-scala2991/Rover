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


#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "stm32g4xx_hal.h"

/**
 * @brief Structure to hold ultrasonic sensor configuration.
 */
typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;
    TIM_HandleTypeDef *timer;
    uint8_t error_count;  // Contatore degli errori consecutivi
    int16_t last_valid_distance; // Ultima distanza valida
} UltrasonicSensor;


/**
 * @brief Initializes an ultrasonic sensor structure.
 * @param sensor: Pointer to the UltrasonicSensor structure.
 * @param trig_port: GPIO port for the TRIG pin.
 * @param trig_pin: GPIO pin for the TRIG pin.
 * @param echo_port: GPIO port for the ECHO pin.
 * @param echo_pin: GPIO pin for the ECHO pin.
 * @param timer: Pointer to the TIM_HandleTypeDef structure for timing.
 */
void Ultrasonic_Init(UltrasonicSensor *sensor, GPIO_TypeDef *trig_port, uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin, TIM_HandleTypeDef *timer);

/**
 * @brief Measures the distance using the ultrasonic sensor.
 * @param sensor: Pointer to the UltrasonicSensor structure.
 * @retval Distance in cm, or -1 if timeout occurs.
 */
int16_t Ultrasonic_GetDistance(UltrasonicSensor *sensor);

#endif // ULTRASONIC_H
