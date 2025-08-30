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


#ifndef INC_COM_MASTER_H_
#define INC_COM_MASTER_H_

#include "main.h"
#include "MPU-6050.h"
#include "controller.h"

typedef struct global_s {
    controller_t controller;
    accelerometer_t accelerometer;
    int16_t temperatura1;
    int16_t temperatura2;
    uint16_t sonar1;
    uint16_t sonar2;
    uint16_t sonar3;
    uint16_t led1;
    uint16_t led2;
    uint16_t encoder1;
    uint16_t encoder2;
    uint16_t encoder3;
    uint16_t encoder4;
    uint8_t B1_state;
    uint8_t B2_state;
    uint8_t B1_decision;
    uint8_t B2_decision;
} global_t;
typedef struct partial_master_s {
    controller_t controller;
    accelerometer_t accelerometer;
    int16_t temperatura2;
    uint16_t sonar1;
    uint16_t sonar2;
    uint16_t sonar3;
    uint8_t B2_state;
    uint8_t B2_decision;

} partial_master_t;

void serialize_partial(const partial_master_t *data_master, uint8_t *buffer_send);
void deserialize_partial(global_t* global_data,partial_master_t* data_master, const uint8_t* buffer_recived);
void debug_print_comunication(global_t* global_data, UART_HandleTypeDef* huart);
#endif /* INC_COM_MASTER_H_ */
