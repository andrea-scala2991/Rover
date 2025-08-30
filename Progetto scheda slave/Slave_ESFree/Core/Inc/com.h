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

#ifndef INC_COM_H_
#define INC_COM_H_

#include "main.h"
#include "controller.h"
#include "MPU-6050.h"

typedef struct global_s{
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
	uint8_t B2_decision;
	uint8_t B1_decision;
}global_t;

typedef struct partial_slave_s{
	int16_t temperatura1;
	uint16_t led1;
	uint16_t led2;
	uint16_t encoder1;
	uint16_t encoder2;
	uint16_t encoder3;
	uint16_t encoder4;
	uint8_t B1_state;
	uint8_t B1_decision;

}partial_slave_t;


void debug_print_comunication(global_t* global_data, UART_HandleTypeDef* huart);
void serialize_partial(const partial_slave_t *data_slave, uint8_t *buffer_send);
void deserialize_partial(global_t* global_data,partial_slave_t* data_slave, const uint8_t* buffer_recived);


#endif /* INC_COM_H_ */
