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

#include "com.h"
void deserialize_partial(global_t* global_data,partial_slave_t* data_slave, const uint8_t* buffer_recived) {

	global_data->controller.ax = (buffer_recived[0] << 8) | buffer_recived[1];
	global_data->controller.ay = (buffer_recived[2] << 8) | buffer_recived[3];
	global_data->controller.a_btn = buffer_recived[4];
	global_data->controller.bx = (buffer_recived[5] << 8) | buffer_recived[6];
	global_data->controller.by = (buffer_recived[7] << 8) | buffer_recived[8];
	global_data->controller.b_btn = buffer_recived[9];
	global_data->controller.btn1 = buffer_recived[10];
	global_data->controller.btn2 = buffer_recived[11];

	global_data->accelerometer.acc_x=(buffer_recived[12] << 8) | buffer_recived[13];
	global_data->accelerometer.acc_y=(buffer_recived[14] << 8) | buffer_recived[15];
	global_data->accelerometer.acc_z=(buffer_recived[16] << 8) | buffer_recived[17];
	global_data->accelerometer.gyro_x=(buffer_recived[18] << 8) | buffer_recived[19];
	global_data->accelerometer.gyro_y=(buffer_recived[20] << 8) | buffer_recived[21];
	global_data->accelerometer.gyro_z=(buffer_recived[22] << 8) | buffer_recived[23];

	global_data->temperatura1 = data_slave->temperatura1;
	global_data->temperatura2 = (buffer_recived[24] << 8) | buffer_recived[25];
	global_data->sonar1 = (buffer_recived[26] << 8) | buffer_recived[27];
	global_data->sonar2 = (buffer_recived[28] << 8) | buffer_recived[29];
	global_data->sonar3 = (buffer_recived[30] << 8) | buffer_recived[31];
	global_data->led1 = data_slave->led1;
	global_data->led2 = data_slave->led2;
	global_data->encoder1 = data_slave->encoder1;
	global_data->encoder2 = data_slave->encoder2;
	global_data->encoder3 = data_slave->encoder3;
	global_data->encoder4 = data_slave->encoder4;
	global_data->B1_state=data_slave->B1_state;
	global_data->B2_state=buffer_recived[32];
}
void serialize_partial(const partial_slave_t *data_slave, uint8_t *buffer_send) {

	buffer_send[0] = (data_slave->temperatura1 >> 8) & 0xFF;
	buffer_send[1] = data_slave->temperatura1 & 0xFF;

    buffer_send[2] = (data_slave->led1 >> 8) & 0xFF;
    buffer_send[3] = data_slave->led1 & 0xFF;
    buffer_send[4] = (data_slave->led2 >> 8) & 0xFF;
    buffer_send[5] = data_slave->led2 & 0xFF;

    buffer_send[6] = (data_slave->encoder1 >> 8) & 0xFF;
    buffer_send[7] = data_slave->encoder1 & 0xFF;
    buffer_send[8] = (data_slave->encoder2 >> 8) & 0xFF;
    buffer_send[9] = data_slave->encoder2 & 0xFF;
    buffer_send[10] = (data_slave->encoder3 >> 8) & 0xFF;
    buffer_send[11] = data_slave->encoder3 & 0xFF;
    buffer_send[12] = (data_slave->encoder4 >> 8) & 0xFF;
    buffer_send[13] = data_slave->encoder4 & 0xFF;

    buffer_send[14]=data_slave->B1_state;
    buffer_send[15]=data_slave->B1_decision;
}

void debug_print_comunication(global_t* global_data, UART_HandleTypeDef* huart) {
    char usart_buffer2[400];

    snprintf(usart_buffer2, sizeof(usart_buffer2), "//////////////////////////\r\n");
    HAL_UART_Transmit(huart, (uint8_t *)usart_buffer2, strlen(usart_buffer2), HAL_MAX_DELAY);

    /*snprintf(usart_buffer2, sizeof(usart_buffer2),
        "controller.AX: %d\r\ncontroller.AY: %d\r\ncontroller.A_BTN: %d\r\ncontroller.BX: %d\r\ncontroller.BY: %d\r\ncontroller.B_BTN: %d\r\ncontroller.BTN1: %d\r\ncontroller.BTN2: %d\r\n"
        "accelerometer.ACC_X: %d\r\naccelerometer.ACC_Y: %d\r\naccelerometer.ACC_Z: %d\r\naccelerometer.GYRO_X: %d\r\naccelerometer.GYRO_Y: %d\r\naccelerometer.GYRO_Z: %d\r\n"
        "temperatura2: %d\r\nsonar1: %d\r\nsonar2: %d\r\nsonar3: %d\r\n"
        "B2_state: %d\r\n",
        global_data->controller.ax,
        global_data->controller.ay,
        global_data->controller.a_btn,
        global_data->controller.bx,
        global_data->controller.by,
        global_data->controller.b_btn,
        global_data->controller.btn1,
        global_data->controller.btn2,
        global_data->accelerometer.acc_x,
        global_data->accelerometer.acc_y,
        global_data->accelerometer.acc_z,
        global_data->accelerometer.gyro_x,
        global_data->accelerometer.gyro_y,
        global_data->accelerometer.gyro_z,
        global_data->temperatura1,
        global_data->temperatura2,
        global_data->sonar1,
        global_data->sonar2,
        global_data->sonar3,

        global_data->B2_state);*/

    HAL_UART_Transmit(huart, (uint8_t *)usart_buffer2, strlen(usart_buffer2), HAL_MAX_DELAY);
}
