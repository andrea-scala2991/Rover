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


#include "com_master.h"

void serialize_partial(const partial_master_t *data_master, uint8_t *buffer_send) {

	buffer_send[0] = (data_master->controller.ax >> 8) & 0xFF;
	buffer_send[1] = data_master->controller.ax & 0xFF;
	buffer_send[2] = (data_master->controller.ay >> 8) & 0xFF;
	buffer_send[3] = data_master->controller.ay & 0xFF;
	buffer_send[4] = data_master->controller.a_btn;
	buffer_send[5] = (data_master->controller.bx >> 8) & 0xFF;
	buffer_send[6] = data_master->controller.bx & 0xFF;
	buffer_send[7] = (data_master->controller.by >> 8) & 0xFF;
	buffer_send[8] = data_master->controller.by & 0xFF;
	buffer_send[9] = data_master->controller.b_btn;
	buffer_send[10] = data_master->controller.btn1;
	buffer_send[11] = data_master->controller.btn2;

	buffer_send[12] = (data_master->accelerometer.acc_x >> 8) & 0xFF;
	buffer_send[13] = data_master->accelerometer.acc_x & 0xFF;
	buffer_send[14] = (data_master->accelerometer.acc_y >> 8) & 0xFF;
	buffer_send[15] = data_master->accelerometer.acc_y & 0xFF;
	buffer_send[16] = (data_master->accelerometer.acc_z >> 8) & 0xFF;
	buffer_send[17] = data_master->accelerometer.acc_z & 0xFF;
	buffer_send[18] = (data_master->accelerometer.gyro_x >> 8) & 0xFF;
	buffer_send[19] = data_master->accelerometer.gyro_x & 0xFF;
	buffer_send[20] = (data_master->accelerometer.gyro_y >> 8) & 0xFF;
	buffer_send[21] = data_master->accelerometer.gyro_y & 0xFF;
	buffer_send[22] = (data_master->accelerometer.gyro_z >> 8) & 0xFF;
	buffer_send[23] = data_master->accelerometer.gyro_z & 0xFF;

	buffer_send[24] = (data_master->temperatura2 >> 8) & 0xFF;
	buffer_send[25] = data_master->temperatura2 & 0xFF;

	buffer_send[26] = (data_master->sonar1 >> 8) & 0xFF;
	buffer_send[27] = data_master->sonar1 & 0xFF;
	buffer_send[28] = (data_master->sonar2 >> 8) & 0xFF;
	buffer_send[29] = data_master->sonar2 & 0xFF;
	buffer_send[30] = (data_master->sonar3 >> 8) & 0xFF;
	buffer_send[31] = data_master->sonar3 & 0xFF;

	buffer_send[32] = data_master->B2_state;

}
void deserialize_partial(global_t* global_data,partial_master_t* data_master, const uint8_t* buffer_recived) {

	global_data->controller.ax = data_master->controller.ax;
	global_data->controller.ay = data_master->controller.ay;
	global_data->controller.a_btn = data_master->controller.a_btn;
	global_data->controller.bx = data_master->controller.bx;
	global_data->controller.by = data_master->controller.by;
	global_data->controller.b_btn = data_master->controller.b_btn;
	global_data->controller.btn1 = data_master->controller.btn1;
	global_data->controller.btn2 = data_master->controller.btn2;

	global_data->accelerometer.acc_x = data_master->accelerometer.acc_x;
	global_data->accelerometer.acc_y = data_master->accelerometer.acc_y;
	global_data->accelerometer.acc_z = data_master->accelerometer.acc_z;
	global_data->accelerometer.gyro_x = data_master->accelerometer.gyro_x;
	global_data->accelerometer.gyro_y = data_master->accelerometer.gyro_y;
	global_data->accelerometer.gyro_z = data_master->accelerometer.gyro_z;

	global_data->temperatura1 = (buffer_recived[0] << 8) | buffer_recived[1];
	global_data->temperatura2 = data_master->temperatura2;

	global_data->sonar1 = data_master->sonar1;
	global_data->sonar2 = data_master->sonar2;
	global_data->sonar3 = data_master->sonar3;

	global_data->led1 = (buffer_recived[2] << 8) | buffer_recived[3];
	global_data->led2 = (buffer_recived[4] << 8) | buffer_recived[5];

	global_data->encoder1 = (buffer_recived[6] << 8) | buffer_recived[7];
	global_data->encoder2 = (buffer_recived[8] << 8) | buffer_recived[9];
	global_data->encoder3 = (buffer_recived[10] << 8) | buffer_recived[11];
	global_data->encoder4 = (buffer_recived[12] << 8) | buffer_recived[13];

	global_data->B1_state=buffer_recived[14];
	global_data->B2_state=data_master->B2_state;

	global_data->B1_decision=buffer_recived[15];
}

void debug_print_comunication(global_t* global_data, UART_HandleTypeDef* huart) {
    char usart_buffer2[800];

    snprintf(usart_buffer2, sizeof(usart_buffer2), "//////////////////////////\r\n");
    HAL_UART_Transmit(huart, (uint8_t *)usart_buffer2, strlen(usart_buffer2), HAL_MAX_DELAY);

    snprintf(usart_buffer2, sizeof(usart_buffer2),
        "controller.AX: %d\r\ncontroller.AY: %d\r\ncontroller.A_BTN: %d\r\ncontroller.BX: %d\r\ncontroller.BY: %d\r\ncontroller.B_BTN: %d\r\ncontroller.BTN1: %d\r\ncontroller.BTN2: %d\r\n"
        "accelerometer.ACC_X: %d\r\naccelerometer.ACC_Y: %d\r\naccelerometer.ACC_Z: %d\r\naccelerometer.GYRO_X: %d\r\naccelerometer.GYRO_Y: %d\r\naccelerometer.GYRO_Z: %d\r\n"
        "temperatura1: %d\r\ntemperatura2: %d\r\nsonar1: %d\r\nsonar2: %d\r\nsonar3: %d\r\n"
        "led1: %d\r\nled2: %d\r\nencoder1: %d\r\nencoder2: %d\r\nencoder3: %d\r\nencoder4: %d\r\n"
        "B1_state: %d\r\nB2_state: %d\r\n",
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
        global_data->led1,
        global_data->led2,
        global_data->encoder1,
        global_data->encoder2,
        global_data->encoder3,
        global_data->encoder4,
        global_data->B1_state,
        global_data->B2_state);

    HAL_UART_Transmit(huart, (uint8_t *)usart_buffer2, strlen(usart_buffer2), HAL_MAX_DELAY);
}
