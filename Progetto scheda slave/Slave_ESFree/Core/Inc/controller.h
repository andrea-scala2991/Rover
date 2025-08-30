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
 * Questo header definisce le strutture, costanti, e le dichiarazioni di funzioni necessarie
 * per il controllo del rover. Contiene definizioni dei range dei controlli, strutture per
 * rappresentare i dati del controller e le velocità delle ruote, e le dichiarazioni delle funzioni
 * principali utilizzate per il calcolo e la verifica dei comandi ricevuti tramite I2C o input analogici.
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"
#include "usart.h"
#include <stdbool.h>

/* Definizione dei range dei comandi analogici per il movimento: */

/* Range per il comando avanti/indietro tramite PAD1 (movimento verticale) */
#define PAD1_y_value_min_giu 0
#define PAD1_y_value_max_giu 100
#define PAD1_y_value_min_su 270
#define PAD1_y_value_max_su 511

/* Range per il comando sinistra/destra tramite PAD2 (movimento orizzontale) */
#define PAD2_x_value_min_destra 0
#define PAD2_x_value_max_destra 265
#define PAD2_x_value_min_sinistra 280
#define PAD2_x_value_max_sinistra 511


/*
 * Struttura dati per rappresentare i comandi ricevuti dal controller.
 * Contiene valori analogici e input dei pulsanti.
 */
typedef struct controller_s {
    uint16_t ax;
    uint16_t ay;
    uint8_t a_btn;

    uint16_t bx;
    uint16_t by;
    uint8_t b_btn;

    uint8_t btn1;
    uint8_t btn2;
} controller_t;


/*
 * Struttura dati per rappresentare le velocità delle ruote del rover.
 * 'left_speed' è la velocità per le ruote sinistre, mentre 'right_speed' è la velocità
 * per le ruote destre.
 */
typedef struct wheel_speeds_s {
    int left_speed;  // Velocità assegnata alle ruote sinistra (M1, M3).
    int right_speed; // Velocità assegnata alle ruote destra (M2, M4).
} wheel_speeds_t;


/*
 * Funzione: compute
 * Scopo: Elabora i dati del controller per calcolare le velocità delle ruote del rover.
 * Input: puntatore alla struttura controller_t che contiene i dati analogici e pulsanti.
 * Output: struttura wheel_speeds_t contenente le velocità calcolate per le ruote sinistre e destra.
 */
wheel_speeds_t compute(controller_t *controller_data, uint16_t sonar1, uint16_t sonar2, uint16_t sonar3);


/*
 * Funzione: calculate_wheel_speeds
 * Scopo: Calcola le velocità delle ruote in base ai valori analogici letti dai pad.
 * Input: valori analogici PAD2_x_value e PAD1_y_value.
 * Output: struttura wheel_speeds_t contenente le velocità delle ruote sinistra e destra.
 */
wheel_speeds_t calculate_wheel_speeds(int PAD2_x_value, int PAD1_y_value, uint16_t sonar1, uint16_t sonar2, uint16_t sonar3);


/*
 * Funzione: is_invalid_value
 * Scopo: Determina se un valore è fuori dal range specificato.
 * Input: x = valore da verificare, min = limite inferiore, max = limite superiore.
 * Output: true se il valore è fuori dal range, altrimenti false.
 */
bool is_invalid_value(int x, int min, int max);


/*
 * Funzione: are_values_zero
 * Scopo: Determina se tutti e quattro i valori passati sono zero.
 * Input: x1, y1, x2, y2 - valori da verificare.
 * Output: true se tutti i valori sono zero, false altrimenti.
 */
bool are_values_zero(int x1, int y1, int x2, int y2);

#endif /* INC_CONTROLLER_H_ */
