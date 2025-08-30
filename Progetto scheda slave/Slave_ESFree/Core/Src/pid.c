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
 * Questo file implementa le funzioni relative al controllo PID.
 * Contiene l'inizializzazione del controller PID e il calcolo dei comandi PID
 * in risposta agli errori rilevati tra l'angolo attuale e il setpoint desiderato.
 */

#include "pid.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

// Buffer per la comunicazione tramite USART
char usart_buffer[100];

/*
 * Funzione: PID_Init
 * Scopo: Inizializza la struttura PID_Controller con i parametri di guadagno.
 * Input:
 * - pid: puntatore alla struttura PID_Controller da inizializzare.
 * - kp: guadagno proporzionale.
 * - ki: guadagno integrale.
 * - kd: guadagno derivativo.
 * Output: Nessuno.
 */
void PID_Init(PID_Controller *pid, double kp, double ki, double kd) {
    pid->kp = kp;               // Imposta il guadagno proporzionale
    pid->ki = ki;               // Imposta il guadagno integrale
    pid->kd = kd;               // Imposta il guadagno derivativo
    pid->ITerm = 0.0;           // Resetta il termine integrale
    pid->lastInput = 0.0;      // Resetta l'ultimo input
    pid->lastTick = 0;          // Resetta il tick temporale
    pid->lastAngle = 0.0;      // Resetta l'ultimo angolo
}

/*
 * Funzione: PID_Compute
 * Scopo: Calcola la risposta PID basata sulla differenza tra setpoint e velocità angolare attuale.
 * Input:
 * - pid: puntatore alla struttura PID_Controller con i parametri e lo stato corrente.
 * - counterValue: valore corrente del contatore dei sensori.
 * - ticks: valore del tempo trascorso in tick.
 * - setpoint: obiettivo angolare desiderato.
 * - resolution: risoluzione del sensore (gradi per tick).
 * - motore: identificatore del motore su cui applicare il comando.
 * Output:
 * - uint8_t: valore di uscita calcolato dal PID, limitato entro valori massimi e minimi.
 */
uint8_t PID_Compute(PID_Controller *pid, int32_t counterValue, uint32_t ticks, float setpoint, float resolution, int motore) {
    double output = 0.0;

    // Calcola l'angolo corrente in base al valore del contatore e alla risoluzione del sensore
    float angle = (360.0 / resolution) * counterValue;

    // Calcolo del tempo trascorso in secondi
    float deltaTime = (ticks - pid->lastTick) / 1000.0;

    // Calcola la velocità angolare se il tempo trascorso è positivo
    float angularSpeed = 0;
    if (deltaTime > 0) {
        angularSpeed = 60 * (angle - pid->lastAngle) / (deltaTime * 360.0);
    }

    // Caso PID con setpoint negativo
    if (setpoint < 0.0) {
        double powMin = 63.0, powMax = 1.0; // Limiti di potenza minimi e massimi
        double error = setpoint - angularSpeed;

        // Aggiorna il termine integrale con il guadagno integrale
        pid->ITerm += (pid->ki * error);

        // Limita il valore del termine integrale
        if (pid->ITerm > powMin) pid->ITerm = powMin;
        else if (pid->ITerm < powMax) pid->ITerm = powMax;

        // Calcola il termine derivativo
        double dInput = angularSpeed - pid->lastInput;

        // Calcola l'output PID
        output = pid->kp * error + pid->ITerm - pid->kd * dInput;

        // Limita l'output
        if (output > powMin) output = powMin;
        else if (output < powMax) output = powMax;
    }
    // Caso PID con setpoint positivo
    else if (setpoint > 0.0) {
        double powMin = 64.0, powMax = 127.0; // Limiti di potenza minimi e massimi
        double error = setpoint - angularSpeed;

        // Aggiorna il termine integrale con il guadagno integrale
        pid->ITerm += (pid->ki * error);

        // Limita il valore del termine integrale
        if (pid->ITerm > powMax) pid->ITerm = powMax;
        else if (pid->ITerm < powMin) pid->ITerm = powMin;

        // Calcola il termine derivativo
        double dInput = angularSpeed - pid->lastInput;

        // Calcola l'output PID
        output = pid->kp * error + pid->ITerm - pid->kd * dInput;

        // Limita l'output
        if (output > powMax) output = powMax;
        else if (output < powMin) output = powMin;
    }
    // Caso con setpoint vicino a zero
    else {
        double powMin = 64.0, powMax = 64;
        double error = setpoint - angularSpeed;

        pid->ITerm += (pid->ki * error);
        if (pid->ITerm > powMax) pid->ITerm = powMax;
        else if (pid->ITerm < powMin) pid->ITerm = powMin;

        double dInput = angularSpeed - pid->lastInput;

        output = pid->kp * error + pid->ITerm - pid->kd * dInput;

        if (output > powMax) output = powMax;
        else if (output < powMin) output = powMin;
    }

    // Aggiorna i valori precedenti per il prossimo ciclo
    pid->lastTick = ticks;
    pid->lastAngle = angle;
    pid->lastInput = angularSpeed;

    
    return (uint8_t)output; // Restituisce l'output limitato calcolato
}
