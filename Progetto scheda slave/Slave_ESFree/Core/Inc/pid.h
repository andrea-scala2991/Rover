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
 * Questo header definisce la struttura e le funzioni necessarie per il controllo PID del sistema.
 * Contiene la definizione della struttura `PID_Controller`, che memorizza i parametri del controller PID
 * e lo stato corrente, oltre alle funzioni per l'inizializzazione e il calcolo del controllo.
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
#include "main.h"

/*
 * Struttura dati per il controllo PID (Proporzionale, Integrale, Derivativo).
 * Contiene tutti i parametri del controller PID e le variabili di stato per il calcolo.
 */
typedef struct {
    double kp;           // Guadagno proporzionale.
    double ki;           // Guadagno integrale.
    double kd;           // Guadagno derivativo.
    double ITerm;        // Termine integrale accumulato.
    double lastInput;    // Ultimo valore di input utilizzato nel calcolo PID.
    uint32_t lastTick;   // Ultimo tick temporale.
    float lastAngle;     // Ultimo angolo rilevato dal sistema.
} PID_Controller;


/*
 * Funzione: PID_Init
 * Scopo: Inizializza la struttura PID con i guadagni specificati.
 * Input:
 * - pid: puntatore alla struttura PID_Controller da inizializzare.
 * - kp: valore del guadagno proporzionale.
 * - ki: valore del guadagno integrale.
 * - kd: valore del guadagno derivativo.
 * Output: Nessuno. La funzione imposta i parametri della struttura PID.
 */
void PID_Init(PID_Controller *pid, double kp, double ki, double kd);


/*
 * Funzione: PID_Compute
 * Scopo: Calcola la risposta PID in base ai valori di input e al setpoint desiderato.
 * Input:
 * - pid: puntatore alla struttura PID_Controller che contiene i parametri e lo stato corrente.
 * - counterValue: valore di conteggio corrente rilevato dal sensore.
 * - ticks: valore dei tick temporali correnti.
 * - setpoint: obiettivo di riferimento che il sistema deve raggiungere.
 * - resolution: risoluzione del sistema o sensore utilizzato.
 * - motore: identificatore del motore su cui applicare il comando PID.
 * Output:
 * - uint8_t: risultato del calcolo PID (comando da applicare al motore).
 */
uint8_t PID_Compute(PID_Controller *pid, int32_t counterValue, uint32_t ticks, float setpoint, float resolution, int motore);

#endif /* INC_PID_H_ */
