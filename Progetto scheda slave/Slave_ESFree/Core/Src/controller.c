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
Progettare e implementare un rover in grado di eseguire i comandi ricevuti tramite un controller
il rover può trovarsi in tre diversi stati operativi: OK quando il sistema funziona correttamente senza anomalie,
DEGRADATO in caso di malfunzionamento parziale che riduce alcune funzionalità ma mantiene il rover operativo,
EMERGENZA quando il rover entra in uno stato di sicurezza per prevenire danni limitando o sospendendo le operazioni.
*/

#include "controller.h"

/*
 * Questa funzione analizza i dati provenienti dalla struttura passata tramite I2C e prende decisioni
 * in base ai valori ricevuti. Di seguito i passi eseguiti:
 *
 * 1. Controllo dei valori fuori range:
 *    - Verifica se i valori ricevuti sono al di fuori dell'intervallo valido (0-511).
 *      Valori fuori range vengono ignorati per evitare di impartire comandi errati al rover.
 *      Questo accade, ad esempio, quando il dispositivo ESP32 si disconnette momentaneamente, generando
 *      valori casuali per alcuni secondi.
 *    - Inoltre, verifica se tutti i valori sono pari a zero. Se tutti i valori sono zero, significa che il
 *      controller non è collegato. Anche in questo caso, per evitare comandi errati, i valori vengono ignorati
 *      e le velocità del rover (sia della parte destra che sinistra) sono impostate a zero.
 *
 * 2. Frenata di emergenza:
 *    - Controlla se il pulsante `btn1` è premuto. Se `btn1` è premuto, si considera attivata la frenata di emergenza,
 *      il rover deve fermarsi immediatamente, ignorando tutti i comandi ricevuti. Le velocità delle ruote sono impostate a zero.
 *
 * 3. Calcolo delle velocità:
 *    - Se nessuna delle condizioni precedenti è verificata, viene chiamata la funzione per calcolare
 *      la velocità delle ruote e la rotazione, utilizzando i dati ricevuti (`bx` e `ay`).
 */


wheel_speeds_t compute(controller_t *controller_data, uint16_t sonar1, uint16_t sonar2, uint16_t sonar3){
	wheel_speeds_t speeds;

    if (are_values_zero(controller_data->ax, controller_data->ay, controller_data->bx, controller_data->by) ||
    		is_invalid_value(controller_data->ax, 0, 511) ||
			is_invalid_value(controller_data->ay, 0, 511) ||
			is_invalid_value(controller_data->bx, 0, 511) ||
			is_invalid_value(controller_data->by, 0, 511)){

		  speeds.left_speed=0;
		  speeds.right_speed=0;
    }
    else{
        if(controller_data->btn1 == 0){
        	speeds = calculate_wheel_speeds(controller_data->bx,controller_data->ay, sonar1, sonar2, sonar3);
        }
        else{
        	speeds.left_speed=0;
        	speeds.right_speed=0;
        }
    }

    return speeds;
}

/*
 * Questa funzione calcola le velocità delle ruote del rover in base ai valori degli input analogici
 * forniti dai pad (PAD1_y_value e PAD2_x_value). Di seguito il comportamento della funzione:
 *
 * 1. Calcolo del setpoint:
 *    - Viene calcolato il rapporto (`ratio`) tra il valore di `PAD1_y_value` e il range compreso tra
 *      `PAD1_y_value_min_su` e `PAD1_y_value_max_su`.
 *    - Questo rapporto viene utilizzato per calcolare il `setpoint`, che rappresenta il valore base
 *      della velocità da assegnare alle ruote.
 *
 * 2. Controllo della posizione dell'analogico Y (PAD1_y_value):
 *    a) Se `PAD1_y_value` è fermo (compreso tra `PAD1_y_value_max_giu` e `PAD1_y_value_min_su`):
 *       - Controlla il valore di `PAD2_x_value` per determinare il comportamento:
 *         - Se `PAD2_x_value` è compreso tra 350 e 511, il rover gira a sinistra da fermo
 *           (ruota sinistra indietro, ruota destra avanti).
 *         - Se `PAD2_x_value` è compreso tra 0 e 150, il rover gira a destra da fermo
 *           (ruota sinistra avanti, ruota destra indietro).
 *         - Se `PAD2_x_value` è compreso tra 150 e 350, entrambi gli analogici sono fermi,
 *           quindi il rover rimane fermo (velocità delle ruote pari a zero).
 *
 *    b) Se `PAD1_y_value` è tirato indietro (compreso tra `PAD1_y_value_min_giu` e `PAD1_y_value_max_giu`):
 *       - Il rover ruota su sé stesso di 180° (ruota sinistra avanti, ruota destra indietro).
 *
 *    c) Se `PAD1_y_value` è spinto in avanti (compreso tra `PAD1_y_value_min_su` e `PAD1_y_value_max_su`):
 *       - Entrambe le ruote ricevono la velocità calcolata dal `setpoint`, e il rover si muove in avanti.
 *
 * 3. Restituzione delle velocità:
 *    - La funzione restituisce una struttura `wheel_speeds_t` contenente le velocità calcolate
 *      per la ruota sinistra (`left_speed`) e la ruota destra (`right_speed`).
 */

wheel_speeds_t calculate_wheel_speeds(int PAD2_x_value, int PAD1_y_value, uint16_t sonar1, uint16_t sonar2, uint16_t sonar3) {

    wheel_speeds_t speeds;
    float ratio = (float)(PAD1_y_value - PAD1_y_value_min_su) / (PAD1_y_value_max_su - PAD1_y_value_min_su);
    int setpoint = ratio * 160.0;

    //Verifico se l'analogico è fermo y del movimento avanti o indietro è fermo
    if(PAD1_y_value >= PAD1_y_value_max_giu && PAD1_y_value <= PAD1_y_value_min_su){

    	if (PAD2_x_value >= 350 && PAD2_x_value <= 511) { //fermo gira verso sinitra
        	speeds.left_speed=20;
        	speeds.right_speed=-20;
        }
        else if(PAD2_x_value >= 0 && PAD2_x_value <= 150){ //fermo gira verso destra
        	speeds.left_speed=-20;
          	speeds.right_speed=20;
        }
        else if(PAD2_x_value>150 && PAD2_x_value<350){//fermo sia nalogico pad1_y e fermo analogico pad2_x allora il rover sta fermo
        	speeds.left_speed=0;
        	speeds.right_speed=0;
        }
    }
    else if(PAD1_y_value >= PAD1_y_value_min_giu && PAD1_y_value < PAD1_y_value_max_giu){//se tiro in dietro la leveta si gira su se stesso di 180 gradi
    	speeds.left_speed=20;
    	speeds.right_speed=-20;
    }
    else if(PAD1_y_value > PAD1_y_value_min_su && PAD1_y_value <= PAD1_y_value_max_su){//se trio in avanti la levetta va in avanti


    	if(sonar2 < 70 || sonar1 < 23 || sonar3 < 23){

    	    	speeds.left_speed=0;
    	    	speeds.right_speed=0;

    	}else{
			speeds.left_speed=setpoint;
			speeds.right_speed=setpoint;
    	}
    }

    return speeds;
}

/*
 * Funzione: is_invalid_value
 * Scopo: Verifica se un valore è al di fuori dell'intervallo specificato.
 *
 * Parametri:
 * - int x: il valore da controllare.
 * - int min: il limite inferiore dell'intervallo.
 * - int max: il limite superiore dell'intervallo.
 *
 * Ritorna:
 * - true se il valore è al di fuori dell'intervallo [min, max].
 * - false altrimenti.
 */
bool is_invalid_value(int x, int min, int max) {
    return (x < min || x > max);
}
/*
 * Funzione: are_values_zero
 * Scopo: Verifica se tutti e quattro i valori passati sono zero.
 *
 * Parametri:
 * - int x1: il primo valore da verificare.
 * - int y1: il secondo valore da verificare.
 * - int x2: il terzo valore da verificare.
 * - int y2: il quarto valore da verificare.
 *
 * Ritorna:
 * - true se tutti i valori sono zero.
 * - false se almeno uno dei valori non è zero.
 */
bool are_values_zero(int x1, int y1, int x2, int y2) {
    return (x1 == 0 && y1 == 0 && x2 == 0 && y2 == 0);
}

//LA CONSERVO SE SERVE MA INUTILE

/*Questa funzione fa due cose principalmente prende il valore del primo analogico e vede l'asse y
 * 1.Se tale valore è maggiore  di PAD1_y_value_min_su=270 e minore e uguale di PAD1_y_value_max_su=511
 * 	 fa la conversione tra valore dell'analogico e setpoint.
 * 2.Se tale valore invece è fermo quindi compreso tra PAD1_y_value_max_giu=260 e PAD1_y_value_min_su=270
 * 	 allora il controller imposta il setpoint a 0 in quanto il rover deve stare fermo
 * 3.Se tale valore è maggiore e uguale di PAD1_y_value_min_giu=0 e PAD1_y_value_max_giu= 260 significa
 * 	 che il pad sta verso il basso e non potendo fare la retromarcia sta fermo*/

/*int convert_to_setpoint(int PAD1_y_value) {
    int setpoint = 0;

    // Controllo per valori in senso orario (positivo)
    if (PAD1_y_value > PAD1_y_value_min_su && PAD1_y_value <= PAD1_y_value_max_su) {//Verifica se andare avnti
        setpoint = ((PAD1_y_value - PAD1_y_value_min_su) / (PAD1_y_value_max_su - PAD1_y_value_min_su)) * 160.0;//conversione setpoint
    }
    // Controllo per valori fermi
    else if (PAD1_y_value >= PAD1_y_value_max_giu && PAD1_y_value <= PAD1_y_value_min_su) {//verifico se rimanere fermp
        setpoint = 0;
    }
    else if (PAD1_y_value >= PAD1_y_value_min_giu && PAD1_y_value < PAD1_y_value_max_giu){//verifico se il pad è verso il basso e deve rimanere femro in quanto non può fare retromarcia
    	setpoint = 0;
    }

    // Controllo per valori in senso antiorario (negativo)
    //else if (PAD1_y_value >= PAD1_y_value_min_giu && PAD1_y_value < PAD1_y_value_max_giu) {//verifico se andare all'indietro
      //  setpoint = -((PAD1_y_value_max_giu - PAD1_y_value) / PAD1_y_value_max_giu) * 160.0;
    //}

    return setpoint;
}
*/
