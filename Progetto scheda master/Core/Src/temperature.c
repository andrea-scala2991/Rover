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

#include "temperature.h"
#include "adc.h"

// Indirizzi di memoria per la calibrazione del sensore di temperatura
#define VREFINT_CAL_ADDR                   ((uint16_t*) (0x1FFF75AAUL)) // VrefInt ADC raw data a 30°C
#define VREFINT_CAL_VREF                   (3300UL)                    // Vref+ durante la calibrazione

#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) (0x1FFF75A8UL)) // TS_CAL1 a 30°C
#define TEMPSENSOR_CAL2_ADDR               ((uint16_t*) (0x1FFF75CAUL)) // TS_CAL2 a 110°C
#define TEMPSENSOR_CAL1_TEMP               (30L)                        // Temperatura corrispondente a TS_CAL1 (°C)
#define TEMPSENSOR_CAL2_TEMP               (110L)                       // Temperatura corrispondente a TS_CAL2 (°C)


// Funzione per inizializzare il sensore di temperatura e ADC
void TemperatureSensor_Init(void) {
    // Inizializzazione dell'ADC, della UART, ecc. (già implementata in main)
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        printf("Errore calibrazione ADC\r\n");
    } else {
        printf("ADC Calibrato\r\n");
    }
}

// Funzione per leggere la temperatura in °C
int16_t TemperatureSensor_Read(void) {
    uint16_t rawValue;
    uint16_t temperature;
    uint16_t cal1 = *(uint16_t*)TEMPSENSOR_CAL1_ADDR;
    uint16_t cal2 = *(uint16_t*)TEMPSENSOR_CAL2_ADDR;

    // Avvio della lettura ADC
    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        rawValue = HAL_ADC_GetValue(&hadc1);

        // Calcolo della tensione sul sensore di temperatura
        uint16_t ts_voltage = rawValue * (VREFINT_CAL_VREF / *VREFINT_CAL_ADDR);

        // Calcolo della temperatura in base ai valori di calibrazione
        temperature = ((TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (cal2 - cal1)) * (ts_voltage - cal1) + TEMPSENSOR_CAL1_TEMP;

        return temperature;
    }

    return -1; // Se la lettura non va a buon fine, restituisce un valore di errore
}
