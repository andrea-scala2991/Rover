/*
 * hc-sr04.h
 *
 *  Created on: Nov 26, 2024
 *      Author: andrea
 */
#include "gpio.h"

#ifndef SRC_HC_SR04_H_
#define SRC_HC_SR04_H_

#define SOUND_SPEED 0.0343
#define TIMER_RESOLUTION 170000000

typedef struct {
	GPIO_TypeDef	*echo_port;
	uint16_t		echo_pin;
	GPIO_TypeDef	*trigger_port;
	uint16_t		trigger_pin;
	uint16_t		distance;
} sonar_t;

void sonar_init(sonar_t *sonar, GPIO_TypeDef *echo_port, uint16_t echo_pin, GPIO_TypeDef *trigger_port, uint16_t trigger_pin);

uint16_t sonar_compute_distance(sonar_t *sonar, TIM_HandleTypeDef* htim);

#endif /* SRC_HC_SR04_H_ */
