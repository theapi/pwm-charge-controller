/*
 * led.h
 *
 *  Created on: 5 Jun 2019
 *      Author: peter
 */

#ifndef LED_H_
#define LED_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdint.h"

typedef enum {
    LED_STATE_NONE,
	LED_STATE_TURN_ON,
	LED_STATE_ON,
	LED_STATE_TURN_OFF,
	LED_STATE_OFF,
	LED_STATE_SEQUENCE_CHECK,
	LED_STATE_PAUSE,
} LED_StateTypeDef;

typedef struct {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	uint32_t onDuration;
	uint32_t offDuration;
	uint32_t pauseDuration;
	uint16_t pulses;
	uint16_t pulseCnt;
	uint32_t previousMillis;
	LED_StateTypeDef state;
} LED_HandleTypeDef;

void LED_Init(LED_HandleTypeDef* led, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void LED_on(LED_HandleTypeDef* led);
void LED_off(LED_HandleTypeDef* led);
void LED_flash(LED_HandleTypeDef* led);

#ifdef __cplusplus
}
#endif
#endif /* LED_H_ */
