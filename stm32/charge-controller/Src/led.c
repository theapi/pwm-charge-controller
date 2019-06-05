#include "led.h"

void LED_Init(LED_HandleTypeDef* led, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	led->GPIOx = GPIOx;
	led->GPIO_Pin = GPIO_Pin;
	led->onDuration = 0;
	led->offDuration = 0;
	led->pauseDuration = 0;
	led->pulses = 0;
	led->pulseCnt = 0;
	led->previousMillis = 0;
	led->state = LED_STATE_NONE;
}

void LED_on(LED_HandleTypeDef* led) {
	HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
}

void LED_off(LED_HandleTypeDef* led) {
	HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
}

void LED_flash(LED_HandleTypeDef* led) {
	uint32_t currentMillis = HAL_GetTick();

	switch (led->state) {

	case LED_STATE_NONE:
		led->state = LED_STATE_TURN_ON;
		break;

	case LED_STATE_TURN_ON:
		HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
		led->pulseCnt++; // Increase loop counter
		led->previousMillis = currentMillis;
		led->state = LED_STATE_ON;
		break;

	case LED_STATE_ON:
		if (currentMillis - led->previousMillis >= led->onDuration) {
			led->state = LED_STATE_TURN_OFF;
		}
		break;

	case LED_STATE_TURN_OFF:
		HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
		led->previousMillis = currentMillis;
		led->state = LED_STATE_OFF;
		break;

	case LED_STATE_OFF:
		if (currentMillis - led->previousMillis >= led->offDuration) {
			led->state = LED_STATE_SEQUENCE_CHECK;
		}
		break;

	case LED_STATE_SEQUENCE_CHECK:
		if (led->pulseCnt >= led->pulses) {
			led->pulseCnt = 0;
			led->previousMillis = currentMillis;
			led->state = LED_STATE_PAUSE; // Sequence finished
		} else {
			led->state = LED_STATE_NONE;
		}
		break;

	case LED_STATE_PAUSE:
		if (currentMillis - led->previousMillis >= led->pauseDuration) {
			led->state = LED_STATE_NONE;
		}
		break;
	}

}

