/*
 * App.c
 *
 *  Created on: 16.03.2019
 *      Author: Bora Ozgen
 */

#include "App.h"
#include "main.h"
#include "cmsis_os.h"

void Timer_Callback (void const *arg);
osTimerDef (Timer, Timer_Callback);
osTimerId timerId;

int App_init(void)
{
	/* Initialize always-on channels */
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_0, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
						  |GPIO_PIN_9, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

	/* Initialize timers */
	timerId = osTimerCreate (osTimer(Timer), osTimerOnce, NULL);

	return 0;
}

int App_turnOffChannel(uint8_t channel, uint32_t time_ms)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // Test Ch1

	osTimerStart(timerId, time_ms);

	return 0;
}

void Timer_Callback (void const *arg) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); // Test Ch1
}
