/*
 * App.c
 *
 *  Created on: 16.03.2019
 *      Author: Bora Ozgen
 */

#include "App.h"

App_channel_t App_channels[CHANNEL_COUNT];

void Timer_Callback (void const *arg);

int App_init(void)
{
	/* Initialize channel structs */
	App_channels[CH1].input_port = GPIOC;
	App_channels[CH1].input_pin = GPIO_PIN_11;
	App_channels[CH1].auto_retry = true;
	App_channels[CH1].retry_time_ms = 1000;
	App_channels[CH1].timer_def.ptimer = Timer_Callback;
	App_channels[CH1].fuse_current = 10.0;

	App_channels[CH2].input_port = GPIOD;
	App_channels[CH2].input_pin = GPIO_PIN_15;
	App_channels[CH2].auto_retry = true;
	App_channels[CH2].retry_time_ms = 750;
	App_channels[CH2].timer_def.ptimer = Timer_Callback;
	App_channels[CH2].fuse_current = 10.0;

	App_channels[CH3].input_port = GPIOA;
	App_channels[CH3].input_pin = GPIO_PIN_9;
	App_channels[CH3].auto_retry = true;
	App_channels[CH3].retry_time_ms = 500;
	App_channels[CH3].timer_def.ptimer = Timer_Callback;
	App_channels[CH3].fuse_current = 10.0;

	App_channels[CH4].input_port = GPIOC;
	App_channels[CH4].input_pin = GPIO_PIN_7;
	App_channels[CH4].auto_retry = true;
	App_channels[CH4].retry_time_ms = 250;
	App_channels[CH4].timer_def.ptimer = Timer_Callback;
	App_channels[CH4].fuse_current = 10.0;

	App_channels[CH5].input_port = GPIOC;
	App_channels[CH5].input_pin = GPIO_PIN_9;
	App_channels[CH5].auto_retry = true;
	App_channels[CH5].retry_time_ms = 1250;
	App_channels[CH5].timer_def.ptimer = Timer_Callback;
	App_channels[CH5].fuse_current = 10.0;

	// TODO: add rest of the channels

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
	for (uint32_t ch=0; ch<CHANNEL_COUNT; ch++) {
		App_channels[ch].timer_id = osTimerCreate(&App_channels[ch].timer_def, osTimerOnce, (void*)ch);
	}

	return 0;
}

int App_turnOffChannel(uint8_t ch)
{
	/* Turn off channel input */

	HAL_GPIO_WritePin(
			App_channels[ch].input_port,
			App_channels[ch].input_pin,
			GPIO_PIN_RESET);

	/* Start auto-retry timer if enabled */
	if (App_channels[ch].auto_retry) {
		osTimerStart(App_channels[ch].timer_id, App_channels[ch].retry_time_ms);
	}

	return 0;
}

void Timer_Callback (void const *arg) {
	/* Get channel ID from argument */
	uint32_t ch = (uint32_t)pvTimerGetTimerID((TimerHandle_t)arg);

	/* Turn on channel input */
	HAL_GPIO_WritePin(
				App_channels[ch].input_port,
				App_channels[ch].input_pin,
				GPIO_PIN_SET);
}
