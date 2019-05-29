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
	App_channels[CH1].auto_retry = false;
	App_channels[CH1].retry_time_ms = 1000;
	App_channels[CH1].timer_def.ptimer = Timer_Callback;
	App_channels[CH1].fuse_current = CH1_FUSE_CURRENT;

	App_channels[CH2].input_port = GPIOA;
	App_channels[CH2].input_pin = GPIO_PIN_9;
	App_channels[CH2].auto_retry = false;
	App_channels[CH2].retry_time_ms = 1000;
	App_channels[CH2].timer_def.ptimer = Timer_Callback;
	App_channels[CH2].fuse_current = CH2_FUSE_CURRENT;

	App_channels[CH3].input_port = GPIOD;
	App_channels[CH3].input_pin = GPIO_PIN_15;
	App_channels[CH3].auto_retry = false;
	App_channels[CH3].retry_time_ms = 1000;
	App_channels[CH3].timer_def.ptimer = Timer_Callback;
	App_channels[CH3].fuse_current = CH3_FUSE_CURRENT;

	App_channels[CH4].input_port = GPIOC;
	App_channels[CH4].input_pin = GPIO_PIN_7;
	App_channels[CH4].auto_retry = false;
	App_channels[CH4].retry_time_ms = 1000;
	App_channels[CH4].timer_def.ptimer = Timer_Callback;
	App_channels[CH4].fuse_current = CH4_FUSE_CURRENT;

	App_channels[CH5].input_port = GPIOC;
	App_channels[CH5].input_pin = GPIO_PIN_9;
	App_channels[CH5].auto_retry = false;
	App_channels[CH5].retry_time_ms = 1000;
	App_channels[CH5].timer_def.ptimer = Timer_Callback;
	App_channels[CH5].fuse_current = CH5_FUSE_CURRENT;

	App_channels[CH6].input_port = GPIOD;
	App_channels[CH6].input_pin = GPIO_PIN_4;
	App_channels[CH6].auto_retry = false;
	App_channels[CH6].retry_time_ms = 1000;
	App_channels[CH6].timer_def.ptimer = Timer_Callback;
	App_channels[CH6].fuse_current = CH6_FUSE_CURRENT;

	App_channels[CH7].input_port = GPIOD;
	App_channels[CH7].input_pin = GPIO_PIN_5;
	App_channels[CH7].auto_retry = false;
	App_channels[CH7].retry_time_ms = 1000;
	App_channels[CH7].timer_def.ptimer = Timer_Callback;
	App_channels[CH7].fuse_current = CH7_FUSE_CURRENT;

	App_channels[CH8].input_port = GPIOB;
	App_channels[CH8].input_pin = GPIO_PIN_9;
	App_channels[CH8].auto_retry = false;
	App_channels[CH8].retry_time_ms = 1000;
	App_channels[CH8].timer_def.ptimer = Timer_Callback;
	App_channels[CH8].fuse_current = CH8_FUSE_CURRENT;

	App_channels[CH9].input_port = GPIOE;
	App_channels[CH9].input_pin = GPIO_PIN_0;
	App_channels[CH9].auto_retry = false;
	App_channels[CH9].retry_time_ms = 1000;
	App_channels[CH9].timer_def.ptimer = Timer_Callback;
	App_channels[CH9].fuse_current = CH9_FUSE_CURRENT;

	App_channels[CH10].input_port = GPIOE;
	App_channels[CH10].input_pin = GPIO_PIN_2;
	App_channels[CH10].auto_retry = false;
	App_channels[CH10].retry_time_ms = 1000;
	App_channels[CH10].timer_def.ptimer = Timer_Callback;
	App_channels[CH10].fuse_current = CH10_FUSE_CURRENT;

	App_channels[CH11].input_port = GPIOE;
	App_channels[CH11].input_pin = GPIO_PIN_3;
	App_channels[CH11].auto_retry = false;
	App_channels[CH11].retry_time_ms = 1000;
	App_channels[CH11].timer_def.ptimer = Timer_Callback;
	App_channels[CH11].fuse_current = CH11_FUSE_CURRENT;

	App_channels[CH12].input_port = GPIOA;
	App_channels[CH12].input_pin = GPIO_PIN_2;
	App_channels[CH12].auto_retry = false;
	App_channels[CH12].retry_time_ms = 1000;
	App_channels[CH12].timer_def.ptimer = Timer_Callback;
	App_channels[CH12].fuse_current = CH12_FUSE_CURRENT;

	App_channels[CH13].input_port = GPIOA;
	App_channels[CH13].input_pin = GPIO_PIN_3;
	App_channels[CH13].auto_retry = false;
	App_channels[CH13].retry_time_ms = 1000;
	App_channels[CH13].timer_def.ptimer = Timer_Callback;
	App_channels[CH13].fuse_current = CH13_FUSE_CURRENT;

	App_channels[CH14].input_port = GPIOA;
	App_channels[CH14].input_pin = GPIO_PIN_5;
	App_channels[CH14].auto_retry = false;
	App_channels[CH14].retry_time_ms = 1000;
	App_channels[CH14].timer_def.ptimer = Timer_Callback;
	App_channels[CH14].fuse_current = CH14_FUSE_CURRENT;

	App_channels[CH15].input_port = GPIOA;
	App_channels[CH15].input_pin = GPIO_PIN_4;
	App_channels[CH15].auto_retry = false;
	App_channels[CH15].retry_time_ms = 1000;
	App_channels[CH15].timer_def.ptimer = Timer_Callback;
	App_channels[CH15].fuse_current = CH15_FUSE_CURRENT;

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

int App_triggerFuse(uint8_t ch)
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

int App_turnOffChannel(uint8_t ch)
{
	/* Turn off channel input */
	HAL_GPIO_WritePin(
			App_channels[ch].input_port,
			App_channels[ch].input_pin,
			GPIO_PIN_RESET);

	return 0;
}

int App_turnOnChannel(uint8_t ch)
{
	/* Turn on channel input */
	HAL_GPIO_WritePin(
			App_channels[ch].input_port,
			App_channels[ch].input_pin,
			GPIO_PIN_SET);

	return 0;
}

void Timer_Callback (void const *arg)
{
	/* Get channel ID from argument */
	uint32_t ch = (uint32_t)pvTimerGetTimerID((TimerHandle_t)arg);

	/* Turn on channel input */
	HAL_GPIO_WritePin(
			App_channels[ch].input_port,
			App_channels[ch].input_pin,
			GPIO_PIN_SET);
}
