/*
 * AdcDrv.c
 *
 *  Created on: 12.03.2019
 *      Author: Bora Ozgen
 */

#include "AdcDrv.h"
#include "stm32f3xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

int AdcDrv_init(void)
{
	if (HAL_OK !=  HAL_ADC_Start(&hadc1)) {
		return -1;
	}

	if (HAL_OK !=  HAL_ADC_Start(&hadc2)) {
		return -1;
	}

	if (HAL_OK !=  HAL_ADC_Start(&hadc3)) {
		return -1;
	}

	if (HAL_OK !=  HAL_ADC_Start(&hadc4)) {
		return -1;
	}

	return 0;
}

float AdcDrv_readCurrent(uint8_t channel)
{
	uint32_t adcValue = 0;

	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		adcValue = HAL_ADC_GetValue(&hadc1);
	} else {
		return -1;
	}

	return adcValue;
}

uint8_t AdcDrv_readCurrentMapped(uint8_t channel)
{
	return 0;
}

uint8_t AdcDrv_readBatVoltageMapped(void)
{
	return 0;
}

uint8_t AdcDrv_readTemperatureMapped(void)
{
	return 0;
}

