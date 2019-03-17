/*
 * AdcDrv.c
 *
 *  Created on: 12.03.2019
 *      Author: Bora Ozgen
 */

#include "AdcDrv.h"
#include "stm32f3xx_hal.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

__IO uint16_t   ADC1ConvertedValues[ADC1_CH_COUNT];
__IO uint16_t   ADC2ConvertedValues[ADC2_CH_COUNT];
__IO uint16_t   ADC3ConvertedValues[ADC3_CH_COUNT];
__IO uint16_t   ADC4ConvertedValues[ADC4_CH_COUNT];

int AdcDrv_init(void)
{
	if (HAL_OK !=  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1ConvertedValues, ADC1_CH_COUNT)) {
		return -1;
	}

	if (HAL_OK !=  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC2ConvertedValues, ADC2_CH_COUNT)) {
		return -1;
	}

	if (HAL_OK !=  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)ADC3ConvertedValues, ADC3_CH_COUNT)) {
		return -1;
	}

	if (HAL_OK !=  HAL_ADC_Start_DMA(&hadc4, (uint32_t *)ADC4ConvertedValues, ADC4_CH_COUNT)) {
		return -1;
	}

	return 0;
}

extern float testFuseCurrent;

float AdcDrv_readCurrent(uint8_t channel)
{
	uint16_t adcValue = 0;
	float retVal = 0.0;

	switch (channel) {
	case 0:
		//adcValue = ADC4ConvertedValues[5]; // Test, channel 1
		retVal = testFuseCurrent;
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	case 8:
		break;
	case 9:
		break;
	case 10:
		break;
	case 11:
		break;
	case 12:
		break;
	case 13:
		break;
	case 14:
		break;
	default:
		Error_Handler();
	}

	return retVal;
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

