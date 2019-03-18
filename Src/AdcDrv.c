/*
 * AdcDrv.c
 *
 *  Created on: 12.03.2019
 *      Author: Bora Ozgen
 */

#include "AdcDrv.h"
#include "stm32f3xx_hal.h"
#include "main.h"
#include "App.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

__IO uint16_t   ADC1ConvertedValues[ADC1_CH_COUNT];
__IO uint16_t   ADC2ConvertedValues[ADC2_CH_COUNT];
__IO uint16_t   ADC3ConvertedValues[ADC3_CH_COUNT];
__IO uint16_t   ADC4ConvertedValues[ADC4_CH_COUNT];

extern App_channel_t App_channels[CHANNEL_COUNT];

int AdcDrv_init(void)
{
#if 0
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
#endif
	/* Assign ADC value pointers to channels */
	App_channels[CH1].adc_value_ptr = &ADC4ConvertedValues[5];
	App_channels[CH2].adc_value_ptr = &ADC4ConvertedValues[4];
	App_channels[CH3].adc_value_ptr = &ADC3ConvertedValues[2];
	App_channels[CH4].adc_value_ptr = &ADC4ConvertedValues[2];
	App_channels[CH5].adc_value_ptr = &ADC4ConvertedValues[3];
	App_channels[CH6].adc_value_ptr = &ADC3ConvertedValues[1];
	App_channels[CH7].adc_value_ptr = &ADC4ConvertedValues[0];
	App_channels[CH8].adc_value_ptr = &ADC3ConvertedValues[4];
	App_channels[CH9].adc_value_ptr = &ADC3ConvertedValues[5];
	App_channels[CH10].adc_value_ptr = &ADC3ConvertedValues[0];
	App_channels[CH11].adc_value_ptr = &ADC3ConvertedValues[3];
	App_channels[CH12].adc_value_ptr = &ADC1ConvertedValues[1];
	App_channels[CH13].adc_value_ptr = &ADC1ConvertedValues[0];
	App_channels[CH14].adc_value_ptr = &ADC2ConvertedValues[1];
	App_channels[CH15].adc_value_ptr = &ADC2ConvertedValues[0];

	/* Assign K factor and Rsense values to channels */
	App_channels[CH1].k_factor = K_FACTOR_VN7003ALH;
	App_channels[CH1].r_sense = R_SENSE_VN7003ALH;
	App_channels[CH2].k_factor = K_FACTOR_VN7003ALH;
	App_channels[CH2].r_sense = R_SENSE_VN7003ALH;
	App_channels[CH3].k_factor = K_FACTOR_VN7003ALH;
	App_channels[CH3].r_sense = R_SENSE_VN7003ALH;
	App_channels[CH4].k_factor = K_FACTOR_VN7003ALH;
	App_channels[CH4].r_sense = R_SENSE_VN7003ALH;
	App_channels[CH5].k_factor = K_FACTOR_VN7003ALH;
	App_channels[CH5].r_sense = R_SENSE_VN7003ALH;

	App_channels[CH6].k_factor = K_FACTOR_VND5012AK;
	App_channels[CH6].r_sense = R_SENSE_VND5012AK;
	App_channels[CH7].k_factor = K_FACTOR_VND5012AK;
	App_channels[CH7].r_sense = R_SENSE_VND5012AK;
	App_channels[CH8].k_factor = K_FACTOR_VND5012AK;
	App_channels[CH8].r_sense = R_SENSE_VND5012AK;
	App_channels[CH9].k_factor = K_FACTOR_VND5012AK;
	App_channels[CH9].r_sense = R_SENSE_VND5012AK;
	App_channels[CH10].k_factor = K_FACTOR_VND5012AK;
	App_channels[CH10].r_sense = R_SENSE_VND5012AK;
	App_channels[CH11].k_factor = K_FACTOR_VND5012AK;
	App_channels[CH11].r_sense = R_SENSE_VND5012AK;

	App_channels[CH12].k_factor = K_FACTOR_VND5050AK;
	App_channels[CH12].r_sense = R_SENSE_VND5050AK;
	App_channels[CH13].k_factor = K_FACTOR_VND5050AK;
	App_channels[CH13].r_sense = R_SENSE_VND5050AK;
	App_channels[CH14].k_factor = K_FACTOR_VND5050AK;
	App_channels[CH14].r_sense = R_SENSE_VND5050AK;
	App_channels[CH15].k_factor = K_FACTOR_VND5050AK;
	App_channels[CH15].r_sense = R_SENSE_VND5050AK;

	return 0;
}

float AdcDrv_readCurrent(uint8_t ch)
{
	uint16_t adcVal = 0;
	float floatVal = 0.0;

	/* Sanity check */
	if (ch > CHANNEL_COUNT - 1) {
		Error_Handler();
	} else {
		/* Retrieve last ADC value from the DMA buffer */
		adcVal = *(App_channels[ch].adc_value_ptr);
	}

	/* Calculate current value */
	// TODO: map K value to measurement for more accuracy
	// TODO: use Vrefint measurement instead of VREF?
	floatVal = ((float)adcVal * ADC_VREF * App_channels[ch].k_factor) / (App_channels[ch].r_sense * 4095.0F);

	return floatVal;
}

uint8_t AdcDrv_readCurrentMapped(uint8_t ch)
{
	float floatVal = 0.0;
	uint8_t mappedVal = 0;

	/* Read float value */
	floatVal = AdcDrv_readCurrent(ch);

	/* Sanity check */
	// TODO: use some error code
	if (floatVal < 0.0F) {
		return 0;
	} else if (floatVal > 25.5F ){
		return 255;
	}

	/* Map 0-25.5A to 0-255A */
	// TODO: do rounding
	mappedVal = (uint8_t)(floatVal * 10.0F);

	return mappedVal;
}

uint8_t AdcDrv_readBatVoltageMapped(void)
{
	uint16_t adcVal = 0;
	float floatVal = 0.0;
	uint8_t mappedVal = 0;

	/* Retrieve last ADC value from the DMA buffer */
	adcVal = ADC1ConvertedValues[2];

	/* Sanity check */
	// TODO: use some error code
	if (floatVal < 0.0F) {
		return 0;
	} else if (floatVal > 25.5F ){
		return 255;
	}

	/* Calculate float value */
	floatVal = ((float)adcVal * ADC_VREF) / (11.0F * 4095.0F);

	/* Map 0-25.5V to 0-255 */
	mappedVal = (uint8_t)(floatVal * 10.0F);

	return mappedVal;
}

uint8_t AdcDrv_readTemperatureMapped(void)
{
	// TODO: NTC calculation
	return 0;
}

