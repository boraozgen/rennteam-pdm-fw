/*
 * AdcDrv.h
 *
 *  Created on: 12.03.2019
 *      Author: Bora Ozgen
 */

#ifndef ADCDRV_H_
#define ADCDRV_H_

#include "stdint.h"

#define ADC_VREF 			3.286F
#define K_FACTOR_VN7003ALH	16650.0F
#define K_FACTOR_VND5012AK	5100.0F
#define K_FACTOR_VND5050AK	2020.0F
#define R_SENSE_VN7003ALH	2000.0F
#define R_SENSE_VND5012AK	1000.0F
#define R_SENSE_VND5050AK	2000.0F

#define ADC1_CH_COUNT	4
#define ADC2_CH_COUNT	2
#define ADC3_CH_COUNT	6
#define ADC4_CH_COUNT	6

int AdcDrv_init(void);
float AdcDrv_readCurrent(uint8_t ch);
uint8_t AdcDrv_readCurrentMapped(uint8_t ch);
uint8_t AdcDrv_readBatVoltageMapped(void);
uint8_t AdcDrv_readTemperatureMapped(void);

#endif /* ADCDRV_H_ */
