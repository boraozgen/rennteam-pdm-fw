/*
 * AdcDrv.h
 *
 *  Created on: 12.03.2019
 *      Author: Bora Ozgen
 */

#ifndef ADCDRV_H_
#define ADCDRV_H_

#include "stdint.h"

int AdcDrv_init(void);
float AdcDrv_readCurrent(uint8_t channel);
uint8_t AdcDrv_readCurrentMapped(uint8_t channel);
uint8_t AdcDrv_readBatVoltageMapped(void);
uint8_t AdcDrv_readTemperatureMapped(void);

#endif /* ADCDRV_H_ */
