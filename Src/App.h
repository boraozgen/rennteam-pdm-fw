/*
 * App.h
 *
 *  Created on: 16.03.2019
 *      Author: Bora Ozgen
 */

#ifndef APP_H_
#define APP_H_

#include "stdint.h"

#define CHANNEL_COUNT			15
#define CURRENT_FRAME_A_SIZE	8
#define CURRENT_FRAME_B_SIZE	7
#define STATUS_FRAME_SIZE		3

int App_init(void);
int App_turnOffChannel(uint8_t channel, uint32_t time_ms);

#endif /* APP_H_ */
