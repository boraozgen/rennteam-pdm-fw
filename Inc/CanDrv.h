/*
 * CanDrv.h
 *
 *  Created on: 12.03.2019
 *      Author: Papa Motec
 */

#ifndef CANDRV_H_
#define CANDRV_H_

#include "stdint.h"

int CanDrv_init(void);
int CanDrv_sendCurrentFrameA(uint8_t* currentFrameA_data);
int CanDrv_sendCurrentFrameB(uint8_t* currentFrameB_data);
int CanDrv_sendStatusFrame(uint8_t* statusFrame_data);
int CanDrv_sendFuseMessage(uint8_t channel);

#endif /* CANDRV_H_ */
