/*
 * CanDrv.h
 *
 *  Created on: 12.03.2019
 *      Author: Papa Motec
 */

#ifndef CANDRV_H_
#define CANDRV_H_

#include "stdint.h"

/* Frame IDs */
#define CANDRV_ID_CURRENT_A		0x500
#define CANDRV_ID_CURRENT_B		0x501
#define CANDRV_ID_STATUS		0x502
#define CANDRV_ID_EVENT			0x503

/* Frame DLCs */
#define CANDRV_DLC_CURRENT_A	8
#define CANDRV_DLC_CURRENT_B	7
#define CANDRV_DLC_STATUS		3
#define CANDRV_DLC_EVENT		2

/* Event frame indexes */
#define CANDRV_EVENT_TYPE		0
#define CANDRV_EVENT_CHANNEL	1

/* Status frame indexes */
#define CANDRV_STATUS_TEMP		0
#define CANDRV_STATUS_VBAT		1
#define CANDRV_STATUS_CODE		2

/* Event frame defines */
#define CANDRV_EVENT_TYPE_FUSE	0

/* Receiver defines */
#define CANDRV_PDM_CTRL_ID		0x0060
#define CANDRV_PDM_CTRL_MASK	0xFFFF

int CanDrv_init(void);
int CanDrv_sendCurrentFrameA(uint8_t* currentFrameA_data);
int CanDrv_sendCurrentFrameB(uint8_t* currentFrameB_data);
int CanDrv_sendStatusFrame(uint8_t* statusFrame_data);
int CanDrv_sendFuseMessage(uint8_t channel);

#endif /* CANDRV_H_ */
