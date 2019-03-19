/*
 * CanDrv.c
 *
 *  Created on: 12.03.2019
 *      Author: Bora Ozgen
 */

#include "CanDrv.h"
#include "stm32f3xx_hal.h"
#include "main.h"

CAN_HandleTypeDef hcan;
uint32_t TxMailbox;

int CanDrv_sendFrame(uint8_t* data, uint8_t dlc, uint16_t id);

int CanDrv_init(void)
{
	/* Start the CAN module */
	HAL_CAN_Start(&hcan);
	return 0;
}

int CanDrv_sendCurrentFrameA(uint8_t* currentFrameA_data)
{
	/* Pass the array and parameters to sendFrame */
	return CanDrv_sendFrame(currentFrameA_data, CANDRV_DLC_CURRENT_A, CANDRV_ID_CURRENT_A);
}

int CanDrv_sendCurrentFrameB(uint8_t* currentFrameB_data)
{
	/* Pass the array and parameters to sendFrame */
	return CanDrv_sendFrame(currentFrameB_data, CANDRV_DLC_CURRENT_B, CANDRV_ID_CURRENT_B);
}

int CanDrv_sendStatusFrame(uint8_t* statusFrame_data)
{
	uint8_t TxDataChannel[CANDRV_DLC_STATUS] = {0};

	TxDataChannel[CANDRV_STATUS_TEMP] = statusFrame_data[CANDRV_STATUS_TEMP];
	TxDataChannel[CANDRV_STATUS_VBAT] = statusFrame_data[CANDRV_STATUS_VBAT];
	TxDataChannel[CANDRV_STATUS_CODE] = 0; // Not yet defined

	/* Pass the array and parameters to sendFrame */
	return CanDrv_sendFrame(TxDataChannel, CANDRV_DLC_STATUS, CANDRV_ID_STATUS);
}

int CanDrv_sendFuseMessage(uint8_t ch)
{
	uint8_t TxDataChannel[CANDRV_DLC_EVENT] = {0};

	TxDataChannel[CANDRV_EVENT_TYPE] = CANDRV_EVENT_TYPE_FUSE;
	TxDataChannel[CANDRV_EVENT_CHANNEL] = ch;

	/* Pass the array and parameters to sendFrame */
	return CanDrv_sendFrame(TxDataChannel, CANDRV_DLC_EVENT, CANDRV_ID_EVENT);
}

int CanDrv_sendFrame(uint8_t* data, uint8_t dlc, uint16_t id)
{
	/* Set frame parameters */
	CAN_TxHeaderTypeDef TxHeaderChannel = {0};
	TxHeaderChannel.StdId = id;
	TxHeaderChannel.RTR = CAN_RTR_DATA;
	TxHeaderChannel.IDE = CAN_ID_STD;
	TxHeaderChannel.DLC = dlc;
	TxHeaderChannel.TransmitGlobalTime = DISABLE;

	/* Wait for a mailbox to get freed */
	while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan));

	/* Start the transmission process */
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderChannel, data, &TxMailbox) != HAL_OK)
	{
	  /* Transmission request error */
	  Error_Handler();
	}

	return 0;
}
