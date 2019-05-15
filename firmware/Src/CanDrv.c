/*
 * CanDrv.c
 *
 *  Created on: 12.03.2019
 *      Author: Bora Ozgen
 */

#include "CanDrv.h"
#include "App.h"
#include "stm32f3xx_hal.h"
#include "main.h"

CAN_HandleTypeDef hcan;
uint32_t TxMailbox;

CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];

int CanDrv_sendFrame(uint8_t* data, uint8_t dlc, uint16_t id);

int CanDrv_init(void)
{
	CAN_FilterTypeDef  sFilterConfig = {0};

	/* Configure the CAN Filter */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = CANDRV_PDM_CTRL_ID << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = CANDRV_PDM_CTRL_MASK << 5;
	sFilterConfig.FilterMaskIdLow = 0xFFFF;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	/* Start the CAN module */
	HAL_CAN_Start(&hcan);

	/* Activate CAN RX notification */
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}
	return 0;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  /* Process command */
  // TODO: optimize / delegate this to a task?
  if (RxHeader.StdId == CANDRV_PDM_CTRL_ID) {
	  for (uint8_t ch=0; ch < 8; ch++) {
		  if ((RxData[0] & (0x01 << ch)) == 0x00) {
			App_turnOffChannel(ch);
		  } else {
			App_turnOnChannel(ch);
		  }
	  }

	  for (uint8_t ch=8; ch < CHANNEL_COUNT; ch++) {
		  if ((RxData[1] & (0x01 << (ch - 8))) == 0x00) {
			App_turnOffChannel(ch);
		  } else {
			App_turnOnChannel(ch);
		  }
	  }
  }
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
