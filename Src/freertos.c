/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AdcDrv.h"
#include "CanDrv.h"
#include "App.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern App_channel_t App_channels[CHANNEL_COUNT];

/* USER CODE END Variables */
osThreadId measurementTaskHandle;
osThreadId fuseTaskHandle;
osThreadId testTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartMeasurementTask(void const * argument);
void StartFuseTask(void const * argument);
void StartTestTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of measurementTask */
  osThreadDef(measurementTask, StartMeasurementTask, osPriorityNormal, 0, 128);
  measurementTaskHandle = osThreadCreate(osThread(measurementTask), NULL);

  /* definition and creation of fuseTask */
  osThreadDef(fuseTask, StartFuseTask, osPriorityNormal, 0, 128);
  fuseTaskHandle = osThreadCreate(osThread(fuseTask), NULL);

  /* definition and creation of testTask */
  osThreadDef(testTask, StartTestTask, osPriorityIdle, 0, 128);
  testTaskHandle = osThreadCreate(osThread(testTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartMeasurementTask */
/**
  * @brief  Function implementing the measurementTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartMeasurementTask */
void StartMeasurementTask(void const * argument)
{

  /* USER CODE BEGIN StartMeasurementTask */
  uint8_t i = 0;
  uint8_t currentsMapped[CHANNEL_COUNT] = {0};
  uint8_t batVoltageMapped = 0;
  uint8_t temperatureMapped = 0;
  uint8_t currentFrameA_data[CURRENT_FRAME_A_SIZE] = {0};
  uint8_t currentFrameB_data[CURRENT_FRAME_B_SIZE] = {0};
  uint8_t statusFrame_data[STATUS_FRAME_SIZE] = {0};
  TickType_t xLastWakeTime = xTaskGetTickCount();

  /* Initialize ADC driver */
  if (AdcDrv_init()) {
	  Error_Handler();
  }

  /* Initialize CAN driver */
  if (CanDrv_init()) {
	  Error_Handler();
  }

  /* Initialize application */
  if (App_init()) {
	  Error_Handler();
  }

  /* Infinite loop */
  for(;;) {
	  /* Wait a given interval */
	  osDelayUntil(&xLastWakeTime, CURRENT_FRAME_INTERVAL_MS);

	  /* Read ADC channels */
	  for (uint8_t channel=0; channel<CHANNEL_COUNT; channel++) {
		  currentsMapped[channel] = AdcDrv_readCurrentMapped(channel);
	  }
	  batVoltageMapped = AdcDrv_readBatVoltageMapped();
	  temperatureMapped = AdcDrv_readTemperatureMapped();

	  /* Prepare CAN frames */
	  for (i=0; i<CURRENT_FRAME_A_SIZE; i++) {
		  currentFrameA_data[i] = currentsMapped[i];
	  }
	  for (i=0; i<CURRENT_FRAME_B_SIZE; i++) {
		  currentFrameB_data[i] = currentsMapped[i+8];
	  }
	  statusFrame_data[0] = batVoltageMapped;
	  statusFrame_data[1] = temperatureMapped;

	  /* Send CAN frames */
	  CanDrv_sendCurrentFrameA(currentFrameA_data);
	  CanDrv_sendCurrentFrameB(currentFrameB_data);
	  CanDrv_sendStatusFrame(statusFrame_data);
  }
  /* USER CODE END StartMeasurementTask */
}

/* USER CODE BEGIN Header_StartFuseTask */
/**
* @brief Function implementing the fuseTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFuseTask */
void StartFuseTask(void const * argument)
{
  /* USER CODE BEGIN StartFuseTask */
  float current;

  /* Infinite loop */
  for(;;)
  {
	/* Wait a given interval */
	osDelay(FUSE_TASK_INTERVAL_MS);

	/* Read ADC channels */
	for (uint8_t ch=0; ch<CHANNEL_COUNT; ch++) {
		current = AdcDrv_readCurrent(ch);

		/* Check if fuse current is exceeded */
		if (current > App_channels[ch].fuse_current) {
			/* Turn off channel */
			App_turnOffChannel(ch);

			/* Send fuse message to CAN bus */
			CanDrv_sendFuseMessage(ch);
		}
	}
  }
  /* USER CODE END StartFuseTask */
}

/* USER CODE BEGIN Header_StartTestTask */
/**
* @brief Function implementing the testTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTestTask */
void StartTestTask(void const * argument)
{
  /* USER CODE BEGIN StartTestTask */
  /* Infinite loop */
  for(;;)
  {
#ifdef TEST_MODE
	  /* Write test function here */
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4, GPIO_PIN_RESET);
	  *(App_channels[CH1].adc_value_ptr) = 0;
	  *(App_channels[CH2].adc_value_ptr) = 0;
	  *(App_channels[CH3].adc_value_ptr) = 0;
	  *(App_channels[CH4].adc_value_ptr) = 0;
	  *(App_channels[CH5].adc_value_ptr) = 0;
	  osDelay(3000);
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4, GPIO_PIN_SET);
	  *(App_channels[CH1].adc_value_ptr) = 4000;
	  *(App_channels[CH2].adc_value_ptr) = 4000;
	  *(App_channels[CH3].adc_value_ptr) = 4000;
	  *(App_channels[CH4].adc_value_ptr) = 4000;
	  *(App_channels[CH5].adc_value_ptr) = 4000;
	  osDelay(100);
#else
	  /* Terminate this task */
	  osThreadTerminate(NULL);
#endif
  }
  /* USER CODE END StartTestTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
