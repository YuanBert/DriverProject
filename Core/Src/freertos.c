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
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
osThreadId scanBSPATaskHandle;
osThreadId scanBSPBTaskHandle;
osThreadId scanUartsTaskHandle;

xSemaphoreHandle xScanBSPABoardSemaphore;
xSemaphoreHandle xScanBSPBBoardSemaphore;
xSemaphoreHandle xScanCoreBoardSemaphore;

extern uint8_t 	CoreBoardReceiveBuff[RXBUFFERSIZE];
extern uint8_t 	CoreBoardReceiveInfo[RXBUFFERSIZE];
extern uint8_t	CoreBoardRx_BufferLen;
extern uint8_t 	CoreBoardRx_InfoLen;
extern uint8_t 	BSPABoardReceiveBuff[RXBUFFERSIZE];
extern uint8_t 	BSPABoardReceiveInfo[RXBUFFERSIZE];
extern uint8_t	BSPABoardRx_BufferLen;
extern uint8_t 	BSPABoardRx_InfoLen;
extern uint8_t 	BSPBBoardReceiveBuff[RXBUFFERSIZE];
extern uint8_t 	BSPBBoardReceiveInfo[RXBUFFERSIZE];
extern uint8_t	BSPBBoardRx_BufferLen;
extern uint8_t 	BSPBBoardRx_InfoLen;

portBASE_TYPE tReturn;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void vApplicationIdleHook(void);
void StartScanBSPATask(void const * argument);
void StartScanBSPBTask(void const * argument);
void StartScanBSPCTask(void const * argument);
void StartScanUsartsTask(void const * argument);


/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  xScanCoreBoardSemaphore = xSemaphoreCreateBinary();
  xScanBSPABoardSemaphore = xSemaphoreCreateBinary();
  xScanBSPBBoardSemaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 1, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /*
  tReturn = xTaskCreate((TaskFunction_t)StartScanBSPATask,
  				(const char *)"scanBSPATask",
  				(uint16_t    )128,
  				(void *		 )NULL,
  				(UBaseType_t )0,
  				(TaskHandle_t*)&scanBSPATaskHandle);
  if(pdPASS == tReturn)
  {
	printf(" Scan BSPA Task Create OK!\r\n");
  }
  
  tReturn = xTaskCreate((TaskFunction_t)StartScanBSPBTask,
  				(const char *)"scanBSPBTask",
  				(uint16_t    )64,
  				(void *		 )NULL,
  				(UBaseType_t )1,
  				(TaskHandle_t*)&scanBSPBTaskHandle);
   if(pdPASS == tReturn)
  {
	printf(" Scan BSPB Task Create OK! \r\n");
  }
  */
  
  tReturn = xTaskCreate((TaskFunction_t)StartScanUsartsTask,
  				(const char *)"scanUartsTask",
  				(uint16_t    )256,
  				(void *		 )NULL,
  				(UBaseType_t )1,
  				(TaskHandle_t*)&scanUartsTaskHandle);
  if(pdPASS == tReturn)
  {
	printf(" Scan Core Board Task Create OK! \r\n");
  }
  
  /*
  osThreadDef(scanBSPATask,StartScanBSPATask,osPriorityAboveNormal,0,128);
  scanBSPATaskHandle = osThreadCreate(osThread(scanBSPATask), NULL);
  
  osThreadDef(scanBSPBTask,StartScanBSPBTask,osPriorityAboveNormal,0,128);
  scanBSPBTaskHandle = osThreadCreate(osThread(scanBSPBTask), NULL);
  
  osThreadDef(scanCoreBoardTask, StartScanCoreBoardTask, osPriorityHigh,0,128);
  scanCoreBoardTaskHandle = osThreadCreate(osThread(scanCoreBoardTask), NULL);
  */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
	printf("Welcome to USE FreeRTOS \r\n");
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
void vApplicationIdleHook(void)
{
	for(;;)
	{
		//printf("Idle Hook Function\r\n");
		//
	}
}
void StartScanBSPATask(void const * argument)
{
	for(;;)
	{
		vTaskDelay(500);
		printf("ScanBSPA \r\n");
	}
}
void StartScanBSPBTask(void const * argument)
{
	for(;;)
	{
		vTaskDelay(500);
		printf("ScanBSPB \r\n");
	}
}
void StartScanUsartsTask(void const * argument)
{
	BaseType_t err = pdFALSE;
	xSemaphoreTake(xScanBSPABoardSemaphore, 0);
	xSemaphoreTake(xScanBSPBBoardSemaphore, 0);
	xSemaphoreTake(xScanCoreBoardSemaphore, 0);
	for(;;)
	{
		err = xSemaphoreTake(xScanBSPABoardSemaphore, 5);
		if(err == pdTRUE)
		{
			printf("BSPABoardRx_Len = %03d\n %s\r\n",BSPABoardRx_InfoLen,BSPABoardReceiveInfo);
			memset(BSPABoardReceiveInfo,0x00,BSPABoardRx_InfoLen);
			BSPABoardRx_InfoLen = 0;
		}
		
		err = xSemaphoreTake(xScanBSPBBoardSemaphore, 5);
		if(err == pdTRUE)
		{
			printf("BSPBBoardRx_Len = %03d\n %s\r\n",BSPBBoardRx_InfoLen,BSPBBoardReceiveInfo);
			memset(BSPBBoardReceiveInfo,0x00,BSPBBoardRx_InfoLen);
			BSPBBoardRx_InfoLen = 0;
		}
		
		err = xSemaphoreTake(xScanCoreBoardSemaphore, 5);//portMAX_DELAY);
		if(err == pdTRUE)
		{
			printf("CoreBoardRx_Len = %03d\n %s\r\n",CoreBoardRx_InfoLen,CoreBoardReceiveInfo);
			memset(CoreBoardReceiveInfo,0x00,CoreBoardRx_InfoLen);
			CoreBoardRx_InfoLen = 0;
		}

		if(err == pdFALSE)
		{
			vTaskDelay(500);
			printf("Scan Uarts Task\r\n");
		}
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
