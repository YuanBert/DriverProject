/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CommunicationLED_Pin GPIO_PIN_13
#define CommunicationLED_GPIO_Port GPIOC
#define CTR485B_EN_Pin GPIO_PIN_0
#define CTR485B_EN_GPIO_Port GPIOC
#define CTR485A_EN_Pin GPIO_PIN_1
#define CTR485A_EN_GPIO_Port GPIOC
#define LightSensor_Pin GPIO_PIN_2
#define LightSensor_GPIO_Port GPIOC
#define BSPA_TX_Pin GPIO_PIN_2
#define BSPA_TX_GPIO_Port GPIOA
#define BSPA_RX_Pin GPIO_PIN_3
#define BSPA_RX_GPIO_Port GPIOA
#define W25Q64_NSS_Pin GPIO_PIN_4
#define W25Q64_NSS_GPIO_Port GPIOA
#define W25Q64_SCK_Pin GPIO_PIN_5
#define W25Q64_SCK_GPIO_Port GPIOA
#define W25Q64_MISO_Pin GPIO_PIN_6
#define W25Q64_MISO_GPIO_Port GPIOA
#define W25Q64_MOSI_Pin GPIO_PIN_7
#define W25Q64_MOSI_GPIO_Port GPIOA
#define GentleSensor_Pin GPIO_PIN_4
#define GentleSensor_GPIO_Port GPIOC
#define GentleSensor_EXTI_IRQn EXTI4_IRQn
#define BSPB_TX_Pin GPIO_PIN_10
#define BSPB_TX_GPIO_Port GPIOB
#define BSPB_RX_Pin GPIO_PIN_11
#define BSPB_RX_GPIO_Port GPIOB
#define MCUAtmosphereLEDR_Pin GPIO_PIN_12
#define MCUAtmosphereLEDR_GPIO_Port GPIOB
#define MCUAtmosphereLEDG_Pin GPIO_PIN_13
#define MCUAtmosphereLEDG_GPIO_Port GPIOB
#define MCU_DS18B20_Pin GPIO_PIN_6
#define MCU_DS18B20_GPIO_Port GPIOC
#define CoreBoard_TX_Pin GPIO_PIN_9
#define CoreBoard_TX_GPIO_Port GPIOA
#define CoreBoard_RX_Pin GPIO_PIN_10
#define CoreBoard_RX_GPIO_Port GPIOA

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
  /** enum: DS_StatusTypeDef
  **
  ** DESCRIPTION:
  **  --����������
  **
  ** CREATED: 2017/12/7, by bert
  **
  ** FILE: DS_Protocol.h
  **
  ** AUTHOR: Bert.Zhang
  ********************************************************************************
  */
  typedef enum
  {
    DS_OK       = 0x00U,
    DS_ERROR    = 0x01U,
    DS_BUSY     = 0x02U,
    DS_TIMEOUT  = 0x03U,
    DS_NOCMD    = 0x04U
  }DS_StatusTypeDef;

#define RXBUFFERSIZE		128
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
