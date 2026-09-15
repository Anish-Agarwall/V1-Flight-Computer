/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TELEM_CS_Pin GPIO_PIN_0
#define TELEM_CS_GPIO_Port GPIOC
#define TELEM_BUSY_Pin GPIO_PIN_1
#define TELEM_BUSY_GPIO_Port GPIOC
#define TXEN_Pin GPIO_PIN_2
#define TXEN_GPIO_Port GPIOC
#define RXEN_Pin GPIO_PIN_3
#define RXEN_GPIO_Port GPIOC
#define Voltage_ADC_Pin GPIO_PIN_0
#define Voltage_ADC_GPIO_Port GPIOA
#define MAIN_DETECT_Pin GPIO_PIN_1
#define MAIN_DETECT_GPIO_Port GPIOA
#define DROGUE_DETECT_Pin GPIO_PIN_2
#define DROGUE_DETECT_GPIO_Port GPIOA
#define Heartbeat_LED_Pin GPIO_PIN_4
#define Heartbeat_LED_GPIO_Port GPIOA
#define SPI_CLK_Pin GPIO_PIN_5
#define SPI_CLK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define TELEM_CLK_Pin GPIO_PIN_4
#define TELEM_CLK_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_12
#define FLASH_CS_GPIO_Port GPIOB
#define SPI2_CLK_Pin GPIO_PIN_13
#define SPI2_CLK_GPIO_Port GPIOB
#define MISO2_Pin GPIO_PIN_14
#define MISO2_GPIO_Port GPIOB
#define MOSI2_Pin GPIO_PIN_15
#define MOSI2_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_9
#define SD_CS_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_9
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOA
#define USB_D__Pin GPIO_PIN_11
#define USB_D__GPIO_Port GPIOA
#define USB_D_A12_Pin GPIO_PIN_12
#define USB_D_A12_GPIO_Port GPIOA
#define SPI3_CLK_Pin GPIO_PIN_10
#define SPI3_CLK_GPIO_Port GPIOC
#define MISO3_Pin GPIO_PIN_11
#define MISO3_GPIO_Port GPIOC
#define MOSI3_Pin GPIO_PIN_12
#define MOSI3_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
