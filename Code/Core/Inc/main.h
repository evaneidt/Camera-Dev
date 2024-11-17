/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define QSPI_IO2_Pin GPIO_PIN_2
#define QSPI_IO2_GPIO_Port GPIOE
#define YOUT4_Pin GPIO_PIN_4
#define YOUT4_GPIO_Port GPIOE
#define YOUT6_Pin GPIO_PIN_5
#define YOUT6_GPIO_Port GPIOE
#define YOUT7_Pin GPIO_PIN_6
#define YOUT7_GPIO_Port GPIOE
#define LSE_OSC_Pin GPIO_PIN_14
#define LSE_OSC_GPIO_Port GPIOC
#define HSE_OSC_Pin GPIO_PIN_0
#define HSE_OSC_GPIO_Port GPIOH
#define DIP_1_Pin GPIO_PIN_0
#define DIP_1_GPIO_Port GPIOC
#define DIP_3_Pin GPIO_PIN_2
#define DIP_3_GPIO_Port GPIOC
#define DIP_4_Pin GPIO_PIN_3
#define DIP_4_GPIO_Port GPIOC
#define I2S_WS_Pin GPIO_PIN_0
#define I2S_WS_GPIO_Port GPIOA
#define QSPI_IO3_Pin GPIO_PIN_1
#define QSPI_IO3_GPIO_Port GPIOA
#define DAC_nCS_Pin GPIO_PIN_4
#define DAC_nCS_GPIO_Port GPIOA
#define I2S_SCK_Pin GPIO_PIN_5
#define I2S_SCK_GPIO_Port GPIOA
#define CAM_CLK_Pin GPIO_PIN_6
#define CAM_CLK_GPIO_Port GPIOA
#define RAM_CLK_Pin GPIO_PIN_7
#define RAM_CLK_GPIO_Port GPIOA
#define DIP_5_Pin GPIO_PIN_4
#define DIP_5_GPIO_Port GPIOC
#define DIP_6_Pin GPIO_PIN_5
#define DIP_6_GPIO_Port GPIOC
#define QSPI_IO1_Pin GPIO_PIN_0
#define QSPI_IO1_GPIO_Port GPIOB
#define QSPI_IO0_Pin GPIO_PIN_1
#define QSPI_IO0_GPIO_Port GPIOB
#define QSPI_CLK_Pin GPIO_PIN_2
#define QSPI_CLK_GPIO_Port GPIOB
#define HDMI_CLK_Pin GPIO_PIN_9
#define HDMI_CLK_GPIO_Port GPIOE
#define EEPROM_nWC_Pin GPIO_PIN_10
#define EEPROM_nWC_GPIO_Port GPIOE
#define QSPI_nCS_Pin GPIO_PIN_11
#define QSPI_nCS_GPIO_Port GPIOE
#define DIP_7_Pin GPIO_PIN_12
#define DIP_7_GPIO_Port GPIOE
#define DIP_8_Pin GPIO_PIN_13
#define DIP_8_GPIO_Port GPIOE
#define DIP_9_Pin GPIO_PIN_14
#define DIP_9_GPIO_Port GPIOE
#define DIP_10_Pin GPIO_PIN_15
#define DIP_10_GPIO_Port GPIOE
#define I2S_SCL_Pin GPIO_PIN_10
#define I2S_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_11
#define I2C_SDA_GPIO_Port GPIOB
#define I2S_MCLK_CTRL_Pin GPIO_PIN_12
#define I2S_MCLK_CTRL_GPIO_Port GPIOB
#define YOUT2_Pin GPIO_PIN_13
#define YOUT2_GPIO_Port GPIOB
#define HDMI_INT_Pin GPIO_PIN_14
#define HDMI_INT_GPIO_Port GPIOB
#define nREC_Pin GPIO_PIN_15
#define nREC_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_8
#define LED_R_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_9
#define LED_G_GPIO_Port GPIOD
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOD
#define ADDR_nOE_Pin GPIO_PIN_11
#define ADDR_nOE_GPIO_Port GPIOD
#define DATA_nOE_Pin GPIO_PIN_12
#define DATA_nOE_GPIO_Port GPIOD
#define DATA_CLK_Pin GPIO_PIN_13
#define DATA_CLK_GPIO_Port GPIOD
#define DATA_IN_Pin GPIO_PIN_14
#define DATA_IN_GPIO_Port GPIOD
#define ADDR_CLK_Pin GPIO_PIN_15
#define ADDR_CLK_GPIO_Port GPIOD
#define YOUT0_Pin GPIO_PIN_6
#define YOUT0_GPIO_Port GPIOC
#define ADDR_IN_Pin GPIO_PIN_8
#define ADDR_IN_GPIO_Port GPIOC
#define YOUT3_Pin GPIO_PIN_9
#define YOUT3_GPIO_Port GPIOC
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define YOUT1_Pin GPIO_PIN_10
#define YOUT1_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define HDMI_CE_Pin GPIO_PIN_15
#define HDMI_CE_GPIO_Port GPIOA
#define DAC_CLK_Pin GPIO_PIN_10
#define DAC_CLK_GPIO_Port GPIOC
#define DIP_2_Pin GPIO_PIN_11
#define DIP_2_GPIO_Port GPIOC
#define DAC_MOSI_Pin GPIO_PIN_12
#define DAC_MOSI_GPIO_Port GPIOC
#define RAM_nPD_Pin GPIO_PIN_0
#define RAM_nPD_GPIO_Port GPIOD
#define CAM_PD_Pin GPIO_PIN_1
#define CAM_PD_GPIO_Port GPIOD
#define SR_nCLR_Pin GPIO_PIN_2
#define SR_nCLR_GPIO_Port GPIOD
#define YOUT5_Pin GPIO_PIN_3
#define YOUT5_GPIO_Port GPIOD
#define RAM_nDQM_Pin GPIO_PIN_4
#define RAM_nDQM_GPIO_Port GPIOD
#define RAM_nWE_Pin GPIO_PIN_5
#define RAM_nWE_GPIO_Port GPIOD
#define RAM_nCAS_Pin GPIO_PIN_6
#define RAM_nCAS_GPIO_Port GPIOD
#define RAM_nRAS_Pin GPIO_PIN_7
#define RAM_nRAS_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define I2S_SD_Pin GPIO_PIN_4
#define I2S_SD_GPIO_Port GPIOB
#define OSD_A2_Pin GPIO_PIN_5
#define OSD_A2_GPIO_Port GPIOB
#define OSD_A1_Pin GPIO_PIN_6
#define OSD_A1_GPIO_Port GPIOB
#define OSD_A0_Pin GPIO_PIN_7
#define OSD_A0_GPIO_Port GPIOB
#define OSD_EN_Pin GPIO_PIN_8
#define OSD_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
