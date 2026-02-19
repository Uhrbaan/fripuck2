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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PULSE_2_Pin GPIO_PIN_2
#define PULSE_2_GPIO_Port GPIOE
#define PULSE_3_Pin GPIO_PIN_3
#define PULSE_3_GPIO_Port GPIOE
#define CAM_D4_Pin GPIO_PIN_4
#define CAM_D4_GPIO_Port GPIOE
#define CAM_D6_Pin GPIO_PIN_5
#define CAM_D6_GPIO_Port GPIOE
#define CAM_D7_Pin GPIO_PIN_6
#define CAM_D7_GPIO_Port GPIOE
#define SEL_0_Pin GPIO_PIN_13
#define SEL_0_GPIO_Port GPIOC
#define SEL_1_Pin GPIO_PIN_14
#define SEL_1_GPIO_Port GPIOC
#define SEL_2_Pin GPIO_PIN_15
#define SEL_2_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define IR6_AN_Pin GPIO_PIN_0
#define IR6_AN_GPIO_Port GPIOC
#define IR7_AN_Pin GPIO_PIN_1
#define IR7_AN_GPIO_Port GPIOC
#define IR0_AN_Pin GPIO_PIN_2
#define IR0_AN_GPIO_Port GPIOC
#define IR1_AN_Pin GPIO_PIN_3
#define IR1_AN_GPIO_Port GPIOC
#define CAM_MCLK_Pin GPIO_PIN_0
#define CAM_MCLK_GPIO_Port GPIOA
#define BATT_AN_Pin GPIO_PIN_1
#define BATT_AN_GPIO_Port GPIOA
#define c_Pin GPIO_PIN_2
#define c_GPIO_Port GPIOA
#define MIC_TIMER_OUT_Pin GPIO_PIN_3
#define MIC_TIMER_OUT_GPIO_Port GPIOA
#define CAM_HSYNC_Pin GPIO_PIN_4
#define CAM_HSYNC_GPIO_Port GPIOA
#define AUDIO_SPEAKER_Pin GPIO_PIN_5
#define AUDIO_SPEAKER_GPIO_Port GPIOA
#define CAM_PCLK_Pin GPIO_PIN_6
#define CAM_PCLK_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define IR2_AN_Pin GPIO_PIN_4
#define IR2_AN_GPIO_Port GPIOC
#define IR3_AN_Pin GPIO_PIN_5
#define IR3_AN_GPIO_Port GPIOC
#define IR5_AN_Pin GPIO_PIN_0
#define IR5_AN_GPIO_Port GPIOB
#define IR4_AN_Pin GPIO_PIN_1
#define IR4_AN_GPIO_Port GPIOB
#define LED_BODY_Pin GPIO_PIN_2
#define LED_BODY_GPIO_Port GPIOB
#define SD_DETECT_Pin GPIO_PIN_7
#define SD_DETECT_GPIO_Port GPIOE
#define MOT_L_IN2_Pin GPIO_PIN_8
#define MOT_L_IN2_GPIO_Port GPIOE
#define MOT_L_IN1_Pin GPIO_PIN_9
#define MOT_L_IN1_GPIO_Port GPIOE
#define MOT_L_IN4_Pin GPIO_PIN_10
#define MOT_L_IN4_GPIO_Port GPIOE
#define MOT_L_IN3_Pin GPIO_PIN_11
#define MOT_L_IN3_GPIO_Port GPIOE
#define MOT_R_IN2_Pin GPIO_PIN_12
#define MOT_R_IN2_GPIO_Port GPIOE
#define MOT_R_IN1_Pin GPIO_PIN_13
#define MOT_R_IN1_GPIO_Port GPIOE
#define MOT_R_IN3_Pin GPIO_PIN_14
#define MOT_R_IN3_GPIO_Port GPIOE
#define MOT_R_IN4_Pin GPIO_PIN_15
#define MOT_R_IN4_GPIO_Port GPIOE
#define SPI1_CS_ENC_L_Pin GPIO_PIN_10
#define SPI1_CS_ENC_L_GPIO_Port GPIOB
#define ENC_L_INT_Pin GPIO_PIN_11
#define ENC_L_INT_GPIO_Port GPIOB
#define PULSE_0_Pin GPIO_PIN_12
#define PULSE_0_GPIO_Port GPIOB
#define MIC_SPI2_SCK_Pin GPIO_PIN_13
#define MIC_SPI2_SCK_GPIO_Port GPIOB
#define PULSE_1_Pin GPIO_PIN_14
#define PULSE_1_GPIO_Port GPIOB
#define MIC12_DATA_Pin GPIO_PIN_15
#define MIC12_DATA_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_8
#define UART_TX_GPIO_Port GPIOD
#define UART_RX_Pin GPIO_PIN_9
#define UART_RX_GPIO_Port GPIOD
#define LED5_Pin GPIO_PIN_10
#define LED5_GPIO_Port GPIOD
#define LED7_Pin GPIO_PIN_11
#define LED7_GPIO_Port GPIOD
#define SPI1_CS_ENC_R_Pin GPIO_PIN_12
#define SPI1_CS_ENC_R_GPIO_Port GPIOD
#define ENC_R_INT_Pin GPIO_PIN_13
#define ENC_R_INT_GPIO_Port GPIOD
#define LED_FRONT_Pin GPIO_PIN_14
#define LED_FRONT_GPIO_Port GPIOD
#define IMU_INT_Pin GPIO_PIN_15
#define IMU_INT_GPIO_Port GPIOD
#define CAM_D0_Pin GPIO_PIN_6
#define CAM_D0_GPIO_Port GPIOC
#define CAM_D1_Pin GPIO_PIN_7
#define CAM_D1_GPIO_Port GPIOC
#define SDIO_D0_Pin GPIO_PIN_8
#define SDIO_D0_GPIO_Port GPIOC
#define SDIO_D1_Pin GPIO_PIN_9
#define SDIO_D1_GPIO_Port GPIOC
#define GEN_IO_1_Pin GPIO_PIN_8
#define GEN_IO_1_GPIO_Port GPIOA
#define USB_PRESENT_Pin GPIO_PIN_9
#define USB_PRESENT_GPIO_Port GPIOA
#define DIST_INT_Pin GPIO_PIN_10
#define DIST_INT_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI1_CS_ESP32_Pin GPIO_PIN_15
#define SPI1_CS_ESP32_GPIO_Port GPIOA
#define MIC_SPI3_SCK_Pin GPIO_PIN_10
#define MIC_SPI3_SCK_GPIO_Port GPIOC
#define SDIO_D3_Pin GPIO_PIN_11
#define SDIO_D3_GPIO_Port GPIOC
#define SDIO_CLK_Pin GPIO_PIN_12
#define SDIO_CLK_GPIO_Port GPIOC
#define CAN_RX_Pin GPIO_PIN_0
#define CAN_RX_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_1
#define CAN_TX_GPIO_Port GPIOD
#define SDIO_CMD_Pin GPIO_PIN_2
#define SDIO_CMD_GPIO_Port GPIOD
#define REMOTE_Pin GPIO_PIN_3
#define REMOTE_GPIO_Port GPIOD
#define SEL_3_Pin GPIO_PIN_4
#define SEL_3_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOD
#define AUDIO_PWR_Pin GPIO_PIN_7
#define AUDIO_PWR_GPIO_Port GPIOD
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define MIC34_DATA_Pin GPIO_PIN_5
#define MIC34_DATA_GPIO_Port GPIOB
#define CAM_D5_Pin GPIO_PIN_6
#define CAM_D5_GPIO_Port GPIOB
#define CAM_VSYNC_Pin GPIO_PIN_7
#define CAM_VSYNC_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
#define CAM_D2_Pin GPIO_PIN_0
#define CAM_D2_GPIO_Port GPIOE
#define CAM_D3_Pin GPIO_PIN_1
#define CAM_D3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
