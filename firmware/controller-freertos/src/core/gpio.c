/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PE4   ------> DCMI_D4
     PE5   ------> DCMI_D6
     PE6   ------> DCMI_D7
     PA0-WKUP   ------> S_TIM5_CH1
     PA2   ------> S_TIM9_CH1
     PA3   ------> S_TIM5_CH4
     PA4   ------> DCMI_HSYNC
     PA6   ------> DCMI_PIXCLK
     PB0   ------> SharedAnalog_PB0
     PB13   ------> SPI2_SCK
     PB15   ------> I2S2_SD
     PC6   ------> DCMI_D0
     PC7   ------> DCMI_D1
     PC8   ------> SDIO_D0
     PC9   ------> SDIO_D1
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> SPI3_SCK
     PC11   ------> SDIO_D3
     PC12   ------> SDIO_CK
     PD2   ------> SDIO_CMD
     PB5   ------> I2S3_SD
     PB6   ------> DCMI_D5
     PB7   ------> DCMI_VSYNC
     PE0   ------> DCMI_D2
     PE1   ------> DCMI_D3
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PULSE_2_Pin|PULSE_3_Pin|MOT_L_IN2_Pin|MOT_L_IN1_Pin
                          |MOT_L_IN4_Pin|MOT_L_IN3_Pin|MOT_R_IN2_Pin|MOT_R_IN1_Pin
                          |MOT_R_IN3_Pin|MOT_R_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_BODY_Pin|PULSE_0_Pin|PULSE_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_ENC_L_GPIO_Port, SPI1_CS_ENC_L_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED5_Pin|LED7_Pin|SPI1_CS_ENC_R_Pin|REMOTE_Pin
                          |LED1_Pin|LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_FRONT_Pin|AUDIO_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_ESP32_GPIO_Port, SPI1_CS_ESP32_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PULSE_2_Pin PULSE_3_Pin MOT_L_IN2_Pin MOT_L_IN1_Pin
                           MOT_L_IN4_Pin MOT_L_IN3_Pin MOT_R_IN2_Pin MOT_R_IN1_Pin
                           MOT_R_IN3_Pin */
  GPIO_InitStruct.Pin = PULSE_2_Pin|PULSE_3_Pin|MOT_L_IN2_Pin|MOT_L_IN1_Pin
                          |MOT_L_IN4_Pin|MOT_L_IN3_Pin|MOT_R_IN2_Pin|MOT_R_IN1_Pin
                          |MOT_R_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_D4_Pin CAM_D6_Pin CAM_D7_Pin CAM_D2_Pin
                           CAM_D3_Pin */
  GPIO_InitStruct.Pin = CAM_D4_Pin|CAM_D6_Pin|CAM_D7_Pin|CAM_D2_Pin
                          |CAM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL_0_Pin SEL_1_Pin SEL_2_Pin */
  GPIO_InitStruct.Pin = SEL_0_Pin|SEL_1_Pin|SEL_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM_MCLK_Pin */
  GPIO_InitStruct.Pin = CAM_MCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(CAM_MCLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BATT_AN_Pin AUDIO_SPEAKER_Pin */
  GPIO_InitStruct.Pin = BATT_AN_Pin|AUDIO_SPEAKER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : c_Pin */
  GPIO_InitStruct.Pin = c_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(c_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_TIMER_OUT_Pin */
  GPIO_InitStruct.Pin = MIC_TIMER_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(MIC_TIMER_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_HSYNC_Pin CAM_PCLK_Pin */
  GPIO_InitStruct.Pin = CAM_HSYNC_Pin|CAM_PCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PROX5_Pin */
  GPIO_InitStruct.Pin = PROX5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PROX5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_BODY_Pin SPI1_CS_ENC_L_Pin PULSE_0_Pin PULSE_1_Pin */
  GPIO_InitStruct.Pin = LED_BODY_Pin|SPI1_CS_ENC_L_Pin|PULSE_0_Pin|PULSE_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOT_R_IN4_Pin */
  GPIO_InitStruct.Pin = MOT_R_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOT_R_IN4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_L_INT_Pin */
  GPIO_InitStruct.Pin = ENC_L_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_L_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = MIC_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(MIC_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC12_DATA_Pin */
  GPIO_InitStruct.Pin = MIC12_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(MIC12_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED5_Pin LED7_Pin SPI1_CS_ENC_R_Pin LED_FRONT_Pin
                           LED1_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|LED7_Pin|SPI1_CS_ENC_R_Pin|LED_FRONT_Pin
                          |LED1_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_R_INT_Pin */
  GPIO_InitStruct.Pin = ENC_R_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_R_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_D0_Pin CAM_D1_Pin */
  GPIO_InitStruct.Pin = CAM_D0_Pin|CAM_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SDIO_D0_Pin SDIO_D1_Pin SDIO_D3_Pin SDIO_CLK_Pin */
  GPIO_InitStruct.Pin = SDIO_D0_Pin|SDIO_D1_Pin|SDIO_D3_Pin|SDIO_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GEN_IO_1_Pin */
  GPIO_InitStruct.Pin = GEN_IO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GEN_IO_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_PRESENT_Pin DIST_INT_Pin */
  GPIO_InitStruct.Pin = USB_PRESENT_Pin|DIST_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_ESP32_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_ESP32_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_ESP32_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_SPI3_SCK_Pin */
  GPIO_InitStruct.Pin = MIC_SPI3_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(MIC_SPI3_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CMD_Pin */
  GPIO_InitStruct.Pin = SDIO_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(SDIO_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : REMOTE_Pin AUDIO_PWR_Pin */
  GPIO_InitStruct.Pin = REMOTE_Pin|AUDIO_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SEL_3_Pin */
  GPIO_InitStruct.Pin = SEL_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SEL_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC34_DATA_Pin */
  GPIO_InitStruct.Pin = MIC34_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(MIC34_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM_D5_Pin */
  GPIO_InitStruct.Pin = CAM_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(CAM_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM_VSYNC_Pin */
  GPIO_InitStruct.Pin = CAM_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(CAM_VSYNC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
