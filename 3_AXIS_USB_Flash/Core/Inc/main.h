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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SSM3ST4840C.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern ADC_HandleTypeDef hadc1;

#define AXIS_1_SPI &hspi3  // Axis 1 uses SPI3 (hspi3)
#define AXIS_2_SPI &hspi1  // Axis 2 uses SPI1 (hspi1)
#define AXIS_3_SPI &hspi2  // Axis 3 uses SPI2 (hspi2)

extern const int ENC_ENB_A1;  // 0 = disable Encoder , 1 = enable Encoder
extern const int ENC_ENB_A2;  // 0 = disable Encoder , 1 = enable Encoder
extern const int ENC_ENB_A3;  // 0 = disable Encoder , 1 = enable Encoder
extern const int ENC_HOME_A1; // 0 = don't reset ENC counter at Z-event, 1 = reset at Z-event (for homing)
extern const int ENC_HOME_A2; // 0 = don't reset ENC counter at Z-event, 1 = reset at Z-event (for homing)
extern const int ENC_HOME_A3; // 0 = don't reset ENC counter at Z-event, 1 = reset at Z-event (for homing)
extern const int STG_ENB_A1;  // 0 = disable Stallguard, 1 = enable Stallguard
extern const int STG_ENB_A2;  // 0 = disable Stallguard, 1 = enable Stallguard
extern const int STG_ENB_A3;  // 0 = disable Stallguard, 1 = enable Stallguard
extern const int32_t ENC_Resolution_A1; // Encoder resolution Axis 1
extern const int32_t ENC_Resolution_A2; // Encoder resolution Axis 2
extern const int32_t ENC_Resolution_A3; // Encoder resolution Axis 3
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define SYS_MEM_BOOT_ADDR 0x1FFF0000UL  // start of system memory (ROM) bootloader
typedef void (*boot_func_t)(void);
void EnterBootloader(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EXT_IN_1_Pin GPIO_PIN_13
#define EXT_IN_1_GPIO_Port GPIOC
#define EXT_IN_2_Pin GPIO_PIN_14
#define EXT_IN_2_GPIO_Port GPIOC
#define RST_Pin GPIO_PIN_2
#define RST_GPIO_Port GPIOF
#define RS485_Receive_Pin GPIO_PIN_1
#define RS485_Receive_GPIO_Port GPIOA
#define RS485_Transmit_Pin GPIO_PIN_4
#define RS485_Transmit_GPIO_Port GPIOA
#define A2_TMC_SCK_Pin GPIO_PIN_5
#define A2_TMC_SCK_GPIO_Port GPIOA
#define A2_TMC_SDO_Pin GPIO_PIN_6
#define A2_TMC_SDO_GPIO_Port GPIOA
#define A2_TMC_SDI_Pin GPIO_PIN_7
#define A2_TMC_SDI_GPIO_Port GPIOA
#define A2_TMC_CSN_Pin GPIO_PIN_0
#define A2_TMC_CSN_GPIO_Port GPIOB
#define A2_DRV_ENN_Pin GPIO_PIN_1
#define A2_DRV_ENN_GPIO_Port GPIOB
#define EXT_IN_3_Pin GPIO_PIN_2
#define EXT_IN_3_GPIO_Port GPIOB
#define EXT_IN_4_Pin GPIO_PIN_10
#define EXT_IN_4_GPIO_Port GPIOB
#define EXT_IN_6_Pin GPIO_PIN_11
#define EXT_IN_6_GPIO_Port GPIOB
#define EXT_IN_5_Pin GPIO_PIN_12
#define EXT_IN_5_GPIO_Port GPIOB
#define A3_TMC_SCK_Pin GPIO_PIN_13
#define A3_TMC_SCK_GPIO_Port GPIOB
#define A3_TMC_SDO_Pin GPIO_PIN_14
#define A3_TMC_SDO_GPIO_Port GPIOB
#define A3_TMC_SDI_Pin GPIO_PIN_15
#define A3_TMC_SDI_GPIO_Port GPIOB
#define A3_TMC_CSN_Pin GPIO_PIN_8
#define A3_TMC_CSN_GPIO_Port GPIOA
#define A3_DRV_ENN_Pin GPIO_PIN_9
#define A3_DRV_ENN_GPIO_Port GPIOA
#define CAN_ENB_Pin GPIO_PIN_15
#define CAN_ENB_GPIO_Port GPIOA
#define A1_DRV_ENN_Pin GPIO_PIN_2
#define A1_DRV_ENN_GPIO_Port GPIOD
#define A1_TMC_CSN_Pin GPIO_PIN_3
#define A1_TMC_CSN_GPIO_Port GPIOD
#define A1_TMC_SCK_Pin GPIO_PIN_3
#define A1_TMC_SCK_GPIO_Port GPIOB
#define A1_TMC_SDO_Pin GPIO_PIN_4
#define A1_TMC_SDO_GPIO_Port GPIOB
#define A1_TMC_SDI_Pin GPIO_PIN_5
#define A1_TMC_SDI_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOB
#define EXT_OUT_1_Pin GPIO_PIN_7
#define EXT_OUT_1_GPIO_Port GPIOB
#define EXT_OUT_2_Pin GPIO_PIN_8
#define EXT_OUT_2_GPIO_Port GPIOB
#define EXT_OUT_3_Pin GPIO_PIN_9
#define EXT_OUT_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
