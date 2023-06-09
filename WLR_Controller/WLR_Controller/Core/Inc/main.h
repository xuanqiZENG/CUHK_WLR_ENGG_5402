/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include "bsp_delay.h"
extern CAN_TxHeaderTypeDef   TxHeader_1;
extern CAN_RxHeaderTypeDef   RxHeader_1;
extern uint8_t               TxData_1[8];
extern uint8_t               RxData_1[8];

extern CAN_TxHeaderTypeDef   TxHeader_2;
extern CAN_RxHeaderTypeDef   RxHeader_2;
extern uint8_t               TxData_2[8];
extern uint8_t               RxData_2[8];

void data_process(void);
void data_process_2(void);


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void Encoder_Check(void);
void v_cal(void);
void robot_state_init();
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint8_t Motor_Set_FeedBack(unsigned char FeedBack,uint8_t ID);
void IMU_check(void);
uint8_t ID_Set(uint8_t ID);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanHandle);
void uint8touint32(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32);
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);
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
uint32_t data_checksum(uint32_t* data_to_check, uint32_t check_length);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
