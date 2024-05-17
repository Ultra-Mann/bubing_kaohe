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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_receive.h"
#include "gpio.h"
#include "can.h"
#include "bsp_can.h"
#include "zhiru.h"
#include "struct_typedef.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "bsp_rc.h"
#include "PID.h"

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
#define RED_Pin GPIO_PIN_12
#define RED_GPIO_Port GPIOH
#define GREEN_Pin GPIO_PIN_11
#define GREEN_GPIO_Port GPIOH
#define BULE_Pin GPIO_PIN_10
#define BULE_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */


extern  const fp32 PID_Speed_3508[3];  //3508�ٶȻ�PID����
extern  const fp32 PID_Speed[3];  //�ٶȻ�PID����
extern  const fp32 PID_Angle[3];
extern const fp32 PID_Speed_yaw[3];
extern const fp32 PID_Angle_yaw[3];
extern float seth;
extern float setb;
extern int a;
extern fp32 Angle_Current_pitch;
extern fp32 Angle_Current_yaw;
float Angle_Change(int a);
//extern fp32 Angle_Current;    //6020���ת���Ƕ�(����ecd��ΧΪ0-8191  ��ת��Ϊ0-360 1��Ϊo=fp32(8191/360))
//extern const motor_measure_t *motor_data_Speed_yaw;
//extern const motor_measure_t *motor_data_Angle_yaw;
//extern const motor_measure_t *motor_data_Speed_pich;
//extern const motor_measure_t *motor_data_Angle_pich;
//extern const motor_measure_t *motor_data_Speed[4];
extern int speed;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
