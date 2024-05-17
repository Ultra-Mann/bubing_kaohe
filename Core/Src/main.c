/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "gimbal_task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

const fp32 PID_Speed_3508[3]={10000,10,10};  //3508速度环PID参数
const fp32 PID_Speed[3]={40,0.2,10};  //速度环PID参数
const fp32 PID_Angle[3]={150,0,3};
const fp32 PID_Speed_yaw[3] = {20,0,10};
const fp32 PID_Angle_yaw[3] = {500,1,80};
float seth = 175;
float setb = 10;
int a = 0;
int temp = 0;

//const RC_ctrl_t *local_rc_ctrl;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float Angle_Change(int a)
{
	return (float)((a/8191.0)*2*180);
}
//**//
float RC_change(int a)
{
	if(a>=0)
	{
		a = (float)((a/660.0)*180+180);
		if(a == 360)
		{
			a = 355;
		}
		return a;
	}
	else
	{
		a = -(float)((-a/660.0)*180-180);
		if(a == 0)
		{
			a = 5;
		}
		return a;
	}
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
fp32 Angle_Current_pitch;    //6020电机转换角度(返回ecd范围为0-8191  需转换为0-360 1度为o=fp32(8191/360))
fp32 Angle_Current_yaw; 
int speed;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();            //滤波器初始化
  //**//
  remote_control_init();
  //**//
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET);
  //**//
  HAL_TIM_Base_Start(&htim9);
  HAL_TIM_Base_Start_IT(&htim9);
  //**//

//  motor_data_Angle_yaw = get_yaw_gimbal_motor_measure_point(); //获取电机数据指针编号
//  motor_data_Speed[0] = get_chassis_motor_measure_point(0);
  //**//
	HAL_Delay(10);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(Angle_Current>42&Angle_Current<48||Angle_Current>88&Angle_Current<92||Angle_Current>133&Angle_Current<138||Angle_Current>178&Angle_Current<182||Angle_Current>223&Angle_Current<227||Angle_Current>268&Angle_Current<272||Angle_Current>313&Angle_Current<317||Angle_Current>358&Angle_Current<361)
//	  {
//		  seth = seth+45;
//		  if(Angle_Current>360)
//		  {
//			  seth = seth - 360;
//		  }
//		  
//		  HAL_Delay(1000);
//	  }
//		seth = seth + 0.001*(local_rc_ctrl->rc.ch[0]+25);
//	    if(seth >= 355||seth <= 5)
//		{
//			if(seth >= 355)
//			{
//				seth = 355;
//			}
//			else if(seth <= 5)
//			{
//				seth = 5;
//			}
//		}
//		HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
//		Angle_Current=Angle_Change(motor_data_Angle_yaw->ecd);
//		PID_calc(&pid_yaw_angle,Angle_Current,seth); //外环
//		PID_calc(&pid_yaw_speed,motor_data_Angle_yaw->speed_rpm,pid_yaw_angle.out); //内环
//		
//		CAN_cmd_gimbal(pid_yaw_speed.out,0,0,0);
		
	  
	  //PID_calc(&pid_s,motor_data_Speed->speed_rpm,1000); //内环
	  //CAN_cmd_chassis(pid_s.out,0,0,0);
	  //CAN_cmd_gimbal(0,0,0,0);
//		if(Angle_Current == 90)
//		{
//			seth = 0;
//		}
		HAL_Delay(2);
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{ 
    if(htim == &htim9) 
    { 
       
    }
}

void green_led_task(void const * argument)
{
	while(1)
	{
	    if(seth >= 355||seth <= 5)
		{
			if(seth >= 210)
			{
				seth = 210;
			}
			else if(seth <= 175)
			{
				seth = 175;
			}
		}

//		Angle_Current=Angle_Change(motor_data_Angle_yaw->ecd);
//		PID_calc(&pid_yaw_angle,Angle_Current,seth); //外环
//		PID_calc(&pid_yaw_speed,motor_data_Angle_yaw->speed_rpm,pid_yaw_angle.out); //内环
//		
//		CAN_cmd_gimbal(pid_yaw_speed.out,0,0,0);
	}
}

void red_led_task(void const * argument)
{
  while(1)
  {
//		HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET);
//		HAL_Delay(500);
  }
}

void bule_led_task(void const * argument)
{
	while(1)
	{
//		HAL_GPIO_WritePin(BULE_GPIO_Port,BULE_Pin,GPIO_PIN_RESET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(BULE_GPIO_Port,BULE_Pin,GPIO_PIN_SET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(BULE_GPIO_Port,BULE_Pin,GPIO_PIN_RESET);
//		HAL_Delay(500);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
