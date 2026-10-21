/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "gpio.h"
/* USER CODE BEGIN Includes */
#include "CO_driver.h"
#include "CANopen.h"
#include "OD.h"
/* USER CODE END Includes */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CO_t* CO = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1); /* 启动CAN：退出初始化模式，进入正常模式 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* 暂时不跑 CANopen，改用裸 CAN 发目标位置给 F103 */
  static uint32_t master_target = 16384;  // 第一个目标：90 度
  static uint32_t last_switch = 0;

  /* 每 3 秒切换目标，切换时只发一次（避免反复发打断 F103 定位） */
  if (HAL_GetTick() - last_switch > 3000) {
      last_switch = HAL_GetTick();
      master_target = (master_target == 16384) ? 32768 : 16384;

      uint8_t pos_data[4];
      pos_data[0] = (master_target >> 24) & 0xFF;  // 大端
      pos_data[1] = (master_target >> 16) & 0xFF;
      pos_data[2] = (master_target >> 8) & 0xFF;
      pos_data[3] = master_target & 0xFF;

      CAN_TxHeaderTypeDef tx_header;
      tx_header.StdId = 0x181;            // F103 收这个 ID
      tx_header.ExtId = 0;
      tx_header.IDE = CAN_ID_STD;
      tx_header.RTR = CAN_RTR_DATA;
      tx_header.DLC = 4;
      tx_header.TransmitGlobalTime = DISABLE;

      uint32_t tx_mailbox;
      HAL_CAN_AddTxMessage(&hcan1, &tx_header, pos_data, &tx_mailbox);
  }

  HAL_Delay(50);

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  /* RCC振荡器初始化结构体 */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  /* RCC时钟初始化结构体 */

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  /* RCC振荡器类型 */
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;  /* RCC HSE 状态 */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;  /* RCC PLL 状态 */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;  /* RCC PLL 源选择 */
  RCC_OscInitStruct.PLL.PLLM = 8;  /* RCC PLL 分频因子M */
  RCC_OscInitStruct.PLL.PLLN = 336;  /* RCC PLL 分频因子N */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  /* RCC PLL 分频因子P */
  RCC_OscInitStruct.PLL.PLLQ = 4;  /* RCC PLL 分频因子Q */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)    /* 如果配置失败 */
  {
    Error_Handler();  /* 错误处理 */
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK  /* RCC时钟类型 */
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;  /* RCC时钟类型 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  /* RCC时钟源 */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  /* RCC时钟分频因子 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  /* RCC时钟分频因子 */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  /* RCC时钟分频因子 */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)    /* 如果配置失败 */
  {
    Error_Handler();  /* 错误处理 */
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if(CO != NULL && CO->CANmodule != NULL) {
    CO_CANinterrupt(CO->CANmodule); /* 处理CAN中断 */
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)  /* 错误处理函数 */
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();  /* 禁用中断 */
  while (1)  /* 无限循环 */
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT  /* 如果使用全断言 */
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)  /* 断言失败函数 */
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */  /* 如果使用全断言 */

