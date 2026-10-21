/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronic`s.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo42c.h" // 包含 42C 电机控制头文件
#include "pid.h"      // 包含 PID 控制器头文件
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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int32_t target_pos = 32768;    // 目标偏移（从回零点起）：输出 180°=8192 计数（实测减速比1:1）
float speed = 0.0f;           // 当前速度累加器
int32_t encoder_pos = 0;          // 编码器位置（提到全局，方便 watch 观察）
int32_t home_full = 0;      // 回零参考位置（按按键时记录的编码器完整位置）
uint8_t new_target_flag = 0;  // 目标标志位，1=收到新目标
#define HOME_BUTTON_Pin GPIO_PIN_0
#define HOME_BUTTON_GPIO_Port GPIOA
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  CAN_FilterTypeDef sFilterConfig; // CAN 过滤器配置结构体
  sFilterConfig.FilterBank = 0; // 从 0 开始滤波器
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // 过滤器模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器规模
  sFilterConfig.FilterIdHigh = 0x181 << 5; // 高ID字节
  sFilterConfig.FilterIdLow = 0x0000; // 低ID字节
  sFilterConfig.FilterMaskIdHigh = 0xFFFF; // 高掩码ID字节
  sFilterConfig.FilterMaskIdLow = 0x0000; // 低掩码ID字节
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // FIFO 分配
  sFilterConfig.FilterActivation = ENABLE; // 过滤器激活
  sFilterConfig.SlaveStartFilterBank = 14; // 单 CAN 用不到，填 14

  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); // 配置 CAN 过滤器
  HAL_CAN_Start(&hcan); // 启动 CAN
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能接收中断
  HAL_Delay(2000);   // 等 SERVO42C 启动 2 秒
  Servo_Enable(); // 使能 42C 电机


  // ===== 手动回零 =====
  // 上电后：手动把输出轴对齐到物理 0° 标记，然后按 PA0 按键（按下=低电平）
  while (HAL_GPIO_ReadPin(HOME_BUTTON_GPIO_Port, HOME_BUTTON_Pin) == GPIO_PIN_SET)
  {
      // 等待按键按下
  }
  HAL_Delay(50);          // 按键消抖
  encoder_pos = Servo_ReadEncoder();  // 读当前位置（作为 home 参考）
  if (encoder_pos < 0) encoder_pos = Servo_ReadEncoder();  // 失败重读一次
  home_full = encoder_pos;            // 记录 home 位置（完整多圈位置）
  new_target_flag = 1;    // 触发一次定位（回到 target_pos = 输出 180°）
IWDG->KR = 0x5555; // 使能 IWDG 写入
  IWDG->PR = 4; // 设置预分频器为 4
  IWDG->RLR = 1249; // 设置重装载寄存器为 1249
  IWDG->KR = 0xCCCC; // 启动独立看门狗
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    IWDG->KR = 0xAAAA; // 喂狗，防止看门狗复位

    /* 收到新目标标志 -> 执行一次定位 */
    if (new_target_flag == 1)
    {
        new_target_flag = 0;                  // 清标志
        encoder_pos = Servo_ReadEncoder();     // 读当前位置
        if (encoder_pos < 0) encoder_pos = Servo_ReadEncoder(); // 读失败重试一次
        int32_t delta_enc = (home_full + target_pos) - encoder_pos;   // 到目标的差值
        int32_t delta_step = delta_enc * 25 / 512;       // 换算成步数
        uint8_t dir = 1;                       // 拆符号
        if (delta_step < 0) { dir = 0; delta_step = -delta_step; }
        Servo_SetPosition(dir, 10, delta_step);  // 发定位指令
        HAL_Delay(500); // 等电机转完，避免运动期间读编码器打断定位
    }
     // HAL_GPIO_TogglePin(User_Label___LED_GPIO_Port, User_Label___LED_Pin); // LED 心跳暂时停用
     // HAL_Delay(500); // 拖慢 PID 循环，先去掉
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   // encoder_pos = Servo_ReadEncoder(); // 读取编码器值
   if (encoder_pos >= 0)
    {
       // HAL_GPIO_WritePin(User_Label___LED_GPIO_Port, User_Label___LED_Pin, GPIO_PIN_SET);   // 收到-> LED 亮
    }
    else
    {
       // HAL_GPIO_WritePin(User_Label___LED_GPIO_Port, User_Label___LED_Pin, GPIO_PIN_RESET); //没收到 -> LED 灭
    }
    uint8_t encoder_data[8] = {0};
    encoder_data[0] = (encoder_pos >> 24) & 0xFF; // 高位字节
    encoder_data[1] = (encoder_pos >> 16) & 0xFF; // 中位字节
    encoder_data[2] = (encoder_pos >> 8) & 0xFF; // 中位字节
    encoder_data[3] = encoder_pos & 0xFF; // 低位字节

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = 0x201; // 标准 ID
    tx_header.ExtId = 0; 
    tx_header.IDE = CAN_ID_STD; // 标准 ID
    tx_header.RTR = CAN_RTR_DATA; // 数据帧
    tx_header.DLC = 4; // 数据长度
    tx_header.TransmitGlobalTime = DISABLE; // 不传输全局时间戳

    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(&hcan, &tx_header, encoder_data, &tx_mailbox); // 发送 CAN 消息

    // 位置模式：定位交给 SERVO42C 内部闭环，MCU 只读位置反馈
    //（target 由 CAN 更新时，在这里重新调用 Servo_SetPosition）

    HAL_Delay(50); // 延时 50ms
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(User_Label___LED_GPIO_Port, User_Label___LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Label___LED_Pin */
  GPIO_InitStruct.Pin = User_Label___LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(User_Label___LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* 手动回零按键 PA0：输入+上拉（按键接 PA0 与 GND，按下为低电平） */
  GPIO_InitStruct.Pin = HOME_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HOME_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

    target_pos = (RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3]; // 收目标位置
    new_target_flag = 1;  // 置标志：有新目标来了
    // 处理接收到的CAN数据
    // 例如：根据RxHeader.StdId判断消息类型，并处理RxData中的数据
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
#ifdef USE_FULL_ASSERT
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
