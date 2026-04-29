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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <dog_control.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Uart1串口相关
float uart1_rx_float1;
float uart1_rx_float2;
uint8_t uart1_cmd_ready = 0;    // 保持，主循环通过它判断新数据就绪

// 接收缓冲区大小改为 9 字节：8 字节数据 + 1 字节校验和
uint8_t uart1_rx_buffer[9];
uint8_t uart1_rx_index;
uint8_t uart1_rx_state;
uint8_t uart1_rx_byte;      // 中断接收的单个字节

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART6_UART_Init(void);
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
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan1);
	
	Foot foot1;
	foot1.X = 0;
	foot1.Y = -0.1;
	foot1.L_R_side = Right;
	
	foot1.mi.ID = 0x01;
	foot1.mi.MST_ID = 0x11;
	foot1.mi.mode = MIT;
	foot1.mi.kp = 7;
	foot1.mi.kd = 0.1;
	foot1.mi.torq = 0.05;
	foot1.mi.vel = 0.001;
	foot1.mi.pos = 0;
	foot1.mi.able_state = Enable;
	
	foot1.mo.ID = 0x02;
	foot1.mo.MST_ID = 0x12;
	foot1.mo.mode = MIT;
	foot1.mo.kp = 7;
	foot1.mo.kd = 0.1;
	foot1.mo.torq = 0.05;
	foot1.mo.vel = 0.001;
	foot1.mo.pos = 0;
	foot1.mo.able_state = Enable;

	Foot foot2;
	foot2.X = 0;
	foot2.Y = -0.1;
	foot2.L_R_side = Left;
	
	foot2.mi.ID = 0x03;
	foot2.mi.MST_ID = 0x13;
	foot2.mi.mode = MIT;
	foot2.mi.kp = 7;
	foot2.mi.kd = 0.1;
	foot2.mi.torq = 0.05;
	foot2.mi.vel = 0.001;
	foot2.mi.pos = 0;
	foot2.mi.able_state = Enable;
	
	foot2.mo.ID = 0x04;
	foot2.mo.MST_ID = 0x14;
	foot2.mo.mode = MIT;
	foot2.mo.kp = 7;
	foot2.mo.kd = 0.1;
	foot2.mo.torq = 0.05;
	foot2.mo.vel = 0.001;
	foot2.mo.pos = 0;
	foot2.mo.able_state = Enable;
	//两个前腿
	
	//两个后腿
	Foot foot3;
	foot3.X = 0;
	foot3.Y = -0.1;
	foot3.L_R_side = Right;
	
	foot3.mi.ID = 0x08;
	foot3.mi.MST_ID = 0x18;
	foot3.mi.mode = MIT;
	foot3.mi.kp = 7;
	foot3.mi.kd = 0.1;
	foot3.mi.torq = 0.05;
	foot3.mi.vel = 0.001;
	foot3.mi.pos = 0;
	foot3.mi.able_state = Enable;
	
	foot3.mo.ID = 0x07;
	foot3.mo.MST_ID = 0x17;
	foot3.mo.mode = MIT;
	foot3.mo.kp = 7;
	foot3.mo.kd = 0.1;
	foot3.mo.torq = 0.05;
	foot3.mo.vel = 0.001;
	foot3.mo.pos = 0;
	foot3.mo.able_state = Enable;

	Foot foot4;
	foot4.X = 0;
	foot4.Y = -0.1;
	foot4.L_R_side = Left;
	
	foot4.mi.ID = 0x06;
	foot4.mi.MST_ID = 0x16;
	foot4.mi.mode = MIT;
	foot4.mi.kp = 7;
	foot4.mi.kd = 0.1;
	foot4.mi.torq = 0.05;
	foot4.mi.vel = 0.001;
	foot4.mi.pos = 0;
	foot4.mi.able_state = Enable;
	
	foot4.mo.ID = 0x05;
	foot4.mo.MST_ID = 0x15;
	foot4.mo.mode = MIT;
	foot4.mo.kp = 7;
	foot4.mo.kd = 0.1;
	foot4.mo.torq = 0.05;
	foot4.mo.vel = 0.001;
	foot4.mo.pos = 0;
	foot4.mo.able_state = Enable;
	
	
	
	HAL_Delay(2000);//留时间给电机自检
	
	DM_Motor_Init(&hcan1,&foot1.mi);
	HAL_Delay(10);
	DM_Motor_Init(&hcan1,&foot1.mo);
	HAL_Delay(10);
	DM_Motor_Init(&hcan1,&foot2.mi);
	HAL_Delay(10);
	DM_Motor_Init(&hcan1,&foot2.mo);
	HAL_Delay(10);
	DM_Motor_Init(&hcan1,&foot3.mi);
	HAL_Delay(10);
	DM_Motor_Init(&hcan1,&foot3.mo);
	HAL_Delay(10);
	DM_Motor_Init(&hcan1,&foot4.mi);
	HAL_Delay(10);
	DM_Motor_Init(&hcan1,&foot4.mo);
	HAL_Delay(10);
	HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*
		DM_Motor_Send(&hcan1,&foot1.mo);
		HAL_Delay(1);
		DM_Motor_Send(&hcan1,&foot1.mi);
		HAL_Delay(1);
		*/
		if(uart1_cmd_ready)
		{
			uart1_cmd_ready = 0;
			foot1.X =  uart1_rx_float1;
			foot1.Y = uart1_rx_float2;
			foot2.X =  uart1_rx_float1;
			foot2.Y = uart1_rx_float2;
			foot3.X = -uart1_rx_float1;
			foot3.Y = uart1_rx_float2;
			foot4.X = -uart1_rx_float1;
			foot4.Y = uart1_rx_float2;
		}//收到再改
		
		HAL_Delay(1);
		Foot_control(&foot1,&hcan1);
		HAL_Delay(1);
		Foot_control(&foot2,&hcan1);
		HAL_Delay(1);
		Foot_control(&foot3,&hcan1);
		HAL_Delay(1);
		Foot_control(&foot4,&hcan1);
		HAL_Delay(1);
		
		//蓝牙模块发送
			float val1 = foot1.X;   // 待发送的第一个浮点数
			float val2 = foot1.Y;   // 待发送的第二个浮点数

			uint8_t tx_buffer[11];

			// 帧头
			tx_buffer[0] = 0xA5;

			// 将 float 的二进制表示按小端存入缓冲区（避免直接对 float 移位）
			uint32_t raw1, raw2;
			memcpy(&raw1, &val1, 4);
			memcpy(&raw2, &val2, 4);

			tx_buffer[1] = (uint8_t)(raw1 & 0xFF);
			tx_buffer[2] = (uint8_t)((raw1 >> 8) & 0xFF);
			tx_buffer[3] = (uint8_t)((raw1 >> 16) & 0xFF);
			tx_buffer[4] = (uint8_t)((raw1 >> 24) & 0xFF);

			tx_buffer[5] = (uint8_t)(raw2 & 0xFF);
			tx_buffer[6] = (uint8_t)((raw2 >> 8) & 0xFF);
			tx_buffer[7] = (uint8_t)((raw2 >> 16) & 0xFF);
			tx_buffer[8] = (uint8_t)((raw2 >> 24) & 0xFF);

			// 校验和：8 个数据字节之和的低 8 位
			uint16_t sum = 0;
			for (int i = 1; i <= 8; i++) {
					sum += tx_buffer[i];
			}
			tx_buffer[9] = (uint8_t)(sum & 0xFF);

			// 帧尾
			tx_buffer[10] = 0x5A;

			// 然后通过 UART 发送 tx_buffer，长度 11
			HAL_UART_Transmit(&huart1, tx_buffer, 11, 100);
		
		DM_Motor_FootData_SendVOFA_Firewater(&foot1,&huart6);
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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	
	// 配置CAN过滤器 - 接收所有标准ID的消息
  CAN_FilterTypeDef sFilterConfig;
  
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // 接收到FIFO0
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        switch (uart1_rx_state)
        {
            case 0: // 等待帧头 0xA5
                if (uart1_rx_byte == 0xA5)
                {
                    uart1_rx_state = 1;
                    uart1_rx_index = 0;
                }
                break;

            case 1: // 接收 8 字节数据 + 1 字节校验和
                uart1_rx_buffer[uart1_rx_index++] = uart1_rx_byte;
                if (uart1_rx_index == 9)   // 收满 9 字节（8数据+1校验和）
                {
                    // 计算 8 字节数据的校验和（低 8 位）
                    uint8_t calc_sum = 0;
                    for (int i = 0; i < 8; i++)
                    {
                        calc_sum += uart1_rx_buffer[i];
                    }

                    if (calc_sum == uart1_rx_buffer[8]) // 校验通过
                    {
                        uart1_rx_state = 2;   // 等待帧尾
                    }
                    else
                    {
                        // 校验失败，丢弃整帧，复位
                        uart1_rx_state = 0;
                        uart1_rx_index = 0;
                    }
                }
                break;

            case 2: // 等待帧尾 0x5A
                if (uart1_rx_byte == 0x5A)
                {
                    // 帧尾正确，解析两个 float（小端模式）
                    uint32_t raw1, raw2;

                    // 组合第一个 uint32_t
                    raw1 = (uint32_t)uart1_rx_buffer[0] |
                           ((uint32_t)uart1_rx_buffer[1] << 8) |
                           ((uint32_t)uart1_rx_buffer[2] << 16) |
                           ((uint32_t)uart1_rx_buffer[3] << 24);

                    // 组合第二个 uint32_t
                    raw2 = (uint32_t)uart1_rx_buffer[4] |
                           ((uint32_t)uart1_rx_buffer[5] << 8) |
                           ((uint32_t)uart1_rx_buffer[6] << 16) |
                           ((uint32_t)uart1_rx_buffer[7] << 24);

                    // 转换为 float（避免直接对 float 进行位操作）
                    memcpy(&uart1_rx_float1, &raw1, sizeof(float));
                    memcpy(&uart1_rx_float2, &raw2, sizeof(float));

                    uart1_cmd_ready = 1;      // 通知主循环有新数据
                }
                // 无论帧尾是否匹配，都复位状态机，准备下一帧
                uart1_rx_state = 0;
                uart1_rx_index = 0;
                break;

            default:
                uart1_rx_state = 0;
                uart1_rx_index = 0;
                break;
        }

        // 重新启动中断接收下一个字节
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
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
