/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Caculate.h"
#include "DJI.h"
//#include "uart_printf.c"
#include "wtr_can.h"
#include "M2006.h"
#include "stdlib.h"
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
/* ============================================================================
 * 全局状态（用户变量）
 * receive_flag/rx_done/ref：由串口接收回调更新，主循环与控制逻辑读取
 * tx_once_flag：到位反馈一次性发送标志
 * rx_buffer：串口 DMA + IDLE 模式的接收缓冲（长度10），注意回调中会补 '\0'
 * ack：向主控返回的单字节确认码（0xAA）
 * ========================================================================== */
uint8_t receive_flag = '0';
uint8_t rx_buffer[10];//接收数组
volatile uint8_t rx_done = 0;//接收完成标志
volatile uint8_t tx_once_flag = 0;//发送检验标志位
volatile double ref = 0.0;
int rx_index = 0;
const uint8_t ack = 0xAA;       // 向主控的反馈信息
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//float_to_byte place;//目标位置

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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(1000); //等待电调/总线准备就绪，避免上电瞬间误动作
	CANFilterInit(&hcan1);//用于初始化can1总线、开启过滤器，无法使用can2
	M2006_init(); //设置 M2006 参数/减速比等，并初始化 PID 默认值
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR); //开启 UART 错误中断（含 ORE/FE/PE/NE）
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6,rx_buffer,sizeof(rx_buffer)); //开启 USART6 DMA + 空闲中断接收
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_HT); //关闭 DMA 半传中断：避免半包触发处理
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/* ============================================================================
 * 主循环职责：
 * - 读取接收完成标志（rx_done）并一次性锁存“本次到位反馈”的触发位（tx_once_flag）
 * - 连续调用 set_M2006_position_cm(ref) 执行位置伺服（外环：位置到速度；内环：速度到电流）
 * - 每轮发送一次 1~4 路电机电流命令（CanTransmit_DJI_1234）
 * - 若判定到位且 tx_once_flag==1，则向主控发送 0xAA 确认，并清零 tx_once_flag
 * - HAL_Delay(5)：粗粒度控制循环节拍（简单系统可接受；如需硬实时建议迁移到定时器中断）
 * ========================================================================== */
  while (1)
  {
		/* 接收到 'A' 命令：锁存本次运动任务，tx_once_flag=1 确保“到位反馈”仅发送一次 */
		if(rx_done == 1 && rx_buffer[0] == 'A')
		{
			//set_M2006_position_cm(ref);
			rx_done = 0;
			tx_once_flag = 1;
		}
		set_M2006_position_cm(ref);
		/* 位置伺服（positionServo→speedServo→PID_Calc）：ref 单位为“厘米”，内部转换为角度 */
		CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,hDJI[1].speedPID.output,hDJI[2].speedPID.output,hDJI[3].speedPID.output); // 自行选择�?要的电机ID填入
		
		if(M2006_done() == 1 && tx_once_flag == 1)//到位
		{
			/* 到位一次性反馈：主控仅需接收单字节 0xAA 作为完成信号（非 ASCII，调试请用十六进制显示） */
			HAL_UART_Transmit(&huart6, (uint8_t*)&ack, 1, 0xFF);
			tx_once_flag = 0;//复位tx_once_flag
		}
		/* 周期节拍：5ms（简单节拍控制）。若后续并发任务增多，建议迁移控制循环至定时器中断以获得确定性周期 */
		HAL_Delay(5);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if(huart == &huart6) // 这里用你的实际 UART 句柄
    {
        // --- 1. 基本长度检查 ---
        if (Size < 2) // 至少需要命令+1个参数字符
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
            __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
            return;
        }

        // --- 2. 添加字符串结束符，防止 atof 越界 ---
        if (Size < sizeof(rx_buffer))
        {
            rx_buffer[Size] = '\0';
        }
        else
        {
            rx_buffer[sizeof(rx_buffer) - 1] = '\0';
        }

        // --- 3. 解析指令与参数 ---
        char command = rx_buffer[0];
        if (command == 'A') // 你自己的协议命令，可以加更多分支
        {
            double parsed_value = atof((const char *)&rx_buffer[1]);
            ref = parsed_value;
            receive_flag = command;
            rx_done = 1;
        }

        // --- 4. 重新启动 DMA 接收 ---
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    }
}

/* ============================================================================
 * UART 错误回调（最简 ORE 兜底）：
 * - 识别 ORE/FE/PE/NE 并逐一清旗标
 * - 立即停止 DMA 并重启 ReceiveToIdle_DMA，迅速恢复接收状态
 * - 不做复杂容错与日志，仅保证“不断收”
 * ========================================================================== */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6) // 只处理 huart1
    {
        uint32_t err = huart->ErrorCode;

        // 清除所有接收相关错误标志
        if (err & HAL_UART_ERROR_ORE) __HAL_UART_CLEAR_OREFLAG(huart);
        if (err & HAL_UART_ERROR_FE)  __HAL_UART_CLEAR_FEFLAG(huart);
        if (err & HAL_UART_ERROR_PE)  __HAL_UART_CLEAR_PEFLAG(huart);
        if (err & HAL_UART_ERROR_NE)  __HAL_UART_CLEAR_NEFLAG(huart);

        // 重启接收
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
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
