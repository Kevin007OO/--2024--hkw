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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "ctype.h"
#include "PID.h"
#include "motor.h"
#include "stp_23l.h"
#include "lidar_processing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WHEEL_DIAMETER 5.5
#define PI 3.14159265358
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
float data[4];
extern int wflag1,wflag2,endflag,startflag,once_transmit_flag,correct_enable;
extern double distance_traveled1,start_position1;
extern double distance_traveled2,start_position2;
uint8_t receive_flag = '0';
uint8_t receiveData[10];
volatile double ref;
volatile static double last_ref = 200.0; // 直接把要用到的距离“上次目标值”设置为场地中心位置
volatile double delta_cm;
double ref1;
motor_data motor1;
motor_data motor2;
double pfdb;
char arr[6];
int rxindex=0;
// 这里是给HAL库使用的单字节接收缓冲
uint8_t stp_Receive_buf1[1];
uint8_t stp_Receive_buf2[1];

/* ------------ 超时机制所需变量 ------------ */
uint8_t timeout_enabled = 0;      // 超时功能开关: 1 = 启用, 0 = 禁用
uint32_t timeout_ticks = 1600;    // 超时时间 (单位: TIM1中断次数) 当前TIM1中断周期为2.5ms，设置为1000就是设置超时时间为5000ms
uint32_t movement_timer = 0;      // 运动计时器
/* ---------------------------------------- */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lidar_Process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// === 动态超时估算：由 |距离(cm)| 推算秒数，再换成 ticks ===
static inline uint32_t compute_timeout_ticks_cm(double d_cm, int is_end_phase)
{
    // 线性拟合（来自实测）：t = a + b * |d|
    // a 和 b 来自拟合：a≈1.0，b≈0.01
    double t = 1.0 + 0.01 * (d_cm >= 0 ? d_cm : -d_cm);

    // 最小 / 最大保护
    if (t < 1.0) t = 1.0;        // 不少于 1 秒
    if (t > 10.0) t = 10.0;      // 不超过 60 秒（可按需调大）

    // 末段慢速（指令 'e'）可略调裕量
    double margin = is_end_phase ? 1.10 : 1.20;   // 普通段 1.20，末段 1.10（可按实际再调）
    t *= margin;
		t += 0.3; // 再加0.3秒的冗余

    // 秒 -> tick（TIM1: 2.5ms 一次）= 秒 * 400
    uint32_t ticks = (uint32_t)(t * 400.0 + 0.5);

    // 再次做一下保底与上限钳制
    if (ticks < 400)   ticks = 400;    // ≥ 1s
    if (ticks > 4000) ticks = 4000;  // ≤ 10s
    return ticks;
}


void justfloat_output(){
    data[0]=motor2.motor_pid_v.fdb;
    data[1]=motor2.motor_pid_v.ref;
	  data[2]=motor1.motor_pid_v.fdb;
	  data[3]=motor1.motor_pid_v.ref;
	
    HAL_UART_Transmit(&huart3,(uint8_t*)data,sizeof(float)*4,0xffff);
    HAL_UART_Transmit(&huart3,tail,4,0xffff);
}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // 初始化两个电机的数据结构
  motor_init(htim2,htim1,TIM_CHANNEL_1,&motor1);
  motor_init(htim3,htim1,TIM_CHANNEL_2,&motor2);
  
  // 启动TIM1作为主控制循环的定时器，并开启中断
  HAL_TIM_Base_Start_IT(&htim1);
  
  // 启动USART3的DMA接收，使用空闲中断(IDLE)来检测一帧数据的结束
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,receiveData,10);
  
	// 在main函数中开启启动接收前先使能 UART 错误中断
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);

  // 启动USART1和USART2的单字节中断接收，用于处理LiDAR数据流
  HAL_UART_Receive_IT(&huart1,stp_Receive_buf1,sizeof(stp_Receive_buf1));
  HAL_UART_Receive_IT(&huart2,stp_Receive_buf2,sizeof(stp_Receive_buf2));
	
   __HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR); //加这一句，使能指令接收dma错误中断
	 
  // 禁用DMA的半传输中断，我们只关心整帧完成和空闲中断
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// 2. 在主循环中调用新的处理函数
    Lidar_Process();
		set_motor_direction(&motor1,AIN1_GPIO_Port,AIN1_Pin,AIN2_GPIO_Port,AIN2_Pin);
		set_motor_direction(&motor2,BIN1_GPIO_Port,BIN1_Pin,BIN2_GPIO_Port,BIN2_Pin);
		if(receive_flag =='a'){
			// --- 松开刹车，准备新的移动 ---
			set_motor_hold(&motor1, 0);
			set_motor_hold(&motor2, 0);
			// ---------------------------
			wflag1=1;
			wflag2=1;
			correct_enable=0;
			once_transmit_flag=1;
			movement_timer = 0; //复位计时器
			delta_cm = abs(ref - last_ref);
			timeout_ticks = compute_timeout_ticks_cm(delta_cm, /*is_end_phase=*/0); // 计算动态超时
			ref1=ref/WHEEL_DIAMETER/PI;//ref参数单位为cm
		  set_pidp_ref(&motor1,ref1);
		  set_pidp_ref(&motor2,ref1);
			receiveData[0]='0';
			receive_flag='0';
			startflag = 1;
			start_position1 = distance1/10/WHEEL_DIAMETER/PI;
			start_position2 = distance2/10/WHEEL_DIAMETER/PI;
			last_ref = ref;
		}
		if(receive_flag =='e'){//end
			// --- 松开刹车，准备新的移动 ---
			set_motor_hold(&motor1, 0);
			set_motor_hold(&motor2, 0);
			// ---------------------------
			endflag = 1;
			wflag1=1;
			wflag2=1;
			movement_timer = 0; //复位计时器
			delta_cm = abs(ref - last_ref);
			timeout_ticks = compute_timeout_ticks_cm(delta_cm, /*is_end_phase=*/1); // 计算动态超时
			ref1=ref/WHEEL_DIAMETER/PI;//ref参数单位为cm
		  set_pidp_ref(&motor1,ref1);
		  set_pidp_ref(&motor2,ref1);
			receiveData[0]='0';
			receive_flag='0';
			last_ref = ref;
		}
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim1){
		motor_pid_control();
//		justfloat_output();
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
	if(huart->Instance == USART3)
	{
		// --- 1. 对接收长度进行基础验证 ---
		// 至少需要2个字节 (例如: 'a' + '0') 才能构成一条有效指令
		if (Size < 2)
		{
			// 长度不足，认为是无效数据，直接准备下一次接收
			HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, 10);
			__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
			return;
		}

		// --- 2. 手动添加字符串结束符 '\0' ---
		// 这是解决问题的关键步骤，确保atof等字符串函数不会读到缓冲区里的旧数据
		// Size 是接收到的字节数，所以我们将结束符放在 receiveData[Size] 的位置
        // 同时要防止越界，虽然Size理论上不会超过10
		if (Size < 10)
		{
			receiveData[Size] = '\0';
		}
		else
		{
			receiveData[9] = '\0'; // 缓冲区满了，强制在末尾加结束符
		}


		// --- 3. 解析指令和数据 ---
		char command = receiveData[0]; // 第一个字节是指令

		// 检查是否为我们支持的指令
		if(command == 'a' || command == 'e')
		{
			// 指令有效，才进行后续操作
			// 将指令后的部分 (&receiveData[1]) 转换为浮点数
			double parsed_value = atof((const char *)&receiveData[1]);

			// 更新全局变量，传递指令和目标值
			ref = parsed_value;
			receive_flag = command; // receive_flag现在在主循环中触发电机运动
		}
		// 如果指令不是 'a' 或 'e'，则忽略这条消息，不进行任何操作

		// --- 4. 重新启动DMA空闲中断接收 ---
		// 为下一次数据接收做准备
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, 10);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 将接收到的数据存入环形缓冲区1
        uint16_t next_head = (uart1_rx_buffer.head + 1) % 256;
        if (next_head != uart1_rx_buffer.tail) { // 防止缓冲区写满
            uart1_rx_buffer.buffer[uart1_rx_buffer.head] = stp_Receive_buf1[0];
            uart1_rx_buffer.head = next_head;
        }
        // 立即重新启动中断，准备接收下一个字节
        HAL_UART_Receive_IT(&huart1, stp_Receive_buf1, 1);
    }
    else if (huart->Instance == USART2)
    {
        // 将接收到的数据存入环形缓冲区2
        uint16_t next_head = (uart2_rx_buffer.head + 1) % 256;
        if (next_head != uart2_rx_buffer.tail) { // 防止缓冲区写满
            uart2_rx_buffer.buffer[uart2_rx_buffer.head] = stp_Receive_buf2[0];
            uart2_rx_buffer.head = next_head;
        }
        // 立即重新启动中断，准备接收下一个字节
        HAL_UART_Receive_IT(&huart2, stp_Receive_buf2, 1);
    }
}

// HAL 库的错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) 
    {
        // 逐个清除错误标志
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);

        // 重新打开接收
        HAL_UART_Receive_IT(&huart1, stp_Receive_buf1, sizeof(stp_Receive_buf1)); //需要修改各个参数
    }
		if (huart->Instance == USART2) 
    {
        // 逐个清除错误标志
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);

        // 重新打开接收
        HAL_UART_Receive_IT(&huart2, stp_Receive_buf2, sizeof(stp_Receive_buf2)); //需要修改各个参数
    }
		if (huart == &huart3) 
    {
        uint32_t err = huart->ErrorCode;

        // 清除所有接收相关错误标志
        if (err & HAL_UART_ERROR_ORE) __HAL_UART_CLEAR_OREFLAG(huart);
        if (err & HAL_UART_ERROR_FE)  __HAL_UART_CLEAR_FEFLAG(huart);
        if (err & HAL_UART_ERROR_PE)  __HAL_UART_CLEAR_PEFLAG(huart);
        if (err & HAL_UART_ERROR_NE)  __HAL_UART_CLEAR_NEFLAG(huart);

        // 重启接收
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, sizeof(receiveData));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
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
