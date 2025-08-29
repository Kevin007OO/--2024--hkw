/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 自动化仓储机器人主控程序
  *                   实现机器人从货架抓取箱子并放置到指定区域的完整流程
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "robot_config.h"
#include "robot_state.h"
#include "robot_actions.h"
#include "task_planner.h"
#include "receive_from_pi.h"  // 树莓派通信模块
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

// --- 通信相关变量 ---
uint8_t receiveData[4];             // 串口接收数据缓冲区
volatile int wheel_ack = 0;         // 轮轨电机 ACK计数
volatile int jixiebi_ack = 0;       // 机械臂 ACK计数
volatile int dajiang_ack = 0;       // 水平滑台 ACK计数

// --- 系统状态变量 ---
volatile RobotPosition_t current_robot_position = ROBOT_POS_UNKNOWN; // 当前机器人位置
RobotState_t current_state = STATE_INIT;            // 当前状态机状态
TaskInfo_t task_info;                               // 当前任务信息
RecognitionResult_t recognition_data;               // 从树莓派接收的原始数据

// --- 控制变量 ---
double start_time, current_time, duration; // 时间相关变量
// 记录6个目标放置区域(a-f)的占用状态
// 数组索引0-5对应a-f区域，1表示true(已占用)，0表示false(未占用)
int is_area_occupied[6] = {0, 0, 0, 0, 0, 0};
volatile uint32_t jixiebi_next_ok = 0;  // 下次允许计数的时刻（ms）

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // 刚上电等待一会儿再开启接收，防止刚上电时电平抖动喷出乱码
  HAL_Delay(DELAY_STARTUP_MS); 
  
  // 启用各串口的错误中断和接收中断
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
  HAL_UART_Receive_IT(&huart1, &receiveData[0], 1);   // 轮轨电机

//   __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
//   HAL_UART_Receive_IT(&huart2,&receiveData[1],1);
  
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
  HAL_UART_Receive_IT(&huart3, &receiveData[2], 1);   // 机械臂
  
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
  HAL_UART_Receive_IT(&huart6, &receiveData[3], 1);   // 水平滑台
  
  
  // 测试数据初始化（实际使用时由树莓派提供）
//  recognition_data.shelf_positions[0] = 4;  // 货架位1放箱子4
//  recognition_data.shelf_positions[1] = 3;  // 货架位2放箱子3
//  recognition_data.shelf_positions[2] = 2;  // 货架位3放箱子2
//  recognition_data.shelf_positions[3] = 5;  // 货架位4放箱子5
//  recognition_data.shelf_positions[4] = 6;  // 货架位5放箱子6
//  recognition_data.shelf_positions[5] = 1;  // 货架位6放箱子1
//  
//  recognition_data.area_positions[0] = 1;   // a区域对应箱子1
//  recognition_data.area_positions[1] = 2;   // b区域对应箱子2
//  recognition_data.area_positions[2] = 0;   // c区域无对应箱子
//  recognition_data.area_positions[3] = 4;   // d区域对应箱子4
//  recognition_data.area_positions[4] = 5;   // e区域对应箱子5
//  recognition_data.area_positions[5] = 6;   // f区域对应箱子6

//  current_state = STATE_PROCESS_TASK_DATA;
  
  // 初始化树莓派接收模块，告诉它使用UART5
  rpi_init_polling(&huart5);
  HAL_TIM_Base_Init(&htim2);
  
  // 状态指示LED初始化
  HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 检测启动按钮
    if(HAL_GPIO_ReadPin(BUTTON_START_PORT, BUTTON_START_PIN) == 1 && current_state == STATE_BEFORE_START) {
        current_state = STATE_START_ROBOT_POSE;
        HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
    }
    
    // 主状态机
    switch (current_state) {
        case STATE_BEFORE_START: {
            // 等待启动信号
            break;
        }
        
        case STATE_INIT: {
            // 初始化任务变量
            memset(&task_info, 0, sizeof(TaskInfo_t));
            wheel_ack = dajiang_ack = jixiebi_ack = 0;
            // 直接进入下一个状态
            current_state = STATE_WAIT_RPI_DATA;
            break;
        }
        
        case STATE_WAIT_RPI_DATA: {
            // 向树莓派请求识别数据
            rpi_dual_signal_request_and_receive_polling(
                &recognition_data,
					      "PERFECT\n", // 先发送 PERFECT 信号
                100,         // 延迟 100ms
                "REQUEST\n", // 主动发送这个信号给树莓派
                true,    // 启用修复
                7,       // 尝试7次后修复货架
                true,    // 启用超时
                90000);  // 90秒超时
            current_state = STATE_PROCESS_TASK_DATA;
            break;
        }
        
        case STATE_PROCESS_TASK_DATA: {
            // --- 这是核心规划逻辑 ---
            process_and_plan_task(&recognition_data, &task_info);
            current_state = STATE_BEFORE_START;
            HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);
            break;
        }
        
        case STATE_START_ROBOT_POSE: {
            // 发送指令，让机器人移动到货架区的初始位置
            sendCommand_MoveToStart(); // 启动
            wheel_ack = 1;
            current_state = STATE_WAIT_START_POSE_ACK;
            break;
        }
        
        case STATE_WAIT_START_POSE_ACK: {
            if(wheel_ack == 1) { 
                wheel_ack = 0; // 重置ACK
                current_state = STATE_GET_NEXT_COLUMN;
            }
            // 此处可以添加超时逻辑
            break;
        }

        case STATE_GET_NEXT_COLUMN: {
            if (task_info.current_column_idx >= 3) { // 所有6个箱子都处理完了
                current_state = STATE_ALL_TASKS_COMPLETED;
            } else {
                current_state = STATE_MOVE_TO_SHELF_COLUMN;
            }
            break;
        }

        case STATE_MOVE_TO_SHELF_COLUMN: {
            // 确定当前要处理的箱子信息
            int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
            int target_id = is_orphan ? task_info.last_placed_target_id : 
                           task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
            int shelf_column_id = task_info.column_run_order[task_info.current_column_idx];
            
            // 获取当前列的上下两个箱子ID
            task_info.bottom_box_id = recognition_data.shelf_positions[shelf_column_id + 2];
            task_info.top_box_id = recognition_data.shelf_positions[shelf_column_id - 1];
            
            wheel_ack = dajiang_ack = 0;
            
            sendCommand_MoveToShelf(shelf_column_id, target_id);
            start_clock();
            current_state = STATE_WAIT_MOVE_TO_SHELF_ACK;
            break;
        }

        case STATE_WAIT_MOVE_TO_SHELF_ACK: {
            calculateDuration();
            if ((dajiang_ack >= 1) && (wheel_ack >= 1)) { 
                wheel_ack = dajiang_ack = jixiebi_ack = 0;
                current_state = STATE_EXECUTE_DOUBLE_PICK;
                current_robot_position = ROBOT_POS_SHELF;
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
            } else if(duration > TIMEOUT_MOVE_BETWEEN_SHELF_AND_AREA_S) {
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);
                current_state = STATE_MOVE_TO_SHELF_OUTOFTIME;
            }
            break;
        }
        
        case STATE_MOVE_TO_SHELF_OUTOFTIME: {
            // 超时处理：手动设置ACK
            dajiang_ack = 1;
            wheel_ack = 1;
            start_clock();
            current_state = STATE_WAIT_MOVE_TO_SHELF_ACK;
            break;
        }

        case STATE_EXECUTE_DOUBLE_PICK: {
            sendCommand_PickBox();
            // 根据取货动作序列，设置期望的ACK数量
            start_clock();
            current_state = STATE_WAIT_DOUBLE_PICK_ACK;
            break;
        }

        case STATE_WAIT_DOUBLE_PICK_ACK: {
            calculateDuration();
            if (jixiebi_ack >= 1) { // 检查是否收到所有ACK
                jixiebi_ack = 0;
                current_state = STATE_PLAN_TARGET_A_MOVE;
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
            } else if(duration >= TIMEOUT_DOUBLE_PICK_S) {
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);
                current_state = STATE_OUTOFTIME_DOUBLEPICK;
            }
            break;
        }
        
        case STATE_OUTOFTIME_DOUBLEPICK: {
            // 双取超时处理
            jixiebi_ack = 1;
            current_state = STATE_WAIT_DOUBLE_PICK_ACK;
            start_clock();
            break;
        }
                
        case STATE_PLAN_TARGET_A_MOVE: {
            // 规划上层箱子A的放置路径
            int is_orphan = (task_info.top_box_id == task_info.orphan_box_id);
            int target_id;

            if (is_orphan) {
                // 如果是孤儿箱子，需要寻找最佳临时放置位置
                target_id = find_best_target_for_orphan();
            } else {
                // 如果是普通箱子，直接使用其对应的目标区域
                target_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
            }

            RobotPosition_t required_pos = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;

            if (current_robot_position != required_pos) {
                // 需要进行大的跨区域移动
                current_state = STATE_EXECUTE_MAJOR_MOVE_A;
            } else {
                current_state = STATE_PLACE_BOX_A;
            }
            break;
        }

        case STATE_EXECUTE_MAJOR_MOVE_A: { // 执行大的跨区域移动
            int is_orphan = (task_info.top_box_id == task_info.orphan_box_id);
            int target_id; // 目标区域ID
            
            if (is_orphan) {
                target_id = find_best_target_for_orphan();
            } else {
                target_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
            }
            
            RobotPosition_t required_pos = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
            
            // 执行移动函数
            wheel_ack = dajiang_ack = 0;
            
            sendCommand_MoveTargetToTarget(required_pos, target_id);
            HAL_UART_Transmit_DMA(&huart2, (uint8_t*)CMD_FRONT_LIFT_FOR_REGRAB, strlen(CMD_FRONT_LIFT_FOR_REGRAB));  // 前端抬升
            HAL_Delay(DELAY_MAJOR_MOVE_LIFT_MS);
            
            // 根据目标ID精确定位水平滑台
            if(target_id == 0) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_A, strlen(CMD_SLIDER_POS_AREA_A));
            } else if(target_id == 1) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_B, strlen(CMD_SLIDER_POS_AREA_B));
            } else if(target_id == 2) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_C, strlen(CMD_SLIDER_POS_AREA_C));
            } else if(target_id == 3) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_D, strlen(CMD_SLIDER_POS_AREA_D));
            } else if(target_id == 4) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_E, strlen(CMD_SLIDER_POS_AREA_E));
            } else if(target_id == 5) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_F, strlen(CMD_SLIDER_POS_AREA_F));
            }
            
            current_state = STATE_WAIT_MAJOR_MOVE_A_ACK;
            start_clock();
            break;
        }

        case STATE_WAIT_MAJOR_MOVE_A_ACK: {
            calculateDuration();
            if (wheel_ack >= 1 && dajiang_ack >= 1) { // 等待轮轨电机(huart1)的确认
                wheel_ack = dajiang_ack = 0;
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
                
                // 移动完成后，更新当前位置状态
                int is_orphan = (task_info.top_box_id == task_info.orphan_box_id);
                int target_id;
                if (is_orphan) {
                    target_id = find_best_target_for_orphan();
                } else {
                    target_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
                }
                current_robot_position = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
                current_state = STATE_PLACE_BOX_A;
            }
            else if (duration >= TIMEOUT_MOVE_BETWEEN_SHELF_AND_AREA_S) {
                current_state = STATE_MOVE_A_OUTOFTIME;
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);
            }
            break;
        } // 等待大的跨区域移动完成
        
        case STATE_MOVE_A_OUTOFTIME: {
            // A移动超时处理
            wheel_ack = 1; // 手动伪造 ACK
            dajiang_ack = 1;
            current_state = STATE_WAIT_MAJOR_MOVE_A_ACK; // 返回等待
            start_clock();
            break;
        }
        
        case STATE_PLACE_BOX_A: {
            int box_id_to_place = task_info.top_box_id;
            int area_idx_to_go;
            
            if (box_id_to_place == task_info.orphan_box_id) {
                area_idx_to_go = find_best_target_for_orphan();
            } else {
                // 对于普通箱子，直接查找它自己应该去的区域索引
                for(int i = 1; i <= 6; i++) {
                    if(task_info.shelf_to_target[i] == box_id_to_place) {
                         area_idx_to_go = task_info.shelf_slot_to_area_idx[i];
                         break;
                    }
                }
            }   
            
            wheel_ack = dajiang_ack = jixiebi_ack = 0;
            
            // 执行放置函数
            sendCommand_PlaceBox(box_id_to_place, &task_info, area_idx_to_go);
            
            current_state = STATE_WAIT_PLACE_BOX_A_ACK;
            start_clock();
            break;
        }

        case STATE_WAIT_PLACE_BOX_A_ACK: {
            calculateDuration(); // 在开头更新计时
            
            int area_idx_to_go = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
            if(task_info.orphan_box_id == task_info.top_box_id) {
                area_idx_to_go = find_best_target_for_orphan();
            }
            
            if((area_idx_to_go >= 1 && area_idx_to_go <= 4) && jixiebi_ack >= 2) {
                jixiebi_ack = 0;
                
                int placed_box_id = task_info.top_box_id;
                if (placed_box_id != task_info.orphan_box_id) {
                    task_info.last_placed_target_id = area_idx_to_go;
                    is_area_occupied[area_idx_to_go] = 1; // 标记区域已占用
                }
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
                current_state = STATE_PLAN_TARGET_B_MOVE;
                
            } else if((area_idx_to_go == 0 || area_idx_to_go == 5) && jixiebi_ack >= 2) {
                wheel_ack = jixiebi_ack = 0;
                int placed_box_id = task_info.top_box_id;
                if (placed_box_id != task_info.orphan_box_id) {
                    task_info.last_placed_target_id = area_idx_to_go;
                    is_area_occupied[area_idx_to_go] = 1; // 标记区域已占用
                }
                current_state = STATE_PLAN_TARGET_B_MOVE;
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
            }
            else if (duration >= TIMEOUT_PLACE_BOX_S) { // 如果超时
                current_state = STATE_PLACE_A_OUTOFTIME; // 跳转到新的超时处理状态
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);
            }
            break;
        }

        case STATE_PLACE_A_OUTOFTIME: {
            // A放置超时处理
            jixiebi_ack = 2; // 手动伪造 ACK
            current_state = STATE_WAIT_PLACE_BOX_A_ACK; // 返回等待
            start_clock();
            break;
        }
        
        // ----- 放置下层箱子 (B) -----
        case STATE_PLAN_TARGET_B_MOVE: {
            int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
            int target_id = is_orphan ? task_info.last_placed_target_id : 
                           task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
            RobotPosition_t required_pos = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
            current_state = STATE_EXECUTE_MAJOR_MOVE_B;
            
            // 机械臂重新抓取下层箱子
            sendCommand_ArmRegrab();
            
            // 水平滑台定位
            if(target_id == 0) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_A, strlen(CMD_SLIDER_POS_AREA_A));
            } else if(target_id == 1) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_B, strlen(CMD_SLIDER_POS_AREA_B));
            } else if(target_id == 2) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_C, strlen(CMD_SLIDER_POS_AREA_C));
            } else if(target_id == 3) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_D, strlen(CMD_SLIDER_POS_AREA_D));
            } else if(target_id == 4) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_E, strlen(CMD_SLIDER_POS_AREA_E));
            } else if(target_id == 5) {
                HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_AREA_F, strlen(CMD_SLIDER_POS_AREA_F));
            }
            break;
        }
        
        case STATE_EXECUTE_MAJOR_MOVE_B: {
            int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
            int target_id = is_orphan ? task_info.last_placed_target_id : 
                           task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
            RobotPosition_t required_pos = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
            
            // 执行移动函数
            wheel_ack = 0;
            sendCommand_MoveTargetToTarget(required_pos, target_id);
            
            current_state = STATE_WAIT_MAJOR_MOVE_B_ACK;
            start_clock();
            break;
        }
        
        case STATE_WAIT_MAJOR_MOVE_B_ACK: {
            calculateDuration(); // 新增
            if(wheel_ack >= 1 && dajiang_ack >= 1) {
                wheel_ack = dajiang_ack = 0;
                int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
                int target_id = is_orphan ? task_info.last_placed_target_id : 
                               task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
                current_robot_position = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
                current_state = STATE_PLACE_BOX_B;
                
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
                start_clock(); // STATE_PLACE_BOX_B里的jixiebi_ack超时启动
            }
            else if (duration >= TIMEOUT_MAJOR_MOVE_B_S) {
                current_state = STATE_MOVE_B_OUTOFTIME;
                HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);
            }
            break;
        }
        
        case STATE_MOVE_B_OUTOFTIME: {
            // B移动超时处理
            wheel_ack = 1; // 手动伪造 ACK
            dajiang_ack = 1;
            current_state = STATE_WAIT_MAJOR_MOVE_B_ACK; // 返回等待
            start_clock(); // STATE_WAIT_MAJOR_MOVE_B_ACK超时
            break;
        }
        
        case STATE_PLACE_BOX_B: {
            int box_id_to_place = task_info.bottom_box_id;
            int area_idx_to_go;
            int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
            int target_id = is_orphan ? task_info.last_placed_target_id : 
                           task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
            
            if (box_id_to_place == task_info.orphan_box_id) {
                // 如果是孤儿箱子，我们要找到它占放的目标箱子所在的区域
                // 这需要我们反向查找一下 (或者在规划时也存储下这个映射)
                // 为了简单，我们先假设能找到
                if(task_info.last_placed_target_id == task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]]) {
                    area_idx_to_go = task_info.last_placed_target_id;
                } else if(task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]] == 0) {
                    if(recognition_data.area_positions[1] == 0) {
                        target_id = 2;
                    } else if(recognition_data.area_positions[2] == 0) {
                        target_id = 1;
                    } else {
                        target_id = 2;
                    }
                } else if(task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]] == 5) {
                    if(recognition_data.area_positions[4] == 0) {
                        target_id = 3;
                    } else if(recognition_data.area_positions[3] == 0) {
                        target_id = 4;
                    } else {
                        target_id = 3;
                    }
                }
            } else {
                // 对于普通箱子，直接查找它自己应该去的区域索引
                for(int i = 1; i <= 6; i++) {
                    if(task_info.shelf_to_target[i] == box_id_to_place) {
                         area_idx_to_go = task_info.shelf_slot_to_area_idx[i];
                         break;
                    }
                }
            }   
            
            wheel_ack = dajiang_ack = 0;
            
            // 执行放置函数
            calculateDuration();
            if(jixiebi_ack >= 2) {
                jixiebi_ack = 0;
                sendCommand_PlaceBox(box_id_to_place, &task_info, area_idx_to_go);
                
                current_state = STATE_WAIT_PLACE_BOX_B_ACK;
                start_clock();
            } else if(duration > TIMEOUT_WAIT_FOT_ARMREGRAB_S) {
                jixiebi_ack = 2;
                start_clock();
            }
            break;
        }
        
    case STATE_WAIT_PLACE_BOX_B_ACK: {
        calculateDuration();
        if(jixiebi_ack >= 2) {
            wheel_ack = jixiebi_ack = 0;

            // 确定箱子B最终的放置区域ID
            int final_area_id;
            if(task_info.orphan_box_id == task_info.bottom_box_id) {
                final_area_id = task_info.last_placed_target_id;
            } else {
                final_area_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
            }

            finalize_column_and_prepare_for_next(final_area_id); // 使用新函数
            
        } else if (duration >= TIMEOUT_PLACE_BOX_S) {
            current_state = STATE_PLACE_B_OUTOFTIME;
            HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);
        }
        break;
    }
        
        case STATE_PLACE_B_OUTOFTIME: {
            // B放置超时处理
            jixiebi_ack = 2; // 手动伪造 ACK
            current_state = STATE_WAIT_PLACE_BOX_B_ACK; // 返回等待
            start_clock();
            break;
        }
         
        case STATE_ALL_TASKS_COMPLETED: {
            // 所有任务完成，可以在这里闪灯或发送完成信息
            // 然后可以回到空闲状态或停止
            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CMD_WHEEL_END_POS, strlen(CMD_WHEEL_END_POS)); // 移动到中心线左侧停止
            HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_COL_2, strlen(CMD_SLIDER_POS_COL_2));
            HAL_UART_Transmit_DMA(&huart2, (uint8_t*)CMD_FRONT_DOWN_TO_GROUND, strlen(CMD_FRONT_DOWN_TO_GROUND));
            current_state = STATE_WAIT_OFF;
            start_clock();
            break;
        }
        
        case STATE_WAIT_OFF: {
            calculateDuration();
            if(wheel_ack >= 1) {
                current_state = STATE_SWITCH_OFF;
            } else if(duration > TIMEOUT_MOVE_BETWEEN_SHELF_AND_AREA_S) {
                return 0;
            }
            break;
        }

        case STATE_SWITCH_OFF: {
            return 0;
        }

        case STATE_ERROR: {
            // 停止所有电机，闪烁错误LED
            while(1) {
                HAL_GPIO_TogglePin(LED_STATUS_PORT, LED_STATUS_PIN);
                HAL_Delay(DELAY_ERROR_LED_BLINK_MS);
            }
            break;
        }
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
  RCC_OscInitStruct.PLL.PLLM = 4;
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

/**
 * @brief  串口接收完成中断回调函数
 * @param  huart: 串口句柄指针
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uint32_t now = HAL_GetTick();
    
    if(huart->Instance == USART1) {
        // 轮轨电机确认
        wheel_ack++;
        HAL_UART_Receive_IT(&huart1, &receiveData[0], 1);
    }
    
    if (huart->Instance == USART3) {  // 机械臂
        // 仅在"到点"时计数，并在这里重置下一窗口
        if ((int32_t)(now - jixiebi_next_ok) >= 0) {
            jixiebi_ack++;                              // 不做上限限制
            jixiebi_next_ok = now + JIXIEBI_COOLDOWN_MS; // 这里重置时间戳
        }
        HAL_UART_Receive_IT(&huart3, &receiveData[2], 1); // 立刻重启接收
        return;
    }
    
    if(huart->Instance == USART6) {
        // 水平滑台确认
        dajiang_ack++;
        HAL_UART_Receive_IT(&huart6, &receiveData[3], 1);
    }
}

/**
 * @brief  串口错误回调函数
 * @param  huart: 串口句柄指针
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // 清除所有常见错误标志
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    // 重启该口的"1字节接收"
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &receiveData[0], 1);
    } else if (huart->Instance == USART3) {
        HAL_UART_Receive_IT(&huart3, &receiveData[2], 1);
    } else if (huart->Instance == USART6) {
        HAL_UART_Receive_IT(&huart6, &receiveData[3], 1);
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
