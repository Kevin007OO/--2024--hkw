#include "robot_actions.h"
#include "robot_config.h"
#include "task_planner.h"
#include "main.h" // 为了 HAL_Delay
#include "string.h"



/**
 * @brief  发送移动到起始位置的命令
 *         huart1-wheel(轮轨), huart2-front(前端), huart3-jixiebi(机械臂)
 */
void sendCommand_MoveToStart(){
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_COMMON_INIT, strlen(CMD_COMMON_INIT));  // 机械臂初始化
    HAL_Delay(DELAY_INIT_BETWEEN_ARM_FRONT_MS);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)CMD_COMMON_INIT, strlen(CMD_COMMON_INIT));  // 前端初始化
}

/**
 * @brief  发送移动到指定货架列的命令
 * @param  column_id: 目标货架列号(1-3)
 * @param  target_id: 目标区域ID
 */
void sendCommand_MoveToShelf(int column_id, int target_id) {
    // 计算上一个放置的目标ID，用于路径优化
    int last_target_id = 0;
    if(task_info.current_column_idx != 0) {
        last_target_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx-1]+3];
        if(last_target_id == -1) {
            last_target_id = task_info.last_placed_target_id;
        }
    }
    
    HAL_Delay(DELAY_CMD_INTERVAL_MS);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)CMD_FRONT_PREPARE_FOT_LIFT, strlen(CMD_FRONT_PREPARE_FOT_LIFT));  // 前端抬升
    HAL_Delay(DELAY_CMD_INTERVAL_MS);
    
    // 根据列号控制水平滑台移动
    if(column_id == 1) {
        HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_COL_1, strlen(CMD_SLIDER_POS_COL_1));     // 移动到列1
    } else if(column_id == 2) {
        HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_COL_2, strlen(CMD_SLIDER_POS_COL_2));      // 移动到列2
    } else if(column_id == 3) {
        HAL_UART_Transmit_DMA(&huart6, (uint8_t*)CMD_SLIDER_POS_COL_3, strlen(CMD_SLIDER_POS_COL_3)); // 移动到列3
    }
    
		// 机械臂定位
		if(task_info.current_column_idx==0){
		  HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_ROTATE_NOLOADED_ZERO, strlen(CMD_ARM_ROTATE_NOLOADED_ZERO));
	  }else{
		  HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_ROTATE_NOLOADED_ONE, strlen(CMD_ARM_ROTATE_NOLOADED_ONE));
	  }
		HAL_Delay(10);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)CMD_FRONT_CLAW_OPEN, strlen(CMD_FRONT_CLAW_OPEN));   // 前端张开
	if((last_target_id == 0||last_target_id == 1)&&task_info.column_run_order[task_info.current_column_idx]==1){
		HAL_Delay(800);
	}else if((last_target_id == 4||last_target_id == 5)&&task_info.column_run_order[task_info.current_column_idx]==3){
		HAL_Delay(800);
	}
//        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_ROTATE_NOLOADED_ZERO, strlen(CMD_ARM_ROTATE_NOLOADED_ZERO));  

    // 前端张开和轮轨移动
    
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CMD_WHEEL_TO_SHELF, strlen(CMD_WHEEL_TO_SHELF));  // 轮轨移动到货架
}

/**
 * @brief  发送目标区域间移动命令
 * @param  targetPosition: 目标位置类型
 * @param  target_id: 目标区域ID(0-5对应a-f)
 */
void sendCommand_MoveTargetToTarget(RobotPosition_t targetPosition, int target_id){
    // 如果已在目标位置且是前方区域，无需移动
    if(current_robot_position == targetPosition && target_id >= 1 && target_id <= 4) {
        wheel_ack = 1;
        return;
    }
    
    // 根据目标位置发送对应的轮轨移动命令
    if(targetPosition == ROBOT_POS_SIDE_AREA && target_id == 0) {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CMD_WHEEL_TO_SIDE_AREA_A, strlen(CMD_WHEEL_TO_SIDE_AREA_A));   // 移动到侧方区域a位置
    }
    else if(targetPosition == ROBOT_POS_FRONT_AREA) {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CMD_WHEEL_TO_AREA_BCDE, strlen(CMD_WHEEL_TO_AREA_BCDE)); // 移动到前方区域
    } else if(targetPosition == ROBOT_POS_SIDE_AREA && target_id == 5) {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CMD_WHEEL_TO_SIDE_AREA_F, strlen(CMD_WHEEL_TO_SIDE_AREA_F));   // 移动到侧方区域f位置
    }
}

/**
 * @brief  机械臂重新抓取命令（从上爪转移到下爪）
 */
void sendCommand_ArmRegrab(){
    int last_target_id = 0;
    last_target_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
    
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_ROTATE_NOLOADED_ZERO, strlen(CMD_ARM_ROTATE_NOLOADED_ZERO));  // 机械臂复位
    start_clock();
    
    // 等待机械臂完成动作
    while(jixiebi_ack == 0) {
        calculateDuration();
        if(duration > TIMEOUT_ARM_ROTATE_TO_ZERO_S) {
            jixiebi_ack = 1;
            break;
        }
    }
    
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PICK_FROM_CLAW, strlen(CMD_ARM_PICK_FROM_CLAW)); // 执行重抓动作
}

/**
 * @brief  执行抓取箱子的动作序列
 */
void sendCommand_PickBox(){
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PICK_FROM_SHELF, strlen(CMD_ARM_PICK_FROM_SHELF)); // 机械臂夹起箱子
    HAL_Delay(DELAY_PICK_ARM_ACTION_MS);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)CMD_FRONT_CLAW_GRAB, strlen(CMD_FRONT_CLAW_GRAB));  // 收缩夹具夹住箱子
    HAL_Delay(DELAY_PICK_GRAB_CONFIRM_MS);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)CMD_FRONT_LIFT_BOX, strlen(CMD_FRONT_LIFT_BOX)); // 提升前爪
}

/**
 * @brief  执行放置箱子的动作序列
 * @param  box_id: 要放置的箱子ID
 * @param  task: 任务信息结构体指针
 * @param  serial_number: 目标区域序号(0-5对应a-f)
 */
void sendCommand_PlaceBox(int box_id, TaskInfo_t* task, int serial_number) {
    if(serial_number == 0) {  // a区域
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_ROTATE_AREA_A, strlen(CMD_ARM_ROTATE_AREA_A));  // 机械臂旋转90度
        start_clock();
        
        while(jixiebi_ack == 0) {
            calculateDuration();
            if(duration > TIMEOUT_ARM_ROTATE_90_S) {
                jixiebi_ack = 1;
                break;
            }
        }

        // 根据是否为孤儿箱子选择放置高度
        if (box_id == task->orphan_box_id) {
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PLACE_HIGH, strlen(CMD_ARM_PLACE_HIGH)); // 孤儿箱子高位放置
        } else {
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PLACE_LOW, strlen(CMD_ARM_PLACE_LOW)); // 普通箱子低位放置
        }
        
    } else if(serial_number >= 1 && serial_number <= 2) { // b,c区域
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_ROTATE_AREA_BC, strlen(CMD_ARM_ROTATE_AREA_BC)); // 机械臂旋转-180度
        start_clock();
        
        while(jixiebi_ack == 0) {
            calculateDuration();
            if(duration > TIMEOUT_ARM_ROTATE_180_S) {
                jixiebi_ack = 1;
                break;
            }
        }
        
        if (box_id == task->orphan_box_id) {
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PLACE_HIGH, strlen(CMD_ARM_PLACE_HIGH)); // 孤儿箱子高位放置
        } else {
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PLACE_LOW, strlen(CMD_ARM_PLACE_LOW)); // 普通箱子低位放置
        }
        
    } else if(serial_number >= 1 && serial_number <= 4) { // b,c,d,e区域
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_ROTATE_AREA_DE, strlen(CMD_ARM_ROTATE_AREA_DE)); // 机械臂旋转-180度
        start_clock();
        
        while(jixiebi_ack == 0) {
            calculateDuration();
            if(duration > TIMEOUT_ARM_ROTATE_180_S) {
                jixiebi_ack = 1;
                break;
            }
        }
        
        if (box_id == task->orphan_box_id) {
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PLACE_HIGH, strlen(CMD_ARM_PLACE_HIGH)); // 孤儿箱子高位放置
        } else {
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PLACE_LOW, strlen(CMD_ARM_PLACE_LOW)); // 普通箱子低位放置
        }
        
    }
			else if(serial_number == 5) { // f区域
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_ROTATE_AREA_F, strlen(CMD_ARM_ROTATE_AREA_F)); // 机械臂旋转-90度
        start_clock();
        
        while(jixiebi_ack == 0) {
            calculateDuration();
            if(duration > TIMEOUT_ARM_ROTATE_90_S) {
                jixiebi_ack = 1;
                break;
            }
        }
        
        if (box_id == task->orphan_box_id) {
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PLACE_HIGH, strlen(CMD_ARM_PLACE_HIGH)); // 孤儿箱子高位放置
        } else {
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)CMD_ARM_PLACE_LOW, strlen(CMD_ARM_PLACE_LOW)); // 普通箱子低位放置
        }
    }
}
