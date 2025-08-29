#include "task_planner.h"
#include "tim.h"
#include "gpio.h"
#include "robot_config.h"

/**
 * @brief  启动计时器
 */
void start_clock(){
    __HAL_TIM_SetCounter(&htim2, 0);
    start_time = HAL_GetTick();
    HAL_TIM_Base_Start(&htim2);
}

/**
 * @brief  计算持续时间
 */
void calculateDuration(){
    current_time = HAL_GetTick();
    duration = (current_time - start_time) / 1000.0f; // 转换为秒
}

/**
 * @brief  根据货架格号(1-6)返回其所在的列号(1-3)
 * @param  slot: 货架格号 (1-6)
 * @retval 列号 (1-3)，错误返回0
 */
int get_column_from_shelf_slot(int slot) {
    if (slot == 1 || slot == 4) return 1;  // 第1列：上层1，下层4
    if (slot == 2 || slot == 5) return 2;  // 第2列：上层2，下层5
    if (slot == 3 || slot == 6) return 3;  // 第3列：上层3，下层6
    return 0; // 错误
}

/**
 * @brief  处理数据，规划所有任务和映射
 * @param  rec_data: 从树莓派接收的识别结果
 * @param  task: 要填充的任务信息结构体
 */
void process_and_plan_task(RecognitionResult_t* rec_data, TaskInfo_t* task) {
    // 1. 反向映射，几号纸垫放置在哪个位置
    int box_id_to_area_idx[7] = {-1, -1, -1, -1, -1, -1, -1};
    for (int j = 0; j < 6; j++) {
        if (rec_data->area_positions[j] != 0) {
            box_id_to_area_idx[rec_data->area_positions[j]] = j;
        }
    }

    // 2. 找到孤儿箱子ID（没有对应放置区域的箱子）
    task->orphan_box_id = 0;
    for (int k = 1; k <= 6; k++) {
        if (box_id_to_area_idx[k] == -1) {
            task->orphan_box_id = k;
            break;
        }
    }

    // 3. 构建 货架位 -> 纸垫ID 和 货架位 -> 区域索引 的双重映射
    for (int i = 0; i < 6; i++) {
        int shelf_slot = i + 1;
        int box_id = rec_data->shelf_positions[i];
        task->shelf_to_target[shelf_slot] = box_id; 
        // box_id_to_area_idx[box_id]返回0-5索引，对应a-f区域
        // shelf_slot_to_area_idx[shelf_slot]中shelf_slot是1-6
        task->shelf_slot_to_area_idx[shelf_slot] = box_id_to_area_idx[box_id];
        
        // 记录孤儿箱子最初所在的货架列
        if (box_id == task->orphan_box_id) {
            task->orphan_box_initial_shelf = (shelf_slot-1)%3+1;
        }
    }
    
    // 4. 【按列搬运】规划执行顺序：孤儿箱子所在列最后处理
    int orphan_column = get_column_from_shelf_slot(task->orphan_box_initial_shelf);
    int column_order_idx = 0;
    
    // 先处理非孤儿列
    for (int col = 1; col <= 3; col++) {
        if (col != orphan_column) {
            task->column_run_order[column_order_idx++] = col;
        }
    }
    // 最后处理孤儿列
    task->column_run_order[2] = task->orphan_box_initial_shelf;

    // 5. 根据列顺序生成最终的箱子搬运顺序 (box_run_order)
    int run_order_idx = 0;
    for (int i = 0; i < 3; i++) {
        int current_col = task->column_run_order[i];
        int top_shelf_slot = current_col;       // 上层货架位
        int bottom_shelf_slot = current_col + 3; // 下层货架位
        task->box_run_order[run_order_idx++] = rec_data->shelf_positions[top_shelf_slot - 1];
        task->box_run_order[run_order_idx++] = rec_data->shelf_positions[bottom_shelf_slot - 1];
    }
    
    // 6. 初始化任务指针
    task->current_column_idx = 0;
    task->last_placed_target_id = 0;
}

/**
 * @brief  为孤儿箱子寻找最佳放置位置的策略函数 (最终修正版)
 * @retval int 目标区域的索引号 (0-5对应a-f)
 */
int find_best_target_for_orphan(void) {
    // 1. 明确下一个大目标（下层箱子B要去哪里），用于"顺路"判断
    int target_b_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];

    // 2. 根据刚取货的货架列，判断当前大致区域
    int current_column = task_info.column_run_order[task_info.current_column_idx];
    int current_pos_zone; // 0=左侧区域(对应a,b), 1=中间区域(c,d), 2=右侧区域(e,f)
    
    // 逻辑：3列是左，2列是中，1列是右
    if (current_column == 3) {
        current_pos_zone = 0; // 从第3列过来，现在在场地的左侧
    } else if (current_column == 2) {
        current_pos_zone = 1; // 从第2列过来，现在在场地的中间
    } else { // current_column == 1
        current_pos_zone = 2; // 从第1列过来，现在在场地的右侧
    }

    // 3. 根据"就近"和"尽量不放a,f"原则，确定两个理想的候选位置
    int candidate1 = -1, candidate2 = -1, candidate3 = -1;
    switch (current_pos_zone) {
        case 0: candidate1 = 1; candidate2 = 2; candidate3 = 3; break; // 左侧 -> 优先 b(1), c(2)
        case 1: candidate1 = 2; candidate2 = 3; candidate3 = 1; break; // 中间 -> 优先 c(2), d(3)
        case 2: candidate1 = 4; candidate2 = 3; candidate3 = 2; break; // 右侧 -> 优先 e(4), d(3)
    }

    // 4. 检查这些候选位置是否真的已经被占用
    int is_cand1_valid = (candidate1 != -1 && is_area_occupied[candidate1] == 1);
    int is_cand2_valid = (candidate2 != -1 && is_area_occupied[candidate2] == 1);
    int is_cand3_valid = (candidate3 != -1 && is_area_occupied[candidate3] == 1);

    // 5. 根据有效性和"顺路"原则，做出最终决定
    if (is_cand1_valid && is_cand2_valid && is_cand3_valid) {
        // 如果所有候选都有效，考虑顺路因素
        if (current_pos_zone == 0 && target_b_id >= 2) return candidate2;
        if (current_pos_zone == 2 && target_b_id <= 3) return candidate2;
        return candidate1;
    } 
    else if (is_cand1_valid) {
        return candidate1;
    }
    else if (is_cand2_valid) {
        return candidate2;
    }
    else if (is_cand3_valid) {
        return candidate3;
    }
    else {
        // 保底策略：如果两个就近的位置都无效，就使用上一个放置点
        return task_info.last_placed_target_id;
    }
}

/**
 * @brief  当一列的任务（两个箱子）全部完成后，更新状态并准备处理下一列
 * @param  placed_area_id: 最后一个箱子实际放置的区域ID
 */
void finalize_column_and_prepare_for_next(int placed_area_id) {
    int placed_box_id = task_info.bottom_box_id;
    if (placed_box_id != task_info.orphan_box_id) {
        task_info.last_placed_target_id = placed_area_id;
        is_area_occupied[placed_area_id] = 1; // 标记区域已占用
    }
    task_info.current_column_idx++;
    current_state = STATE_GET_NEXT_COLUMN;
    HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
}
