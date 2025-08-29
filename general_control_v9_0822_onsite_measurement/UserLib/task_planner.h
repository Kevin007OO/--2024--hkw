#ifndef INC_TASK_PLANNER_H_
#define INC_TASK_PLANNER_H_

#include "robot_state.h"

// 函数原型声明
void start_clock(void);
void calculateDuration(void);
int get_column_from_shelf_slot(int slot);
void process_and_plan_task(RecognitionResult_t* rec_data, TaskInfo_t* task);
int find_best_target_for_orphan(void);

/**
 * @brief  当一列的任务（两个箱子）全部完成后，更新状态并准备处理下一列
 * @param  placed_area_id: 最后一个箱子实际放置的区域ID
 */
void finalize_column_and_prepare_for_next(int placed_area_id);

#endif /* INC_TASK_PLANNER_H_ */
