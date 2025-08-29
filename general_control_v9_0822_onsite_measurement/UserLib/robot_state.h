#ifndef INC_ROBOT_STATE_H_
#define INC_ROBOT_STATE_H_

#include "receive_from_pi.h" // 因为TaskInfo_t等可能需要此头文件中的类型

/* ============================================================================
 * 类型定义
 * ============================================================================ */
// 机器人运行的主状态枚举
typedef enum {
    STATE_BEFORE_START,         // 启动前状态，等待开始信号
    STATE_INIT,                 // 初始化状态
    STATE_WAIT_RPI_DATA,        // 等待树莓派识别数据
    STATE_PROCESS_TASK_DATA,    // 处理任务数据，进行路径规划
    STATE_START_ROBOT_POSE,     // 机器人移动到起始位置
    STATE_WAIT_START_POSE_ACK,  // 等待起始位置移动确认

    /* --- 以货架列为单位的循环 --- */
    STATE_GET_NEXT_COLUMN,          // 获取下一个要处理的货架列
    STATE_MOVE_TO_SHELF_COLUMN,     // 移动到目标货架列
    STATE_WAIT_MOVE_TO_SHELF_ACK,   // 等待移动到货架确认
    STATE_MOVE_TO_SHELF_OUTOFTIME,  // 移动到货架超时处理
    
    STATE_EXECUTE_DOUBLE_PICK,      // 执行"双取"动作（同时抓取上下两个箱子）
    STATE_WAIT_DOUBLE_PICK_ACK,     // 等待双取动作确认
    STATE_OUTOFTIME_DOUBLEPICK,     // 双取动作超时处理

    /* --- 处理第一个箱子 (上层箱子A) --- */
    STATE_PLAN_TARGET_A_MOVE,       // 规划去往A目标(上层箱子)的移动
    STATE_EXECUTE_MAJOR_MOVE_A,     // 执行A目标的主要移动
    STATE_WAIT_MAJOR_MOVE_A_ACK,    // 等待A移动确认
    STATE_MOVE_A_OUTOFTIME,         // A移动超时处理
    // STATE_EXECUTE_FINE_TUNE_MOVE_A,
    // STATE_WAIT_FINE_TUNE_MOVE_A_ACK,
    STATE_PLACE_BOX_A,              // 放置上层箱子A
    STATE_WAIT_PLACE_BOX_A_ACK,     // 等待放置A确认
    STATE_PLACE_A_OUTOFTIME,        // 放置A超时处理

    /* --- 内部转接 --- */
    // STATE_EXECUTE_ARM_REGRAB,    // 执行机械臂从前爪的重新抓取动作
    // STATE_WAIT_ARM_REGRAB_ACK,   // 等待机械臂重抓确认

    /* --- 处理第二个箱子 (下层箱子B) --- */
    STATE_PLAN_TARGET_B_MOVE,       // 规划去往B目标(下层箱子)的移动
    STATE_EXECUTE_MAJOR_MOVE_B,     // 执行B目标的主要移动
    STATE_WAIT_MAJOR_MOVE_B_ACK,    // 等待B移动确认
    STATE_MOVE_B_OUTOFTIME,         // B移动超时处理
    // STATE_EXECUTE_FINE_TUNE_MOVE_B,
    // STATE_WAIT_FINE_TUNE_MOVE_B_ACK,
    STATE_PLACE_BOX_B,              // 放置下层箱子B
    STATE_WAIT_PLACE_BOX_B_ACK,     // 等待放置B确认
    STATE_PLACE_B_OUTOFTIME,        // 放置B超时处理

    STATE_ALL_TASKS_COMPLETED,      // 所有任务完成
    STATE_WAIT_OFF,                 // 等待关机
    STATE_SWITCH_OFF,               // 关机状态
    STATE_ERROR                     // 错误状态
} RobotState_t;

// 机器人当前位置枚举
typedef enum {
    ROBOT_POS_UNKNOWN,         // 未知位置
    ROBOT_POS_SHELF,           // 在货架区
    ROBOT_POS_SIDE_AREA,       // 在侧方放置区 (对应1, 6号纸垫)
    ROBOT_POS_FRONT_AREA,      // 在前方放置区 (对应2, 3, 4, 5号纸垫)
} RobotPosition_t;

// 封装所有任务信息的结构体
typedef struct {
    int shelf_to_target[7];         // 货架格上箱子编号映射(1-6)
    int shelf_slot_to_area_idx[7];  // 货架位(索引1-6) -> 目标区域索引(0-5, 对应a-f)

    // 孤儿箱子信息（没有对应放置区域的箱子）
    int orphan_box_id;              // "孤儿箱子"的编号 (1-6)
    int orphan_box_initial_shelf;   // "孤儿箱子"最初所在的货架位(1-3)

    // 执行顺序规划
    int column_run_order[3];        // 列的执行顺序(1-3)
    int box_run_order[6];           // 箱子的执行顺序 (根据列顺序生成)

    // 运行时状态
    int current_column_idx;         // 当前执行到 column_run_order 的哪个索引
    int current_box_idx;            // 当前箱子索引
    int last_placed_target_id;      // 上一个放置的箱子对应的纸垫编号
    
    // 用于临时存储当前正在处理的两个箱子的信息
    int top_box_id;                 // 上层箱子ID
    int bottom_box_id;              // 下层箱子ID
} TaskInfo_t;

/* ============================================================================
 * 全局变量声明 (extern)
 * ============================================================================ */

// --- 通信相关变量 ---
extern volatile int wheel_ack;
extern volatile int jixiebi_ack;
extern volatile int dajiang_ack;

// --- 系统状态变量 ---
extern volatile RobotPosition_t current_robot_position;
extern RobotState_t current_state;
extern TaskInfo_t task_info;
extern RecognitionResult_t recognition_data;

// --- 控制变量 ---
extern double start_time, current_time, duration;
extern int is_area_occupied[6];
extern volatile uint32_t jixiebi_next_ok;

// --- 硬件句柄声明 (让其他文件能调用HAL库函数) ---
#include "usart.h" // 包含所有 huart 句柄的定义

#endif /* INC_ROBOT_STATE_H_ */
