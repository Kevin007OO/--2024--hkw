#ifndef INC_ROBOT_CONFIG_H_
#define INC_ROBOT_CONFIG_H_

#define JIXIEBI_COOLDOWN_MS  2u      // 机械臂口令去抖窗口: 1~3ms后可用

/* ============================================================================
 * 通信指令宏定义
 * ============================================================================ */
// 通用指令 (用于机械臂、前夹爪的启动)
#define CMD_COMMON_INIT                 "s"             // 启动指令

// 前端爪开合指令 (h系列) (控制前夹爪M2006)
#define CMD_FRONT_CLAW_OPEN             "h-2"           // 张开
#define CMD_FRONT_CLAW_GRAB             "h0.1"            // 夹紧

// 前端爪升降指令 (v系列) (控制前夹爪舵机)
// 参数为距离地面的位置
#define CMD_FRONT_DOWN_TO_GROUND        "v0"            // 夹爪降到地面上, 用于完赛后把前夹爪复位
#define CMD_FRONT_PREPARE_FOT_LIFT      "v3"            // 在货架时夹爪夹紧前略微抬升方便夹起下层箱子
#define CMD_FRONT_LIFT_BOX              "v10"           // 把下层箱�j��夹起便于移出货架
#define CMD_FRONT_LIFT_FOR_REGRAB       "v18"           // 到了纸垛后把下层箱子升起一些便于ArmRegrab动作

// 轮轨移动指令 (a系列) (控制车轮电机)
// 参数为目标位置 (激光测距传感器与激光测距板子的距离)
#define CMD_WHEEL_TO_SHELF              "a9.0"          // 移动到货架前
#define CMD_WHEEL_TO_SIDE_AREA_A        "a264.9"          // 移动到纸垛a
#define CMD_WHEEL_TO_SIDE_AREA_F        "a264.9"          // 移动到纸垛f
#define CMD_WHEEL_TO_AREA_BCDE          "a285.1"        // 移动到纸垛bcde

// 轮轨移动指令 (e系列) (控制车轮电机) - 任务结束时使用
// 参数为目标位置 (激光测距传感器与激光测距板子的距离)
#define CMD_WHEEL_END_POS               "e100"           // 结束时快速移动至A区域

// 水平滑台移动指令 (A系列) (控制水平滑台M2006) 
// 参数为目标位置，零点为正中间，负为靠纸垛a侧，正为靠纸垛f侧
#define CMD_SLIDER_POS_COL_1            "A50"           // 移动到第一列货架
#define CMD_SLIDER_POS_COL_2            "A0"            // 移动到第二列货架
#define CMD_SLIDER_POS_COL_3            "A-50.5"        // 移动到第三列货架
#define CMD_SLIDER_POS_AREA_A           "A-45.0"        // 移动到纸垛a
#define CMD_SLIDER_POS_AREA_B           "A-68.2"          // 移动到纸垛b
#define CMD_SLIDER_POS_AREA_C           "A-23.2"          // 移动到纸垛c
#define CMD_SLIDER_POS_AREA_D           "A23"           // 移动到纸垛d
#define CMD_SLIDER_POS_AREA_E           "A68.3"           // 移动到纸垛e
#define CMD_SLIDER_POS_AREA_F           "A45.2"         // 移动到纸垛f

// 机械臂空载旋转指令 (f系列) (控制小米电机)
// 参数为旋转角度，正为逆时针，负为顺时针
#define CMD_ARM_ROTATE_NOLOADED_ZERO    "f0"            // 空载下旋转回零点
#define CMD_ARM_ROTATE_NOLOADED_ONE    "f1"            // 空载下旋转回正（修复取箱子偏左问题）

// 机械臂带载旋转指令 (n系列) (控制小米电机) 
// 参数为旋转角度，正为逆时针，负为顺时针
#define CMD_ARM_ROTATE_AREA_A           "n90"           // 带载旋转至纸垛a
#define CMD_ARM_ROTATE_AREA_F           "n-90"          // 带载旋转至纸垛f
#define CMD_ARM_ROTATE_AREA_BC        	"n-180"         // 带载旋转至纸垛bc
#define CMD_ARM_ROTATE_AREA_DE					"n180"					// 带载旋转至纸垛de

// 机械臂抓取指令 (g系列) (控制步进电机+舵机)
// 参数为夹爪下降距离
#define CMD_ARM_PICK_FROM_SHELF         "g35"           // 抓取货架上的上层箱子
#define CMD_ARM_PICK_FROM_CLAW          "g17"           // 抓取前夹爪上的箱子

// 机械臂放置指令 (p系列) (控制步进电机+舵机) 
// 参数为夹爪下降距离
#define CMD_ARM_PLACE_HIGH              "p24"           // 放置孤儿箱子高度
#define CMD_ARM_PLACE_LOW               "p12"           // 放置其他5个箱子高度


/* ============================================================================
 * 超时时间宏定义 (单位: 秒)
 * 部分分控已设超时机制，这部分的超时，主控的超时必须略微长于分控的最长超时
 * TIMEOUT_PLACE_BOX_S必须大于TIMEOUT_ARM_ROTATE_180_S
 * TIMEOUT_WAIT_FOT_ARMREGRAB_S必须大于TIMEOUT_ARM_ROTATE_TO_ZERO_S
 * ============================================================================ */
/*分控最长超时记录 (8月15日更新)
 * 轮子最长超时 (从8.5移动到285) 5.5秒
 * 小米电机最长超时 (带载旋转180度并到位) 3.8秒
 * 小米电机最长超时 (带载旋转90度并到位) 2.04秒
 * 小米电机回零点最长超时 (空载旋转180度并到位) 1.35秒
*/
#define TIMEOUT_MOVE_BETWEEN_SHELF_AND_AREA_S   6     // 轮子在货架和纸垛间移动的超时
#define TIMEOUT_MAJOR_MOVE_B_S              5           // 在纸垛附近,车轮在纸垛a或f的位置与纸垛bcde的位置之间移动的超时与2006的超时时间取最大值

#define TIMEOUT_ARM_ROTATE_90_S             2.6         // 小米电机带载旋转90度最长超时
#define TIMEOUT_ARM_ROTATE_180_S            4.3         // 小米电机带载旋转180度最长超时
#define TIMEOUT_ARM_ROTATE_TO_ZERO_S        2.0         // 小米电机带载旋转回零点最长超时

#define TIMEOUT_DOUBLE_PICK_S               3          // 在货架前取两个箱子的超时
#define TIMEOUT_PLACE_BOX_S                 3.2    // 在纸垛前放置箱子的总超时 (包括: 放置箱子再升起夹爪的整个过程)
#define TIMEOUT_WAIT_FOT_ARMREGRAB_S        4    // 从车轮与2006都执行完开始计时，正常情况下应该已经完成夹取动作，最少耗时情况为2006就地，车轮保持，此时耗时为机械臂重新夹取需要的时间（不包括小米电机旋转），大约为3s，这里设置到4s


/* ============================================================================
 * 延时时间宏定义 (单位: 毫秒)
 * ============================================================================ */
#define DELAY_STARTUP_MS                800     // 主控刚上电时的延时，等待外设电平稳定
#define DELAY_INIT_BETWEEN_ARM_FRONT_MS 50      // 启动时, 机械臂和前夹爪初始化指令之间的延时
#define DELAY_CMD_INTERVAL_MS           10      // 发送两个不同指令间的最小间隔延时
#define DELAY_PICK_GRAB_CONFIRM_MS      1000    // 抓取时，前夹爪夹紧后等待物理夹实稳定的延时
#define DELAY_MAJOR_MOVE_LIFT_MS        2000    // 大范围移动前，抬升箱子后等待稳定的延时
#define DELAY_PICK_ARM_ACTION_MS        50      // 抓取时，机械臂动作和前夹爪动作间的协调延时
#define DELAY_ERROR_LED_BLINK_MS        200     // 错误状态下LED闪烁的间隔


/* ============================================================================
 * 硬件引脚宏定义
 * ============================================================================ */
#define BUTTON_START_PORT               GPIOA
#define BUTTON_START_PIN                GPIO_PIN_0
#define LED_STATUS_PORT                 GPIOA
#define LED_STATUS_PIN                  GPIO_PIN_1

#endif /* INC_ROBOT_CONFIG_H_ */
