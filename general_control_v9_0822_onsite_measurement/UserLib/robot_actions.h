#ifndef INC_ROBOT_ACTIONS_H_
#define INC_ROBOT_ACTIONS_H_

#include "robot_state.h"

// 函数原型声明
void sendCommand_MoveToStart(void);
void sendCommand_MoveToShelf(int column_id, int target_id);
void sendCommand_MoveTargetToTarget(RobotPosition_t targetPosition, int target_id);
void sendCommand_ArmRegrab(void);
void sendCommand_PickBox(void);
void sendCommand_PlaceBox(int box_id, TaskInfo_t* task, int serial_number);


#endif /* INC_ROBOT_ACTIONS_H_ */
