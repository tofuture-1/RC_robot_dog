#ifndef __CASCADEPID_CONTROL_H
#define __CASCADEPID_CONTROL_H
#include "main.h"           /* 主头文件，包含HAL库定义 */
#include "Motor-control.h"
#include <stdlib.h>
#endif



// 通用PID计算函数
float PID_Calculate(PID_Controller *pid, float error, float dt);

// 初始化串级PID控制器参数
void CascadePID_Init(Motor_Data *motor);

// 串级PID控制主函数
void Position_Speed_Cascade_Control(Motor_Data *motor);


