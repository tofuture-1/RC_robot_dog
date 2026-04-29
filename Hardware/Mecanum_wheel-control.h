#ifndef __MECANUM_WHEEL_CONTROL_H
#define __MECANUM_WHEEL_CONTROL_H
#endif

#include "main.h"
#include "Motor-control.h"

void Drive_mutiple_motor_speed(CAN_HandleTypeDef *hcan, Motor_Group *motor_group, int16_t speed1, int16_t speed2, int16_t speed3, int16_t speed4);


//运动模式切换-未进行RPM对应M/s
//电机位置规定
//左上1，右上2
//左下3，右下4
void Mecanum_Move(CAN_HandleTypeDef *hcan,Motor_Group *motor_group, char direction ,int16_t speed);
