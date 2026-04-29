#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"           /* 主头文件，包含HAL库定义 */
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include <string.h>
#define MOTOR_FEEDBACK_BASIC_ID 0x204
#define MAX_CURRENT          10000		//设置电流限制
#define MIN_CURRENT          -10000		//设置电流限制
#define MOTOR_IDE_GROUP1 0x1FF		
#define MOTOR_IDE_GROUP2 0x2FF		
#define ENCODER_RESOLUTION   8191    // 编码器分辨率（0-8191对应0-360度）
#define ANGLE_TO_ENCODER     22.75f  // 8191/360 = 22.75
	
#endif



typedef enum {
    MOTOR_MODE_IDLE = 0,        // 空闲模式，电机停止
    MOTOR_MODE_VELOCITY,        // 速度环控制模式
    MOTOR_MODE_POSITION,        // 位置环控制模式
    MOTOR_MODE_CASCADE_PV,      // 串级控制：位置外环 + 速度内环
}Motor_Mode;

	// PID结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output_max;   // 输出最大值
    float output_min;   // 输出最小值
    float integral_max; // 积分限幅
    uint32_t last_time; // 上一次计算时间
} PID_Controller;
// 电机数据结构体
typedef struct {
    float current_angle;    // 当前角度（度）
    float target_angle;     // 目标角度（度）
		uint16_t current_raw;   // 当前原始编码器值（未转换为角度）
    uint16_t target_raw;    // 目标原始编码器值
		int16_t real_speed;		//实际速度值（RPM）
		int16_t target_speed;	//目标速度值（RPM）
		float degree;						//电机温度
    int16_t output_current; // 设定输出电流值
		int16_t real_current;		// 电机实际电流值
    uint8_t position_reached; // 是否到达目标位置
		uint8_t speed_reached; // 是否到达目标速度
		int16_t speed_error;   // 速度误差
    float position_error;   // 位置误差
    PID_Controller pid;     // 通用PID控制器
		uint8_t ID;							//电调ID
		uint32_t uart_last_time; // 上一次串口发送数据时间
		    // 两个PID控制器：位置环和速度环
    PID_Controller position_pid;  // 位置环PID
    PID_Controller speed_pid;     // 速度环PID
	
		Motor_Mode mode;		//电机控制模式
} Motor_Data;

typedef struct	{
	Motor_Data motor1;
	Motor_Data motor2;
	Motor_Data motor3;
	Motor_Data motor4;
	uint16_t IDE;
}Motor_Group;
//创建电机群


HAL_StatusTypeDef Motor_SetCurrent(CAN_HandleTypeDef *hcan,Motor_Data *Motor_x);
uint8_t Motorx_GetInfo(CAN_HandleTypeDef *hcan,Motor_Data *Motor_x);
void Motor_SendVOFA_Firewater(Motor_Data *Motor_x,CAN_HandleTypeDef *hcan,UART_HandleTypeDef *huart);
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float output_max);
HAL_StatusTypeDef Multiple_Motor_SetCurrent(CAN_HandleTypeDef *hcan,Motor_Group *motor_group);

int16_t Position_PID_Calculate(Motor_Data *motor);
void Motor_SetTargetAngle(Motor_Data *motor, float target_angle_deg);
void PositionLoop_Control(Motor_Data *motor);

// 速度环PID计算函数
int16_t Speed_PID_Calculate(Motor_Data *motor);
// 设置目标速度（RPM）
void Motor_SetTargetSpeed(Motor_Data *motor, int16_t target_speed_rpm);
// 速度环控制函数
void SpeedLoop_Control(Motor_Data *motor);

//VOFA串口接收数据转换成float
float parse_float_from_array(uint8_t *rxdata, uint8_t size);

