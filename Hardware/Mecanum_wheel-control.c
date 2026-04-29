#include "Mecanum_wheel-control.h"



//控制每个电机速度-未解算
void Drive_mutiple_motor_speed(CAN_HandleTypeDef *hcan, Motor_Group *motor_group, int16_t speed1, int16_t speed2, int16_t speed3, int16_t speed4)
{
	PID_Init(&motor_group->motor1.pid, 5.5f, 0.1f, 0.01f,1000);
	PID_Init(&motor_group->motor2.pid, 6.5f, 0.5f, 0.01f,1000);
	PID_Init(&motor_group->motor3.pid, 5.5f, 0.1f, 0.01f,1000);
	PID_Init(&motor_group->motor4.pid, 5.0f, 0.1f, 0.01f,1000);
	
	Motor_SetTargetSpeed(&motor_group->motor1,speed1);
	Motor_SetTargetSpeed(&motor_group->motor2,speed2);
	Motor_SetTargetSpeed(&motor_group->motor3,speed3);
	Motor_SetTargetSpeed(&motor_group->motor4,speed4);
	
	//四个电机速度环控制
	Motorx_GetInfo(hcan,&motor_group->motor1);
	SpeedLoop_Control(&motor_group->motor1);
	
	Motorx_GetInfo(hcan,&motor_group->motor2);
	SpeedLoop_Control(&motor_group->motor2);
	
	Motorx_GetInfo(hcan,&motor_group->motor3);
	SpeedLoop_Control(&motor_group->motor3);
	
	Motorx_GetInfo(hcan,&motor_group->motor4);
	SpeedLoop_Control(&motor_group->motor4);
	
	//发送电流值
	Multiple_Motor_SetCurrent(hcan,motor_group);
	
}

//运动模式切换-未进行RPM对应M/s
//电机位置规定
//左上1，右上2
//左下3，右下4
//单电机正速度为逆时针转
void Mecanum_Move(CAN_HandleTypeDef *hcan,Motor_Group *motor_group, char direction ,int16_t speed)
{
	if(direction == 'W')
	{
		Drive_mutiple_motor_speed(hcan,motor_group,speed,-speed,speed,-speed);
	}else if(direction == 'S')
	{
		Drive_mutiple_motor_speed(hcan,motor_group,-speed,speed,-speed,speed);
	}else if(direction =='A')
	{
		Drive_mutiple_motor_speed(hcan,motor_group,-speed,-speed,speed,speed);
	}else if(direction == 'D')
	{
		Drive_mutiple_motor_speed(hcan,motor_group,speed,speed,-speed,-speed);
	}else if(direction =='R')
	{
		Drive_mutiple_motor_speed(hcan,motor_group,speed,speed,speed,speed);
	}else if(direction == 'L')
	{
		Drive_mutiple_motor_speed(hcan,motor_group,-speed,-speed,-speed,-speed);
	}else if(direction == 'T')
	{
		Drive_mutiple_motor_speed(hcan,motor_group,0,0,0,0);
	}else
	{
		Drive_mutiple_motor_speed(hcan,motor_group,0,0,0,0);
	}
	
	
	
}



