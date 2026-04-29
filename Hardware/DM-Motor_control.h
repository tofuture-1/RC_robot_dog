#ifndef __DM_MOTOR_CONTROL_H
#define __DM_MOTOR_CONTROL_H


#include "main.h"
#include "string.h"
#include "math.h"
#include "stdio.h"

#define Disable 0
#define Enable 1

#define P_MAX 12.5f
#define V_MAX 45.0f
#define T_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f



enum mode{
	MIT = 0,
	Position_Speed,
	Speed,
	Force_Position
};

// 电机数据结构体
typedef struct {
		uint8_t able_state;//电机使能状态
		uint8_t ID;							//电调ID
		uint8_t MST_ID;
		uint32_t uart_last_time; // 上一次串口发送数据时间
		uint16_t mode;
		
		float pos;
		float vel;
		float kp;
		float kd;
		float torq;
	//mit用
	
		uint16_t I_des;
		uint16_t V_des_PVT;
	//力位混控用
	
	uint8_t controller_id_r;
	uint8_t error_code_r;
	uint16_t pos_raw_r;
	uint16_t vel_raw_r;
	uint16_t torq_raw_r;
	uint8_t t_mos_r;
	uint8_t t_rotor_r;
	//回传数据
	
	//测试用
	double test1;
	double test2;
} DM_Motor_Data;





HAL_StatusTypeDef DM_Motor_Init(CAN_HandleTypeDef *hcan,DM_Motor_Data *Motorx);
HAL_StatusTypeDef DM_Motor_Send(CAN_HandleTypeDef *hcan,DM_Motor_Data *Motorx);
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
HAL_StatusTypeDef DM_Motorx_GetInfo(CAN_HandleTypeDef *hcan,DM_Motor_Data *Motorx);
void DM_Motor_SendVOFA_Firewater(DM_Motor_Data *Motor_x,CAN_HandleTypeDef *hcan,UART_HandleTypeDef *huart);

#endif




