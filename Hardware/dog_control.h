#ifndef __DOG_CONTROL_H
#define __DOG_CONTROL_H

#include <DM-Motor_control.h>
#define Long_L 0.1275
#define Short_L 0.085

#define Pi 3.1415926
#define Low_limit -1.0
#define High_limit 0.8
//#define Map_unit 1.8325957 // Pi*105/180
#define Right 1
#define Left 0

//！！！注意一定要设置准确的0点，以主动杆水平时为0
typedef struct{
	double X;
	double Y;
	double d_AD;
	uint8_t L_R_side;
	
	double motor_angle_raw_inside;//内侧电机用
	double motor_angle_raw_outside;
	
	//映射至达妙//外侧电机以水平x轴负向为基准
	double motor_angle_DM_inside;
	double motor_angle_DM_outside;
	
	uint32_t foot_uart_last_time;
	
	//需要设置绝对值防止超？
	DM_Motor_Data mi;
	DM_Motor_Data mo;
}Foot;

//Uart1串口相关




	

HAL_StatusTypeDef FootXY_map_MotorAngle(Foot *footx);
HAL_StatusTypeDef Foot_control(Foot *footx,CAN_HandleTypeDef *hcanx);
void DM_Motor_FootData_SendVOFA_Firewater(Foot *footx,UART_HandleTypeDef *huart);
#endif

