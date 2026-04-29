#include <dog_control.h>

HAL_StatusTypeDef FootXY_map_MotorAngle(Foot *footx)
{
	
	footx->d_AD = sqrt(footx->X*footx->X + footx->Y*footx->Y);
	
		if (footx->d_AD > Long_L + Short_L || footx->d_AD < fabs(Long_L - Short_L))
	{
    return HAL_ERROR;
	}
	
	//提前，防止除0错误
	
	double a = (Short_L*Short_L + footx->d_AD*footx->d_AD - Long_L*Long_L)/(2.0*Short_L*footx->d_AD);
	
	//适用于atan2恒为负值时
	footx->motor_angle_raw_inside = acos(a) + atan2(footx->Y,footx->X);
	footx->motor_angle_raw_outside = -acos(a) + atan2(footx->Y,footx->X);	
	//计算角度原数据,均以水平x轴正向为基准,后面要考虑越轴-只有外侧电机要考虑？
	
	
	
	
	
	footx->motor_angle_DM_inside =  footx->motor_angle_raw_inside;
	footx->motor_angle_DM_outside = (-Pi - footx->motor_angle_raw_outside);
	
	
	
	
	//限位，不应该直接大跳，保持上一次可行的发送
	if(footx->motor_angle_DM_inside > High_limit)	
		return HAL_ERROR;
	if(footx->motor_angle_DM_inside < Low_limit)	
		return HAL_ERROR;
	if(footx->motor_angle_DM_outside > High_limit)	
		return HAL_ERROR;
	if(footx->motor_angle_DM_outside < Low_limit)	
		return HAL_ERROR;
	
	
	if(footx->L_R_side == Right)
	{
		footx->mi.pos = footx->motor_angle_DM_inside;
		footx->mo.pos = footx->motor_angle_DM_outside;
	}else if(footx->L_R_side == Left)
	{
		footx->mi.pos =  - footx->motor_angle_DM_inside;
		footx->mo.pos =  - footx->motor_angle_DM_outside;
	}else
	{
		return HAL_ERROR;
	}
	//设置限位
	return HAL_OK;
}

HAL_StatusTypeDef Foot_control(Foot *footx,CAN_HandleTypeDef *hcanx)
{
	FootXY_map_MotorAngle(footx);
	DM_Motor_Send(hcanx,&footx->mi);
	DM_Motor_Send(hcanx,&footx->mo);
	
	return HAL_OK;
}

/*
HAL_StatusTypeDef Bluetooth_control(UART_HandleTypeDef *huartx) 
{
			
if (uart1_cmd_ready)
{
    uart1_cmd_ready = 0;

    // 将接收到的int值转换为short（如果需要）
    speed = (int16_t)uart1_rx_value;
    Motor_SetTargetSpeed(&motor1, speed);

    // 构造发送数据包（11字节）：帧头 + 原数据(4+4字节) + 校验和 + 帧尾
    uint8_t tx_buffer[11];
    tx_buffer[0] = 0xA5;                          // 帧头
    // 原数据（小端模式，与接收解析一致）
    tx_buffer[1] = (uint8_t)(uart1_rx_value & 0xFF);？？？
    tx_buffer[2] = (uint8_t)((uart1_rx_value >> 8) & 0xFF);
    tx_buffer[3] = (uint8_t)((uart1_rx_value >> 16) & 0xFF);
    tx_buffer[4] = (uint8_t)((uart1_rx_value >> 24) & 0xFF);
    // 校验和：原数据4字节之和的低8位
    tx_buffer[5] = tx_buffer[1] + tx_buffer[2] + tx_buffer[3] + tx_buffer[4];
    tx_buffer[6] = 0x5A;                          // 帧尾

    HAL_UART_Transmit(huartx, tx_buffer, 7, 100);
}
}
*/

void DM_Motor_FootData_SendVOFA_Firewater(Foot *footx,UART_HandleTypeDef *huart)
{
		uint32_t current_time = HAL_GetTick();
		//设置UART发送缓冲
		char buffer[100];
		int len;
		//将电机回传数据使用串口发送到VOFA，使用firewater协议
		if((current_time - footx->foot_uart_last_time) > 100)
		{
		
		
		len = sprintf(buffer, "%lf,%lf,%lf,%lf,%lf,%lf\n",
			footx->motor_angle_DM_inside,footx->motor_angle_DM_outside,
			footx->motor_angle_raw_inside,footx->motor_angle_raw_outside,
			footx->X,footx->Y);
		HAL_UART_Transmit(huart, (uint8_t *)buffer, len, 1000);
		footx->foot_uart_last_time = HAL_GetTick();
		}
		
		
}
