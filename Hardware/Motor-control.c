#include "Motor-control.h"




/**
*@brief 发送特定电流给GM6020电机--只能控制单个电机-其余会被置0
	*@param hcan:CAN句柄指针,current_value:特定电流
	*@retval 无返回值
	*/
HAL_StatusTypeDef  Motor_SetCurrent(CAN_HandleTypeDef *hcan,Motor_Data *Motor_x)
{
	//定义can通信所需数据
		uint32_t mailbox;
		CAN_TxHeaderTypeDef tx_header;
		uint8_t Txdata[8] = {0};
		
	if(Motor_x->output_current > MAX_CURRENT)
	{
		Motor_x->output_current = MAX_CURRENT;
	}else if(Motor_x->output_current < MIN_CURRENT)
	{
		Motor_x->output_current = MIN_CURRENT;
	}//电流安全限制
	
	//按电调can通信协议处理数据
	uint8_t high_byte = (Motor_x->output_current >> 8) & 0xFF;  // 高8位：0x03
	uint8_t low_byte = Motor_x->output_current & 0xFF;          // 低8位：0xE8

	switch(Motor_x->ID){
		case 1 :
			Txdata[0] = high_byte;
			Txdata[1] = low_byte;
			break;
		case 2 :
			Txdata[2] = high_byte;
			Txdata[3] = low_byte;
			break;
		case 3 :
			Txdata[4] = high_byte;
			Txdata[5] = low_byte;
			break;
		case 4 :
			Txdata[6] = high_byte;
			Txdata[7] = low_byte;
			break;
		case 5 :
			Txdata[0] = high_byte;
			Txdata[1] = low_byte;
			break;
		case 6 :
			Txdata[2] = high_byte;
			Txdata[3] = low_byte;
			break;
		case 7 :
			Txdata[4] = high_byte;
			Txdata[5] = low_byte;
			break;
		case 8 :
			Txdata[6] = high_byte;
			Txdata[7] = low_byte;
			break;
		default:
			memset(Txdata, 0, 8);  // 清零全部 8 字节
			return HAL_ERROR;
			
	}
		
	if(Motor_x->ID > 0 && Motor_x->ID <= 4)
	{

		tx_header.StdId = MOTOR_IDE_GROUP1;
		tx_header.ExtId = 0;
		tx_header.IDE = CAN_ID_STD;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.DLC = 8;
		tx_header.TransmitGlobalTime = DISABLE;
	}
	else if(Motor_x->ID > 4 && Motor_x->ID <= 8)
	{
		tx_header.StdId = MOTOR_IDE_GROUP2;
		tx_header.ExtId = 0;
		tx_header.IDE = CAN_ID_STD;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.DLC = 8;
		tx_header.TransmitGlobalTime = DISABLE;
	}else
	{
		return HAL_ERROR;
	}//判定电调ID并分配CAN发送标识符
	
	//发送数据
	HAL_CAN_AddTxMessage(hcan, &tx_header, Txdata, &mailbox);

	return HAL_OK;
}

// 接收电机回传数据函数
uint8_t Motorx_GetInfo(CAN_HandleTypeDef *hcan,Motor_Data *Motor_x)
{
		//定义接受头及缓冲数组
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
	
    // 先检查FIFO0
    uint32_t fifoLevel0 = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
    for (uint32_t i = 0; i < fifoLevel0; i++) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) 
        {

            
            // 如果收到电机反馈ID，返回成功
            if (rxHeader.StdId == (MOTOR_FEEDBACK_BASIC_ID + Motor_x->ID)) 
            {
															//处理并传输回传数据到对应motor结构体变量中
								Motor_x->current_angle = ((rxData[0]<<8)|rxData[1])*360/8191;
								Motor_x->real_speed = (rxData[2]<<8)|rxData[3];
								Motor_x->real_current = (rxData[4]<<8)|rxData[5];
								Motor_x->degree = rxData[6];
                return 1;
            }
        }
    }
    
    // 再检查FIFO1
    uint32_t fifoLevel1 = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1);
    for (uint32_t i = 0; i < fifoLevel1; i++) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK) 
        {
   
            
            // 如果收到电机反馈ID，返回成功
            if (rxHeader.StdId == (MOTOR_FEEDBACK_BASIC_ID + Motor_x->ID)) 
            {
															//处理并传输回传数据到对应motor结构体变量中
								Motor_x->current_angle = ((rxData[0]<<8)|rxData[1])*360/8191;
								Motor_x->real_speed = (rxData[2]<<8)|rxData[3];
								Motor_x->real_current = (rxData[4]<<8)|rxData[5];
								Motor_x->degree = rxData[6];
                return 1;
            }
        }
    }		
    
    return 0;  // 没有收到目标消息

}




void Motor_SendVOFA_Firewater(Motor_Data *Motor_x,CAN_HandleTypeDef *hcan,UART_HandleTypeDef *huart)
{
		uint32_t current_time = HAL_GetTick();
		//设置UART发送缓冲
		char buffer[100];
		int len;
		//将电机回传数据使用串口发送到VOFA，使用firewater协议
		if((current_time - Motor_x->uart_last_time) > 100)
		{
		Motorx_GetInfo(hcan,Motor_x);
		
		len = sprintf(buffer, "%.1f,%d,%d,%.1f\n",Motor_x->current_angle,Motor_x->real_speed,Motor_x->real_current,Motor_x->degree);
		HAL_UART_Transmit(huart, (uint8_t *)buffer, len, 1000);
		Motor_x->uart_last_time = HAL_GetTick();
		}
}


// 初始化电机PID控制器-单环通用
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float output_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_max = output_max;
    pid->output_min = -output_max;
    pid->integral_max = output_max * 2; // 积分限幅为输出的2倍
    pid->last_time = HAL_GetTick();
}


// 速度环PID计算函数
int16_t Speed_PID_Calculate(Motor_Data *motor)
{
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - motor->pid.last_time) / 1000.0f; // 转换为秒
    if (dt <= 0) dt = 0.01f; // 最小时间间隔
    
    float error = motor->target_speed - motor->real_speed;
    
    // 比例项
    float proportional = motor->pid.Kp * error;
    
    // 积分项（带抗饱和）
    motor->pid.integral += motor->pid.Ki * error * dt;
    
    // 积分限幅
    if (motor->pid.integral > motor->pid.integral_max)
        motor->pid.integral = motor->pid.integral_max;
    else if (motor->pid.integral < -motor->pid.integral_max)
        motor->pid.integral = -motor->pid.integral_max;
    
    // 微分项
    float derivative = motor->pid.Kd * (error - motor->pid.prev_error) / dt;
    
    // 计算输出
    float output = proportional + motor->pid.integral + derivative;
    
    // 输出限幅
    if (output > motor->pid.output_max)
        output = motor->pid.output_max;
    else if (output < motor->pid.output_min)
        output = motor->pid.output_min;
    
    // 更新状态
    motor->pid.prev_error = error;
    motor->pid.last_time = current_time;
    
    return (int16_t)output;
}

// 设置目标速度（RPM）
void Motor_SetTargetSpeed(Motor_Data *motor, int16_t target_speed_rpm)
{
		motor->target_speed = target_speed_rpm;
    motor->speed_reached = 0;//表示false
}

// 速度环控制函数
void SpeedLoop_Control(Motor_Data *motor)
{
    // 计算速度误差
    int16_t error = motor->target_speed - motor->real_speed;
    motor->speed_error = error;
	
        // 检查是否超速需要制动
    int16_t overshoot_threshold = 500; // 超速阈值
    
    if (motor->target_speed >= 0) {
        // 正转方向
        if (motor->real_speed > motor->target_speed + overshoot_threshold) {
            // 显著超速，应用主动制动
            motor->output_current = -300; // 固定制动电流
            motor->speed_reached = 0;
            return; // 直接返回，跳过PID计算
        }
    } else {
        // 反转方向
        if (motor->real_speed < motor->target_speed - overshoot_threshold) {
            // 显著超速，应用主动制动
            motor->output_current = 300; // 固定制动电流
            motor->speed_reached = 0;
            return;
        }
    }
    // 计算PID输出
    motor->output_current = Speed_PID_Calculate(motor);
    
}


// 位置环PID计算函数
int16_t Position_PID_Calculate(Motor_Data *motor)
{
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - motor->pid.last_time) / 1000.0f; // 转换为秒
    if (dt <= 0) dt = 0.01f; // 最小时间间隔
    
    float error = motor->target_angle - motor->current_angle;
    
    // 比例项
    float proportional = motor->pid.Kp * error;
    
    // 积分项（带抗饱和）
    motor->pid.integral += motor->pid.Ki * error * dt;
    
    // 积分限幅
    if (motor->pid.integral > motor->pid.integral_max)
        motor->pid.integral = motor->pid.integral_max;
    else if (motor->pid.integral < -motor->pid.integral_max)
        motor->pid.integral = -motor->pid.integral_max;
    
    // 微分项
    float derivative = motor->pid.Kd * (error - motor->pid.prev_error) / dt;
    
    // 计算输出
    float output = proportional + motor->pid.integral + derivative;
    
    // 输出限幅
    if (output > motor->pid.output_max)
        output = motor->pid.output_max;
    else if (output < motor->pid.output_min)
        output = motor->pid.output_min;
    
    // 更新状态
    motor->pid.prev_error = error;
    motor->pid.last_time = current_time;
    
    return (int16_t)output;
}



// 设置目标位置（角度）
void Motor_SetTargetAngle(Motor_Data *motor, float target_angle_deg)
{
    // 将目标角度归一化到0-360度
    while (target_angle_deg >= 360.0f)
        target_angle_deg -= 360.0f;
    while (target_angle_deg < 0)
        target_angle_deg += 360.0f;
    
    motor->target_angle = target_angle_deg;
    motor->target_raw = (uint16_t)(target_angle_deg * ANGLE_TO_ENCODER);
    motor->position_reached = 0;//表示false
}

// 位置环控制函数
void PositionLoop_Control(Motor_Data *motor)
{
    // 计算位置误差（考虑角度循环特性）
    float error = motor->target_angle - motor->current_angle;
    
    // 处理角度循环（找到最短路径）
    if (error > 180.0f)
        error -= 360.0f;
    else if (error < -180.0f)
        error += 360.0f;

    
    motor->position_error = error;
    
    // 计算PID输出（将角度误差转换为电流）
    // 注意：这里的PID参数需要根据实际系统调整
    motor->output_current = Position_PID_Calculate(motor);
    
    // 检查是否到达目标位置（误差小于阈值）
    if (fabsf(error) < 5.0f) // 5度误差范围内
    {
				motor->position_reached = 1;//表示true
        motor->output_current = 0; // 到达后停止输出
    }
}

//VOFA+、串口-需要约定发送协议-发送数据解析

float parse_float_from_array(uint8_t *rxdata, uint8_t size)
{
    char str_buffer[16] = {0};
    uint8_t str_index = 0;
    uint8_t dot_count = 0;
    uint8_t sign_pos = 0;
    
    for(uint8_t i = 0; i < size; i++)
    {
        uint8_t ch = rxdata[i];
        
        // 遇到换行符或字符串结束符就停止
        if(ch == '\n' || ch == '\r' || ch == '\0')
            break;
            
        // 只接受数字、小数点、负号
        if(ch == '-')
        {
            if(str_index > 0) break; // 负号只能在开头
            str_buffer[str_index++] = ch;
            sign_pos = str_index;
        }
        else if(ch == '.')
        {
            if(dot_count > 0) break; // 只能有一个小数点
            str_buffer[str_index++] = ch;
            dot_count++;
        }
        else if(ch >= '0' && ch <= '9')
        {
            str_buffer[str_index++] = ch;
        }
        else
        {
            // 遇到非法字符，停止解析
            break;
        }
        
        if(str_index >= sizeof(str_buffer) - 1)
            break;
    }
    
    // 检查是否有有效数字
    if(str_index == 0 || 
       (str_index == 1 && str_buffer[0] == '-') ||
       (str_index == 1 && str_buffer[0] == '.'))
    {
        return 0.0f;
    }
    
    str_buffer[str_index] = '\0';
    
    float result = 0.0f;
    if(sscanf(str_buffer, "%f", &result) != 1)
        return 0.0f;
    
    return result;
}

HAL_StatusTypeDef Multiple_Motor_SetCurrent(CAN_HandleTypeDef *hcan,Motor_Group *motor_group)
{
	//定义can通信所需数据
		uint32_t mailbox;
		CAN_TxHeaderTypeDef tx_header;
		uint8_t Txdata[8] = {0};
	//motor1
	if(motor_group->motor1.output_current > MAX_CURRENT)
	{
		motor_group->motor1.output_current = MAX_CURRENT;
	}else if(motor_group->motor1.output_current < MIN_CURRENT)
	{
		motor_group->motor1.output_current = MIN_CURRENT;
	}
	//motor2
	if(motor_group->motor2.output_current > MAX_CURRENT)
	{
		motor_group->motor2.output_current = MAX_CURRENT;
	}else if(motor_group->motor2.output_current < MIN_CURRENT)
	{
		motor_group->motor2.output_current = MIN_CURRENT;
	}
	//motor3
	if(motor_group->motor3.output_current > MAX_CURRENT)
	{
		motor_group->motor3.output_current = MAX_CURRENT;
	}else if(motor_group->motor3.output_current < MIN_CURRENT)
	{
		motor_group->motor3.output_current = MIN_CURRENT;
	}
	//motor4
	if(motor_group->motor4.output_current > MAX_CURRENT)
	{
		motor_group->motor4.output_current = MAX_CURRENT;
	}else if(motor_group->motor4.output_current < MIN_CURRENT)
	{
		motor_group->motor4.output_current = MIN_CURRENT;
	}	
	//对四个电机进行电流安全限制
	
	//按电调can通信协议处理数据
		uint8_t high_byte = (motor_group->motor1.output_current >> 8) & 0xFF;  // 高8位：0x03
		uint8_t low_byte = motor_group->motor1.output_current & 0xFF;          // 低8位：0xE8
		Txdata[0] = high_byte;
		Txdata[1] = low_byte;

	
		high_byte = (motor_group->motor2.output_current >> 8) & 0xFF;  // 高8位：0x03
		low_byte = motor_group->motor2.output_current & 0xFF;          // 低8位：0xE8
		Txdata[2] = high_byte;
		Txdata[3] = low_byte;

		high_byte = (motor_group->motor3.output_current >> 8) & 0xFF;  // 高8位：0x03
		low_byte = motor_group->motor3.output_current & 0xFF;          // 低8位：0xE8
		Txdata[4] = high_byte;
		Txdata[5] = low_byte;

		high_byte = (motor_group->motor4.output_current >> 8) & 0xFF;  // 高8位：0x03
		low_byte = motor_group->motor4.output_current & 0xFF;          // 低8位：0xE8
		Txdata[6] = high_byte;
		Txdata[7] = low_byte;



		
	if(motor_group->IDE == MOTOR_IDE_GROUP1 || motor_group->IDE == MOTOR_IDE_GROUP2)
	{

		tx_header.StdId = motor_group->IDE;
		tx_header.ExtId = 0;
		tx_header.IDE = CAN_ID_STD;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.DLC = 8;
		tx_header.TransmitGlobalTime = DISABLE;
	}else
	{
		return HAL_ERROR;
	}
		
	//判定电调ID并分配CAN发送标识符
	
	//发送数据
	HAL_CAN_AddTxMessage(hcan, &tx_header, Txdata, &mailbox);

	return HAL_OK;
}


