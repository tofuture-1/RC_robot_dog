#include "DM-Motor_control.h"

HAL_StatusTypeDef DM_Motor_Init(CAN_HandleTypeDef *hcan,DM_Motor_Data *Motorx)
{
	uint16_t base_id;
	switch(Motorx->mode) {
			case MIT:
					base_id = 0;
					break;
			case Position_Speed:
					base_id = 0x100;
					break;
			case Speed:
					base_id = 0x200;
					break;
			case Force_Position:
					base_id = 0x300;
					break;
			default:
					// 建议添加默认处理，例如错误处理或默认值
					base_id = 0;  // 或其他合适的默认值
					break;
	}
	
		uint32_t mailbox;
		CAN_TxHeaderTypeDef tx_header;
		tx_header.StdId = base_id + Motorx->ID;
		tx_header.ExtId = 0;
		tx_header.IDE = CAN_ID_STD;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.DLC = 8;
		tx_header.TransmitGlobalTime = DISABLE;
		
		if(Motorx->able_state == Enable)
		{
		uint8_t enable[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc};
		HAL_CAN_AddTxMessage(hcan, &tx_header, enable, &mailbox);
		//电机使能
		}else if(Motorx->able_state == Disable)
		{
			uint8_t disable[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd};
			HAL_CAN_AddTxMessage(hcan, &tx_header, disable, &mailbox);
		}else
		{
			return HAL_ERROR;
		}
		
		return HAL_OK;
}

HAL_StatusTypeDef DM_Motor_Send(CAN_HandleTypeDef *hcan,DM_Motor_Data *Motorx)
{
	uint16_t base_id;
	switch(Motorx->mode) {
			case MIT:
					base_id = 0;
					break;
			case Position_Speed:
					base_id = 0x100;
					break;
			case Speed:
					base_id = 0x200;
					break;
			case Force_Position:
					base_id = 0x300;
					break;
			default:
					// 建议添加默认处理，例如错误处理或默认值
					base_id = 0;  // 或其他合适的默认值
					break;
	}
	
		uint32_t mailbox;
		CAN_TxHeaderTypeDef tx_header;
		tx_header.StdId = base_id + Motorx->ID;
		tx_header.ExtId = 0;
		tx_header.IDE = CAN_ID_STD;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.DLC = 8;
		tx_header.TransmitGlobalTime = DISABLE;
		
	if(Motorx->mode == MIT)
	{
		uint8_t data[8];
		uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;


		//将浮点数据等比例转换成整数
		pos_tmp = float_to_uint(Motorx->pos, -P_MAX, P_MAX, 16);//（-12.5~12.5）
		vel_tmp = float_to_uint(Motorx->vel, -V_MAX, V_MAX, 12);//（-45.0~45.0）
		kp_tmp = float_to_uint(Motorx->kp, KP_MIN, KP_MAX, 12);//（0.0~500.0）
		kd_tmp = float_to_uint(Motorx->kd, KD_MIN, KD_MAX, 12);//（0.0~5.0）
		tor_tmp = float_to_uint(Motorx->torq, -T_MAX, T_MAX, 12);//（-50.0~50.0）
		
		data[0] = (pos_tmp >> 8);
		data[1] = pos_tmp;
		data[2] = (vel_tmp >> 4);
		data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
		data[4] = kp_tmp;
		data[5] = (kd_tmp >> 4);
		data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
		data[7] = tor_tmp;
		
		HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
	}else if(Motorx->mode == Position_Speed)
	{
		uint8_t data[8];

		memcpy(data, &Motorx->pos, sizeof(Motorx->pos));
		memcpy(data + 4, &Motorx->vel, sizeof(Motorx->vel));


		HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
	}else if(Motorx->mode == Speed)
	{
		uint8_t data[8];

		memcpy(data, &Motorx->vel, sizeof(Motorx->vel));
		
		HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
	}else if(Motorx->mode == Force_Position)
	{
		uint8_t data[8];
		memcpy(data, &Motorx->pos, sizeof(Motorx->pos));
		memcpy(data + 4, &Motorx->V_des_PVT, sizeof(Motorx->V_des_PVT));
		memcpy(data+6, &Motorx->I_des, sizeof(Motorx->I_des));
		
		HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
	}else
	{
		return HAL_ERROR;
	}
	
		return HAL_OK;
}


/**
 * @brief 将浮点数线性映射到无符号整数
 * @param x         输入浮点数
 * @param x_min     浮点数最小值
 * @param x_max     浮点数最大值
 * @param bits      输出整数位数（如 16 表示 -32768 ~ 32767，12 表示 -2048 ~ 2047）
 * @return int16_t  映射后的有符号整数值
 */
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    int16_t int_min = 0;
    uint16_t int_max = (1 << bits) - 1;

    // 限制输入范围
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;

    // 线性映射
    float ratio = (x - x_min) / (x_max - x_min);
    float uint_float = ratio * (int_max - int_min) + int_min;

    return (uint16_t)roundf(uint_float);   // 四舍五入
}

// 接收电机回传数据函数
HAL_StatusTypeDef DM_Motorx_GetInfo(CAN_HandleTypeDef *hcan,DM_Motor_Data *Motorx)
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
            if (rxHeader.StdId == (Motorx->MST_ID)) 
            {
							Motorx->controller_id_r = rxData[0] & 0x0F;       // 低4位为控制器ID
							Motorx->error_code_r = (rxData[0] >> 4) & 0x0F;   // 高4位为故障码ERR

							// 2. 提取16位有符号位置值 (POS)
							Motorx->pos_raw_r = (rxData[1] << 8) | rxData[2];  // 组合成16位有符号整数
							// 3. 提取12位有符号速度值 (VEL) 和高12位有符号扭矩值 (T)
							Motorx->vel_raw_r = ((rxData[3] << 4) | ((rxData[4] >> 4) & 0x0F)); // 12位速度
							Motorx->torq_raw_r = (((rxData[4] & 0x0F) << 8) | rxData[5]);        // 12位扭矩
							// 注意：vel_raw和torq_raw是12位数，但存储在16位变量中，其最高位（符号位）在第11位。

							// 4. 提取温度值
							Motorx->t_mos_r = rxData[6]; // MOS管平均温度，单位℃
							Motorx->t_rotor_r = rxData[7]; // 电机线圈平均温度，单位℃
							return HAL_OK;
            }
        }
    }
    
    // 再检查FIFO1
    uint32_t fifoLevel1 = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1);
    for (uint32_t i = 0; i < fifoLevel1; i++) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK) 
        {
   
            
            // 如果收到电机反馈ID，返回成功
            if (rxHeader.StdId == (Motorx->MST_ID)) 
            {
							Motorx->controller_id_r = rxData[0] & 0x0F;       // 低4位为控制器ID
							Motorx->error_code_r = (rxData[0] >> 4) & 0x0F;   // 高4位为故障码ERR

							// 2. 提取16位有符号位置值 (POS)
							Motorx->pos_raw_r = (rxData[1] << 8) | rxData[2];  // 组合成16位有符号整数
							// 3. 提取12位有符号速度值 (VEL) 和高12位有符号扭矩值 (T)
							Motorx->vel_raw_r = ((rxData[3] << 4) | ((rxData[4] >> 4) & 0x0F)); // 12位速度
							Motorx->torq_raw_r = (((rxData[4] & 0x0F) << 8) | rxData[5]);        // 12位扭矩
							// 注意：vel_raw和torq_raw是12位数，但存储在16位变量中，其最高位（符号位）在第11位。

							// 4. 提取温度值
							Motorx->t_mos_r = rxData[6]; // MOS管平均温度，单位℃
							Motorx->t_rotor_r = rxData[7]; // 电机线圈平均温度，单位℃
							return HAL_OK;
            }
        }
    }		
    
    return HAL_ERROR;  // 没有收到目标消息

}




void DM_Motor_SendVOFA_Firewater(DM_Motor_Data *Motorx,CAN_HandleTypeDef *hcan,UART_HandleTypeDef *huart)
{
		uint32_t current_time = HAL_GetTick();
		//设置UART发送缓冲
		char buffer[100];
		int len;
		//将电机回传数据使用串口发送到VOFA，使用firewater协议
		if((current_time - Motorx->uart_last_time) > 100)
		{
		DM_Motorx_GetInfo(hcan,Motorx);
		
		len = sprintf(buffer, "%d,%d,%d,%d,%d,%d\n",Motorx->pos_raw_r,Motorx->vel_raw_r,Motorx->error_code_r,Motorx->torq_raw_r,Motorx->t_mos_r,Motorx->t_rotor_r);
		HAL_UART_Transmit(huart, (uint8_t *)buffer, len, 1000);
		Motorx->uart_last_time = HAL_GetTick();
		}
}




