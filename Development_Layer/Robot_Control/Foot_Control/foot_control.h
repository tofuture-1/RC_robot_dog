//
// Created by 24158 on 2026/6/15.
//

#ifndef RC_ROBOTDOG_REBUILD20260605_FOOT_CONTROL_H
#define RC_ROBOTDOG_REBUILD20260605_FOOT_CONTROL_H

#include "Motor_DM.h"


#define Long_L 0.1275
#define Short_L 0.085

#define Pi 3.14159
#define Low_limit (-1.10)//低位软件主动杆限位（未考虑从动杆与下限位干涉）
#define High_limit 0.80//高位软件主动杆限位，注意应用在实际比较中需要考虑实际大小关系；dm电机逆时针旋转角度值减少
//#define Map_unit 1.8325957 // Pi*105/180
#define Right 1
#define Left 0



/**
 * @brief RC-Dog单腿控制类
 *
 *
 *
 */
class Class_SingleDogFoot_DM{
public:
    void SingleFoot_Init(CAN_HandleTypeDef *Mo_hcan, uint16_t Mo_CAN_Rx_ID,uint16_t  Mo_CAN_Tx_ID,
    CAN_HandleTypeDef *Mi_hcan,uint16_t Mi_CAN_Rx_ID, uint16_t Mi_CAN_Tx_ID,uint8_t L_R_sidex,
    float kp, float kd, float Angle = 0.0f,float Omega = 0.0f,float Torque = 0.0f);

    int8_t FootXY_map_MotorAngle();
    HAL_StatusTypeDef Foot_control();
    void DM_Motor_FootData_SendVOFA_Firewater(UART_HandleTypeDef *huart);

    Class_Motor_DM_Normal mi;
    Class_Motor_DM_Normal mo;



    inline void Set_x_y(float x_new, float y_new);
protected:
    float X;
    float Y;
    float d_AD;
    uint8_t L_R_side;
    // //！！！注意一定要设置准确的0点，以主动杆水平时为0
    float motor_angle_raw_inside;//内侧电机用
    float motor_angle_raw_outside;

    //映射至达妙//外侧电机以水平x轴负向为基准
    float motor_angle_DM_inside = 0;
    float motor_angle_DM_outside = 0;
    float pre_motor_angle_DM_inside = 0;
    float pre_motor_angle_DM_outside = 0;

    uint32_t foot_uart_last_time;


    CAN_HandleTypeDef *mi_hcan;
    uint16_t mi_CAN_Rx_ID;
    uint16_t mi_CAN_Tx_ID;

    CAN_HandleTypeDef *mo_hcan;
    uint16_t mo_CAN_Rx_ID;
    uint16_t mo_CAN_Tx_ID;
};

inline void Class_SingleDogFoot_DM::Set_x_y(float x_new, float y_new)
{
    X = x_new;
    Y = y_new;
}




#endif //RC_ROBOTDOG_REBUILD20260605_FOOT_CONTROL_H
