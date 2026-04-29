#include "CascadePID_control.h"
#include <math.h>

// 通用PID计算函数
float PID_Calculate(PID_Controller *pid, float error, float dt)
{
    // 比例项
    float proportional = pid->Kp * error;
    
    // 积分项（带抗饱和）
    pid->integral += pid->Ki * error * dt;
    
    // 积分限幅
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;
    
    // 微分项
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    
    // 计算输出
    float output = proportional + pid->integral + derivative;
    
    // 输出限幅
    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;
    
    // 更新状态
    pid->prev_error = error;
    
    return output;
}

// 初始化串级PID控制器参数
void CascadePID_Init(Motor_Data *motor)
{
    // 位置环PID参数（输出为目标速度）
    // 注意：位置环的输出是速度，所以output_max是最大速度限制（单位：RPM）
    PID_Init(&motor->position_pid, 8.0f, 0.5f, 0.0f, 500.0f); // 位置环
    
    // 速度环PID参数（输出为电流）
    // 注意：速度环的输出是电流，所以output_max是最大电流限制
    PID_Init(&motor->speed_pid, 6.5f, 0.1f, 0.01f, 1000.0f); // 速度环
}

// 串级PID控制主函数
void Position_Speed_Cascade_Control(Motor_Data *motor)
{
    static uint32_t last_cascade_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_cascade_time) / 1000.0f; // 转换为秒
    
    if (dt <= 0) dt = 0.001f; // 最小时间间隔1ms
    
    // ========== 第一层：位置环（外环） ==========
    // 计算位置误差（考虑角度循环）
    float position_error = motor->target_angle - motor->current_angle;
    
    // 处理角度循环，找到最短路径
    if (position_error > 180.0f)
        position_error -= 360.0f;
    else if (position_error < -180.0f)
        position_error += 360.0f;
    
    motor->position_error = position_error;
    
    // 位置环PID计算，输出为目标速度
    float target_speed_from_position = PID_Calculate(&motor->position_pid, position_error, dt);
    
    // 位置环限速（可配置最大速度）
    float max_position_speed = 500.0f; // RPM
    if (target_speed_from_position > max_position_speed)
        target_speed_from_position = max_position_speed;
    else if (target_speed_from_position < -max_position_speed)
        target_speed_from_position = -max_position_speed;
    
    // 位置接近目标时降低速度（梯形速度规划）
    float distance_to_target = fabsf(position_error);
    float deceleration_distance = 30.0f; // 开始减速的距离（度）
    
    if (distance_to_target < deceleration_distance)
    {
        // 线性减速
        float speed_factor = distance_to_target / deceleration_distance;
        target_speed_from_position *= speed_factor;
        
        // 确保最小速度
        float min_speed = 10.0f; // 最小速度（RPM）
        if (fabsf(target_speed_from_position) < min_speed)
        {
            target_speed_from_position = (target_speed_from_position >= 0) ? min_speed : -min_speed;
        }
    }
    
    // 设置速度环的目标速度
    motor->target_speed = target_speed_from_position;
    
    // ========== 第二层：速度环（内环） ==========
    // 计算速度误差
    float speed_error = motor->target_speed - motor->real_speed;
    motor->speed_error = speed_error;
    
    // 速度环PID计算，输出为电流
    float output_current = PID_Calculate(&motor->speed_pid, speed_error, dt);
    
    // 安全限制
    if (output_current > MAX_CURRENT)
        output_current = MAX_CURRENT;
    else if (output_current < MIN_CURRENT)
        output_current = MIN_CURRENT;
    
    // 设置输出电流
    motor->output_current = (int16_t)output_current;
    
    // ========== 位置到达判断 ==========
    float position_tolerance = 0.5f; // 位置容差（度）
    float speed_tolerance = 5.0f;    // 速度容差（RPM）
    
    if (fabsf(position_error) < position_tolerance && abs(motor->real_speed) < speed_tolerance)
    {
        motor->position_reached = 1;
        motor->output_current = 0; // 到达后停止输出
    }
    else
    {
        motor->position_reached = 0;
    }
    
    // 更新上次计算时间
    last_cascade_time = current_time;
}

