//
// Created by 24158 on 2026/6/18.
//

#ifndef RC_ROBOTDOG_REBUILD20260605_DRV_TIM_H
#define RC_ROBOTDOG_REBUILD20260605_DRV_TIM_H

#include "main.h"
#include "tim.h"
/**
 * @brief TIM定时器回调函数数据类型
 *
 */
typedef void (*TIM_Call_Back)();

/**
 * @brief TIM定时器处理结构体
 *
 */
typedef struct
{
    TIM_HandleTypeDef *TIM_Handler;
    TIM_Call_Back Callback_Function;
} Struct_TIM_Manage_Object;

extern bool init_finished;


extern Struct_TIM_Manage_Object TIM1_Manage_Object;
extern Struct_TIM_Manage_Object TIM2_Manage_Object;
extern Struct_TIM_Manage_Object TIM3_Manage_Object;
extern Struct_TIM_Manage_Object TIM4_Manage_Object;
extern Struct_TIM_Manage_Object TIM5_Manage_Object;
extern Struct_TIM_Manage_Object TIM6_Manage_Object;
extern Struct_TIM_Manage_Object TIM7_Manage_Object;
extern Struct_TIM_Manage_Object TIM8_Manage_Object;
extern Struct_TIM_Manage_Object TIM12_Manage_Object;
extern Struct_TIM_Manage_Object TIM13_Manage_Object;
extern Struct_TIM_Manage_Object TIM14_Manage_Object;
extern Struct_TIM_Manage_Object TIM15_Manage_Object;
extern Struct_TIM_Manage_Object TIM16_Manage_Object;
extern Struct_TIM_Manage_Object TIM17_Manage_Object;
extern Struct_TIM_Manage_Object TIM23_Manage_Object;
extern Struct_TIM_Manage_Object TIM24_Manage_Object;
/* Exported function declarations --------------------------------------------*/

void TIM_Init(TIM_HandleTypeDef *htim, TIM_Call_Back Callback_Function);

#endif //RC_ROBOTDOG_REBUILD20260605_DRV_TIM_H
