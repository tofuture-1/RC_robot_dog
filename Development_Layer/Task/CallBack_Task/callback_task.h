//
// Created by 24158 on 2026/6/16.
//

#ifndef RC_ROBOTDOG_REBUILD20260605_CALLBACK_TASK_H
#define RC_ROBOTDOG_REBUILD20260605_CALLBACK_TASK_H

#include "drv_can.h"
#include "drv_uart.h"
#include "drv_tim.h"

#ifdef __cplusplus
#include "robotdog_class.h"
#endif


//区分C和C++编译器，C++编译器会定义__cplusplus宏
#ifdef __cplusplus
extern "C" {    //告诉C++编译器，内部代码按C规则编译
#endif


void Robot_Init();
void Tim6_Robot_1ms_Callback();
void CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage);
void CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage);
void UART1_Callback(Struct_UART_Rx_Buffer *UART_RxMessage);
#ifdef __cplusplus
}//结束extern "C"
#endif

#endif //RC_ROBOTDOG_REBUILD20260605_CALLBACK_TASK_H
