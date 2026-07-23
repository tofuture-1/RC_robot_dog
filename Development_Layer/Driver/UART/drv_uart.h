//
// Created by 24158 on 2026/6/17.
//

#ifndef RC_ROBOTDOG_REBUILD20260605_DRV_UART_H
#define RC_ROBOTDOG_REBUILD20260605_DRV_UART_H

#include "main.h"
#include "usart.h"

/**
 * @brief UART 接收缓存结构体，用于保存最近一次接收到的数据。
 */
typedef struct
{
    uint8_t Data[32];    // IBUS 帧固定 32 字节
    uint16_t Length;     // 实际接收长度
} Struct_UART_Rx_Buffer;

/**
 * @brief UART 接收回调函数类型，在 DMA 接收完成后通知上层处理。
 * @param rx_buffer 指向接收缓存结构体的指针。
 */
typedef void (*UART_Call_Back)(Struct_UART_Rx_Buffer *rx_buffer);

/**
 * @brief UART 管理对象，统一维护句柄、接收缓存和回调函数。
 */
typedef struct
{
    UART_HandleTypeDef *UART_Handler;
    Struct_UART_Rx_Buffer Rx_Buffer;
    UART_Call_Back Callback_Function;
} Struct_UART_Manage_Object;

extern Struct_UART_Manage_Object UART6_Manage_Object;

/**
 * @brief 初始化指定的 UART 外设，开启 DMA 接收并注册回调函数。
 * @param huart  UART 句柄指针
 * @param Callback_Function 接收完成后的回调函数指针
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function);

#endif //RC_ROBOTDOG_REBUILD20260605_DRV_UART_H
