//
// Created by 24158 on 2026/6/17.
//

#include "drv_uart.h"
#include <string.h>

Struct_UART_Manage_Object UART6_Manage_Object = {0};
Struct_UART_Manage_Object UART1_Manage_Object = {0};
// 帧同步缓冲区
static uint8_t uart1_frame_buf[32];
static uint8_t uart1_rx_byte = 0;

/**
 * @brief 初始化指定的 UART 外设，开启逐字节中断接收并注册回调函数。
 * @param huart  UART 句柄指针
 * @param Callback_Function 接收完成后的回调函数指针
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function)
{
    Struct_UART_Manage_Object *manage_object = nullptr;

    if (huart->Instance == USART6)
    {
        manage_object = &UART6_Manage_Object;
    }else if (huart->Instance == USART1)
    {
        manage_object = &UART1_Manage_Object;
    }
    else
    {
        return;
    }

    manage_object->UART_Handler = huart;
    manage_object->Callback_Function = Callback_Function;

    // 开始逐字节中断接收
    HAL_UART_Receive_IT(huart, &uart1_rx_byte, 1);
}

/**
 * @brief UART 中断接收回调函数。
 *        逐字节接收，同步到 IBUS 帧头 0x20 0x40 后接收完整 32 字节帧。
 */
extern "C"
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *manage_object = nullptr;

    if (huart->Instance == USART1)
    {
        manage_object = &UART1_Manage_Object;
    }else if (huart->Instance == USART6)
    {
        manage_object = &UART6_Manage_Object;
    }
    else
    {
        return;
    }

    // 帧同步状态机
    static uint8_t sync_state = 0;

    switch (sync_state)
    {
    case 0: // 等待帧头第一个字节 0x20
        sync_state = 0;
        if (uart1_rx_byte == 0x20)
        {
            uart1_frame_buf[0] = uart1_rx_byte;
            sync_state = 1;
        }
        HAL_UART_Receive_IT(huart, &uart1_rx_byte, 1);
        break;

    case 1: // 等待帧头第二个字节 0x40
        sync_state = 0;
        if (uart1_rx_byte == 0x40)
        {
            uart1_frame_buf[1] = uart1_rx_byte;
            sync_state = 2;
            // 帧头同步成功，批量接收剩余 30 字节
            HAL_UART_Receive_IT(huart, uart1_frame_buf + 2, 30);
        }
        else
        {
            HAL_UART_Receive_IT(huart, &uart1_rx_byte, 1);
        }
        break;

    case 2: // 已收到完整 32 字节帧
        sync_state = 0;

        // 复制到管理对象的接收缓冲区
        memcpy(manage_object->Rx_Buffer.Data, uart1_frame_buf, 32);
        manage_object->Rx_Buffer.Length = 32;

        // 通知上层回调
        if (manage_object->Callback_Function != nullptr)
        {
            manage_object->Callback_Function(&manage_object->Rx_Buffer);
        }

        // 继续接收下一帧
        HAL_UART_Receive_IT(huart, &uart1_rx_byte, 1);
        break;

    default:
        sync_state = 0;
        HAL_UART_Receive_IT(huart, &uart1_rx_byte, 1);
        break;
    }
}
