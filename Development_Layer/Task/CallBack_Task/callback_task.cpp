//
// Created by 24158 on 2026/6/16.
//

#include "callback_task.h"




Class_DOG dog;
Struct_CAN_Status *status;//
float kp = 4.5f,kd = 0.1f,angle = 0.0f,Omega = 0.0f,Torque = 0.00f;//测试用
uint8_t init_flag = 0;//实现状态机中只初始化一次

void Robot_Init()
{


    CAN_Init(&hcan1,CAN1_Callback);
    CAN_Init(&hcan2,CAN2_Callback);
    UART_Init(&huart1,UART1_Callback);

    dog.Chassis_Init();


    TIM_Init(&htim6,Tim6_Robot_1ms_Callback);
    HAL_TIM_Base_Start_IT(&htim6);


    //HAL_Delay(100);慎用，前面已经启用中断，所以可能会在多个中断之间来回跳导致100ms的阻塞计时一直不能完成，进而导致init_finished = true;永远不能执行
    init_finished = true;


}


void Tim6_Robot_1ms_Callback()
{
    robot_1ms_cycle_flag = true;

    // 1ms_检测
    static uint8_t cnt_100ms = 0;
    static uint16_t cnt_500ms = 0;
    static uint16_t cnt_1000ms = 0;

    cnt_100ms++;
    cnt_500ms++;
    cnt_1000ms++;

    if (cnt_100ms == 100) {
        robot_100ms_cycle_flag = true;
        cnt_100ms = 0;
    }
    if (cnt_1000ms == 1000) {
        robot_1000ms_cycle_flag = true;
        cnt_1000ms = 0;

    }



    dog.Chassis_Control();





}

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    // dog.Foot1_Left_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // dog.Foot1_Left_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // dog.Foot2_Right_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // dog.Foot2_Right_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
switch (CAN_RxMessage->Header.StdId & 0x0f)
{
    case(0x01):
    {
            dog.Foot1_Left_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
    }
    case(0x02):
    {
            dog.Foot1_Left_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
    }
case(0x03):
        {
            dog.Foot2_Right_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
case(0x04):
        {
            dog.Foot2_Right_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
case(0x05):
        {
            dog.Foot3_Left_Front.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
case(0x06):
        {
            dog.Foot3_Left_Front.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
case(0x07):
        {
            dog.Foot4_Right_Front.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
case(0x08):
        {
            dog.Foot4_Right_Front.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
default:
    // 所有case都不匹配时执行
    break;
}

    // if(CAN_RxMessage->Header.StdId  == dog.Foot1_Left_Back.mi.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot1_Left_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
    //
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot2_Right_Back.mi.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot2_Right_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);//要改
    //
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot3_Left_Front.mi.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot3_Left_Front.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot4_Right_Front.mi.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot4_Right_Front.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // }
    //
    // else if(CAN_RxMessage->Header.StdId == dog.Foot1_Left_Back.mo.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot1_Left_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
    //
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot2_Right_Back.mo.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot2_Right_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);//要改
    //
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot3_Left_Front.mo.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot3_Left_Front.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot4_Right_Front.mo.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot4_Right_Front.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // }else
    // {
    //     return;
    // }



}


/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{

    // if(CAN_RxMessage->Header.StdId  == dog.Foot1_Left_Back.mi.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot1_Left_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
    //
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot2_Right_Back.mi.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot2_Right_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);//要改
    //
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot3_Left_Front.mi.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot3_Left_Front.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot4_Right_Front.mi.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot4_Right_Front.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // }
    //
    // else if(CAN_RxMessage->Header.StdId == dog.Foot1_Left_Back.mo.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot1_Left_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
    //
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot2_Right_Back.mo.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot2_Right_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);//要改
    //
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot3_Left_Front.mo.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot3_Left_Front.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // }else if (CAN_RxMessage->Header.StdId == dog.Foot4_Right_Front.mo.Get_CAN_Tx_ID() )
    // {
    //     dog.Foot4_Right_Front.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
    // }else
    // {
    //     return;
    // }
    switch (CAN_RxMessage->Header.StdId & 0x0f)
    {
    case(0x01):
        {
            dog.Foot1_Left_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    case(0x02):
        {
            dog.Foot1_Left_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    case(0x03):
        {
            dog.Foot2_Right_Back.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    case(0x04):
        {
            dog.Foot2_Right_Back.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    case(0x05):
        {
            dog.Foot3_Left_Front.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    case(0x06):
        {
            dog.Foot3_Left_Front.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    case(0x07):
        {
            dog.Foot4_Right_Front.mi.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    case(0x08):
        {
            dog.Foot4_Right_Front.mo.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    default:
         // 所有case都不匹配时执行
        break;
    }


}


/**
 * @brief UART1回调函数，接收FS-IA6B遥控器IBUS数据
 *
 * @param UART_RxMessage UART1收到的32字节IBUS帧
 */
void UART1_Callback(Struct_UART_Rx_Buffer *UART_RxMessage)
{
    dog.Handle_RC_Data(UART_RxMessage->Data);
}
