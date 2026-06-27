//
// Created by 24158 on 2026/6/8.
//

#include "drv_can.h"
//#include "stm32F4xx_hal_can.h"//虽然drv_can.h中包含了该项，但显式包含可提高代码可读性及稳定性，重复包含留给编译器自行处理

//#include "string.h"

bool init_finished = false;

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};



//  SuperCapacitor Tx Data buffer
uint8_t CAN_Supercap_Tx_Data[8];

/*-----------------Private Variables-----------------*/
// CAN1 Tx buffer
uint8_t CAN1_0x1fe_Tx_Data[8] = {0};
uint8_t CAN1_0x1ff_Tx_Data[8] = {0};
uint8_t CAN1_0x200_Tx_Data[8] = {0};
uint8_t CAN1_0x2fe_Tx_Data[8] = {0};
uint8_t CAN1_0x2ff_Tx_Data[8] = {0};
uint8_t CAN1_0x3fe_Tx_Data[8] = {0};
uint8_t CAN1_0x4fe_Tx_Data[8] = {0};

// CAN2 Tx buffer
uint8_t CAN2_0x1fe_Tx_Data[8] = {0};
uint8_t CAN2_0x1ff_Tx_Data[8] = {0};
uint8_t CAN2_0x200_Tx_Data[8] = {0};
uint8_t CAN2_0x2fe_Tx_Data[8] = {0};
uint8_t CAN2_0x2ff_Tx_Data[8] = {0};
uint8_t CAN2_0x3fe_Tx_Data[8] = {0};
uint8_t CAN2_0x4fe_Tx_Data[8] = {0};

/**
 * @brief 根据 CAN 句柄获取对应的管理对象
 * @param hcan CAN 句柄指针，用于匹配对应的 CAN 实例
 * @return Struct_CAN_Manage_Object* 匹配到的管理对象指针，未匹配到时返回空指针。
 */
static Struct_CAN_Manage_Object *CAN_Get_Manage_Object(
    CAN_HandleTypeDef *hcan) {
  if (hcan == nullptr) {
    return nullptr;
  }

  if (hcan->Instance == CAN1) {
    return &CAN1_Manage_Object;
  } else if (hcan->Instance == CAN2) {
    return &CAN2_Manage_Object;
  }

  return nullptr;
}

/**
 * @brief 重置指定管理对象中的状态统计信息。
 * @param manage_object CAN 管理对象指针，用于指定待重置状态记录。
 */
static void CAN_Reset_Status(Struct_CAN_Manage_Object *manage_object) {
  if (manage_object == nullptr) {
    return;
  }

  // 直接清空状态统计结构体，不保留历史状态。
  memset(&manage_object->Status, 0, sizeof(manage_object->Status));
}

/**
 * @brief 更新发送统计信息，并记录本次发送时 FIFO 空闲等级。
 * @param manage_object CAN 管理对象指针，用于保存发送统计结果。
 * @param send_status 本次发送接口返回的 HAL 状态值。
 * @param tx_free_level 之前获取到的 Tx FIFO 空闲等级。
 */
static void CAN_Update_Tx_Status(Struct_CAN_Manage_Object *manage_object,
                                   HAL_StatusTypeDef send_status,
                                   uint32_t tx_free_level) {
  if (manage_object == nullptr) {
    return;
  }

  manage_object->Status.Last_Tx_Free_Level = tx_free_level;

  if (send_status == HAL_OK) {
    manage_object->Status.Tx_Success_Count++;
    manage_object->Status.Tx_Consecutive_Fail_Count = 0;
  } else {
    manage_object->Status.Tx_Fail_Count++;
    manage_object->Status.Tx_Consecutive_Fail_Count++;

    if (tx_free_level == 0U) {
      manage_object->Status.Tx_Fifo_Full_Count++;
    }
  }
}

/**
 * @brief 初始化指定 CAN 实例的滤波器、通知和总线管理信息。
 * @param hcan CAN 句柄指针，用于指定待初始化的硬件实例。
 * @param Callback_Function 接收回调函数指针，用于在接收后通知上层处理。
 */
// F407 BXCAN 初始化流程
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function) {
    // ===== 根据CAN实例选择滤波器bank =====
    // bank 0~13 属于 CAN1, bank 14~27 属于 CAN2
    uint32_t filter_bank_offset = (hcan->Instance == CAN2) ? 14 : 0;

    // ===== 1. 配置滤波器1（掩码过滤 0x200-0x20F → FIFO0）=====
    CAN_FilterTypeDef CAN_FilterConfig1 = {0};

    // bxCAN 滤波器配置（32位掩码模式）
    CAN_FilterConfig1.FilterBank = filter_bank_offset;           // CAN1用bank0, CAN2用bank14
    CAN_FilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;    // 掩码模式
    CAN_FilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;   // 32位宽
    CAN_FilterConfig1.FilterIdHigh = 0x200 << 5;            // 标准ID左移5位
  // CAN_FilterConfig1.FilterIdHigh = 0x000 << 5;            // 匹配所有ID
    CAN_FilterConfig1.FilterIdLow = 0x0000;
    CAN_FilterConfig1.FilterMaskIdHigh = 0x7F0 << 5;        // 掩码（匹配高7位）
  // CAN_FilterConfig1.FilterMaskIdHigh = 0x000 << 5;        // 掩码全0（匹配所有）
    CAN_FilterConfig1.FilterMaskIdLow = 0x0000;
    CAN_FilterConfig1.FilterFIFOAssignment = CAN_RX_FIFO0;  // 匹配后送入FIFO0
    CAN_FilterConfig1.FilterActivation = ENABLE;             // 启用滤波器
    CAN_FilterConfig1.SlaveStartFilterBank = 14;             // CAN2从14号滤波器开始

    HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfig1);

    // ===== 2. 配置滤波器2（接收所有标准帧 → FIFO1）=====
    CAN_FilterTypeDef CAN_FilterConfig2 = {0};

    CAN_FilterConfig2.FilterBank = filter_bank_offset + 1;       // CAN1用bank1, CAN2用bank15
    CAN_FilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfig2.FilterIdHigh = 0x000 << 5;            // 匹配所有ID
    CAN_FilterConfig2.FilterIdLow = 0x0000;
    CAN_FilterConfig2.FilterMaskIdHigh = 0x000 << 5;        // 掩码全0（匹配所有）
    CAN_FilterConfig2.FilterMaskIdLow = 0x0000;
    CAN_FilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO1;  // 送入FIFO1
    CAN_FilterConfig2.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfig2);

    // ===== 3. 配置管理对象 =====
    Struct_CAN_Manage_Object *manage_object = CAN_Get_Manage_Object(hcan);
    if (manage_object != nullptr) {
        manage_object->CAN_Handler = hcan;
        manage_object->Callback_Function = Callback_Function;
        CAN_Reset_Status(manage_object);
    }

    // ===== 4. 激活中断通知 =====
    HAL_CAN_ActivateNotification(hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING |    // FIFO0有新消息
        CAN_IT_RX_FIFO1_MSG_PENDING |    // FIFO1有新消息
        CAN_IT_ERROR_WARNING |           // 错误警告
        CAN_IT_ERROR_PASSIVE |           // 被动错误
        CAN_IT_BUSOFF | CAN_IT_TX_MAILBOX_EMPTY);                 // 总线关闭及发送邮箱空（启动tx中断）

    // ===== 5. 启动 CAN =====
    HAL_CAN_Start(hcan);
    CAN_Update_Bus_Status(hcan);
}

/**
 * @brief 通过指定 CAN 实例发送一帧数据，并更新发送统计状态。
 * @param hcan CAN 句柄指针，用于指定要使用的硬件实例。
 * @param ID 标准帧标识符。
 * @param Data 待发送的数据缓冲区指针。
 * @param Length 发送 DLC 长度值。
 * @return HAL_StatusTypeDef 返回 HAL 层发送结果。
 */
HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID,
                                  uint8_t *Data, uint8_t Length) {
  CAN_TxHeaderTypeDef TxHeader;

  TxHeader.StdId = ID;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = Length;
  TxHeader.TransmitGlobalTime = DISABLE;

  Struct_CAN_Manage_Object *manage_object = CAN_Get_Manage_Object(hcan);
  uint32_t tx_mailbox;
  HAL_StatusTypeDef send_status;
  uint32_t retry_cnt = 200U;  // 重试上限约 2ms（@1Mbps 一帧约 111μs）

  do {
    send_status = HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &tx_mailbox);
    if (send_status != HAL_OK) {
      /* 所有 3 个邮箱均已满，等待 TX 中断释放其中一个邮箱 */
      for (volatile uint32_t i = 0U; i < 500U; i++) { }
      retry_cnt--;
    }
  } while ((send_status != HAL_OK) && (retry_cnt > 0U));

  uint32_t tx_free_level = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
  CAN_Update_Tx_Status(manage_object, send_status, tx_free_level);
  if (send_status != HAL_OK) {
    CAN_Update_Bus_Status(hcan);
  }

  return send_status;
}

/**
 * @brief 以实时方式发送报文，当 FIFO 满时直接返回失败。
 * @param hcan CAN 句柄指针，用于指定要使用的硬件实例。
 * @param ID 标准帧标识符。
 * @param Data 待发送的数据缓冲区指针。
 * @param Length 发送 DLC 长度值。
 * @return HAL_StatusTypeDef 返回实时发送结果。
 */
HAL_StatusTypeDef CAN_Send_Data_Realtime(CAN_HandleTypeDef *hcan,
                                           uint16_t ID, uint8_t *Data,
                                           uint8_t Length) {
  uint32_t tx_free_level = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
  if (tx_free_level == 0U) {
    CAN_Update_Tx_Status(CAN_Get_Manage_Object(hcan), HAL_ERROR,
                           tx_free_level);
    CAN_Update_Bus_Status(hcan);
    return HAL_ERROR;
  }

  return CAN_Send_Data(hcan, ID, Data, Length);
}

/**
 * @brief 刷新指定 CAN 实例的协议状态、错误状态和 FIFO 空闲等级。
 * @param hcan CAN 句柄指针，用于指定需更新状态的硬件实例。
 */
void CAN_Update_Bus_Status(CAN_HandleTypeDef *hcan) {
  Struct_CAN_Manage_Object *manage_object = CAN_Get_Manage_Object(hcan);
  if (manage_object == nullptr) {
    return;
  }

  // BXCAN 通过直接读取 CAN 控制器寄存器获取协议状态
  manage_object->Status.Last_Protocol_Status = 0U;
  
  // 读取 CAN 控制器状态寄存器
  uint32_t msr = hcan->Instance->MSR;  // 模式状态寄存器
  uint32_t esr = hcan->Instance->ESR;  // 错误状态寄存器
  
  // bit[0:2]: 最后错误代码 (LEC)
  manage_object->Status.Last_Protocol_Status |= (esr & 0x7U);
  
  // bit[3:5]: 数据段最后错误代码 (BXCAN不支持，设为0)
  manage_object->Status.Last_Protocol_Status |= (0U << 3);
  
  // bit[6:7]: 总线活动状态
  manage_object->Status.Last_Protocol_Status |= ((((msr >> 1) & 0x1U)) << 6);
  manage_object->Status.Last_Protocol_Status |= ((((msr >> 0) & 0x1U)) << 7);
  
  // bit[8]: 错误被动状态 (EP)
  manage_object->Status.Last_Protocol_Status |= ((((esr >> 4) & 0x1U)) << 8);
  
  // bit[9]: 警告状态 (EWGF)
  manage_object->Status.Last_Protocol_Status |= ((((esr >> 2) & 0x1U)) << 9);
  
  // bit[10]: 总线关闭状态 (BS)
  manage_object->Status.Last_Protocol_Status |= ((((msr >> 2) & 0x1U)) << 10);

  manage_object->Status.Last_Error_Status = HAL_CAN_GetError(hcan);
  manage_object->Status.Last_Tx_Free_Level =
      HAL_CAN_GetTxMailboxesFreeLevel(hcan);
}

/**
 * @brief 获取指定 CAN 外设对应的状态统计结构体指针。
 * @param hcan CAN 外设句柄指针，用于匹配对应的管理对象。
 * @return Struct_CAN_Status* 返回状态结构体指针，未匹配到时返回空指针。
 */
Struct_CAN_Status *CAN_Get_Status(CAN_HandleTypeDef *hcan) {
    Struct_CAN_Manage_Object *manage_object = CAN_Get_Manage_Object(hcan);
    if (manage_object == nullptr) {
        return nullptr;
    }
    return &manage_object->Status;
}

/**
 * @brief CAN 电机控制 1ms 周期任务回调函数，用于发送周期性的控制报文。
 */
void RTOS_1ms_CAN_Motor_Callback() {
  // DJI 控制报文

  // 步行机器人控制报文，ID 对应 0x201~0x204。
  // CAN_Send_Data_Realtime(&hcan1, 0x200, CAN1_0x200_Tx_Data,
  //                          8U);
  // 台式扩展机器人控制报文，ID 对应 0x205~0x20B。
  //CAN_Send_Data_Realtime(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data,
  //                           CAN_DLC_BYTES_8);
}

/**
 * @brief CAN FIFO0 消息待处理中断回调，用于读取 FIFO0 中的新报文并通知上层回调
 * @param hcan CAN 句柄指针，用于标识触发中断的硬件实例。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  Struct_CAN_Manage_Object *manager_obj;

  if(hcan->Instance == CAN1)
   manager_obj = &CAN1_Manage_Object;
  else if(hcan->Instance == CAN2)
   manager_obj = &CAN2_Manage_Object;
  else
   return;

  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0, &manager_obj->Rx_Buffer.Header, manager_obj->Rx_Buffer.Data);

  if (!init_finished)
    return;
  //上层回调函数
  if(manager_obj->Callback_Function != nullptr)
  {
    manager_obj->Callback_Function(&manager_obj->Rx_Buffer);
  }
}

/**
 * @brief CAN FIFO1 消息待处理中断回调，用于读取 FIFO1 中的新报文并通知上层回调
 * @param hcan CAN 句柄指针，用于标识触发中断的硬件实例。
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {

  Struct_CAN_Manage_Object* manager_obj;

  if(hcan->Instance == CAN1)
   manager_obj = &CAN1_Manage_Object;
  else if(hcan->Instance == CAN2)
   manager_obj = &CAN2_Manage_Object;
  else
   return;

  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1, &manager_obj->Rx_Buffer.Header, manager_obj->Rx_Buffer.Data);
  if (!init_finished)
    return;

  //上层回调函数
  if(manager_obj->Callback_Function != nullptr)
  {
    manager_obj->Callback_Function(&manager_obj->Rx_Buffer);
  }
}


/**
 * @brief CAN 错误状态中断回调，用于记录 Warning、Error Passive 和 Bus-Off 事件并刷新总线状态。
 * @param hcan CAN 句柄指针，用于标识触发错误状态中断的硬件实例。
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  Struct_CAN_Manage_Object *manage_object = CAN_Get_Manage_Object(hcan);
  uint32_t esr = (manage_object != nullptr) ? hcan->Instance->ESR : 0U;
  uint32_t error_status_it = 0U;

  // 从硬件ESR寄存器读取错误状态标志
  if ((esr & CAN_ESR_EWGF) != 0U)
  {
    error_status_it |= CAN_IT_ERROR_WARNING;
  }
  if ((esr & CAN_ESR_EPVF) != 0U)
  {
    error_status_it |= CAN_IT_ERROR_PASSIVE;
  }
  if ((esr & CAN_ESR_BOFF) != 0U)
  {
    error_status_it |= CAN_IT_BUSOFF;
  }

  if (manage_object != nullptr) {
    manage_object->Status.Last_Error_Status = error_status_it;
  }

  // Warning 和 Error Passive 只是状态变化，可以在此阶段直接刷新总线。
  if ((error_status_it & (CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE)) !=
      0U) {
    CAN_Update_Bus_Status(hcan);
  }

  if ((error_status_it & CAN_IT_BUSOFF) != 0U) {
    if (manage_object != nullptr) {
      manage_object->Status.Bus_Off_Count++;
    }

    HAL_CAN_Stop(hcan);

    if (HAL_CAN_Start(hcan) == HAL_OK) {

      HAL_CAN_ActivateNotification(hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                         CAN_IT_RX_FIFO1_MSG_PENDING |
                                         CAN_IT_ERROR_WARNING |
                                         CAN_IT_ERROR_PASSIVE |
                                         CAN_IT_BUSOFF);
    }
  }

  CAN_Update_Bus_Status(hcan);
}







