/**
 * @file can_fifo1_it.h
 * @brief CAN FIFO1中断处理头文件
 *
 * 提供CAN FIFO1消息挂起中断的回调函数管理接口，支持多回调函数注册
 */

#ifndef __BSP_CAN_FIFO1__
#define __BSP_CAN_FIFO1__

#include "type.h"
#define CAN_FIFO1_CB_LIST_SIZE 32
typedef void (*canFifo1Cb)(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                           u8 data[8]);
typedef struct {
  volatile u32 state;
  canFifo1Cb callbacks[CAN_FIFO1_CB_LIST_SIZE];
} canFifo1CbList;

u8 bsp_can_fifo1_cb_add(canFifo1Cb callback);
void bsp_can_fifo1_cb_remove(u8 idx);

#endif /* __BSP_CAN_FIFO1_IT__ */