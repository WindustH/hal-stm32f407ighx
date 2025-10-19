/**
 * @file can_fifo0_it.h
 * @brief CAN FIFO0中断处理头文件
 *
 * 提供CAN FIFO0消息挂起中断的回调函数管理接口，支持多回调函数注册
 */

#ifndef __BSP_CAN_FIFO0__
#define __BSP_CAN_FIFO0__
#include "type.h"
#define CAN_FIFO0_CB_LIST_SIZE 32

typedef void (*canFifo0Cb)(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                           u8 data[8]);
typedef struct {
  volatile u32 state;
  canFifo0Cb callbacks[CAN_FIFO0_CB_LIST_SIZE];
} canFifo0CbList;

u8 bsp_can_fifo0_cb_add(canFifo0Cb callback);
void bsp_can_fifo0_cb_remove(u8 idx);

#endif /* __BSP_CAN_FIFO0_IT__ */