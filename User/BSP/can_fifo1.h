/**
 * @file can_fifo1_it.h
 * @brief CAN FIFO1中断处理头文件
 *
 * 提供CAN FIFO1消息挂起中断的回调函数管理接口，支持多回调函数注册
 */

#ifndef __BSP_CAN_FIFO1__
#define __BSP_CAN_FIFO1__

#include "type.h"

/**
 * @brief 注册一个 CAN FIFO1 消息挂起回调函数
 * @param callback 要注册的回调函数指针（不可为 NULL）
 * @return 成功返回索引（0 ~ CAN_FIFO1_CB_LIST_SIZE-1），失败返回
 * CAN_FIFO1_CB_LIST_SIZE
 */
u8 bsp_can_fifo1_cb_add(canFifo1Cb callback);

/**
 * @brief 注销一个已注册的 CAN FIFO1 回调函数
 * @param idx 要注销的回调索引（由 add 返回）
 */
void bsp_can_fifo1_cb_remove(u8 idx);

#endif /* __BSP_CAN_FIFO1_IT__ */