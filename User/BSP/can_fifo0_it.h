/**
 * @file can_fifo0_it.h
 * @brief CAN FIFO0中断处理头文件
 *
 * 提供CAN FIFO0消息挂起中断的回调函数管理接口，支持多回调函数注册
 */

#ifndef __BSP_CAN_FIFO0_IT__
#define __BSP_CAN_FIFO0_IT__

#include "type.h"

/**
 * @brief 注册一个 CAN FIFO0 消息挂起回调函数
 * @param callback 要注册的回调函数指针（不可为 NULL）
 * @return 成功返回索引（0 ~ CAN_FIFO0_CB_LIST_SIZE-1），失败返回
 * CAN_FIFO0_CB_LIST_SIZE
 */
u8 bsp_can_fifo0_cb_add(canFifo0Cb callback);

/**
 * @brief 注销一个已注册的 CAN FIFO0 回调函数
 * @param idx 要注销的回调索引（由 add 返回）
 */
void bsp_can_fifo0_cb_remove(u8 idx);

#endif /* __BSP_CAN_FIFO0_IT__ */