/**
 * @file uart_it.h
 * @brief UART中断处理头文件
 *
 * 提供UART接收中断的回调函数管理接口，支持多回调函数注册
 */

#ifndef __BSP_UART_IT__
#define __BSP_UART_IT__

#include "type.h"

/**
 * @brief 注册一个 UART 接收完成回调函数
 * @param callback 要注册的回调函数指针（不可为 NULL）
 * @return 成功返回索引（0 ~ UART_CB_LIST_SIZE-1），失败返回 UART_CB_LIST_SIZE
 */
u8 bsp_uart_rx_callback_add(uartRxCb callback);

/**
 * @brief 注销一个已注册的 UART 回调函数
 * @param idx 要注销的回调索引（由 add 返回）
 */
void bsp_uart_rx_callback_remove(u8 idx);

#endif /* __BSP_UART_IT__ */
