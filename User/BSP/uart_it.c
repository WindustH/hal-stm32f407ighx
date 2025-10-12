/**
 * @file uart_it.c
 * @brief UART中断处理实现文件
 *
 * 实现UART接收中断的回调函数管理，支持多回调函数的注册、注销和调用
 */

#include "uart_it.h"
#include "stm32f4xx_hal.h" // IWYU pragma: keep
#include <string.h>

// 静态全局回调列表
static volatile uartRxCbList uart_cb_list = {0};

/**
 * @brief 注册 UART 接收回调
 * @param callback 要注册的回调函数指针
 * @return 成功返回索引，失败返回 UART_CB_LIST_SIZE
 */
u8 bsp_uart_rx_cb_add(uartRxCb callback) {
  if (callback == NULL) {
    return UART_CB_LIST_SIZE;
  }

  // 临界区：关闭中断防止竞态（适用于 Cortex-M）
  __disable_irq();
  for (u8 i = 0; i < UART_CB_LIST_SIZE; i++) {
    if (!(uart_cb_list.state & (1U << i))) {
      uart_cb_list.callbacks[i] = callback;
      uart_cb_list.state |= (1U << i);
      __enable_irq();
      return i;
    }
  }
  __enable_irq();
  return UART_CB_LIST_SIZE; // 满了
}

/**
 * @brief 注销 UART 回调
 * @param idx 要注销的回调索引
 */
void bsp_uart_rx_cb_remove(u8 idx) {
  if (idx >= UART_CB_LIST_SIZE) {
    return;
  }

  __disable_irq();
  uart_cb_list.state &= ~(1U << idx);
  // 可选：清空指针（非必须，但更安全）
  uart_cb_list.callbacks[idx] = NULL;
  __enable_irq();
}

/**
 * @brief HAL UART 接收事件回调（由 HAL 库调用）
 *        此函数需在 stm32xxx_it.c 或 main.c 中保留为 weak 或直接实现
 * @param huart UART对象指针
 * @param size 接收数据大小
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, u16 size) {
  // 遍历所有已注册的回调并调用
  u32 state_snapshot = uart_cb_list.state; // 读一次避免多次访问 volatile
  for (u8 i = 0; i < UART_CB_LIST_SIZE; i++) {
    if ((state_snapshot & (1U << i)) && uart_cb_list.callbacks[i] != NULL) {
      uart_cb_list.callbacks[i](huart, size);
    }
  }
}