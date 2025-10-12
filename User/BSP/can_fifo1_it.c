/**
 * @file can_fifo1_it.c
 * @brief CAN FIFO1中断处理实现文件
 *
 * 实现CAN FIFO1消息挂起中断的回调函数管理，支持多回调函数的注册、注销和调用
 */

#include "can_fifo1_it.h"
#include "stm32f4xx_hal.h" // IWYU pragma: keep
#include <string.h>

// 静态全局回调列表
static volatile canFifo1MsgPendingCbList can_fifo1_msg_pending_cb_list = {0};

/**
 * @brief 注册 CAN FIFO1 消息挂起回调
 * @param callback 要注册的回调函数指针
 * @return 成功返回索引，失败返回 CAN_FIFO1_CB_LIST_SIZE
 */
u8 bsp_can_fifo1_msg_pending_callback_add(canFifo1MsgPendingCb callback) {
  if (callback == NULL) {
    return CAN_FIFO1_CB_LIST_SIZE;
  }

  // 临界区：关闭中断防止竞态（适用于 Cortex-M）
  __disable_irq();
  for (u8 i = 0; i < CAN_FIFO1_CB_LIST_SIZE; i++) {
    if (!(can_fifo1_msg_pending_cb_list.state & (1U << i))) {
      can_fifo1_msg_pending_cb_list.callbacks[i] = callback;
      can_fifo1_msg_pending_cb_list.state |= (1U << i);
      __enable_irq();
      return i;
    }
  }
  __enable_irq();
  return CAN_FIFO1_CB_LIST_SIZE; // 满了
}

/**
 * @brief 注销 CAN FIFO1 回调
 * @param idx 要注销的回调索引
 */
void bsp_can_fifo1_msg_pending_callback_remove(u8 idx) {
  if (idx >= CAN_FIFO1_CB_LIST_SIZE) {
    return;
  }

  __disable_irq();
  can_fifo1_msg_pending_cb_list.state &= ~(1U << idx);
  // 可选：清空指针（非必须，但更安全）
  can_fifo1_msg_pending_cb_list.callbacks[idx] = NULL;
  __enable_irq();
}

/**
 * @brief HAL CAN FIFO1 消息挂起回调（由 HAL 库调用）
 *        此函数需在 stm32xxx_it.c 或 main.c 中保留为 weak 或直接实现
 * @param hcan CAN对象指针
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  // 遍历所有已注册的回调并调用
  u32 state_snapshot =
      can_fifo1_msg_pending_cb_list.state; // 读一次避免多次访问 volatile
  for (u8 i = 0; i < CAN_FIFO1_CB_LIST_SIZE; i++) {
    if ((state_snapshot & (1U << i)) &&
        can_fifo1_msg_pending_cb_list.callbacks[i] != NULL) {
      can_fifo1_msg_pending_cb_list.callbacks[i](hcan);
    }
  }
}