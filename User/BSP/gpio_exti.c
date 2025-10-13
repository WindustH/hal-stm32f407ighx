/**
 * @file gpio_exti.c
 * @brief GPIO EXTI中断处理实现文件
 *
 * 实现GPIO EXTI中断的回调函数管理，支持多回调函数的注册、注销和调用
 */

#include "gpio_exti.h"
#include "stm32f4xx_hal.h" // IWYU pragma: keep
#include <string.h>

// 静态全局回调列表
static volatile gpioExtiCbList gpio_exti_cb_list = {0};

u8 bsp_gpio_exti_cb_add(gpioExtiCb callback) {
  if (callback == NULL) {
    return GPIO_EXTI_CB_LIST_SIZE;
  }

  // 临界区：关闭中断防止竞态（适用于 Cortex-M）
  __disable_irq();
  for (u8 i = 0; i < GPIO_EXTI_CB_LIST_SIZE; i++) {
    if (!(gpio_exti_cb_list.state & (1U << i))) {
      gpio_exti_cb_list.callbacks[i] = callback;
      gpio_exti_cb_list.state |= (1U << i);
      __enable_irq();
      return i;
    }
  }
  __enable_irq();
  return GPIO_EXTI_CB_LIST_SIZE; // 满了
}

void bsp_gpio_exti_cb_remove(u8 idx) {
  if (idx >= GPIO_EXTI_CB_LIST_SIZE) {
    return;
  }

  __disable_irq();
  gpio_exti_cb_list.state &= ~(1U << idx);
  // 可选：清空指针（非必须，但更安全）
  gpio_exti_cb_list.callbacks[idx] = NULL;
  __enable_irq();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // 遍历所有已注册的回调并调用
  u32 state_snapshot =
      gpio_exti_cb_list.state; // 读一次避免多次访问 volatile
  for (u8 i = 0; i < GPIO_EXTI_CB_LIST_SIZE; i++) {
    if ((state_snapshot & (1U << i)) &&
        gpio_exti_cb_list.callbacks[i] != NULL) {
      gpio_exti_cb_list.callbacks[i](GPIO_Pin);
    }
  }
}