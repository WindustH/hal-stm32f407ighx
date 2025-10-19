/**
 * @file gpio_exti.h
 * @brief GPIO EXTI中断处理头文件
 *
 * 提供GPIO EXTI中断的回调函数管理接口，支持多回调函数注册
 */

#ifndef __BSP_GPIO_EXTI__
#define __BSP_GPIO_EXTI__

#include "type.h"
#define GPIO_EXTI_CB_LIST_SIZE 32
typedef void (*gpioExtiCb)(uint16_t GPIO_Pin);
typedef struct {
  volatile u32 state;
  gpioExtiCb callbacks[GPIO_EXTI_CB_LIST_SIZE];
} gpioExtiCbList;

u8 bsp_gpio_exti_cb_add(gpioExtiCb callback);
void bsp_gpio_exti_cb_remove(u8 idx);

#endif /* __BSP_GPIO_EXTI__ */