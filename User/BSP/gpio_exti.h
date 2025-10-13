/**
 * @file gpio_exti.h
 * @brief GPIO EXTI中断处理头文件
 *
 * 提供GPIO EXTI中断的回调函数管理接口，支持多回调函数注册
 */

#ifndef __BSP_GPIO_EXTI__
#define __BSP_GPIO_EXTI__

#include "type.h"

/**
 * @brief 注册一个 GPIO EXTI 中断回调函数
 * @param callback 要注册的回调函数指针（不可为 NULL）
 * @return 成功返回索引（0 ~ GPIO_EXTI_CB_LIST_SIZE-1），失败返回
 * GPIO_EXTI_CB_LIST_SIZE
 */
u8 bsp_gpio_exti_cb_add(gpioExtiCb callback);

/**
 * @brief 注销一个已注册的 GPIO EXTI 回调函数
 * @param idx 要注销的回调索引（由 add 返回）
 */
void bsp_gpio_exti_cb_remove(u8 idx);

#endif /* __BSP_GPIO_EXTI__ */