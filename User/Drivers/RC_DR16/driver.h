/**
 * @file driver.h
 * @brief DR16遥控器驱动头文件
 *
 * 提供DR16遥控器的UART通信接口和控制消息更新函数声明
 */

#ifndef __USER_DRIVERS_RC_DR16_DRIVER__
#define __USER_DRIVERS_RC_DR16_DRIVER__
#include "protocol.h"
#include "type.h" // IWYU pragma: keep

/**
 * @brief 配置与启用DR16遥控器的UART通信
 * @param uartx UART对象指针
 */
void rc_dr16_setup(UART_HandleTypeDef *uartx);

/**
 * @brief 更新遥控器控制消息
 * @param huart UART对象指针
 * @param size 接收数据大小
 */
void rc_dr16_update_ctrl_msg(UART_HandleTypeDef *huart, u16 size);

volatile rcCtrl_dr16 *rc_dr16_get_ctrl_sig();

#endif /* __USER_DRIVERS_RC_DR16_DRIVER__ */
