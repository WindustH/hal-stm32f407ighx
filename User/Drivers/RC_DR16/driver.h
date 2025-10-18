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

void rc_dr16_setup();

volatile rcCtrl_dr16 *rc_dr16_get_ctrl_sig();

#endif /* __USER_DRIVERS_RC_DR16_DRIVER__ */
