/**
 * @file driver.h
 * @brief DMJ4310电机驱动头文件
 *
 * 提供DMJ4310电机的CAN通信接口和控制函数声明
 */

#ifndef __USER_DRIVERS_MOT_DMJ4310_DRIVER__
#define __USER_DRIVERS_MOT_DMJ4310_DRIVER__

#include "protocol.h"
#include "type.h"

extern volatile u8 DMJ4310_PROTECT_ON;

/**
 * @brief 配置与启用DMJ4310电机的CAN通信
 * @param hcan CAN对象指针
 */
void dmj4310_setup(CAN_HandleTypeDef *hcan, u8 master);

/**
 * @brief 发送控制消息到DMJ4310电机
 */
void dmj4310_send_ctrl_msg();

/**
 * @brief 设置DMJ4310电机的扭矩
 * @param trq 扭矩值
 */
void dmj4310_set_torque(f32 trq);
void dmj4310_update_stat(CAN_HandleTypeDef *hcan);
void dmj4310_reset_pos();
volatile motStat_DMJ4310 *dmj4310_get_stat();
#endif /* __USER_DRIVERS_MOT_DMJ4310_DRIVER__ */
