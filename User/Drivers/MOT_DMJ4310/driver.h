/**
 * @file driver.h
 * @brief DMJ4310电机驱动头文件
 *
 * 提供DMJ4310电机的CAN通信接口和控制函数声明
 */

#ifndef __USER_DRIVERS_MOT_DMJ4310_DRIVER__
#define __USER_DRIVERS_MOT_DMJ4310_DRIVER__

#include "type.h"

/**
 * @brief 配置与启用DMJ4310电机的CAN通信
 * @param hcan CAN对象指针
 */
void mot_setup_can_dmj4310(CAN_HandleTypeDef *hcan);

/**
 * @brief 发送控制消息到DMJ4310电机
 */
void mot_send_ctrl_msg_dmj4310();

/**
 * @brief 设置DMJ4310电机的扭矩
 * @param trq 扭矩值
 */
void mot_set_torque_dmj4310(f32 trq);

void mot_update_stat_dmj4310(CAN_HandleTypeDef *hcan);

#endif /* __USER_DRIVERS_MOT_DMJ4310_DRIVER__ */
