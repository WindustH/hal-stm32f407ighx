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

extern u8 DMJ4310_PROTECT_ON;

/**
 * @brief 配置与启用DMJ4310电机的CAN通信
 * @param hcan CAN对象指针
 */
void setup_mot_dmj4310(CAN_HandleTypeDef *hcan);

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
volatile motStat_DMJ4310 *mot_get_stat_dmj4310();
#endif /* __USER_DRIVERS_MOT_DMJ4310_DRIVER__ */
