/**
 * @file driver.h
 * @brief M3508电机驱动头文件
 *
 * 提供M3508电机的CAN通信接口和控制函数声明，支持多电机控制
 */

#ifndef __USER_DRIVERS_MOT_M3508_DRIVER__
#define __USER_DRIVERS_MOT_M3508_DRIVER__

#include "protocol.h"
#include "type.h"

extern volatile u8 m3508_protect_on;

/**
 * @brief 配置与启用M3508电机的CAN通信
 * @param hcan CAN对象指针
 */
void m3508_setup(CAN_HandleTypeDef *hcan, u8 master, u32 filter_bank);

/**
 * @brief 发送控制消息到M3508电机
 */
void m3508_send_ctrl_msg();

/**
 * @brief 设置M3508电机的电流
 * @param mot_id 电机ID (0-7)
 * @param cur 电流值
 */
void m3508_set_current(u8 mot_id, f32 cur);

void m3508_update_stat(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                       u8 data[8]);

void m3508_reset_pos();

volatile motStat_M3508 *m3508_get_stat(u8 id);

#endif /* __USER_DRIVERS_MOT_M3508_DRIVER__ */
