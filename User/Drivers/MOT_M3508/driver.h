/**
 * @file driver.h
 * @brief M3508电机驱动头文件
 *
 * 提供M3508电机的CAN通信接口和控制函数声明，支持多电机控制
 */

#ifndef __USER_DRIVERS_MOT_M3508_DRIVER__
#define __USER_DRIVERS_MOT_M3508_DRIVER__

#include "type.h"

/**
 * @brief 配置与启用M3508电机的CAN通信
 * @param hcan CAN对象指针
 */
void mot_setup_can_m3508(CAN_HandleTypeDef *hcan);

/**
 * @brief 发送控制消息到M3508电机
 */
void mot_send_ctrl_msg_m3508();

/**
 * @brief 设置M3508电机的电流
 * @param mot_id 电机ID (0-7)
 * @param cur 电流值
 */
void mot_set_current_m3508(u8 mot_id, i16 cur);


void mot_update_stat_m3508(CAN_HandleTypeDef *hcan);

#endif /* __USER_DRIVERS_MOT_M3508_DRIVER__ */
