/**
 * @file driver.h
 * @brief DMJ6006电机驱动头文件
 *
 * 提供DMJ6006电机的CAN通信接口和控制函数声明
 */

#ifndef __USER_DRIVERS_MOT_DMJ6006_DRIVER__
#define __USER_DRIVERS_MOT_DMJ6006_DRIVER__

#include "protocol.h"
#include "type.h"

extern volatile u8 dmj6006_protect_on;

/**
 * @brief 配置与启用DMJ6006电机的CAN通信
 * @param hcan CAN对象指针
 */
void dmj6006_setup(CAN_HandleTypeDef *hcan, u16 can_id, u16 master_id,
                   u32 filter_bank, u8 fifo);

/**
 * @brief 发送控制消息到DMJ6006电机
 */
void dmj6006_send_ctrl_msg();

/**
 * @brief 设置DMJ6006电机的扭矩
 * @param trq 扭矩值
 */
void dmj6006_set_torque(f32 trq);
void dmj6006_update_stat(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                         u8 data[8]);
void dmj6006_reset_pos();
volatile motStat_DMJ6006 *dmj6006_get_stat();
#endif /* __USER_DRIVERS_MOT_DMJ6006_DRIVER__ */
