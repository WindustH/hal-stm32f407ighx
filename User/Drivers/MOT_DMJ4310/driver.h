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

extern volatile u8 dmj4310_protect_on;

/**
 * @brief 配置与启用DMJ4310电机的CAN通信
 * @param hcan CAN对象指针
 */
void dmj4310_setup(CAN_HandleTypeDef *hcan, u32 can_id, u32 master_id,
                   u32 filter_bank, u8 fifo);

/**
 * @brief 发送控制消息到DMJ4310电机
 */
void dmj4310_send_ctrl_msg();

/**
 * @brief 设置DMJ4310电机的扭矩
 * @param trq 扭矩值
 */
void dmj4310_set_torque(f32 trq);
void dmj4310_update_stat(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                         u8 data[8]);
void dmj4310_reset_pos();
void dmj4310_disable();
volatile motStat_DMJ4310 *dmj4310_get_stat();
#endif /* __USER_DRIVERS_MOT_DMJ4310_DRIVER__ */
