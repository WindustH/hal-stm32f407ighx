#ifndef __USER_DRIVERS_MOT_M3508_DRIVER__
#define __USER_DRIVERS_MOT_M3508_DRIVER__

#include "stm32f4xx_hal.h" // IWYU pragma: keep
#include "type.h"

/**
 * @brief 配置与启用M3508电机的CAN通信
 * @param hcan CAN对象指针
 */
void mot_setup_can_m3508(CAN_HandleTypeDef *hcan);

void mot_send_ctrl_msg_m3508();

void mot_set_current_m3508(u8 mot_id, i16 cur);

void mot_update_stat_m3508(const volatile canRxH *msg, const volatile u8 *data);

#endif /* __USER_DRIVERS_MOT_M3508_DRIVER__ */
