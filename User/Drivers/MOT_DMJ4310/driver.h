#ifndef __USER_DRIVERS_MOT_DMJ4310_DRIVER__
#define __USER_DRIVERS_MOT_DMJ4310_DRIVER__

#include "stm32f4xx_hal.h" // IWYU pragma: keep
#include "type.h"

void mot_setup_can_dmj4310(CAN_HandleTypeDef *hcan);

void mot_send_ctrl_msg_dmj4310();

void mot_set_torque_dmj4310(f32 trq);

void mot_update_stat_dmj4310(const volatile canRxH *msg,
                             const volatile u8 *data);

#endif /* __USER_DRIVERS_MOT_DMJ4310_DRIVER__ */
