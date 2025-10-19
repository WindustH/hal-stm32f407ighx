#ifndef __USER_DRIVERS_BOARD_CAN_COM_DRIVER__
#define __USER_DRIVERS_BOARD_CAN_COM_DRIVER__
#include "cb.h"       // IWYU pragma: keep
#include "protocol.h" // IWYU pragma: keep
#include "type.h"

#ifdef BOARD_CHASSIS
void board_chassis_rx_setup(CAN_HandleTypeDef *hcan, u32 can_id,
                            u32 filter_bank, u8 fifo);
void board_com_update_gimbal_data(CAN_HandleTypeDef *hcan,
                                  CAN_RxHeaderTypeDef *header, u8 data[8]);
bComGimDat *board_com_get_gimbal_data();
#endif
#ifdef BOARD_GIMBAL
void board_gimbal_tx_setup(CAN_HandleTypeDef *hcan, u32 can_id);
void board_gimbal_send_msg();
#endif
#endif /* __USER_DRIVERS_BOARD_CAN_COM_DRIVER__ */
