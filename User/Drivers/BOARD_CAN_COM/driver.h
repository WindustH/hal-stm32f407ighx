#ifndef __USER_DRIVERS_BOARD_CAN_COM_DRIVER__
#define __USER_DRIVERS_BOARD_CAN_COM_DRIVER__

#include "cb.h"       // IWYU pragma: keep
#include "protocol.h" // IWYU pragma: keep
#include "type.h"

#ifdef BOARD_CHASSIS
void bc_cha_rx_setup(CAN_HandleTypeDef *hcan, u32 can_id, u32 filter_bank,
                     u8 fifo);
void bc_cha_update_data_from_gim(CAN_HandleTypeDef *hcan,
                                 CAN_RxHeaderTypeDef *header, u8 data[8]);
bComGimDat *bc_cha_get_gim_data();

void bc_cha_tx_setup(CAN_HandleTypeDef *hcan, u32 can_id);
void bc_cha_send_msg();
#endif

#ifdef BOARD_GIMBAL
void bc_gim_tx_setup(CAN_HandleTypeDef *hcan, u32 can_id);
void bc_gim_send_msg();

void bc_gim_rx_setup(CAN_HandleTypeDef *hcan, u32 can_id, u32 filter_bank,
                     u8 fifo);
void bc_gim_update_data_from_cha(CAN_HandleTypeDef *hcan,
                                 CAN_RxHeaderTypeDef *header, u8 data[8]);
bComChaDat *bc_gim_get_cha_data();
#endif

#endif /* __USER_DRIVERS_BOARD_CAN_COM_DRIVER__ */
