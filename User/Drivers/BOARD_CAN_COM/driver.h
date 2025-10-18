#ifndef __USER_DRIVERS_BOARD_CAN_COM_DRIVER__
#define __USER_DRIVERS_BOARD_CAN_COM_DRIVER__
#include "Drivers/RC_DR16/protocol.h"
#include "protocol.h"
#include "type.h"

void board_com_rx_setup(CAN_HandleTypeDef *hcan, u32 can_id, u32 filter_bank);
void board_com_update_rx_data(CAN_HandleTypeDef *hcan,
                              CAN_RxHeaderTypeDef *header, u8 data[8]);

void board_com_tx_setup(CAN_HandleTypeDef *hcan, u32 can_id);
void board_com_send_msg(volatile rcCtrl_dr16 *rc_ctrl);
boardComT *board_com_get_rx_data();
#endif /* __USER_DRIVERS_BOARD_CAN_COM_DRIVER__ */
