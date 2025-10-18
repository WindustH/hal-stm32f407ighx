#include "driver.h"

#include "Tasks/protect_chassis.h"
#include "main.h"

// CAN ID - MIT模式命令帧
volatile u32 board_com_can_id = 0x002U;
static CAN_HandleTypeDef *hcanx;
static boardComT bc_rx_data = {0};

void board_com_rx_setup(CAN_HandleTypeDef *hcan, u8 master, u32 can_id,
                        u32 filter_bank) {

  board_com_can_id = can_id;

  CAN_FilterTypeDef can_filter = {0};
  if (master)
    can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterBank = filter_bank;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

  can_filter.FilterIdHigh = (board_com_can_id << 5);
  can_filter.FilterIdLow = (board_com_can_id << 5);

  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow = 0;

  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 分配到 FIFO0
  can_filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 保存配置的 can 对象
  hcanx = hcan;
}

void board_com_update_rx_data(CAN_HandleTypeDef *hcan) {
  if (hcanx == hcan) {

    canRxH header;
    u8 data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      return;
    }
    board_com_parse_msg(&header, data, &bc_rx_data);
    chassis_protect_refresh_idle_time();
  }
}

void board_com_tx_setup(CAN_HandleTypeDef *hcan, u32 can_id) {
  board_com_can_id = can_id;
  hcanx = hcan;
}
void board_com_send_msg(volatile rcCtrl_dr16 *rc_ctrl) {
  boardComT bcd;
  bcd.ch0 = rc_ctrl->rc.ch0;
  bcd.ch1 = rc_ctrl->rc.ch1;
  bcd.ch2 = rc_ctrl->rc.ch2;
  bcd.ch3 = rc_ctrl->rc.ch2;
  bcd.s1 = rc_ctrl->rc.s1;
  bcd.s2 = rc_ctrl->rc.s2;

  boardComCanMsg can_msg;
  u32 unused_mailbox;
  u8 block[8];
  can_msg.data = block;

  board_com_pack_msg(&can_msg, &bcd);
  while (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0)
    ;
  if (HAL_CAN_AddTxMessage(hcanx, &can_msg.header, can_msg.data,
                           &unused_mailbox) != HAL_OK) {
    return;
  }
}

boardComT *board_com_get_rx_data() { return &bc_rx_data; }