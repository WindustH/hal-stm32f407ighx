#include "driver.h"

#include "BSP/can_tx_queue.h"
#include "Tasks/protect_chassis.h"
#include "main.h"

// CAN ID - MIT模式命令帧
volatile u32 board_com_can_id = 0x007U;
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

  can_filter.FilterIdHigh = (board_com_can_id << 5) & 0xFFFF;
  can_filter.FilterIdLow = (board_com_can_id << 21) & 0xFFFF;

  can_filter.FilterMaskIdHigh = (board_com_can_id << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = (board_com_can_id << 21) & 0xFFFF;

  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 分配到 FIFO0
  can_filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 保存配置的 can 对象
  hcanx = hcan;
}

void board_com_update_rx_data(CAN_HandleTypeDef *hcan,
                              CAN_RxHeaderTypeDef *header, u8 data[8]) {
  if (hcanx == hcan && header->StdId == board_com_can_id) {
    board_com_parse_msg(header, data, &bc_rx_data);
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
  u8 block[8];
  can_msg.data = block;

  board_com_pack_msg(&can_msg, &bcd);
  extern uint8_t debug_point;
  debug_point = can_msg.header.StdId;
  if (can_send_message(hcanx, &can_msg.header, can_msg.data) != HAL_OK) {
    return;
  }
}

boardComT *board_com_get_rx_data() { return &bc_rx_data; }