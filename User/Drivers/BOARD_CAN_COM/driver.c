#include "driver.h"
#include "BSP/can_tx_queue.h"
#include "main.h"

volatile u32 gim_to_cha_can_id = 0x007U;
volatile u32 cha_to_gim_can_id = 0x008U;

#ifdef BOARD_CHASSIS

static bComGimDat bc_gimbal_data = {0};
static CAN_HandleTypeDef *hcanx_chassis_tx = NULL;
static CAN_HandleTypeDef *hcanx_chassis_rx = NULL;
bComGimDat *board_com_get_gimbal_data() { return &bc_gimbal_data; }

void board_chassis_rx_setup(CAN_HandleTypeDef *hcan, u32 can_id,
                            u32 filter_bank, u8 fifo) {
  gim_to_cha_can_id = can_id;
  CAN_FilterTypeDef can_filter = {0};
  can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterBank = filter_bank;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = (gim_to_cha_can_id << 5) & 0xFFFF;
  can_filter.FilterIdLow = (gim_to_cha_can_id << 21) & 0xFFFF;
  can_filter.FilterMaskIdHigh = (gim_to_cha_can_id << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = (gim_to_cha_can_id << 21) & 0xFFFF;
  can_filter.FilterFIFOAssignment = fifo;
  can_filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
  hcanx_chassis_rx = hcan;
}

void board_com_update_gimbal_data(CAN_HandleTypeDef *hcan,
                                  CAN_RxHeaderTypeDef *header, u8 data[8]) {
  if (hcan == hcanx_chassis_rx && header->StdId == gim_to_cha_can_id) {
    board_com_parse_gimbal_msg(header, data, &bc_gimbal_data);
    bcc_cb_call();
  }
}

void board_chassis_tx_setup(CAN_HandleTypeDef *hcan, u32 can_id) {
  cha_to_gim_can_id = can_id;
  hcanx_chassis_tx = hcan;
}

void board_chassis_send_msg(const bComChaDat *data) {
  if (!hcanx_chassis_tx)
    return;

  boardComCanMsg can_msg;
  u8 block[8];
  can_msg.data = block;
  can_msg.header.StdId = cha_to_gim_can_id;
  can_msg.header.IDE = CAN_ID_STD;
  can_msg.header.RTR = CAN_RTR_DATA;
  can_msg.header.DLC = 8;
  can_msg.header.TransmitGlobalTime = DISABLE;

  board_com_pack_chassis_msg(&can_msg, data);
  can_send_message(hcanx_chassis_tx, &can_msg.header, can_msg.data);
}

#endif // BOARD_CHASSIS

#ifdef BOARD_GIMBAL

#include "Drivers/RC_DR16/driver.h"
static CAN_HandleTypeDef *hcanx_gimbal_tx = NULL;
static CAN_HandleTypeDef *hcanx_gimbal_rx = NULL;
void board_gimbal_tx_setup(CAN_HandleTypeDef *hcan, u32 can_id) {
  gim_to_cha_can_id = can_id;
  hcanx_gimbal_tx = hcan;
}

void board_gimbal_send_msg() {
  volatile rcCtrl_dr16 *rc_ctrl = rc_dr16_get_ctrl_sig();
  bComGimDat bcd = {.ch0 = rc_ctrl->rc.ch0,
                    .ch1 = rc_ctrl->rc.ch1,
                    .ch2 = rc_ctrl->rc.ch2,
                    .ch3 = rc_ctrl->rc.ch3,
                    .s1 = rc_ctrl->rc.s1,
                    .s2 = rc_ctrl->rc.s2};

  boardComCanMsg can_msg;
  u8 block[8];
  can_msg.data = block;
  board_com_pack_gimbal_msg(&can_msg, &bcd);
  can_send_message(hcanx_gimbal_tx, &can_msg.header, can_msg.data);
}

static bComChaDat bc_chassis_data = {0};

bComChaDat *board_com_get_chassis_data() { return &bc_chassis_data; }

void board_gimbal_rx_setup(CAN_HandleTypeDef *hcan, u32 can_id, u32 filter_bank,
                           u8 fifo) {
  cha_to_gim_can_id = can_id;
  hcanx_gimbal_rx = hcan;

  CAN_FilterTypeDef can_filter = {0};
  can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterBank = filter_bank;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = (cha_to_gim_can_id << 5) & 0xFFFF;
  can_filter.FilterIdLow = (cha_to_gim_can_id << 21) & 0xFFFF;
  can_filter.FilterMaskIdHigh = (cha_to_gim_can_id << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = (cha_to_gim_can_id << 21) & 0xFFFF;
  can_filter.FilterFIFOAssignment = fifo;
  can_filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void board_com_update_chassis_data(CAN_HandleTypeDef *hcan,
                                   CAN_RxHeaderTypeDef *header, u8 data[8]) {
  if (hcan == hcanx_gimbal_rx && header->StdId == cha_to_gim_can_id) {
    board_com_parse_chassis_msg(header, data, &bc_chassis_data);
    bcc_cb_call();
  }
}

#endif // BOARD_GIMBAL