#include "driver.h"
#include "main.h"
#include "protocol.h"

static CAN_HandleTypeDef *hcan;
static volatile motStat_M3508 mot_stat[8];
static volatile motCtrl_M3508 mot_ctrl[8];

void mot_setup_can_m3508(CAN_HandleTypeDef *hcanx) {
  CAN_FilterTypeDef can_filter = {0}; // 初始化为0更安全

  // 过滤器组 0
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = (0x201U << 5);
  can_filter.FilterIdLow = (0x202U << 5);
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow = 0;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(hcanx, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 1
  can_filter.FilterBank = 1;
  can_filter.FilterIdHigh = (0x203U << 5);
  can_filter.FilterIdLow = (0x204U << 5);

  if (HAL_CAN_ConfigFilter(hcanx, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 2
  can_filter.FilterBank = 2;
  can_filter.FilterIdHigh = (0x205U << 5);
  can_filter.FilterIdLow = (0x206U << 5);

  if (HAL_CAN_ConfigFilter(hcanx, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 3
  can_filter.FilterBank = 3;
  can_filter.FilterIdHigh = (0x207U << 5);
  can_filter.FilterIdLow = (0x208U << 5);

  if (HAL_CAN_ConfigFilter(hcanx, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 启动 CAN 外设
  if (HAL_CAN_Start(hcanx) != HAL_OK) {
    Error_Handler();
  }

  // 激活CAN RX 中断仅用于 FIFO0
  if (HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK) {
    Error_Handler();
  }

  // 保存配置的 can 对象
  hcan = hcanx;
}

void mot_set_current_m3508(u8 mot_id, i16 cur) { mot_ctrl[mot_id].I = cur; }

void mot_send_ctrl_msg_m3508() {
  motCtrlCanMsg_M3508 can_msg;
  u32 unused_mailbox;
  mot_ctrl_pack_msg_m3508(mot_ctrl, &can_msg);

  if (HAL_CAN_AddTxMessage(hcan, &can_msg.header_1_4, can_msg.data_1_4,
                           &unused_mailbox) != HAL_OK) {
    return;
  }

  if (HAL_CAN_AddTxMessage(hcan, &can_msg.header_5_8, can_msg.data_5_8,
                           &unused_mailbox) != HAL_OK) {
    return;
  }
}

void mot_update_stat_m3508(const volatile canRxH *msg,
                           const volatile u8 *data) {
  mot_fb_parse_m3508(msg, data, mot_stat);
}