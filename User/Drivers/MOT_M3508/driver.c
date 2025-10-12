/**
 * @file driver.c
 * @brief M3508电机驱动实现文件
 *
 * 实现M3508电机的CAN通信配置、多电机电流控制和状态更新功能
 */

#include "driver.h"
#include "main.h"

u8 M3508_PROTECT_ON = false;
static CAN_HandleTypeDef *hcanx;           ///< CAN对象指针
static volatile motStat_M3508 mot_stat[8]; ///< 8个电机的状态信息数组
static volatile motCtrl_M3508 mot_ctrl[8]; ///< 8个电机的控制信息数组

void mot_setup_can_m3508(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef can_filter = {0}; // 初始化为0更安全

  // 过滤器组 0 - 配置电机ID 0x201-0x202
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

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 1 - 配置电机ID 0x203-0x204
  can_filter.FilterBank = 1;
  can_filter.FilterIdHigh = (0x203U << 5);
  can_filter.FilterIdLow = (0x204U << 5);

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 2 - 配置电机ID 0x205-0x206
  can_filter.FilterBank = 2;
  can_filter.FilterIdHigh = (0x205U << 5);
  can_filter.FilterIdLow = (0x206U << 5);

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 3 - 配置电机ID 0x207-0x208
  can_filter.FilterBank = 3;
  can_filter.FilterIdHigh = (0x207U << 5);
  can_filter.FilterIdLow = (0x208U << 5);

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 保存配置的 can 对象
  hcanx = hcan;
}

void mot_set_current_m3508(u8 mot_id, i16 cur) {
  if (!M3508_PROTECT_ON)
    mot_ctrl[mot_id].I = cur;
}

void mot_send_ctrl_msg_m3508() {
  if (!M3508_PROTECT_ON) {
    motCtrlCanMsg_M3508 can_msg;
    u32 unused_mailbox;
    mot_ctrl_pack_msg_m3508(mot_ctrl, &can_msg);

    // 发送电机1-4的控制消息
    if (HAL_CAN_AddTxMessage(hcanx, &can_msg.header_1_4, can_msg.data_1_4,
                             &unused_mailbox) != HAL_OK) {
      return;
    }

    // 发送电机5-8的控制消息
    if (HAL_CAN_AddTxMessage(hcanx, &can_msg.header_5_8, can_msg.data_5_8,
                             &unused_mailbox) != HAL_OK) {
      return;
    }
  }
}

void mot_update_stat_m3508(CAN_HandleTypeDef *hcan) {
  if (hcan == hcanx) {
    canRxH header;
    u8 *data = {0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      return;
    }
    mot_fb_parse_m3508(&header, data, mot_stat);
  }
}

volatile motStat_M3508 *mot_get_stat_m3508(u8 id) { return &mot_stat[id]; }