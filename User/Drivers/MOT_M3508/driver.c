/**
 * @file driver.c
 * @brief M3508电机驱动实现文件
 *
 * 实现M3508电机的CAN通信配置、多电机电流控制和状态更新功能
 */

#include "driver.h"
#include "main.h"
#include <stdlib.h>

u8 M3508_PROTECT_ON = false;
static CAN_HandleTypeDef *hcanx;                 ///< CAN对象指针
static volatile motStat_M3508 mot_stat[8] = {0}; ///< 8个电机的状态信息数组
static volatile motCtrl_M3508 mot_ctrl[8] = {0}; ///< 8个电机的控制信息数组

void m3508_setup(CAN_HandleTypeDef *hcan, u8 master) {
  CAN_FilterTypeDef can_filter = {0}; // 初始化为0更安全

  // 过滤器组 0 - 配置电机ID 0x201-0x202
  if (master) {
    can_filter.FilterBank = 0;
    can_filter.SlaveStartFilterBank = 14;
  } else
    can_filter.FilterBank = 14;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = (0x201U << 5);
  can_filter.FilterIdLow = (0x202U << 5);
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow = 0;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 1 - 配置电机ID 0x203-0x204
  if (master)
    can_filter.FilterBank = 1;
  else
    can_filter.FilterBank = 15;
  can_filter.FilterIdHigh = (0x203U << 5);
  can_filter.FilterIdLow = (0x204U << 5);

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 2 - 配置电机ID 0x205-0x206
  if (master)
    can_filter.FilterBank = 2;
  else
    can_filter.FilterBank = 16;
  can_filter.FilterIdHigh = (0x205U << 5);
  can_filter.FilterIdLow = (0x206U << 5);

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 3 - 配置电机ID 0x207-0x208
  if (master)
    can_filter.FilterBank = 3;
  else
    can_filter.FilterBank = 17;
  can_filter.FilterIdHigh = (0x207U << 5);
  can_filter.FilterIdLow = (0x208U << 5);

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 保存配置的 can 对象
  hcanx = hcan;
}

void m3508_set_current(u8 mot_id, f32 cur) {
  if (!M3508_PROTECT_ON)
    mot_ctrl[mot_id].I = (i16)cur;
}

void m3508_send_ctrl_msg() {
  if (!M3508_PROTECT_ON) {

    motCtrlCanMsg_M3508 can_msg;
    u8 block1[8];
    u8 block2[8];
    can_msg.data_1_4 = block1;
    can_msg.data_5_8 = block2;
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

void m3508_update_stat(CAN_HandleTypeDef *hcan) {
  if (hcan == hcanx) {
    canRxH header;
    u8 data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      return;
    }
    mot_fb_parse_m3508(&header, data, mot_stat);
  }
}

volatile motStat_M3508 *m3508_get_stat(u8 id) { return &mot_stat[id]; }

void m3508_reset_pos() {
  for (u8 i = 0; i < 8; i++)
    mot_stat[i].x = 0.0f;
}