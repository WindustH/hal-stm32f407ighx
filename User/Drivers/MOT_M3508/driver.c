/**
 * @file driver.c
 * @brief M3508电机驱动实现文件
 *
 * 实现M3508电机的CAN通信配置、多电机电流控制和状态更新功能
 */

#include "driver.h"
#include "BSP/can_tx_queue.h"
#include "main.h"
volatile u8 m3508_protect_on = false;
static CAN_HandleTypeDef *hcanx;                 ///< CAN对象指针
static volatile motStat_M3508 mot_stat[8] = {0}; ///< 8个电机的状态信息数组
static volatile motCtrl_M3508 mot_ctrl[8] = {0}; ///< 8个电机的控制信息数组

void m3508_setup(CAN_HandleTypeDef *hcan, u32 filter_bank, u8 fifo) {
  CAN_FilterTypeDef can_filter = {0}; // 初始化为0更安全

  // 过滤器组 0 - 配置电机ID 0x201-0x202
  can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterBank = filter_bank;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = (0x201U << 5) & 0xFFFF;
  can_filter.FilterIdLow = (0x201U << 21) & 0xFFFF;
  can_filter.FilterMaskIdHigh = (0x202U << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = (0x202U << 21) & 0xFFFF;
  can_filter.FilterFIFOAssignment = fifo;
  can_filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 1 - 配置电机ID 0x203-0x204
  can_filter.FilterBank = filter_bank + 1;
  can_filter.FilterIdHigh = (0x203U << 5) & 0xFFFF;
  can_filter.FilterIdLow = (0x203U << 21) & 0xFFFF;
  can_filter.FilterMaskIdHigh = (0x204U << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = (0x204U << 21) & 0xFFFF;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 2 - 配置电机ID 0x205-0x206
  can_filter.FilterBank = filter_bank + 2;
  can_filter.FilterIdHigh = (0x205U << 5) & 0xFFFF;
  can_filter.FilterIdLow = (0x205U << 21) & 0xFFFF;
  can_filter.FilterMaskIdHigh = (0x206U << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = (0x206U << 21) & 0xFFFF;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 过滤器组 3 - 配置电机ID 0x207-0x208
  can_filter.FilterBank = filter_bank + 3;
  can_filter.FilterIdHigh = (0x207U << 5) & 0xFFFF;
  can_filter.FilterIdLow = (0x207U << 21) & 0xFFFF;
  can_filter.FilterMaskIdHigh = (0x208U << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = (0x208U << 21) & 0xFFFF;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 保存配置的 can 对象
  hcanx = hcan;
}

void m3508_set_current(u8 mot_id, f32 cur) {
  if (!m3508_protect_on)
    mot_ctrl[mot_id].I = (i16)cur;
}

void m3508_send_ctrl_msg() {
  if (!m3508_protect_on) {

    motCtrlCanMsg_M3508 can_msg;
    u8 block1[8];
    u8 block2[8];
    can_msg.data_1_4 = block1;
    can_msg.data_5_8 = block2;
    m3508_ctrl_pack_msg(mot_ctrl, &can_msg);
    // 发送电机1-4的控制消息
    if (can_send_message(hcanx, &can_msg.header_1_4, can_msg.data_1_4) !=
        HAL_OK) {
      return;
    }
    // 发送电机5-8的控制消息
    if (can_send_message(hcanx, &can_msg.header_5_8, can_msg.data_5_8) !=
        HAL_OK) {
      return;
    }
  }
}

void m3508_update_stat(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                       u8 data[8]) {
  if (hcan == hcanx && header->StdId >= 0x201U && header->StdId <= 0x208U) {
    m3508_fb_parse(header, data, mot_stat);
  }
}

volatile motStat_M3508 *m3508_get_stat(u8 id) { return &mot_stat[id]; }

void m3508_reset_pos() {
  for (u8 i = 0; i < 8; i++)
    mot_stat[i].x = 0.0f;
}