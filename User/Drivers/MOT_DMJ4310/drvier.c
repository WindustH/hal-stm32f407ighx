/**
 * @file drvier.c
 * @brief DMJ4310电机驱动实现文件
 *
 * 实现DMJ4310电机的CAN通信配置、控制命令发送和状态更新功能
 */

#include "driver.h"
#include "main.h"

u8 DMJ4310_PROTECT_ON = false;
static CAN_HandleTypeDef *hcanx;
static volatile motStat_DMJ4310 mot_stat;
static volatile motCtrl_DMJ4310 mot_ctrl;

void setup_mot_dmj4310(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef can_filter = {0}; // 初始化为0更安全

  // 过滤器组 14
  can_filter.FilterBank = 14;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

  can_filter.FilterIdHigh = (0x206U << 5);
  can_filter.FilterIdLow = (0x206U << 5);

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

void mot_set_torque_dmj4310(f32 trq) {
  if (!DMJ4310_PROTECT_ON) {
    mot_ctrl.trq = trq;
    mot_ctrl.kd = 0.0f;
    mot_ctrl.kp = 0.0f;
    mot_ctrl.v = 0.0f;
    mot_ctrl.x = 0.0f;
  }
}

void mot_send_ctrl_msg_dmj4310() {
  if (!DMJ4310_PROTECT_ON) {
    motCtrlCanMsg_DMJ4310 can_msg;
    u32 unused_mailbox;
    mot_ctrl_pack_mit_dmj4310(&mot_ctrl, &can_msg);

    if (HAL_CAN_AddTxMessage(hcanx, &can_msg.header, can_msg.data,
                             &unused_mailbox) != HAL_OK) {
      return;
    }
  }
}

void mot_update_stat_dmj4310(CAN_HandleTypeDef *hcan) {
  if (hcanx == hcan) {
    canRxH header;
    u8 *data = {0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      return;
    }
    mot_fb_parse_dmj4310(&header, data, &mot_stat);
  }
}

volatile motStat_DMJ4310 *mot_get_stat_dmj4310() { return &mot_stat; }