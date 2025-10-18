/**
 * @file drvier.c
 * @brief DMJ4310电机驱动实现文件
 *
 * 实现DMJ4310电机的CAN通信配置、控制命令发送和状态更新功能
 */

#include "conf.h"
#include "driver.h"
#include "main.h"

volatile u8 DMJ4310_PROTECT_ON = false;
static u8 mot_enabled = false;
static CAN_HandleTypeDef *hcanx;
static volatile motStat_DMJ4310 mot_stat = {0};
static volatile motCtrl_DMJ4310 mot_ctrl = {0};

void dmj4310_setup(CAN_HandleTypeDef *hcan, u8 master) {
  CAN_FilterTypeDef can_filter = {0}; // 初始化为0更安全

  // 过滤器组 14
  if (master) {
    can_filter.FilterBank = 0;
    can_filter.SlaveStartFilterBank = 14;
  } else
    can_filter.FilterBank = 14;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

  can_filter.FilterIdHigh = (DMJ4310_MASTER_ID << 5);
  can_filter.FilterIdLow = (DMJ4310_MASTER_ID << 5);

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

void dmj4310_set_torque(f32 trq) {
  if (!DMJ4310_PROTECT_ON) {
    mot_ctrl.trq = trq;
    mot_ctrl.kd = 0.0f;
    mot_ctrl.kp = 0.0f;
    mot_ctrl.v = 0.0f;
    mot_ctrl.x = 0.0f;
  }
}

void dmj4310_send_ctrl_msg() {

  if (!DMJ4310_PROTECT_ON) {
    motCtrlCanMsg_DMJ4310 can_msg;
    u32 unused_mailbox;
    u8 block[8];

    can_msg.data = block;
    if (!mot_enabled) {
      mot_enable_msg_dmj4310(&can_msg);
      if (HAL_CAN_AddTxMessage(hcanx, &can_msg.header, can_msg.data,
                               &unused_mailbox) != HAL_OK) {
        return;
      }
    }

    mot_ctrl_pack_mit_dmj4310(&mot_ctrl, &can_msg);

    if (HAL_CAN_AddTxMessage(hcanx, &can_msg.header, can_msg.data,
                             &unused_mailbox) != HAL_OK) {
      return;
    }
  }
}

void dmj4310_update_stat(CAN_HandleTypeDef *hcan) {
  if (hcanx == hcan) {
    canRxH header;
    u8 data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      return;
    }
    mot_fb_parse_dmj4310(&header, data, &mot_stat);
  }
}

volatile motStat_DMJ4310 *dmj4310_get_stat() { return &mot_stat; }

void dmj4310_reset_pos() { mot_stat.x = 0.0f; }