/**
 * @file drvier.c
 * @brief DMJ6006电机驱动实现文件
 *
 * 实现DMJ6006电机的CAN通信配置、控制命令发送和状态更新功能
 */

#include "BSP/can_tx_queue.h"
#include "driver.h"
#include "main.h"

// CAN ID - MIT模式命令帧
volatile u32 dmj6006_can_id = 0x002U;
// 反馈帧ID - 来自电机
volatile u32 dmj6006_master_id = 0x003U;
volatile u8 dmj6006_protect_on = false;
static u8 dmj6006_enabled = false;
static CAN_HandleTypeDef *hcanx;
static volatile motStat_DMJ6006 dmj6006_mot_stat = {0};
static volatile motCtrl_DMJ6006 dmj6006_mot_ctrl = {0};

void dmj6006_setup(CAN_HandleTypeDef *hcan, u16 can_id, u16 master_id,
                   u32 filter_bank, u8 fifo) {
  dmj6006_can_id = can_id;
  dmj6006_master_id = master_id;

  CAN_FilterTypeDef can_filter = {0};
  can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterBank = filter_bank;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

  can_filter.FilterIdHigh = (dmj6006_master_id << 5) & 0xFFFF;
  can_filter.FilterIdLow = (dmj6006_master_id << 21) & 0xFFFF;

  can_filter.FilterMaskIdHigh = (dmj6006_master_id << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = (dmj6006_master_id << 21) & 0xFFFF;

  can_filter.FilterFIFOAssignment = fifo; // 分配到 FIFO0
  can_filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 保存配置的 can 对象
  hcanx = hcan;
}

void dmj6006_set_torque(f32 trq) {
  if (!dmj6006_protect_on) {
    dmj6006_mot_ctrl.trq = trq;
    dmj6006_mot_ctrl.kd = 0.0f;
    dmj6006_mot_ctrl.kp = 0.0f;
    dmj6006_mot_ctrl.v = 0.0f;
    dmj6006_mot_ctrl.x = 0.0f;
  }
}

void dmj6006_send_ctrl_msg() {

  if (!dmj6006_protect_on) {
    motCtrlCanMsg_DMJ6006 can_msg;
    u8 block[8];

    can_msg.data = block;
    if (!dmj6006_enabled) {
      dmj6006_enable_msg(&can_msg);
      if (can_send_message(hcanx, &can_msg.header, can_msg.data) != HAL_OK) {
        return;
      }
    }

    dmj6006_ctrl_pack_mit(&dmj6006_mot_ctrl, &can_msg);
    if (can_send_message(hcanx, &can_msg.header, can_msg.data) != HAL_OK) {
      return;
    }
  }
}

void dmj6006_disable() {
  dmj6006_enabled = false;
  motCtrlCanMsg_DMJ6006 can_msg;
  u8 block[8];
  can_msg.data = block;
  dmj6006_disable_msg(&can_msg);
  if (can_send_message(hcanx, &can_msg.header, can_msg.data) != HAL_OK) {
    return;
  }
}

void dmj6006_update_stat(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                         u8 data[8]) {
  if (hcanx == hcan && header->StdId == dmj6006_master_id) {
    dmj6006_fb_parse(header, data, &dmj6006_mot_stat);
  }
}

volatile motStat_DMJ6006 *dmj6006_get_stat() { return &dmj6006_mot_stat; }

void dmj6006_reset_pos() { dmj6006_mot_stat.x = 0.0f; }