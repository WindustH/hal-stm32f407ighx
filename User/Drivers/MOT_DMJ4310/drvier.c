/**
 * @file drvier.c
 * @brief DMJ4310电机驱动实现文件
 *
 * 实现DMJ4310电机的CAN通信配置、控制命令发送和状态更新功能
 */

#include "BSP/can_tx_queue.h"
#include "driver.h"
#include "main.h"

// CAN ID - MIT模式命令帧
volatile u32 dmj4310_can_id = 0x002U;
// 反馈帧ID - 来自电机
volatile u32 dmj4310_master_id = 0x003U;
volatile u8 dmj4310_protect_on = false;
static u8 mot_enabled = false;
static CAN_HandleTypeDef *hcanx;
static volatile motStat_DMJ4310 mot_stat = {0};
static volatile motCtrl_DMJ4310 mot_ctrl = {0};

void dmj4310_setup(CAN_HandleTypeDef *hcan, u32 can_id, u32 master_id,
                   u32 filter_bank) {
  dmj4310_can_id = can_id;
  dmj4310_master_id = master_id;

  CAN_FilterTypeDef can_filter = {0};
  can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterBank = filter_bank;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

  can_filter.FilterIdHigh = (dmj4310_master_id << 5) & 0xFFFF;
  can_filter.FilterIdLow = (dmj4310_master_id << 21) & 0xFFFF;
  ;

  can_filter.FilterMaskIdHigh = (dmj4310_master_id << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = (dmj4310_master_id << 21) & 0xFFFF;

  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 分配到 FIFO0
  can_filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 保存配置的 can 对象
  hcanx = hcan;
}

void dmj4310_set_torque(f32 trq) {
  if (!dmj4310_protect_on) {
    mot_ctrl.trq = trq;
    mot_ctrl.kd = 0.0f;
    mot_ctrl.kp = 0.0f;
    mot_ctrl.v = 0.0f;
    mot_ctrl.x = 0.0f;
  }
}

void dmj4310_send_ctrl_msg() {

  if (!dmj4310_protect_on) {
    motCtrlCanMsg_DMJ4310 can_msg;
    u8 block[8];

    can_msg.data = block;
    if (!mot_enabled) {
      dmj4310_enable_msg(&can_msg);
      if (can_send_message(hcanx, &can_msg.header, can_msg.data) != HAL_OK) {
        return;
      }
    }

    dmj4310_ctrl_pack_mit(&mot_ctrl, &can_msg);
    if (can_send_message(hcanx, &can_msg.header, can_msg.data) != HAL_OK) {
      return;
    }
  }
}

void dmj4310_update_stat(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                         u8 data[8]) {
  if (hcanx == hcan && header->StdId == dmj4310_master_id) {
    dmj4310_fb_parse(header, data, &mot_stat);
  }
}

volatile motStat_DMJ4310 *dmj4310_get_stat() { return &mot_stat; }

void dmj4310_reset_pos() { mot_stat.x = 0.0f; }