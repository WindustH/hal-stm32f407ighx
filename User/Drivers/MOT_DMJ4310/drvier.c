#include "driver.h"
#include "main.h"
#include "protocol.h"

static CAN_HandleTypeDef *hcan;
static volatile motStat_DMJ4310 mot_stat;
static volatile motCtrl_DMJ4310 mot_ctrl;

void mot_setup_can_dmj4310(CAN_HandleTypeDef *hcanx) {
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

void mot_set_torque_dmj4310(f32 trq) {
  mot_ctrl.trq = trq;
  mot_ctrl.kd = 0.0f;
  mot_ctrl.kp = 0.0f;
  mot_ctrl.v = 0.0f;
  mot_ctrl.x = 0.0f;
}

void mot_send_ctrl_msg_dmj4310() {
  motCtrlCanMsg_DMJ4310 can_msg;
  u32 unused_mailbox;
  mot_ctrl_pack_mit_dmj4310(&mot_ctrl, &can_msg);

  if (HAL_CAN_AddTxMessage(hcan, &can_msg.header, can_msg.data,
                           &unused_mailbox) != HAL_OK) {
    return;
  }
}

void mot_update_stat_dmj4310(const volatile canRxH *msg,
                             const volatile u8 *data) {
  mot_fb_parse_dmj4310(msg, data, &mot_stat);
}