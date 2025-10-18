#include "protocol.h"
#include "conf.h"

// CAN头和数据缓冲区
static const canTxH tx_header_1_4 = {
    .ExtId = 0,
    .IDE = 0U,
    .RTR = 0U,
    .DLC = 8U,
    .TransmitGlobalTime = 0U,
    .StdId = M3508_CONTROL_ID_1_4,
};
static const canTxH tx_header_5_8 = {
    .ExtId = 0,
    .IDE = 0U,
    .RTR = 0U,
    .DLC = 8U,
    .TransmitGlobalTime = 0U,
    .StdId = M3508_CONTROL_ID_5_8,
};

static u8 first_feedback[8] = {true};

// 位置缩放因子
static f32 s_pos_scale = M3508_X_SCALE;
// 速度缩放因子
static f32 s_vel_scale = M3508_V_SCALE;
// 电流缩放因子
static f32 s_cur_scale = M3508_I_SCALE;

void mot_m3508_set_scaling(f32 pos_scale, f32 vel_scale, f32 cur_scale) {
  s_pos_scale = pos_scale;
  s_vel_scale = vel_scale;
  s_cur_scale = cur_scale;
}

void mot_ctrl_pack_msg_m3508(const volatile motCtrl_M3508 *ctrl,
                             motCtrlCanMsg_M3508 *can_msg) {
  can_msg->header_1_4 = tx_header_1_4;
  can_msg->header_5_8 = tx_header_5_8;

  for (u8 i = 0; i < 4; i++) {
    can_msg->data_1_4[i * 2] = (ctrl[i].I >> 8) & 0xFF;
    can_msg->data_1_4[i * 2 + 1] = ctrl[i].I & 0xFF;
  }
  for (u8 i = 0; i < 4; i++) {
    can_msg->data_5_8[i * 2] = (ctrl[i + 4].I >> 8) & 0xFF;
    can_msg->data_5_8[i * 2 + 1] = ctrl[i + 4].I & 0xFF;
  }
}

void mot_fb_parse_m3508(const volatile canRxH *msg, const volatile u8 *data,
                        volatile motStat_M3508 *mot_stat) {
  u32 std_id = msg->StdId;

  // M3508反馈ID范围: 0x201到0x208
  if (std_id < 0x201U || std_id > 0x208U)
    return;

  u8 motor_id = (u8)(std_id - 0x201U); // 0到7

  // ---- 解析原始数据 ----
  u16 raw_pos = (u16)((data[0] << 8) | data[1]); // 14位位置(0-8191)
  i16 raw_vel = (i16)((data[2] << 8) | data[3]); // 转速(有符号)
  i16 raw_cur = (i16)((data[4] << 8) | data[5]); // 电流(通常为毫安)
  u8 temp = data[6];

  // ---- 多圈位置跟踪 ----
  static u16 prev_raw_pos[8] = {0};
  f32 delta = 0.0f;
  if (first_feedback[motor_id])
    first_feedback[motor_id] = false;
  else {
    i32 sign = 0;
    i16 delta_raw = raw_pos - prev_raw_pos[motor_id];
    // 处理编码器环绕
    if (delta_raw > M3508_PI)
      sign = -1;
    else if (delta_raw < -M3508_PI)
      sign = 1;
    delta = (delta_raw + sign * M3508_PI * 2.0f) * s_pos_scale;
  }

  prev_raw_pos[motor_id] = raw_pos;

  // 应用缩放因子转换为物理单位
  mot_stat[motor_id].x += delta;                // 弧度
  mot_stat[motor_id].v = raw_vel * s_vel_scale; // 弧度/秒
  mot_stat[motor_id].I = raw_cur * s_cur_scale; // 安培
  mot_stat[motor_id].T = temp;
}