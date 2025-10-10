#include "driver/motor_m3508/protocol.h"
#include "driver/motor_m3508/conf.h"

// CAN headers and data buffers
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

static f32 s_pos_scale = M3508_X_SCALE;
static f32 s_vel_scale = M3508_V_SCALE;
static f32 s_cur_scale = M3508_I_SCALE;

// Configure scaling factors
void mot_m3508_set_scaling(f32 pos_scale, f32 vel_scale, f32 cur_scale) {
  s_pos_scale = pos_scale;
  s_vel_scale = vel_scale;
  s_cur_scale = cur_scale;
}

// Pack control message for 8 motors (1-4 and 5-8)
void mot_ctrl_pack_msg_m3508(const volatile motCtrl_M3508 *ctrl,
                             motCtrlCanMsg_M3508 *can_msg) {
  *can_msg = (motCtrlCanMsg_M3508){
      .header_1_4 = tx_header_1_4,
      .header_5_8 = tx_header_5_8,
  };
  for (u8 i = 0; i < 4; i++) {
    can_msg->data_1_4[i * 2] = (ctrl[i].I >> 8) & 0xFF;
    can_msg->data_1_4[i * 2 + 1] = ctrl[i].I & 0xFF;
  }
  for (u8 i = 0; i < 4; i++) {
    can_msg->data_5_8[i * 2] = (ctrl[i + 4].I >> 8) & 0xFF;
    can_msg->data_5_8[i * 2 + 1] = ctrl[i + 4].I & 0xFF;
  }
}

// Parse feedback from CAN (handles motors 1–8)
void mot_fb_parse_m3508(const volatile canRxH *msg, const volatile u8 *data,
                        motStat_M3508 *mot_stat) {
  u32 std_id = msg->StdId;

  // M3508 feedback IDs: 0x201 to 0x208
  if (std_id < 0x201U || std_id > 0x208U) {
    return;
  }

  u8 motor_id = (u8)(std_id - 0x201U); // 0 to 7

  // ---- Parse raw data ----
  u16 raw_pos = (u16)((data[0] << 8) | data[1]); // 14-bit position (0–8191)
  i16 raw_vel = (i16)((data[2] << 8) | data[3]); // RPM (signed)
  i16 raw_cur = (i16)((data[4] << 8) | data[5]); // Current (typically mA)
  u8 temp = data[6];

  // ---- Multi-turn position tracking ----
  static u16 prev_raw_pos[8] = {0};
  static i32 turn_count[8] = {0};

  i16 delta = raw_pos - prev_raw_pos[motor_id];

  // Handle encoder wrap-around (8192 per revolution)
  if (delta > M3508_PI) {
    turn_count[motor_id]--;
  } else if (delta < -M3508_PI) {
    turn_count[motor_id]++;
  }

  prev_raw_pos[motor_id] = raw_pos;

  // Total position in encoder ticks
  i32 total_ticks = (i32)raw_pos + turn_count[motor_id] * 8192;

  // Apply scaling to physical units
  mot_stat[motor_id].x = total_ticks * s_pos_scale; // rad
  mot_stat[motor_id].v = raw_vel * s_vel_scale;     // rad/s
  mot_stat[motor_id].I = raw_cur * s_cur_scale;     // A
  mot_stat[motor_id].T = temp;
}