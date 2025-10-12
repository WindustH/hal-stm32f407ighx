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

// 位置缩放因子
static f32 s_pos_scale = M3508_X_SCALE;
// 速度缩放因子
static f32 s_vel_scale = M3508_V_SCALE;
// 电流缩放因子
static f32 s_cur_scale = M3508_I_SCALE;

/**
 * @brief 配置M3508电机的缩放因子
 * @param pos_scale 位置缩放因子
 * @param vel_scale 速度缩放因子
 * @param cur_scale 电流缩放因子
 */
void mot_m3508_set_scaling(f32 pos_scale, f32 vel_scale, f32 cur_scale) {
  s_pos_scale = pos_scale;
  s_vel_scale = vel_scale;
  s_cur_scale = cur_scale;
}

/**
 * @brief 将控制命令打包为8个电机的控制消息(1-4和5-8)
 * @param ctrl 控制命令结构体指针
 * @param can_msg CAN消息结构体指针
 */
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

/**
 * @brief 从CAN解析反馈数据(处理电机1-8)
 * @param msg CAN接收头指针
 * @param data CAN数据指针
 * @param mot_stat 电机状态结构体指针
 */
void mot_fb_parse_m3508(const volatile canRxH *msg, const volatile u8 *data,
                        volatile motStat_M3508 *mot_stat) {
  u32 std_id = msg->StdId;

  // M3508反馈ID范围: 0x201到0x208
  if (std_id < 0x201U || std_id > 0x208U) {
    return;
  }

  u8 motor_id = (u8)(std_id - 0x201U); // 0到7

  // ---- 解析原始数据 ----
  u16 raw_pos = (u16)((data[0] << 8) | data[1]); // 14位位置(0-8191)
  i16 raw_vel = (i16)((data[2] << 8) | data[3]); // 转速(有符号)
  i16 raw_cur = (i16)((data[4] << 8) | data[5]); // 电流(通常为毫安)
  u8 temp = data[6];

  // ---- 多圈位置跟踪 ----
  static u16 prev_raw_pos[8] = {0};
  static i32 turn_count[8] = {0};

  i16 delta = raw_pos - prev_raw_pos[motor_id];

  // 处理编码器环绕(每圈8192个脉冲)
  if (delta > M3508_PI) {
    turn_count[motor_id]--;
  } else if (delta < -M3508_PI) {
    turn_count[motor_id]++;
  }

  prev_raw_pos[motor_id] = raw_pos;

  // 编码器总脉冲数
  i32 total_ticks = (i32)raw_pos + turn_count[motor_id] * 8192;

  // 应用缩放因子转换为物理单位
  mot_stat[motor_id].x = total_ticks * s_pos_scale; // 弧度
  mot_stat[motor_id].v = raw_vel * s_vel_scale;     // 弧度/秒
  mot_stat[motor_id].I = raw_cur * s_cur_scale;     // 安培
  mot_stat[motor_id].T = temp;
}