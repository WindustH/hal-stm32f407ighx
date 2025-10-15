#include "protocol.h"
#include "conf.h"

// 位置缩放因子 - 原始值转弧度
static f32 s_pos_scale = DMJ4310_X_SCALE;
// 速度缩放因子 - 原始值转弧度/秒
static f32 s_vel_scale = DMJ4310_V_SCALE;
// 扭矩缩放因子 - 原始值转牛顿米
static f32 s_trq_scale = DMJ4310_TOR_SCALE;

void mot_dmj4310_set_scaling(f32 pos_scale, f32 vel_scale, f32 trq_scale) {
  s_pos_scale = pos_scale;
  s_vel_scale = vel_scale;
  s_trq_scale = trq_scale;
}

void mot_fb_parse_dmj4310(const volatile canRxH *msg, const volatile u8 *data,
                          volatile motStat_DMJ4310 *mot_stat_dmj4310) {
  // 只处理预期的反馈CAN ID
  if (msg->StdId != DMJ4310_PITCH_FEEDBACK_ID) {
    return;
  }

  // ---- CAN数据布局(8字节) ----
  // D[0]: [ID (4位) | 错误代码 (4位)]
  // D[1]: 位置[15:8]
  // D[2]: 位置[7:0]
  // D[3]: 速度[11:4]
  // D[4]: [速度[3:0] | 扭矩[11:8]]
  // D[5]: 扭矩[7:0]
  // D[6]: MOS温度(°C)
  // D[7]: 转子温度(°C)

  u8 id_and_error = data[0];
  u8 parsed_id = id_and_error & 0x0F;
  u8 error_code = (id_and_error >> 4) & 0x0F;

  // ---- 位置: 16位有符号整数 ----
  u16 raw_pos_u16 = (u16)((data[1] << 8) | data[2]);
  i16 raw_pos = (i16)raw_pos_u16;

  // ---- 速度: 12位有符号整数 ----
  i16 raw_vel = (i16)((data[3] << 4) | ((data[4] >> 4) & 0x0F));
  if (raw_vel & 0x0800)
    raw_vel |= 0xF000; // 符号扩展

  // ---- 扭矩: 12位有符号整数 ----
  i16 raw_trq = (i16)(((data[4] & 0x0F) << 8) | data[5]);
  if (raw_trq & 0x0800)
    raw_trq |= 0xF000; // 符号扩展

  // ---- 温度 ----
  u8 temp_mos = data[6];
  u8 temp_rotor = data[7];

  // ---- 多圈位置处理(单电机) ----
  static i32 prev_raw_pos = 0;

  i32 delta_raw = raw_pos - prev_raw_pos;
  i8 sign = 0;

  // 检测溢出/下溢
  if (delta_raw > DMJ4310_PI)
    sign = -1;
  else if (delta_raw < -DMJ4310_PI)
    sign = 1;

  // 计算总的位置(去缠绕)并转换为弧度
  f32 delta = (delta_raw + sign * DMJ4310_PI * 2.0f) * s_pos_scale;

  prev_raw_pos = raw_pos;

  // 填充反馈结构体
  mot_stat_dmj4310->x += delta;                  // 弧度(多圈)
  mot_stat_dmj4310->v = raw_vel * s_vel_scale;   // 弧度/秒
  mot_stat_dmj4310->trq = raw_trq * s_trq_scale; // 牛顿米
  mot_stat_dmj4310->error_code = error_code;
  mot_stat_dmj4310->T_mos = temp_mos;
  mot_stat_dmj4310->T_mot = temp_rotor;
  mot_stat_dmj4310->motor_id = parsed_id;
}

void mot_ctrl_pack_mit_dmj4310(const volatile motCtrl_DMJ4310 *ctrl_msg,
                               motCtrlCanMsg_DMJ4310 *can_msg) {
  static canTxH tx_header = {
      .ExtId = 0,
      .IDE = 0,
      .RTR = 0,
      .DLC = 8U,
      .TransmitGlobalTime = 0,
      .StdId = DMJ4310_PITCH_CAN_ID,
  };

  // 使用缩放因子将物理单位转换为原始整数
  i16 p_des_raw = (i16)(ctrl_msg->x / s_pos_scale);
  i16 v_des_raw = (i16)(ctrl_msg->v / s_vel_scale);
  i16 t_ff_raw = (i16)(ctrl_msg->trq / s_trq_scale);

  // 将Kp/Kd从浮点范围映射到12位无符号整数
  u16 kp_raw = (u16)((ctrl_msg->kp - DMJ4310_KP_RANGE_MIN) /
                     (DMJ4310_KP_RANGE_MAX - DMJ4310_KP_RANGE_MIN) * 4095.0f);
  u16 kd_raw = (u16)((ctrl_msg->kd - DMJ4310_KD_RANGE_MIN) /
                     (DMJ4310_KD_RANGE_MAX - DMJ4310_KD_RANGE_MIN) * 4095.0f);

  // 饱和到有效的位范围
  v_des_raw = (v_des_raw > 2047)    ? 2047
              : (v_des_raw < -2048) ? -2048
                                    : v_des_raw;
  t_ff_raw = (t_ff_raw > 2047) ? 2047 : (t_ff_raw < -2048) ? -2048 : t_ff_raw;
  kp_raw = (kp_raw > 4095) ? 4095 : kp_raw;
  kd_raw = (kd_raw > 4095) ? 4095 : kd_raw;

  // 打包到8字节CAN帧
  can_msg->data[0] = DMJ4310_PITCH_CAN_ID;
  can_msg->data[1] = (u8)((p_des_raw >> 8) & 0xFF);
  can_msg->data[2] = (u8)(p_des_raw & 0xFF);
  can_msg->data[3] = (u8)((v_des_raw >> 4) & 0xFF);
  can_msg->data[4] = (u8)(((v_des_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F));
  can_msg->data[5] = (u8)(kp_raw & 0xFF);
  can_msg->data[6] = (u8)((kd_raw >> 4) & 0xFF);
  can_msg->data[7] = (u8)(((kd_raw & 0x0F) << 4) | ((t_ff_raw >> 8) & 0x0F));
  // 注意: 由于CAN的8字节限制，扭矩前馈的低8位被省略。
  // 在CAN-FD中，可以在第9字节发送它们。

  can_msg->header = tx_header;
}