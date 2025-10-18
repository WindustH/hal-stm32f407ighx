// protocol.c
#include "protocol.h"
#include "conf.h"

// 缩放因子（来自 conf.h）
static u8 first_feedback = true;
static f32 s_pos_scale = DMJ4310_X_SCALE;   // 弧度 / raw
static f32 s_vel_scale = DMJ4310_V_SCALE;   // rad/s / raw
static f32 s_trq_scale = DMJ4310_TOR_SCALE; // Nm / raw

extern volatile u32 dmj4310_master_id;
extern volatile u32 dmj4310_can_id;

void mot_dmj4310_set_scaling(f32 pos_scale, f32 vel_scale, f32 trq_scale) {
  s_pos_scale = pos_scale;
  s_vel_scale = vel_scale;
  s_trq_scale = trq_scale;
}

void dmj4310_fb_parse(const volatile canRxH *msg, const volatile u8 *data,
                      volatile motStat_DMJ4310 *mot_stat_dmj4310) {
  if (msg->StdId != dmj4310_master_id) {
    return;
  }

  u8 id_and_error = data[0];
  u8 parsed_id = id_and_error & 0x0F;
  u8 error_code = (id_and_error >> 4) & 0x0F;

  // ---- 位置: 16位有符号整数 ----
  u16 raw_pos_u16 = (u16)((data[1] << 8) | data[2]);
  i16 raw_pos = (i16)raw_pos_u16;

  // ---- 速度: 12位有符号整数 ----
  i16 raw_vel = (i16)((data[3] << 4) | ((data[4] >> 4) & 0x0F));
  if (raw_vel & 0x0800)
    raw_vel |= 0xF000;

  // ---- 扭矩: 12位有符号整数 ----
  i16 raw_trq = (i16)(((data[4] & 0x0F) << 8) | data[5]);
  if (raw_trq & 0x0800)
    raw_trq |= 0xF000;

  // ---- 温度 ----
  u8 temp_mos = data[6];
  u8 temp_rotor = data[7];

  // ---- 多圈位置处理 ----
  static i32 prev_raw_pos = 0;
  f32 delta = 0.0f;
  if (first_feedback)
    first_feedback = false;
  else {
    i32 delta_raw = raw_pos - prev_raw_pos;
    i32 sign = 0;
    if (delta_raw > (i32)DMJ4310_PI)
      sign = -1;
    else if (delta_raw < -(i32)DMJ4310_PI)
      sign = 1;

    delta = (delta_raw + sign * DMJ4310_PI * 2.0f) * s_pos_scale;
  }
  prev_raw_pos = raw_pos;

  mot_stat_dmj4310->x += delta;
  mot_stat_dmj4310->v = raw_vel * s_vel_scale;
  mot_stat_dmj4310->trq = raw_trq * s_trq_scale;
  mot_stat_dmj4310->error_code = error_code;
  mot_stat_dmj4310->T_mos = temp_mos;
  mot_stat_dmj4310->T_mot = temp_rotor;
  mot_stat_dmj4310->motor_id = parsed_id;
}

void dmj4310_ctrl_pack_mit(const volatile motCtrl_DMJ4310 *ctrl_msg,
                           motCtrlCanMsg_DMJ4310 *can_msg) {

  u16 p_des_raw =
      (u16)f32_to_i32(ctrl_msg->x / s_pos_scale, -DMJ4310_PI, +DMJ4310_PI, 16);
  u16 v_des_raw = (u16)f32_to_i32(ctrl_msg->v / s_vel_scale,
                                  DMJ4310_V_RANGE_MIN, DMJ4310_V_RANGE_MAX, 12);
  u16 t_ff_raw = (u16)f32_to_i32(ctrl_msg->trq / s_trq_scale, DMJ4310_MIN_TRQ,
                                 DMJ4310_MAX_TRQ, 12);

  u16 kp_raw = (u16)f32_to_i32(ctrl_msg->kp, DMJ4310_KP_RANGE_MIN,
                               DMJ4310_KP_RANGE_MAX, 12);
  u16 kd_raw = (u16)f32_to_i32(ctrl_msg->kd, DMJ4310_KD_RANGE_MIN,
                               DMJ4310_KD_RANGE_MAX, 12);

  // 按照协议打包 8 字节数据（D[0] ~ D[7]）
  can_msg->data[0] = (u8)((p_des_raw >> 8) & 0xFF); // p[15:8]
  can_msg->data[1] = (u8)(p_des_raw & 0xFF);        // p[7:0]
  can_msg->data[2] = (u8)((v_des_raw >> 4) & 0xFF); // v[11:4]
  can_msg->data[3] = (u8)(((v_des_raw & 0x0F) << 4) |
                          ((kp_raw >> 8) & 0x0F)); // v[3:0] | Kp[11:8]
  can_msg->data[4] = (u8)(kp_raw & 0xFF);          // Kp[7:0]
  can_msg->data[5] = (u8)((kd_raw >> 4) & 0xFF);   // Kd[11:4]
  can_msg->data[6] = (u8)(((kd_raw & 0x0F) << 4) |
                          ((t_ff_raw >> 8) & 0x0F)); // Kd[3:0] | t_ff[11:8]
  can_msg->data[7] = (u8)(t_ff_raw & 0xFF);          // t_ff[7:0]

  can_msg->header.StdId = dmj4310_can_id;
  can_msg->header.IDE = CAN_ID_STD;
  can_msg->header.RTR = CAN_RTR_DATA;
  can_msg->header.DLC = 8U;
  can_msg->header.TransmitGlobalTime = DISABLE;
}

void dmj4310_enable_msg(motCtrlCanMsg_DMJ4310 *can_msg) {
  can_msg->header.StdId = dmj4310_can_id;
  can_msg->header.IDE = CAN_ID_STD;
  can_msg->header.RTR = CAN_RTR_DATA;
  can_msg->header.DLC = 8U;
  can_msg->header.TransmitGlobalTime = DISABLE;

  can_msg->data[0] = 0xFF;
  can_msg->data[1] = 0xFF;
  can_msg->data[2] = 0xFF;
  can_msg->data[3] = 0xFF;
  can_msg->data[4] = 0xFF;
  can_msg->data[5] = 0xFF;
  can_msg->data[6] = 0xFF;
  can_msg->data[7] = 0xFC;
}