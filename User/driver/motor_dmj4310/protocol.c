#include "driver/motor_dmj4310/protocol.h"
#include "driver/motor_dmj4310/conf.h"

static f32 s_pos_scale = DMJ4310_X_SCALE;   // raw -> rad
static f32 s_vel_scale = DMJ4310_V_SCALE;   // raw -> rad/s
static f32 s_trq_scale = DMJ4310_TOR_SCALE; // raw -> Nm

// Configure scaling factors
void mot_dmj4310_set_scaling(f32 pos_scale, f32 vel_scale, f32 trq_scale) {
  s_pos_scale = pos_scale;
  s_vel_scale = vel_scale;
  s_trq_scale = trq_scale;
}

// Parse motor feedback from CAN message
void mot_fb_parse_dmj4310(const volatile canRxH *msg, const volatile u8 *data,
                          motStat_DMJ4310 *mot_stat_dmj4310) {
  // Only process the expected feedback CAN ID
  if (msg->StdId != DMJ4310_PITCH_FEEDBACK_ID) {
    return;
  }

  // ---- CAN Data Layout (8 bytes) ----
  // D[0]: [ID (4b) | Error Code (4b)]
  // D[1]: Position[15:8]
  // D[2]: Position[7:0]
  // D[3]: Velocity[11:4]
  // D[4]: [Velocity[3:0] | Torque[11:8]]
  // D[5]: Torque[7:0]
  // D[6]: MOS Temperature (°C)
  // D[7]: Rotor Temperature (°C)

  u8 id_and_error = data[0];
  u8 parsed_id = id_and_error & 0x0F;
  u8 error_code = (id_and_error >> 4) & 0x0F;

  // ---- Position: 16-bit signed integer ----
  u16 raw_pos_u16 = (u16)((data[1] << 8) | data[2]);
  i16 raw_pos = (i16)raw_pos_u16;

  // ---- Velocity: 12-bit signed integer (bits 11..0) ----
  i16 raw_vel = (i16)((data[3] << 4) | ((data[4] >> 4) & 0x0F));
  if (raw_vel & 0x0800)
    raw_vel |= 0xF000; // sign-extend

  // ---- Torque: 12-bit signed integer (bits 11..0) ----
  i16 raw_trq = (i16)(((data[4] & 0x0F) << 8) | data[5]);
  if (raw_trq & 0x0800)
    raw_trq |= 0xF000; // sign-extend

  // ---- Temperatures ----
  u8 temp_mos = data[6];
  u8 temp_rotor = data[7];

  // ---- Multi-turn position handling (for single motor) ----
  static i16 prev_raw_pos = 0;
  static i32 round_count = 0;

  i16 delta_raw = raw_pos - prev_raw_pos;

  // Detect overflow/underflow (position wraps around ±32768)
  if (delta_raw > DMJ4310_PI) {
    round_count--;
  } else if (delta_raw < -DMJ4310_PI) {
    round_count++;
  }

  prev_raw_pos = raw_pos;

  // Compute total unwrapped position in radians
  f32 pos_total = (raw_pos + round_count * DMJ4310_PI * 2) * s_pos_scale;

  // Fill feedback structure
  mot_stat_dmj4310->x = pos_total;               // rad (multi-turn)
  mot_stat_dmj4310->v = raw_vel * s_vel_scale;   // rad/s
  mot_stat_dmj4310->tor = raw_trq * s_trq_scale; // Nm
  mot_stat_dmj4310->error_code = error_code;
  mot_stat_dmj4310->T_mos = temp_mos;
  mot_stat_dmj4310->T_mot = temp_rotor;
  mot_stat_dmj4310->motor_id = parsed_id;
}

// Pack MIT-mode control command into CAN message
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

  // Convert physical units to raw integers using scaling
  i16 p_des_raw = (i16)(ctrl_msg->x / s_pos_scale);
  i16 v_des_raw = (i16)(ctrl_msg->v / s_vel_scale);
  i16 t_ff_raw = (i16)(ctrl_msg->tor / s_trq_scale);

  // Map Kp/Kd from float range to 12-bit unsigned integers
  u16 kp_raw = (u16)((ctrl_msg->kp - DMJ4310_KP_RANGE_MIN) /
                     (DMJ4310_KP_RANGE_MAX - DMJ4310_KP_RANGE_MIN) * 4095.0f);
  u16 kd_raw = (u16)((ctrl_msg->kd - DMJ4310_KD_RANGE_MIN) /
                     (DMJ4310_KD_RANGE_MAX - DMJ4310_KD_RANGE_MIN) * 4095.0f);

  // Saturate to valid bit ranges
  p_des_raw = (p_des_raw > 32767)    ? 32767
              : (p_des_raw < -32768) ? -32768
                                     : p_des_raw;
  v_des_raw = (v_des_raw > 2047)    ? 2047
              : (v_des_raw < -2048) ? -2048
                                    : v_des_raw;
  t_ff_raw = (t_ff_raw > 2047) ? 2047 : (t_ff_raw < -2048) ? -2048 : t_ff_raw;
  kp_raw = (kp_raw > 4095) ? 4095 : kp_raw;
  kd_raw = (kd_raw > 4095) ? 4095 : kd_raw;

  // Pack into 8-byte CAN frame
  can_msg->data[0] = DMJ4310_PITCH_CAN_ID;
  can_msg->data[1] = (u8)((p_des_raw >> 8) & 0xFF);
  can_msg->data[2] = (u8)(p_des_raw & 0xFF);
  can_msg->data[3] = (u8)((v_des_raw >> 4) & 0xFF);
  can_msg->data[4] = (u8)(((v_des_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F));
  can_msg->data[5] = (u8)(kp_raw & 0xFF);
  can_msg->data[6] = (u8)((kd_raw >> 4) & 0xFF);
  can_msg->data[7] = (u8)(((kd_raw & 0x0F) << 4) | ((t_ff_raw >> 8) & 0x0F));
  // Note: Lower 8 bits of torque feedforward are omitted due to 8-byte CAN
  // limit. On CAN-FD, you could send them in a 9th byte.

  can_msg->header = tx_header;
}