#include "protocol.h"
#include <math.h>

void rc_ctrl_msg_parse_dr16(const volatile u8 *raw_data,
                            volatile rcCtrl_dr16 *out) {
  //   // 通道0: byte0 (8 bits) + byte1[2:0] (3 bits) → bits 0–10
  //   out->rc.ch0 = (raw_data[0] | ((u16)(raw_data[1] & 0x07) << 8)) & 0x07FF;

  //   // 通道1: byte1[7:3] (5 bits) + byte2[5:0] (6 bits) → bits 11–21
  //   out->rc.ch1 =
  //       (((raw_data[1] >> 3) | ((u16)(raw_data[2] & 0x3F) << 5)) & 0x07FF);

  //   // 通道2: byte2[7:6] (2 bits) + byte3 (8 bits) + byte4[0] (1 bit) → bits
  //   22–32 out->rc.ch2 = (((raw_data[2] >> 6) | ((u16)raw_data[3] << 2) |
  //                   ((u16)(raw_data[4] & 0x01) << 10)) &
  //                  0x07FF);

  //   // 通道3: byte4[7:1] (7 bits) + byte5[3:0] (4 bits) → bits 33–43
  //   out->rc.ch3 =
  //       (((raw_data[4] >> 1) | ((u16)(raw_data[5] & 0x0F) << 7)) & 0x07FF);

  //   // 开关 S1/S2: byte5[7:4] → S1 = [7:6], S2 = [5:4]
  //   u8 sw_bits = (raw_data[5] >> 4) & 0x0F;
  //   out->rc.s1 = (sw_bits >> 2) & 0x03; // 高2位
  //   out->rc.s2 = sw_bits & 0x03;        // 低2位

  //   // 鼠标坐标（小端序，16位有符号）
  //   out->mouse.x = (i16)(raw_data[6] | ((u16)raw_data[7] << 8));
  //   out->mouse.y = (i16)(raw_data[8] | ((u16)raw_data[9] << 8));
  //   out->mouse.z = (i16)(raw_data[10] | ((u16)raw_data[11] << 8));

  //   // 鼠标按键：仅 bit0 有效
  //   out->mouse.press_l = raw_data[12] & 0x01;
  //   out->mouse.press_r = raw_data[13] & 0x01;

  // 键盘：小端序 16 位
  out->key.v = raw_data[14] | ((u16)raw_data[15] << 8);

  u16 ch0_raw = (raw_data[0] | (raw_data[1] << 8)) & 0x07ff;
  out->rc.ch0 =
      ((f32)ch0_raw - (364.0f + 1684.0f) / 2) / (-364.0f + 1684.0f) * 2;
  u16 ch1_raw = ((raw_data[1] >> 3) | (raw_data[2] << 5)) & 0x07ff;
  out->rc.ch1 =
      ((f32)ch1_raw - (364.0f + 1684.0f) / 2) / (-364.0f + 1684.0f) * 2;
  u16 ch2_raw =
      ((raw_data[2] >> 6) | (raw_data[3] << 2) | (raw_data[4] << 10)) & 0x07ff;
  out->rc.ch2 =
      -((f32)ch2_raw - (364.0f + 1684.0f) / 2) / (-364.0f + 1684.0f) * 2;
  u16 ch3_raw = ((raw_data[4] >> 1) | (raw_data[5] << 7)) & 0x07ff;
  out->rc.ch3 =
      ((f32)ch3_raw - (364.0f + 1684.0f) / 2) / (-364.0f + 1684.0f) * 2;
  out->rc.ch3 = cbrtf(out->rc.ch3);
  out->rc.s1 = ((raw_data[5] >> 4) & 0x000C) >> 2; //!< Switch left
  out->rc.s2 = ((raw_data[5] >> 4) & 0x0003);      //!< Switch right

  out->mouse.x = raw_data[6] | (raw_data[7] << 8);
  out->mouse.y = raw_data[8] | (raw_data[9] << 8);
  out->mouse.z = raw_data[10] | (raw_data[11] << 8);
  out->mouse.press_l = raw_data[12];
  out->mouse.press_r = raw_data[13];
  out->key.v = raw_data[14] | (raw_data[15] << 8);
}