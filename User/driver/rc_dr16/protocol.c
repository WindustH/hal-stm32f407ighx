#include "driver/rc_dr16/protocol.h"

void rc_ctrl_msg_parse_dr16(const volatile u8 *raw_data, rcCtrl_dr16 *out) {
  // Channel 0: byte0 + low 3 bits of byte1
  out->rc.ch0 = (raw_data[0] | ((u16)(raw_data[1] & 0x07) << 8)) & 0x07FF;

  // Channel 1: high 5 bits of byte1 + low 6 bits of byte2
  out->rc.ch1 =
      (((raw_data[1] >> 3) | ((u16)(raw_data[2] & 0x3F) << 5)) & 0x07FF);

  // Channel 2: high 2 bits of byte2 + byte3 + low 1 bit of byte4
  out->rc.ch2 = (((raw_data[2] >> 6) | ((u16)raw_data[3] << 2) |
                  ((u16)(raw_data[4] & 0x01) << 10)) &
                 0x07FF);

  // Channel 3: high 7 bits of byte4 + low 7 bits of byte5
  out->rc.ch3 =
      (((raw_data[4] >> 1) | ((u16)(raw_data[5] & 0x7F) << 7)) & 0x07FF);

  // Switches: bits 4~7 of byte5
  u8 sw_bits = (raw_data[5] >> 4) & 0x0F;
  out->rc.s1 = (sw_bits >> 2) & 0x03; // upper 2 bits -> s1
  out->rc.s2 = sw_bits & 0x03;        // lower 2 bits -> s2

  // Mouse data (little-endian int16)
  out->mouse.x = (i16)(raw_data[6] | ((u16)raw_data[7] << 8));
  out->mouse.y = (i16)(raw_data[8] | ((u16)raw_data[9] << 8));
  out->mouse.z = (i16)(raw_data[10] | ((u16)raw_data[11] << 8));
  out->mouse.press_l = raw_data[12];
  out->mouse.press_r = raw_data[13];

  // Keyboard
  out->key.v = raw_data[14] | ((u16)raw_data[15] << 8);
}