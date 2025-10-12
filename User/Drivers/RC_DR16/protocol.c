#include "protocol.h"

void rc_ctrl_msg_parse_dr16(const volatile u8 *raw_data,
                            volatile rcCtrl_dr16 *out) {
  // 通道0: byte0 + byte1的低3位
  out->rc.ch0 = (raw_data[0] | ((u16)(raw_data[1] & 0x07) << 8)) & 0x07FF;

  // 通道1: byte1的高5位 + byte2的低6位
  out->rc.ch1 =
      (((raw_data[1] >> 3) | ((u16)(raw_data[2] & 0x3F) << 5)) & 0x07FF);

  // 通道2: byte2的高2位 + byte3 + byte4的低1位
  out->rc.ch2 = (((raw_data[2] >> 6) | ((u16)raw_data[3] << 2) |
                  ((u16)(raw_data[4] & 0x01) << 10)) &
                 0x07FF);

  // 通道3: byte4的高7位 + byte5的低7位
  out->rc.ch3 =
      (((raw_data[4] >> 1) | ((u16)(raw_data[5] & 0x7F) << 7)) & 0x07FF);

  // 开关: byte5的位4~7
  u8 sw_bits = (raw_data[5] >> 4) & 0x0F;
  out->rc.s1 = (sw_bits >> 2) & 0x03; // 高2位 -> s1
  out->rc.s2 = sw_bits & 0x03;        // 低2位 -> s2

  // 鼠标数据(小端序int16)
  out->mouse.x = (i16)(raw_data[6] | ((u16)raw_data[7] << 8));
  out->mouse.y = (i16)(raw_data[8] | ((u16)raw_data[9] << 8));
  out->mouse.z = (i16)(raw_data[10] | ((u16)raw_data[11] << 8));
  out->mouse.press_l = raw_data[12];
  out->mouse.press_r = raw_data[13];

  // 键盘
  out->key.v = raw_data[14] | ((u16)raw_data[15] << 8);
}