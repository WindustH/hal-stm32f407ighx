#include "protocol.h"
#include <math.h>

// 全局变量：CAN 标准帧 ID
extern u32 board_com_can_id;

#define FLOAT_SCALE 8191.0f
#define FLOAT_MAX_INT 8191

// 符号扩展：14位 -> 16位有符号
static inline i16 sign_extend_14_to_16(u16 val) {
  if (val & 0x2000U) {
    return (i16)(val | 0xC000U);
  }
  return (i16)val;
}

void board_com_pack_gimbal_msg(boardComCanMsg *msg, const bComGimDat *data) {
  msg->header.StdId = board_com_can_id;
  msg->header.IDE = CAN_ID_STD;
  msg->header.RTR = CAN_RTR_DATA;
  msg->header.DLC = 8;
  msg->header.TransmitGlobalTime = DISABLE;

  // 缩放 float
  i16 scaled[4];
  const f32 *ch[] = {&data->ch0, &data->ch1, &data->ch2, &data->ch3};
  for (int i = 0; i < 4; i++) {
    f32 v = *ch[i];
    if (v > 1.0f)
      v = 1.0f;
    else if (v < -1.0f)
      v = -1.0f;
    scaled[i] = (i16)roundf(v * FLOAT_SCALE);
    if (scaled[i] > FLOAT_MAX_INT)
      scaled[i] = FLOAT_MAX_INT;
    if (scaled[i] < -FLOAT_MAX_INT)
      scaled[i] = -FLOAT_MAX_INT;
  }

  u8 s1_code = (data->s1 >= 1 && data->s1 <= 3) ? (data->s1 - 1) : 0;
  u8 s2_code = (data->s2 >= 1 && data->s2 <= 3) ? (data->s2 - 1) : 0;

  u8 *d = msg->data;
  // 前 7 字节：56 位 float 数据（14*4）
  // f0: 14 bits -> d[0] (8), d[1] (6)
  d[0] = (u8)((scaled[0] >> 6) & 0xFF);
  d[1] = (u8)((scaled[0] << 2) & 0xFC);

  // f1: d[1] (2), d[2] (8), d[3] (4)
  d[1] |= (u8)((scaled[1] >> 12) & 0x03);
  d[2] = (u8)((scaled[1] >> 4) & 0xFF);
  d[3] = (u8)((scaled[1] << 4) & 0xF0);

  // f2: d[3] (4), d[4] (8), d[5] (2)
  d[3] |= (u8)((scaled[2] >> 10) & 0x0F);
  d[4] = (u8)((scaled[2] >> 2) & 0xFF);
  d[5] = (u8)((scaled[2] << 6) & 0xC0);

  // f3: d[5] (2), d[6] (8)
  d[5] |= (u8)((scaled[3] >> 8) & 0x3F);
  d[6] = (u8)(scaled[3] & 0xFF);

  // 最后一字节 d[7]: s1(2) + s2(2) + pad(4)
  d[7] = (s1_code << 6) | (s2_code << 4); // pad 自动为0
}

void board_com_parse_gimbal_msg(const canRxH *rxHeader, const u8 data[8],
                                bComGimDat *out) {
  if (rxHeader->StdId != board_com_can_id)
    return;
  const u8 *d = data;

  // f0: d[0] + d[1]
  u16 f0 = ((u16)d[0] << 6) | (u16)(d[1] >> 2);
  // f1: d[1]低2 + d[2] + d[3]高4
  u16 f1 = ((u16)(d[1] & 0x03) << 12) | ((u16)d[2] << 4) | (u16)(d[3] >> 4);
  // f2: d[3]低4 + d[4] + d[5]高2
  u16 f2 = ((u16)(d[3] & 0x0F) << 10) | ((u16)d[4] << 2) | (u16)(d[5] >> 6);
  // f3: d[5]低6 + d[6]
  u16 f3 = ((u16)(d[5] & 0x3F) << 8) | (u16)d[6];

  out->ch0 = (f32)sign_extend_14_to_16(f0) / FLOAT_SCALE;
  out->ch1 = (f32)sign_extend_14_to_16(f1) / FLOAT_SCALE;
  out->ch2 = (f32)sign_extend_14_to_16(f2) / FLOAT_SCALE;
  out->ch3 = (f32)sign_extend_14_to_16(f3) / FLOAT_SCALE;

  // s1, s2 from d[7]
  u8 s1_code = (d[7] >> 6) & 0x03;
  u8 s2_code = (d[7] >> 4) & 0x03;

  out->s1 = (s1_code < 3) ? (s1_code + 1) : 1;
  out->s2 = (s2_code < 3) ? (s2_code + 1) : 1;
}