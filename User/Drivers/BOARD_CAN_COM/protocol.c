#include "protocol.h"
#include <math.h>

// 全局变量：CAN 标准帧 ID
extern u32 gim_to_cha_can_id;
extern u32 cha_to_gim_can_id;

// 用于云台 → 底盘：每个通道独立缩放（14位）
static inline i16 scale_gimbal_float_to_i14(f32 value, f32 min, f32 max) {
  if (value > max)
    value = max;
  else if (value < min)
    value = min;
  // 映射 [min, max] → [-8191, 8191]
  f32 normalized = (value - min) / (max - min);        // [0, 1]
  return (i16)roundf(normalized * 16382.0f - 8191.0f); // 16382 = 2*8191
}

// 用于底盘 → 云台：每个通道独立缩放（16位）
static inline i16 scale_chassis_float_to_i16(f32 value, f32 min, f32 max) {
  if (value > max)
    value = max;
  else if (value < min)
    value = min;
  // 映射 [min, max] → [-32767, 32767]
  f32 normalized = (value - min) / (max - min);         // [0, 1]
  return (i16)roundf(normalized * 65534.0f - 32767.0f); // 65534 = 2*32767
}

// 反缩放：14位 → float（云台方向）
static inline f32 unscale_i14_to_gimbal_float(i16 val, f32 min, f32 max) {
  f32 normalized = (val + 8191.0f) / 16382.0f; // [0,1]
  return min + normalized * (max - min);
}

// 反缩放：16位 → float（底盘方向）
static inline f32 unscale_i16_to_chassis_float(i16 val, f32 min, f32 max) {
  f32 normalized = (val + 32767.0f) / 65534.0f; // [0,1]
  return min + normalized * (max - min);
}

// 符号扩展：14位 -> 16位有符号（仅用于解析云台消息）
static inline i16 sign_extend_14_to_16(u16 val) {
  if (val & 0x2000U) {
    return (i16)(val | 0xC000U);
  }
  return (i16)val;
}

void board_com_pack_gimbal_msg(boardComCanMsg *msg, const bComGimDat *data) {
  msg->header.StdId = gim_to_cha_can_id;
  msg->header.IDE = CAN_ID_STD;
  msg->header.RTR = CAN_RTR_DATA;
  msg->header.DLC = 8;
  msg->header.TransmitGlobalTime = DISABLE;

  i16 scaled[4];
  scaled[0] = scale_gimbal_float_to_i14(data->ch0, GIM_TO_CHA_CH0_RANGE_MIN,
                                        GIM_TO_CHA_CH0_RANGE_MAX);
  scaled[1] = scale_gimbal_float_to_i14(data->ch1, GIM_TO_CHA_CH1_RANGE_MIN,
                                        GIM_TO_CHA_CH1_RANGE_MAX);
  scaled[2] = scale_gimbal_float_to_i14(data->ch2, GIM_TO_CHA_CH2_RANGE_MIN,
                                        GIM_TO_CHA_CH2_RANGE_MAX);
  scaled[3] = scale_gimbal_float_to_i14(data->ch3, GIM_TO_CHA_CH3_RANGE_MIN,
                                        GIM_TO_CHA_CH3_RANGE_MAX);

  u8 s1_code = (data->s1 >= 1 && data->s1 <= 3) ? (data->s1 - 1) : 0;
  u8 s2_code = (data->s2 >= 1 && data->s2 <= 3) ? (data->s2 - 1) : 0;

  u8 *d = msg->data;
  d[0] = (u8)((scaled[0] >> 6) & 0xFF);
  d[1] = (u8)((scaled[0] << 2) & 0xFC);
  d[1] |= (u8)((scaled[1] >> 12) & 0x03);
  d[2] = (u8)((scaled[1] >> 4) & 0xFF);
  d[3] = (u8)((scaled[1] << 4) & 0xF0);
  d[3] |= (u8)((scaled[2] >> 10) & 0x0F);
  d[4] = (u8)((scaled[2] >> 2) & 0xFF);
  d[5] = (u8)((scaled[2] << 6) & 0xC0);
  d[5] |= (u8)((scaled[3] >> 8) & 0x3F);
  d[6] = (u8)(scaled[3] & 0xFF);
  d[7] = (s1_code << 6) | (s2_code << 4);
}

void board_com_parse_gimbal_msg(const canRxH *rxHeader, const u8 data[8],
                                bComGimDat *out) {
  if (rxHeader->StdId != gim_to_cha_can_id)
    return;
  const u8 *d = data;

  u16 f0 = ((u16)d[0] << 6) | (u16)(d[1] >> 2);
  u16 f1 = ((u16)(d[1] & 0x03) << 12) | ((u16)d[2] << 4) | (u16)(d[3] >> 4);
  u16 f2 = ((u16)(d[3] & 0x0F) << 10) | ((u16)d[4] << 2) | (u16)(d[5] >> 6);
  u16 f3 = ((u16)(d[5] & 0x3F) << 8) | (u16)d[6];

  out->ch0 = unscale_i14_to_gimbal_float(sign_extend_14_to_16(f0),
                                         GIM_TO_CHA_CH0_RANGE_MIN,
                                         GIM_TO_CHA_CH0_RANGE_MAX);
  out->ch1 = unscale_i14_to_gimbal_float(sign_extend_14_to_16(f1),
                                         GIM_TO_CHA_CH1_RANGE_MIN,
                                         GIM_TO_CHA_CH1_RANGE_MAX);
  out->ch2 = unscale_i14_to_gimbal_float(sign_extend_14_to_16(f2),
                                         GIM_TO_CHA_CH2_RANGE_MIN,
                                         GIM_TO_CHA_CH2_RANGE_MAX);
  out->ch3 = unscale_i14_to_gimbal_float(sign_extend_14_to_16(f3),
                                         GIM_TO_CHA_CH3_RANGE_MIN,
                                         GIM_TO_CHA_CH3_RANGE_MAX);

  u8 s1_code = (d[7] >> 6) & 0x03;
  u8 s2_code = (d[7] >> 4) & 0x03;
  out->s1 = (s1_code < 3) ? (s1_code + 1) : 1;
  out->s2 = (s2_code < 3) ? (s2_code + 1) : 1;
}

void board_com_pack_chassis_msg(boardComCanMsg *msg, const bComChaDat *data) {
  msg->header.StdId = cha_to_gim_can_id;
  msg->header.IDE = CAN_ID_STD;
  msg->header.RTR = CAN_RTR_DATA;
  msg->header.DLC = 8;
  msg->header.TransmitGlobalTime = DISABLE;

  i16 s0 = scale_chassis_float_to_i16(data->ch0, CHA_TO_GIM_CH0_RANGE_MIN,
                                      CHA_TO_GIM_CH0_RANGE_MAX);
  i16 s1 = scale_chassis_float_to_i16(data->ch1, CHA_TO_GIM_CH1_RANGE_MIN,
                                      CHA_TO_GIM_CH1_RANGE_MAX);
  i16 s2 = scale_chassis_float_to_i16(data->ch2, CHA_TO_GIM_CH2_RANGE_MIN,
                                      CHA_TO_GIM_CH2_RANGE_MAX);
  i16 s3 = scale_chassis_float_to_i16(data->ch3, CHA_TO_GIM_CH3_RANGE_MIN,
                                      CHA_TO_GIM_CH3_RANGE_MAX);

  u8 *d = msg->data;
  d[0] = (u8)(s0 >> 8);
  d[1] = (u8)(s0 & 0xFF);
  d[2] = (u8)(s1 >> 8);
  d[3] = (u8)(s1 & 0xFF);
  d[4] = (u8)(s2 >> 8);
  d[5] = (u8)(s2 & 0xFF);
  d[6] = (u8)(s3 >> 8);
  d[7] = (u8)(s3 & 0xFF);
}

void board_com_parse_chassis_msg(const canRxH *rxHeader, const u8 data[8],
                                 bComChaDat *out) {
  if (rxHeader->StdId != cha_to_gim_can_id)
    return;

  const u8 *d = data;
  i16 raw0 = (i16)((d[0] << 8) | d[1]);
  i16 raw1 = (i16)((d[2] << 8) | d[3]);
  i16 raw2 = (i16)((d[4] << 8) | d[5]);
  i16 raw3 = (i16)((d[6] << 8) | d[7]);

  out->ch0 = unscale_i16_to_chassis_float(raw0, CHA_TO_GIM_CH0_RANGE_MIN,
                                          CHA_TO_GIM_CH0_RANGE_MAX);
  out->ch1 = unscale_i16_to_chassis_float(raw1, CHA_TO_GIM_CH1_RANGE_MIN,
                                          CHA_TO_GIM_CH1_RANGE_MAX);
  out->ch2 = unscale_i16_to_chassis_float(raw2, CHA_TO_GIM_CH2_RANGE_MIN,
                                          CHA_TO_GIM_CH2_RANGE_MAX);
  out->ch3 = unscale_i16_to_chassis_float(raw3, CHA_TO_GIM_CH3_RANGE_MIN,
                                          CHA_TO_GIM_CH3_RANGE_MAX);
}