#ifndef __USER_DRIVERS_BOARD_CAN_COM_PROTOCOL__
#define __USER_DRIVERS_BOARD_CAN_COM_PROTOCOL__

#include "type.h"
#define GIM_TO_CHA_CH0_RANGE_MIN -1.0f
#define GIM_TO_CHA_CH0_RANGE_MAX 1.0f
#define GIM_TO_CHA_CH1_RANGE_MIN -1.0f
#define GIM_TO_CHA_CH1_RANGE_MAX 1.0f
#define GIM_TO_CHA_CH2_RANGE_MIN -1.0f
#define GIM_TO_CHA_CH2_RANGE_MAX 1.0f
#define GIM_TO_CHA_CH3_RANGE_MIN -1.0f
#define GIM_TO_CHA_CH3_RANGE_MAX 1.0f

#define CHA_TO_GIM_CH0_RANGE_MIN -2.0f
#define CHA_TO_GIM_CH0_RANGE_MAX 2.0f
#define CHA_TO_GIM_CH1_RANGE_MIN -2.0f
#define CHA_TO_GIM_CH1_RANGE_MAX 2.0f
#define CHA_TO_GIM_CH2_RANGE_MIN -2.0f
#define CHA_TO_GIM_CH2_RANGE_MAX 2.0f
#define CHA_TO_GIM_CH3_RANGE_MIN -2.0f
#define CHA_TO_GIM_CH3_RANGE_MAX 2.0f

typedef struct {
  canTxH header;
  u8 *data;
} boardComCanMsg;

// 云台 → 底盘
typedef struct {
  f32 ch0;
  f32 ch1;
  f32 ch2;
  f32 ch3;
  u8 s1;
  u8 s2;
} bComGimDat;

// 底盘 → 云台（新增）
typedef struct {
  f32 ch0;
  f32 ch1;
  f32 ch2;
  f32 ch3;
} bComChaDat;

// 云台 → 底盘
void board_com_pack_gimbal_msg(boardComCanMsg *msg, const bComGimDat *data);
void board_com_parse_gimbal_msg(const canRxH *rxHeader, const u8 data[8],
                                bComGimDat *out);

// 底盘 → 云台（新增）
void board_com_pack_chassis_msg(boardComCanMsg *msg, const bComChaDat *data);
void board_com_parse_chassis_msg(const canRxH *rxHeader, const u8 data[8],
                                 bComChaDat *out);

#endif /* __USER_DRIVERS_BOARD_CAN_COM_PROTOCOL__ */
