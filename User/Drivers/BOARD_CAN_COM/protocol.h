#ifndef __USER_DRIVERS_BOARD_CAN_COM_PROTOCOL__
#define __USER_DRIVERS_BOARD_CAN_COM_PROTOCOL__
#include "type.h"
typedef struct {
  canTxH header;
  u8 *data;
} boardComCanMsg;
typedef struct {
  f32 ch0;
  f32 ch1;
  f32 ch2;
  f32 ch3;
  u8 s1;
  u8 s2;
} bComGimDat;

void board_com_pack_gimbal_msg(boardComCanMsg *msg, const bComGimDat *data);
void board_com_parse_gimbal_msg(const canRxH *rxHeader, const u8 data[8],
                                bComGimDat *out);
#endif /* __USER_DRIVERS_BOARD_CAN_COM_PROTOCOL__ */
