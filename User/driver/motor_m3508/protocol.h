#ifndef __USER_DRIVER_MOTOR_M3508_PROTOCOL__
#define __USER_DRIVER_MOTOR_M3508_PROTOCOL__

#include "type.h"

#define M3508_CONTROL_ID_1_4 0x200U
#define M3508_CONTROL_ID_5_8 0x1FFU

typedef struct {
  f32 x; // Position in radians (multi-turn)
  f32 v; // Velocity in rad/s
  f32 I; // Current in Amps
  u8 T;  // Temperature (°C)
} motStat_M3508;

typedef struct {
  i16 I; // Desired current (raw, typically mA)
} motCtrl_M3508;

typedef struct {
  canTxH header_1_4;
  canTxH header_5_8;
  u8 *data_1_4;
  u8 *data_5_8;
} motCtrlCanMsg_M3508;

// ====== NEW: Scaling Configuration ======
void mot_m3508_set_scaling(f32 pos_scale, f32 vel_scale, f32 cur_scale);

// Function declarations
void mot_fb_parse_m3508(const volatile canRxH *msg, const volatile u8 *data,
                        motStat_M3508 *mot_stat);
void mot_ctrl_pack_msg_m3508(const volatile motCtrl_M3508 *ctrl,
                             motCtrlCanMsg_M3508 *can_msg);

#endif /* __USER_DRIVER_MOTOR_M3508_PROTOCOL__ */
