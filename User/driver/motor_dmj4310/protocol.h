#ifndef __USER_DRIVER_MOTOR_DMJ4310_PROTOCOL__
#define __USER_DRIVER_MOTOR_DMJ4310_PROTOCOL__

#include "type.h"

// DM-J4310 Error Codes
#define DMJ4310_ERROR_OVERVOLTAGE 0x8U
#define DMJ4310_ERROR_UNDERVOLTAGE 0x9U
#define DMJ4310_ERROR_OVERCURRENT 0xAU
#define DMJ4310_ERROR_MOS_OVERTEMP 0xBU
#define DMJ4310_ERROR_MOTOR_OVERTEMP 0xCU
#define DMJ4310_ERROR_COMM_LOST 0xDU
#define DMJ4310_ERROR_OVERLOAD 0xEU

// MIT Control Parameter Ranges
#define DMJ4310_KP_RANGE_MIN 0.0f
#define DMJ4310_KP_RANGE_MAX 500.0f
#define DMJ4310_KD_RANGE_MIN 0.0f
#define DMJ4310_KD_RANGE_MAX 5.0f

// Motor feedback data structure
typedef struct {
  f32 x;         // Actual position (rad, multi-turn)
  f32 v;         // Actual velocity (rad/s)
  f32 tor;       // Actual torque (Nm)
  u8 error_code; // Error code (0 = no error)
  u8 T_mos;      // MOSFET temperature (°C)
  u8 T_mot;      // Motor (rotor) temperature (°C)
  u8 motor_id;   // Motor ID from feedback
} motStat_DMJ4310;

// MIT-mode control command structure
typedef struct {
  f32 x;   // Desired position (rad)
  f32 v;   // Desired velocity (rad/s)
  f32 kp;  // Position gain
  f32 kd;  // Damping gain
  f32 tor; // Torque feedforward (Nm)
} motCtrl_DMJ4310;

// CAN message container for transmission
typedef struct {
  canTxH header;
  u8 *data;
} motCtrlCanMsg_DMJ4310;

// Function declarations
void mot_fb_parse_dmj4310(const volatile canRxH *msg, const volatile u8 *data,
                          motStat_DMJ4310 *mot_stat_dmj4310);
void mot_ctrl_pack_mit_dmj4310(const volatile motCtrl_DMJ4310 *ctrl_msg,
                               motCtrlCanMsg_DMJ4310 *can_msg);

#endif /* __USER_DRIVER_MOTOR_DMJ4310_PROTOCOL__ */
