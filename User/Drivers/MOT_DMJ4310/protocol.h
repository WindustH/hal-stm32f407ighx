// protocol.h
#ifndef __USER_DRIVERS_MOT_DMJ4310_PROTOCOL__
#define __USER_DRIVERS_MOT_DMJ4310_PROTOCOL__

#include "type.h"

// DMJ4310 错误代码
#define DMJ4310_ERROR_OVERVOLTAGE 0x8U
#define DMJ4310_ERROR_UNDERVOLTAGE 0x9U
#define DMJ4310_ERROR_OVERCURRENT 0xAU
#define DMJ4310_ERROR_MOS_OVERTEMP 0xBU
#define DMJ4310_ERROR_MOTOR_OVERTEMP 0xCU
#define DMJ4310_ERROR_COMM_LOST 0xDU
#define DMJ4310_ERROR_OVERLOAD 0xEU

// MIT 控制参数范围
#define DMJ4310_KP_RANGE_MIN 0.0f
#define DMJ4310_KP_RANGE_MAX 500.0f
#define DMJ4310_KD_RANGE_MIN 0.0f
#define DMJ4310_KD_RANGE_MAX 5.0f
#define DMJ4310_V_RANGE_MIN 0.0f
#define DMJ4310_V_RANGE_MAX 5.0f
#define DMJ4310_MAX_TRQ 7.0f
#define DMJ4310_MIN_TRQ -7.0f

// 电机反馈数据结构体
typedef struct {
  f32 x;         // 实际位置(弧度，多圈)
  f32 v;         // 实际速度(弧度/秒)
  f32 trq;       // 实际扭矩(Nm)
  u8 error_code; // 错误代码(0表示无错误)
  u8 T_mos;      // MOSFET温度(°C)
  u8 T_mot;      // 电机(转子)温度(°C)
  u8 motor_id;   // 来自反馈的电机ID
} motStat_DMJ4310;

// MIT模式控制命令结构体
typedef struct {
  f32 x;   // 期望位置(弧度)
  f32 v;   // 期望速度(弧度/秒)
  f32 kp;  // 位置增益 [0, 500]
  f32 kd;  // 阻尼增益 [0, 5]
  f32 trq; // 扭矩前馈(Nm)
} motCtrl_DMJ4310;

// 用于传输的CAN消息容器（注意：data 是数组而非指针）
typedef struct {
  canTxH header;
  u8 *data;
} motCtrlCanMsg_DMJ4310;

/**
 * @brief 解析DMJ4310电机的CAN反馈消息
 */
void mot_fb_parse_dmj4310(const volatile canRxH *msg, const volatile u8 *data,
                          volatile motStat_DMJ4310 *mot_stat_dmj4310);

/**
 * @brief 将MIT模式控制命令打包为CAN消息
 */
void mot_ctrl_pack_mit_dmj4310(const volatile motCtrl_DMJ4310 *ctrl_msg,
                               motCtrlCanMsg_DMJ4310 *can_msg);

void mot_enable_msg_dmj4310(motCtrlCanMsg_DMJ4310 *can_msg);

#endif /* __USER_DRIVERS_MOT_DMJ4310_PROTOCOL__ */
