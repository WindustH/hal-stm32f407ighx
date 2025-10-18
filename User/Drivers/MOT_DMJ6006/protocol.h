// protocol.h
#ifndef __USER_DRIVERS_MOT_DMJ6006_PROTOCOL__
#define __USER_DRIVERS_MOT_DMJ6006_PROTOCOL__

#include "type.h"

// CAN ID - MIT模式命令帧
extern volatile u16 dmj6006_can_id;
// 反馈帧ID - 来自电机
extern volatile u16 dmg6006_master_id;

// DMJ6006 错误代码
#define DMJ6006_ERROR_OVERVOLTAGE 0x8U
#define DMJ6006_ERROR_UNDERVOLTAGE 0x9U
#define DMJ6006_ERROR_OVERCURRENT 0xAU
#define DMJ6006_ERROR_MOS_OVERTEMP 0xBU
#define DMJ6006_ERROR_MOTOR_OVERTEMP 0xCU
#define DMJ6006_ERROR_COMM_LOST 0xDU
#define DMJ6006_ERROR_OVERLOAD 0xEU

// MIT 控制参数范围
#define DMJ6006_KP_RANGE_MIN 0.0f
#define DMJ6006_KP_RANGE_MAX 500.0f
#define DMJ6006_KD_RANGE_MIN 0.0f
#define DMJ6006_KD_RANGE_MAX 5.0f
#define DMJ6006_V_RANGE_MIN 0.0f
#define DMJ6006_V_RANGE_MAX 5.0f

// 电机反馈数据结构体
typedef struct {
  f32 x;         // 实际位置(弧度，多圈)
  f32 v;         // 实际速度(弧度/秒)
  f32 trq;       // 实际扭矩(Nm)
  u8 error_code; // 错误代码(0表示无错误)
  u8 T_mos;      // MOSFET温度(°C)
  u8 T_mot;      // 电机(转子)温度(°C)
  u8 motor_id;   // 来自反馈的电机ID
} motStat_DMJ6006;

// MIT模式控制命令结构体
typedef struct {
  f32 x;   // 期望位置(弧度)
  f32 v;   // 期望速度(弧度/秒)
  f32 kp;  // 位置增益 [0, 500]
  f32 kd;  // 阻尼增益 [0, 5]
  f32 trq; // 扭矩前馈(Nm)
} motCtrl_DMJ6006;

// 用于传输的CAN消息容器（注意：data 是数组而非指针）
typedef struct {
  canTxH header;
  u8 *data;
} motCtrlCanMsg_DMJ6006;

/**
 * @brief 解析DMJ6006电机的CAN反馈消息
 */
void mot_fb_parse_dmj6006(const volatile canRxH *msg, const volatile u8 *data,
                          volatile motStat_DMJ6006 *mot_stat_dmj6006);

/**
 * @brief 将MIT模式控制命令打包为CAN消息
 */
void mot_ctrl_pack_mit_dmj6006(const volatile motCtrl_DMJ6006 *ctrl_msg,
                               motCtrlCanMsg_DMJ6006 *can_msg);

void mot_enable_msg_dmj6006(motCtrlCanMsg_DMJ6006 *can_msg);

#endif /* __USER_DRIVERS_MOT_DMJ6006_PROTOCOL__ */
