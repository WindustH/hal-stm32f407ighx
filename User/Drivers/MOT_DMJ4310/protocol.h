#ifndef __USER_DRIVER_MOTOR_DMJ4310_PROTOCOL__
#define __USER_DRIVER_MOTOR_DMJ4310_PROTOCOL__

#include "type.h"

// DM-J4310错误代码
#define DMJ4310_ERROR_OVERVOLTAGE 0x8U    // 过压错误
#define DMJ4310_ERROR_UNDERVOLTAGE 0x9U   // 欠压错误
#define DMJ4310_ERROR_OVERCURRENT 0xAU    // 过流错误
#define DMJ4310_ERROR_MOS_OVERTEMP 0xBU   // MOS管过温错误
#define DMJ4310_ERROR_MOTOR_OVERTEMP 0xCU // 电机过温错误
#define DMJ4310_ERROR_COMM_LOST 0xDU      // 通信丢失错误
#define DMJ4310_ERROR_OVERLOAD 0xEU       // 过载错误

// MIT控制参数范围
#define DMJ4310_KP_RANGE_MIN 0.0f   // Kp最小值
#define DMJ4310_KP_RANGE_MAX 500.0f // Kp最大值
#define DMJ4310_KD_RANGE_MIN 0.0f   // Kd最小值
#define DMJ4310_KD_RANGE_MAX 5.0f   // Kd最大值

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
  f32 kp;  // 位置增益
  f32 kd;  // 阻尼增益
  f32 trq; // 扭矩前馈(Nm)
} motCtrl_DMJ4310;

// 用于传输的CAN消息容器
typedef struct {
  canTxH header; // CAN发送头
  u8 *data;      // 数据指针
} motCtrlCanMsg_DMJ4310;

/**
 * @brief 解析DMJ4310电机的CAN反馈消息
 * @param msg CAN接收头指针
 * @param data CAN数据指针
 * @param mot_stat_dmj4310 电机状态结构体指针
 */
void mot_fb_parse_dmj4310(const volatile canRxH *msg, const volatile u8 *data,
                          volatile motStat_DMJ4310 *mot_stat_dmj4310);

/**
 * @brief 将MIT模式控制命令打包为CAN消息
 * @param ctrl_msg 控制命令结构体指针
 * @param can_msg CAN消息结构体指针
 */
void mot_ctrl_pack_mit_dmj4310(const volatile motCtrl_DMJ4310 *ctrl_msg,
                               motCtrlCanMsg_DMJ4310 *can_msg);

#endif /* __USER_DRIVER_MOTOR_DMJ4310_PROTOCOL__ */
