#ifndef __USER_DRIVER_MOTOR_M3508_PROTOCOL__
#define __USER_DRIVER_MOTOR_M3508_PROTOCOL__

#include "type.h"

// M3508电机控制ID
#define M3508_CONTROL_ID_1_4 0x200U // 电机1-4控制ID
#define M3508_CONTROL_ID_5_8 0x1FFU // 电机5-8控制ID

// M3508电机状态结构体
typedef struct {
  f32 x; // 位置(弧度，多圈)
  f32 v; // 速度(弧度/秒)
  f32 I; // 电流(安培)
  u8 T;  // 温度(°C)
} motStat_M3508;

// M3508电机控制结构体
typedef struct {
  i16 I; // 期望电流(原始值，通常为毫安)
} motCtrl_M3508;

// M3508电机CAN消息结构体
typedef struct {
  canTxH header_1_4; // 电机1-4的CAN头
  canTxH header_5_8; // 电机5-8的CAN头
  u8 *data_1_4;      // 电机1-4的数据指针
  u8 *data_5_8;      // 电机5-8的数据指针
} motCtrlCanMsg_M3508;

/**
 * @brief 配置M3508电机的缩放因子
 * @param pos_scale 位置缩放因子
 * @param vel_scale 速度缩放因子
 * @param cur_scale 电流缩放因子
 */
void mot_m3508_set_scaling(f32 pos_scale, f32 vel_scale, f32 cur_scale);

/**
 * @brief 解析M3508电机的CAN反馈消息
 * @param msg CAN接收头指针
 * @param data CAN数据指针
 * @param mot_stat 电机状态结构体指针
 */
void mot_fb_parse_m3508(const volatile canRxH *msg, const volatile u8 *data,
                        motStat_M3508 *mot_stat);

/**
 * @brief 将控制命令打包为M3508电机的CAN消息
 * @param ctrl 控制命令结构体指针
 * @param can_msg CAN消息结构体指针
 */
void mot_ctrl_pack_msg_m3508(const volatile motCtrl_M3508 *ctrl,
                             motCtrlCanMsg_M3508 *can_msg);

#endif /* __USER_DRIVER_MOTOR_M3508_PROTOCOL__ */
