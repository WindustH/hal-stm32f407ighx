#ifndef __USER_TYPE__
#define __USER_TYPE__

#include "def.h"
#include "stm32f4xx_hal.h" // IWYU pragma: keep
#include <stdbool.h>
#include <stdint.h>

// 浮点数类型定义
typedef float f32;
typedef double f64;
// 无符号整数类型定义
typedef uint32_t u32;
typedef uint8_t u8;
typedef uint16_t u16;
// 有符号整数类型定义
typedef int32_t i32;
typedef int8_t i8;
typedef int16_t i16;

// 每秒 Tick 数(常量)
extern const u32 TICK_PER_SECOND;
// 每 Tick 秒数(常量)
extern const f32 SECOND_PER_TICK;

// 函数指针类型定义
typedef void (*cronJob)(void);

// 缓冲区结构体定义
typedef struct {
  u8 dat[DMA_BUFFER_SIZE]; // 数据缓冲区
  u16 len;                 // 数据长度
} buf;

// 进程列表结构体定义
typedef struct {
  cronJob procs[CRON_JOB_MAX_CNT]; // 进程函数指针数组
  u32 state;                       // 进程状态位图
} cronJobList;

// 定义 UART 接收回调函数类型
// 参数说明：
//   huart: 触发中断的 UART 句柄
//   Size:  实际接收到的数据字节数（由 HAL 提供）
typedef void (*uartRxCb)(UART_HandleTypeDef *huart, u16 size);

// 回调函数列表结构
typedef struct {
  volatile u32 state; // 位图：bit i = 1 表示第 i 个槽位被占用
  uartRxCb callbacks[UART_CB_LIST_SIZE];
} uartRxCbList;

// 定义 CAN FIFO0 消息挂起回调函数类型
// 参数说明：
//   hcan: 触发中断的 CAN 句柄
typedef void (*canFifo0Cb)(CAN_HandleTypeDef *hcan);

// 定义 CAN FIFO1 消息挂起回调函数类型
// 参数说明：
//   hcan: 触发中断的 CAN 句柄
typedef void (*canFifo1Cb)(CAN_HandleTypeDef *hcan);

// CAN FIFO0 回调函数列表结构
typedef struct {
  volatile u32 state; // 位图：bit i = 1 表示第 i 个槽位被占用
  canFifo0Cb callbacks[CAN_FIFO0_CB_LIST_SIZE];
} canFifo0CbList;

// CAN FIFO1 回调函数列表结构
typedef struct {
  volatile u32 state; // 位图：bit i = 1 表示第 i 个槽位被占用
  canFifo1Cb callbacks[CAN_FIFO1_CB_LIST_SIZE];
} canFifo1CbList;

typedef CAN_RxHeaderTypeDef canRxH;
typedef CAN_TxHeaderTypeDef canTxH;

typedef struct {
  f32 dt;
  f32 p;
  f32 i;
  f32 d;
  f32 prev_error;
} pidStat;

typedef struct {
  f32 r; // 使用第一套参数的控制域
  f32 R; // 过渡控制域大小
  f32 kpr;
  f32 kir;
  f32 kdr;
  f32 kp;
  f32 ki;
  f32 kd;
  f32 ol;

  f32 target; // Target value

  u8 enabled; // Controller enabled flag
} pwPidArg;

#endif /* __USER_TYPE__ */
