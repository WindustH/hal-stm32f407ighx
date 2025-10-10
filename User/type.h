#ifndef __USER_TYPE__
#define __USER_TYPE__

#include "def.h"
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
typedef void (*proc)(void);

// 缓冲区结构体定义
typedef struct {
  u8 dat[DMA_BUFFER_SIZE];  // 数据缓冲区
  u16 len;                  // 数据长度
} buf;

// 进程列表结构体定义
typedef struct {
  proc procs[PROC_LIST_SIZE];  // 进程函数指针数组
  u32 state;                   // 进程状态位图
} procList;

// CAN发送头结构体定义
typedef struct {
  u32 StdId;              // 标准标识符
  u32 ExtId;              // 扩展标识符
  u32 IDE;                // 标识符类型
  u32 RTR;                // 远程传输请求
  u32 DLC;                // 数据长度码
  u32 TransmitGlobalTime; // 发送全局时间
} canTxH;

// CAN接收头结构体定义
typedef struct {
  u32 StdId;            // 标准标识符
  u32 ExtId;            // 扩展标识符
  u32 IDE;              // 标识符类型
  u32 RTR;              // 远程传输请求
  u32 DLC;              // 数据长度码
  u32 FilterMatchIndex; // 过滤器匹配索引
  u32 Timestamp;        // 时间戳
} canRxH;

#endif /* __USER_TYPE__ */
