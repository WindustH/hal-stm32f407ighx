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

// 缓冲区结构体定义
typedef struct {
  u8 dat[DMA_BUFFER_SIZE]; // 数据缓冲区
  u16 len;                 // 数据长度
} buf;

typedef CAN_RxHeaderTypeDef canRxH;
typedef CAN_TxHeaderTypeDef canTxH;

typedef enum { CHA_NONE, CHA_FREE, CHA_FOLLOW, CHA_ROT } chaMode;
typedef struct {
  volatile f32 *ff_src[FF_MAX_CNT];
  volatile f32 coeff[FF_MAX_CNT];
  u32 state;
} ffSrcList;
i32 clamp_i32(i32 value, i32 min_val, i32 max_val);
f32 clamp_f32(f32 value, f32 min_val, f32 max_val);
f32 i32_to_f32(i32 x_i32, f32 x_min, f32 x_max, i32 bits);
i32 f32_to_i32(f32 x, f32 x_min, f32 x_max, i32 bits);
f32 abs_f32(f32 value);
extern const chaMode cha_mode_map[4];
#endif /* __USER_TYPE__ */
