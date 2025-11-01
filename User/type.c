#include "type.h"

// 计算每秒 Tick 数：基于APB1时钟频率、TIM3预分频器和周期值
const u32 TICK_PER_SECOND =
    2U * APB1_CLK / ((TIM3_PRESCALER + 1) * (TIM3_PERIOD + 1));
// 计算每个 Tick 的秒数：每秒 Tick 数的倒数
const f32 SECOND_PER_TICK = 1.0f / (f32)TICK_PER_SECOND;

i32 clamp_i32(i32 value, i32 min_val, i32 max_val) {
  if (value < min_val)
    return min_val;
  if (value > max_val)
    return max_val;
  return value;
}

f32 clamp_f32(f32 value, f32 min_val, f32 max_val) {
  if (value < min_val)
    return min_val;
  if (value > max_val)
    return max_val;
  return value;
}

f32 abs_f32(f32 value) { return value < 0 ? -value : value; }

f32 i32_to_f32(i32 x_i32, f32 x_min, f32 x_max, i32 bits) {
  f32 span = x_max - x_min;
  f32 offset = x_min;
  return clamp_f32(((f32)x_i32) * span / ((f32)((1 << bits) - 1)) + offset,
                   x_min, x_max);
}

i32 f32_to_i32(f32 x, f32 x_min, f32 x_max, i32 bits) {
  x = clamp_f32(x, x_min, x_max);
  f32 span = x_max - x_min;
  f32 offset = x_min;
  return (i32)((x - offset) * ((f32)((1 << bits) - 1)) / span);
}
