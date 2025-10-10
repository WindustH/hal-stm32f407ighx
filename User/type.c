#include "type.h"

// 计算每秒 Tick 数：基于APB1时钟频率、TIM3预分频器和周期值
const u32 TICK_PER_SECOND = 2U * APB1_CLK / (TIM3_PRESCALER * TIM3_PERIOD);
// 计算每个 Tick 的秒数：每秒 Tick 数的倒数
const f32 SECOND_PER_TICK = 1.0f / (f32)TICK_PER_SECOND;