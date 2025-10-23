#ifndef __USER_DEF__
#define __USER_DEF__

// DMA缓冲区大小
#define DMA_BUFFER_SIZE 256

// 系统时钟频率(Hz)
#define SYSCLK 168000000U
#define SYSCLK_MHZ 168U
// APB1总线时钟频率(Hz)
#define APB1_CLK 42000000U
// APB2总线时钟频率(Hz)
#define APB2_CLK 84000000U
// TIM3预分频器值
#define TIM3_PRESCALER 419
// TIM3周期值
#define TIM3_PERIOD 199
// 遥控器控制帧发送间隔
#define RC_FRAME_INTERVAL_MS 7

#define IS_MASTER_CAN 1
#define IS_SLAVE_CAN 0

#ifndef PI
#define PI 3.14159265359f
#endif

#ifndef MATH_EPSILON
#define MATH_EPSILON 1e-6f
#endif

#define YAW_SPEED 8.0f

#ifndef NULL
#define NULL ((void *)0)
#endif
#endif /* __USER_DEF__ */
