#ifndef __USER_DEF__
#define __USER_DEF__

// DMA缓冲区大小
#define DMA_BUFFER_SIZE 256
// 进程列表大小
#define PROC_LIST_SIZE 32
// 系统时钟频率(Hz)
#define SYSCLK 168000000U
// APB1总线时钟频率(Hz)
#define APB1_CLK 42000000U
// APB2总线时钟频率(Hz)
#define APB2_CLK 84000000U
// TIM3预分频器值
#define TIM3_PRESCALER 419
// TIM3周期值
#define TIM3_PERIOD 99
// volatile关键字简写
#define vlt volatile

#ifndef NULL
// 圆周率π的浮点数表示
#define PI 3.14159265359f
#endif

#ifndef NULL
// 空指针定义
#define NULL ((void *)0)
#endif

#endif /* __USER_DEF__ */
