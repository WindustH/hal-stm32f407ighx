#ifndef __USER_DEF__
#define __USER_DEF__

// DMA缓冲区大小
#define DMA_BUFFER_SIZE 256
// 进程列表大小
#define CRON_JOB_MAX_CNT 32
// 最大支持的 UART 回调任务数
#define UART_CB_LIST_SIZE 32
// 最大支持的 CAN FIFO0 回调任务数
#define CAN_FIFO0_CB_LIST_SIZE 32
// 最大支持的 CAN FIFO1 回调任务数
#define CAN_FIFO1_CB_LIST_SIZE 32
// 最大支持的 GPIO EXTI 回调任务数
#define GPIO_EXTI_CB_LIST_SIZE 32
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
#define TIM3_PERIOD 99
// 遥控器控制帧发送间隔
#define RC_FRAME_INTERVAL_MS 7

#ifndef NULL
// 圆周率π的浮点数表示
#define PI 3.14159265359f
#endif

#ifndef NULL
// 空指针定义
#define NULL ((void *)0)
#endif

#endif /* __USER_DEF__ */
