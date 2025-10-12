#ifndef __USER_BSP_CRON__
#define __USER_BSP_CRON__

#include "type.h"

// 定义进程函数指针类型
typedef void (*cronJob)(void);

/**
 * @brief 添加定时任务到调度列表
 * @param p 要添加的进程函数指针
 * @return 成功时返回任务索引，失败时返回CRON_JOB_MAX_CNT
 */
u8 bsp_cron_job_add(cronJob p);

/**
 * @brief 从调度列表中移除定时任务
 * @param idx 要移除的任务索引
 */
void bsp_cron_job_remove(u8 idx);

#endif /* __USER_BSP_CRON__ */
