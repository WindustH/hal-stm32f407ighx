#include "cron.h"
#include "type.h"

// 静态进程列表变量，用于存储定时任务
static volatile cronJobList cron_job_list = {.state = 0, .procs = {NULL}};

/**
 * @brief 添加定时任务到调度列表
 * @param p 要添加的进程函数指针
 * @return 成功时返回任务索引，失败时返回CRON_JOB_MAX_CNT
 */
u8 bsp_cron_job_add(const cronJob p) {
  // 检查空指针
  if (p == NULL) {
    return CRON_JOB_MAX_CNT;
  }

  // 进入临界区：关闭全局中断防止竞态
  __disable_irq();
  for (u8 i = 0; i < CRON_JOB_MAX_CNT; i++) {
    if (!(cron_job_list.state & (1U << i))) {
      cron_job_list.procs[i] = p;
      cron_job_list.state |= (1U << i);
      __enable_irq(); // 恢复中断
      return i;
    }
  }
  __enable_irq(); // 恢复中断（即使失败也要恢复！）
  return CRON_JOB_MAX_CNT;
}

/**
 * @brief 从调度列表中移除定时任务
 * @param idx 要移除的任务索引
 */
void bsp_cron_job_remove(const u8 idx) {
  // 检查索引边界
  if (idx >= CRON_JOB_MAX_CNT) {
    return;
  }

  // 进入临界区
  __disable_irq();
  cron_job_list.state &= ~(1U << idx);
  // 可选：清空函数指针（增强安全性，防止悬空调用）
  cron_job_list.procs[idx] = NULL;
  __enable_irq();
}

/**
 * @brief HAL TIM3周期结束回调函数
 * @param htim TIM句柄指针
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // 检查中断是否来自TIM3
  if (htim->Instance == TIM3) {
    // 为减少中断关闭时间，只读取一次状态快照
    u32 current_state = cron_job_list.state;
    cronJob *procs_snapshot =
        (cronJob *)
            cron_job_list.procs; // 注意：这里只是取地址，实际仍访问 volatile

    // 执行所有已注册的定时任务
    for (u8 i = 0; i < CRON_JOB_MAX_CNT; i++) {
      if ((current_state & (1U << i)) && (procs_snapshot[i] != NULL)) {
        procs_snapshot[i]();
      }
    }
  }
}