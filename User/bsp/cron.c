#include "cron.h"
#include "tim.h"
#include "type.h"

// 静态进程列表变量，用于存储定时任务
static volatile procList proc_list = {.state = 0, .procs = {NULL}};

/**
 * @brief 添加定时任务到调度列表
 * @param p 要添加的进程函数指针
 * @return 成功时返回任务索引，失败时返回PROC_LIST_SIZE
 */
u8 bsp_cron_job_add(const proc p) {
  // 检查空指针
  if (p == NULL) {
    return PROC_LIST_SIZE;
  }

  // 使用中断控制防止竞态条件
  for (u8 i = 0; i < PROC_LIST_SIZE; i++) {
    // 检查位是否为0（可用）
    if (!(proc_list.state & (1U << i))) {
      proc_list.procs[i] = p;
      proc_list.state |= (1U << i);
      return i;
    }
  }
  return PROC_LIST_SIZE;
}

/**
 * @brief 从调度列表中移除定时任务
 * @param idx 要移除的任务索引
 */
void bsp_cron_job_remove(const u8 idx) {
  // 检查索引边界
  if (idx < PROC_LIST_SIZE) {
    // 使用中断控制防止竞态条件
    proc_list.state &= ~(1U << idx);
  }
}

/**
 * @brief HAL TIM3周期结束回调函数
 * @param htim TIM句柄指针
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // 检查中断是否来自TIM3
  if (htim->Instance == TIM3) {
    // 执行定时任务
    for (u8 i = 0; i < PROC_LIST_SIZE; i++) {
      if ((proc_list.state & (1U << i)) && (proc_list.procs[i] != NULL)) {
        proc_list.procs[i]();
      }
    }
  }
}