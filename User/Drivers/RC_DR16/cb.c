#include "cb.h"
#include "type.h"

// 静态进程列表变量，用于存储定时任务
static volatile dr16CbList cb_list = {.state = 0, .procs = {NULL}};

u8 dr16_cb_add(const dr16Cb p) {
  // 检查空指针
  if (p == NULL) {
    return DR16_CB_MAX_CNT;
  }

  // 进入临界区：关闭全局中断防止竞态
  __disable_irq();
  for (u8 i = 0; i < DR16_CB_MAX_CNT; i++) {
    if (!(cb_list.state & (1U << i))) {
      cb_list.procs[i] = p;
      cb_list.state |= (1U << i);
      __enable_irq(); // 恢复中断
      return i;
    }
  }
  __enable_irq(); // 恢复中断（即使失败也要恢复！）
  return DR16_CB_MAX_CNT;
}

void dr16_cb_remove(const u8 idx) {
  // 检查索引边界
  if (idx >= DR16_CB_MAX_CNT) {
    return;
  }

  // 进入临界区
  __disable_irq();
  cb_list.state &= ~(1U << idx);
  // 可选：清空函数指针（增强安全性，防止悬空调用）
  cb_list.procs[idx] = NULL;
  __enable_irq();
}

void dr16_cb_call() {

  // 为减少中断关闭时间，只读取一次状态快照
  u32 current_state = cb_list.state;
  dr16Cb *procs_snapshot =
      (dr16Cb *)cb_list.procs; // 注意：这里只是取地址，实际仍访问 volatile

  // 执行所有已注册的定时任务
  for (u8 i = 0; i < DR16_CB_MAX_CNT; i++) {
    if ((current_state & (1U << i)) && (procs_snapshot[i] != NULL)) {
      procs_snapshot[i]();
    }
  }
}