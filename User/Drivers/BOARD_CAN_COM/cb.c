#include "cb.h"
#include "type.h"

// 静态进程列表变量，用于存储定时任务
static volatile bccCbList cb_list = {.state = 0, .procs = {NULL}};

u8 bcc_cb_add(const cbT p) {
  // 检查空指针
  if (p == NULL) {
    return BCC_CB_MAX_CNT;
  }

  // 进入临界区：关闭全局中断防止竞态
  __disable_irq();
  for (u8 i = 0; i < BCC_CB_MAX_CNT; i++) {
    if (!(cb_list.state & (1U << i))) {
      cb_list.procs[i] = p;
      cb_list.state |= (1U << i);
      __enable_irq(); // 恢复中断
      return i;
    }
  }
  __enable_irq(); // 恢复中断（即使失败也要恢复！）
  return BCC_CB_MAX_CNT;
}

void bcc_cb_remove(const u8 idx) {
  // 检查索引边界
  if (idx >= BCC_CB_MAX_CNT) {
    return;
  }

  // 进入临界区
  __disable_irq();
  cb_list.state &= ~(1U << idx);
  // 可选：清空函数指针（增强安全性，防止悬空调用）
  cb_list.procs[idx] = NULL;
  __enable_irq();
}

void bcc_cb_call() {

  // 为减少中断关闭时间，只读取一次状态快照
  u32 current_state = cb_list.state;
  cbT *procs_snapshot =
      (cbT *)cb_list.procs; // 注意：这里只是取地址，实际仍访问 volatile

  // 执行所有已注册的定时任务
  for (u8 i = 0; i < BCC_CB_MAX_CNT; i++) {
    if ((current_state & (1U << i)) && (procs_snapshot[i] != NULL)) {
      procs_snapshot[i]();
    }
  }
}