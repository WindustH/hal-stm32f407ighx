#include "cron.h"
#include "tim.h"
#include "type.h"

static volatile procList proc_list = {.state = 0, .procs = {NULL}};

u8 bsp_cron_job_add(const proc p) {
  // Check for null pointer
  if (p == NULL) {
    return PROC_LIST_SIZE;
  }

  // Use interrupt control to prevent race condition
  for (u8 i = 0; i < PROC_LIST_SIZE; i++) {
    // Fix the bitwise operation to check if the bit is 0 (available)
    if (!(proc_list.state & (1U << i))) {
      proc_list.procs[i] = p;
      proc_list.state |= (1U << i);
      return i;
    }
  }
  return PROC_LIST_SIZE;
}

void bsp_cron_job_remove(const u8 idx) {
  // Check bounds
  if (idx < PROC_LIST_SIZE) {
    // Use interrupt control to prevent race condition
    proc_list.state &= ~(1U << idx);
  }
}

// HAL TIM3 Period Elapsed Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // Check if the interrupt is from TIM3
  if (htim->Instance == TIM3) {
    // Execute cron jobs
    for (u8 i = 0; i < PROC_LIST_SIZE; i++) {
      if ((proc_list.state & (1U << i)) && (proc_list.procs[i] != NULL)) {
        proc_list.procs[i]();
      }
    }
  }
}