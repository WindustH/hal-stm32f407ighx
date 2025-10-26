#include "feedforward.h" // 对应你的头文件
#include "Drivers/BMI088/driver.h"
#include "type.h"

volatile ffSrcList dmj4310_pidx_ff_src_list = {.state = 0};
volatile ffSrcList dmj4310_pidv_ff_src_list = {.state = 0};

// 通用添加函数
static u8 ff_add(volatile ffSrcList *list, volatile f32 *ff_src, f32 coeff) {
  if (ff_src == NULL) {
    return FF_MAX_CNT;
  }

  __disable_irq();
  for (u8 i = 0; i < FF_MAX_CNT; i++) {
    if (!(list->state & (1U << i))) {
      list->ff_src[i] = ff_src;
      list->coeff[i] = coeff;
      list->state |= (1U << i);
      __enable_irq();
      return i;
    }
  }
  __enable_irq();
  return FF_MAX_CNT;
}

// 通用移除函数
static void ff_remove(volatile ffSrcList *list, u8 idx) {
  if (idx >= FF_MAX_CNT) {
    return;
  }

  __disable_irq();
  list->state &= ~(1U << idx);
  list->ff_src[idx] = NULL;
  list->coeff[idx] = 0.0f;
  __enable_irq();
}

// 通用求和函数：注意不关闭中断，以避免在控制循环中引入延迟
static f32 ff_sum(volatile const ffSrcList *list) {
  f32 sum = 0.0f;
  u32 current_state = list->state; // 读一次状态快照

  for (u8 i = 0; i < FF_MAX_CNT; i++) {
    if (current_state & (1U << i)) {
      volatile f32 *src = list->ff_src[i];
      if (src != NULL) {
        sum += (*src) * list->coeff[i];
      }
    }
  }
  return sum;
}

// ===== 对外接口 =====

u8 dmj4310_pidx_ff_add(volatile f32 *ff_src, f32 coeff) {
  return ff_add(&dmj4310_pidx_ff_src_list, ff_src, coeff);
}

void dmj4310_pidx_ff_remove(u8 idx) {
  ff_remove(&dmj4310_pidx_ff_src_list, idx);
}

f32 dmj4310_pidx_ff_sum(void) {
  if (bmi088_get_pose()->pitch > FF_MAX_PITCH ||
      bmi088_get_pose()->pitch < FF_MIN_PITCH)
    return 0.0f;
  else
    return ff_sum(&dmj4310_pidx_ff_src_list);
}

u8 dmj4310_pidv_ff_add(volatile f32 *ff_src, f32 coeff) {
  return ff_add(&dmj4310_pidv_ff_src_list, ff_src, coeff);
}

void dmj4310_pidv_ff_remove(u8 idx) {
  ff_remove(&dmj4310_pidv_ff_src_list, idx);
}

f32 dmj4310_pidv_ff_sum(void) { return ff_sum(&dmj4310_pidv_ff_src_list); }