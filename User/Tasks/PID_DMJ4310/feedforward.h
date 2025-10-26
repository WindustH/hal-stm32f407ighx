#ifndef __USER_TASKS_PID_DMJ4310_FEEDFORWARD__
#define __USER_TASKS_PID_DMJ4310_FEEDFORWARD__
#include "type.h"

extern volatile ffSrcList dmj4310_pidv_ff_src_list;
extern volatile ffSrcList dmj4310_pidx_ff_src_list;
u8 dmj4310_pidx_ff_add(volatile f32 *ff_src, f32 coeff);
void dmj4310_pidx_ff_remove(u8 idx);
f32 dmj4310_pidx_ff_sum(void);
u8 dmj4310_pidv_ff_add(volatile f32 *ff_src, f32 coeff);
void dmj4310_pidv_ff_remove(u8 idx);
f32 dmj4310_pidv_ff_sum(void);
#endif /* __USER_TASKS_PID_DMJ4310_FEEDFORWARD__ */