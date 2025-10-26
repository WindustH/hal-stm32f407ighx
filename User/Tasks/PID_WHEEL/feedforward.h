#ifndef __USER_TASKS_PID_WHEEL_FEEDFORWARD__
#define __USER_TASKS_PID_WHEEL_FEEDFORWARD__
#ifdef BOARD_CHASSIS
#include "type.h"

extern volatile ffSrcList wheel_pidv_ff_src_list;
extern volatile ffSrcList wheel_pidx_ff_src_list;
u8 wheel_pidx_ff_add(volatile f32 *ff_src, f32 coeff);
void wheel_pidx_ff_remove(u8 idx);
f32 wheel_pidx_ff_sum(void);
u8 wheel_pidv_ff_add(volatile f32 *ff_src, f32 coeff);
void wheel_pidv_ff_remove(u8 idx);
f32 wheel_pidv_ff_sum(void);
#endif
#endif /* __USER_TASKS_PID_WHEEL_FEEDFORWARD__ */
