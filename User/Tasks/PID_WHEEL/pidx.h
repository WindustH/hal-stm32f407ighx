#ifndef __USER_TASKS_PID_WHEEL_PIDX__
#define __USER_TASKS_PID_WHEEL_PIDX__
#ifdef BOARD_CHASSIS
#include "type.h"

void wheel_pidx_setup(volatile f32 *fb);

void wheel_pidx_update();

void wheel_pidx_set_target(f32 tgt);

void wheel_pidx_start();

void wheel_pidx_target_add(f32 d);
void wheel_pidx_stop();
void wheel_reset_pidx_stat();
f32 wheel_pidx_get_target();
#endif
#endif /* __USER_TASKS_PID_WHEEL_PIDX__ */
