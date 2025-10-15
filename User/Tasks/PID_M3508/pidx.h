#ifndef __USER_TASKS_PID_M3508_PIDX__
#define __USER_TASKS_PID_M3508_PIDX__
#include "type.h"

void m3508_pidx_setup(volatile f32 **fb);

void m3508_pidx_update();

void m3508_pidx_set_target(u8 id, f32 tgt);

void m3508_pidx_start();

void m3508_pidx_stop();

#endif /* __USER_TASKS_PID_M3508_PIDX__ */
