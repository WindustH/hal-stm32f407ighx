#ifndef __USER_TASKS_PID_M3508_PIDV__
#define __USER_TASKS_PID_M3508_PIDV__

#include "type.h"

void m3508_pidv_setup(volatile f32 **fb);

void m3508_pidv_update();

void m3508_pidv_set_target(u8 id, f32 tgt);

void m3508_pidv_start();

void m3508_pidv_stop();

#endif /* __USER_TASKS_PID_M3508_PIDV__ */
