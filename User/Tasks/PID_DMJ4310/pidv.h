#ifndef __USER_TASKS_PID_DMJ4310_PIDV__
#define __USER_TASKS_PID_DMJ4310_PIDV__

#include "type.h"

void dmj4310_pidv_setup(volatile f32 *fb);

void dmj4310_pidv_update();

void dmj4310_pidv_set_target(f32 tgt);

void dmj4310_pidv_start();

void dmj4310_pidv_stop();
void dmj4310_reset_pidv_stat();
#endif /* __USER_TASKS_PID_DMJ4310_PIDV__ */
