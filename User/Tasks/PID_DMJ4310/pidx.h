#ifndef __USER_TASKS_PID_DMJ4310_PIDX__
#define __USER_TASKS_PID_DMJ4310_PIDX__
#include "type.h"

void dmj4310_pidx_setup(volatile f32 *fb);

void dmj4310_pidx_update();

void dmj4310_pidx_set_target(f32 tgt);

void dmj4310_pidx_start();

void dmj4310_pidx_stop();
void dmj4310_reset_pidx_stat();
#endif /* __USER_TASKS_PID_DMJ4310_PIDX__ */
