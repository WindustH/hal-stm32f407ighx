#ifndef __USER_TASKS_PID_DMJ6006_PIDX__
#define __USER_TASKS_PID_DMJ6006_PIDX__
#include "type.h"

void dmj6006_pidx_setup(volatile f32 *fb);

void dmj6006_pidx_update();

void dmj6006_pidx_set_target(f32 tgt);

void dmj6006_pidx_start();

void dmj6006_pidx_stop();

#endif /* __USER_TASKS_PID_DMJ6006_PIDX__ */
