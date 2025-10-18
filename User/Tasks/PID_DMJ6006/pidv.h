#ifndef __USER_TASKS_PID_DMJ6006_PIDV__
#define __USER_TASKS_PID_DMJ6006_PIDV__

#include "type.h"

void dmj6006_pidv_setup(volatile f32 *fb);

void dmj6006_pidv_update();

void dmj6006_pidv_set_target(f32 tgt);

void dmj6006_pidv_start();

void dmj6006_pidv_stop();

#endif /* __USER_TASKS_PID_DMJ6006_PIDV__ */
