#ifndef __USER_TASKS_PID_WHEEL_PIDV__
#define __USER_TASKS_PID_WHEEL_PIDV__
#ifdef BOARD_CHASSIS
#include "type.h"

void wheel_pidv_setup(volatile f32 *fb);

void wheel_pidv_update();

void wheel_pidv_set_target(f32 tgt);

void wheel_pidv_start();

void wheel_pidv_stop();
void wheel_reset_pidv_stat();
#endif
#endif /* __USER_TASKS_PID_WHEEL_PIDV__ */
