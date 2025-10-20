#ifndef __USER_TASKS_CHASSIS__
#define __USER_TASKS_CHASSIS__
#ifdef BOARD_CHASSIS
#include "type.h"
void chassis_update_wheel_speed();
void chassis_set_rot(f32 rot);
#endif
#endif /* __USER_TASKS_CHASSIS__ */
