#ifndef __USER_TASKS_CHASSIS__
#define __USER_TASKS_CHASSIS__
#ifdef BOARD_CHASSIS
#include "type.h"
void chassis_update();
void chassis_mode_switch(chaMode pre_mode, chaMode new_mode);
void chassis_set_rot(f32 rot);
#endif
#endif /* __USER_TASKS_CHASSIS__ */
