#ifndef __USER_TASKS_GIMBAL__
#define __USER_TASKS_GIMBAL__
#ifdef BOARD_GIMBAL
#include "type.h"
void gimbal_update();
void gimbal_switch_mode(chaMode pre_mode, chaMode new_mode);
void gimbal_start();
#endif
#endif /* __USER_TASKS_GIMBAL__ */
