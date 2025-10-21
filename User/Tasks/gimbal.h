#ifndef __USER_TASKS_GIMBAL__
#define __USER_TASKS_GIMBAL__
#ifdef BOARD_GIMBAL
#include "type.h"
void gimbal_rc_setup();
void gimbal_rc_update();
void gimbal_rc_mode_switch(chaMode pre_mode, chaMode new_mode);
#endif
#endif /* __USER_TASKS_GIMBAL__ */
