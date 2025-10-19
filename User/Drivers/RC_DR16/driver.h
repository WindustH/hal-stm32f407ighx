#ifndef __USER_DRIVERS_RC_DR16_DRIVER__
#define __USER_DRIVERS_RC_DR16_DRIVER__
#ifdef BOARD_GIMBAL
#include "cb.h" // IWYU pragma: keep
#include "protocol.h"
#include "type.h" // IWYU pragma: keep
void rc_dr16_setup();

volatile rcCtrl_dr16 *rc_dr16_get_ctrl_sig();
#endif
#endif /* __USER_DRIVERS_RC_DR16_DRIVER__ */
