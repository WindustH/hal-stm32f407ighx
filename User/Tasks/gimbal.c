#ifdef BOARD_GIMBAL
#include "gimbal.h"
#include "Drivers/RC_DR16/driver.h"
void gimbal_update_yaw_pose() {
  volatile rcCtrl_dr16 *ctrl = rc_dr16_get_ctrl_sig();
}
#endif