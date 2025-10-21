#ifdef BOARD_GIMBAL
#include "gimbal.h"
#include "BSP/dwt.h"
#include "Drivers/RC_DR16/driver.h"
#include "Tasks/PID_DMJ6006/pidx.h"

static u32 dwt_cnt;
static volatile u8 gimbal_rc_started = false;
static chaMode mode = CHA_NONE;

#define YAW_SPEED 8.0f
void gimbal_rc_setup() {}
void gimbal_rc_update() {
  if (!gimbal_rc_started)
    return;
  volatile rcCtrl_dr16 *ctrl = rc_dr16_get_ctrl_sig();

  chaMode new_mode = cha_mode_map[ctrl->rc.s1];
  gimbal_rc_mode_switch(mode, new_mode);
  mode = new_mode;

  f32 rc_yaw = ctrl->rc.ch2;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  dmj6006_pidx_target_add(dt * YAW_SPEED * rc_yaw);
}
void gimbal_rc_start() {
  dwt_cnt = DWT->CYCCNT;
  gimbal_rc_started = true;
}
void gimbal_rc_mode_switch(chaMode pre_mode, chaMode new_mode) {
  if (pre_mode == new_mode)
    return;
  if (pre_mode == CHA_NONE) {

  } else if (pre_mode == CHA_FREE) {
  } else if (pre_mode == CHA_FOLLOW) {
  }
}
#endif
