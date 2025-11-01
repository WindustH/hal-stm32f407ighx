#include "gimbal.h"
#include "BSP/dwt.h"
#include "Drivers/RC_DR16/driver.h"
#include "Tasks/PID_DMJ4310/feedforward.h"
#include "Tasks/PID_DMJ4310/pidx.h"
#include "Tasks/PID_DMJ6006/feedforward.h"
#include "Tasks/PID_DMJ6006/pidx.h"

#define YAW_FF_COEFF 0.0f
static u32 dwt_cnt;
static volatile u8 gimbal_started = false;

void gimbal_update() {
  if (!gimbal_started)
    return;
  volatile rcCtrl_dr16 *ctrl = rc_dr16_get_ctrl_sig();

  f32 rc_yaw = ctrl->rc.ch2;
  f32 rc_pitch = ctrl->rc.ch3;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  dmj6006_pidx_target_add(dt * YAW_SPEED * rc_yaw);
  dmj4310_pidx_target_add(dt * PITCH_SPEED * rc_pitch);
}
void gimbal_start() {
  dwt_cnt = DWT->CYCCNT;
  gimbal_started = true;
}