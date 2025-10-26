#ifdef BOARD_GIMBAL
#include "gimbal.h"
#include "BSP/dwt.h"
#include "Drivers/BOARD_CAN_COM/driver.h"
#include "Drivers/RC_DR16/driver.h"
#include "Tasks/PID_DMJ4310/feedforward.h"
#include "Tasks/PID_DMJ4310/pidx.h"
#include "Tasks/PID_DMJ6006/feedforward.h"
#include "Tasks/PID_DMJ6006/pidx.h"

#define YAW_FF_COEFF 0.0f
static u32 dwt_cnt;
static volatile u8 gimbal_started = false;
static chaMode mode = CHA_NONE;
static u8 cha_ff_src_id;

void gimbal_update() {
  if (!gimbal_started)
    return;
  volatile rcCtrl_dr16 *ctrl = rc_dr16_get_ctrl_sig();
  // 模式切换
  chaMode new_mode = cha_mode_map[ctrl->rc.s1];
  gimbal_switch_mode(mode, new_mode);
  mode = new_mode;

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
void gimbal_switch_mode(chaMode pre_mode, chaMode new_mode) {
  if (pre_mode == new_mode)
    return;

  if (pre_mode == CHA_NONE) {
    // 将摇杆值加入前馈
    dmj6006_pidx_ff_add(&rc_dr16_get_ctrl_sig()->rc.ch2, YAW_SPEED);
    dmj4310_pidx_ff_add(&rc_dr16_get_ctrl_sig()->rc.ch3, PITCH_SPEED * 1.5f);
  } else if (pre_mode == CHA_ROT) {
    // 移除底盘陀螺仪速度前馈
    dmj6006_pidx_ff_remove(cha_ff_src_id);
  }

  if (new_mode == CHA_ROT) {
    // 添加底盘陀螺仪速度前馈
    cha_ff_src_id =
        dmj6006_pidx_ff_add(&bc_gim_get_cha_data()->ch0, -YAW_FF_COEFF);
  }
}
#endif
