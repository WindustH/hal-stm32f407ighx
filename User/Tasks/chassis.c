#include "Tasks/chassis.h"
#include "BSP/dwt.h"
#include "Drivers/RC_DR16/driver.h"
#include "Tasks/PID_M3508/pidv.h"
#include "Tasks/PID_WHEEL/pidx.h"
#include "Utils/motion.h"
#include "arm_math.h" // IWYU pragma: keep
#include "def.h"
#include "type.h"

#define MAX_SPEED 3500.0f
#define ROD_ROT_ACCEL 1.0f
#define MAX_ROT_SPEED 10.0f

static void chassis_update_wheel_speed();
static volatile f32 rot_coeff = 0.0f;
static u8 chassis_started = false;
static u32 dwt_cnt;

void chassis_update() {
  if (!chassis_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  volatile rcCtrl_dr16 *ctrl = rc_dr16_get_ctrl_sig();
  f32 rc_yaw = ctrl->rc.ch2;

  // 根据旋转速度增加位置环目标
  wheel_pidx_target_add(dt * YAW_SPEED * rc_yaw);
  // 解算和设置轮速
  chassis_update_wheel_speed();
}

void chassis_start() {
  dwt_cnt = DWT->CYCCNT;
  chassis_started = true;
}

void chassis_set_rot(f32 rot) { rot_coeff = clamp_f32(rot, -4.0f, 4.0f); }

static void chassis_update_wheel_speed() {
  volatile rcCtrl_dr16 *ctrl = rc_dr16_get_ctrl_sig();
  f32 x = ctrl->rc.ch0;
  f32 y = ctrl->rc.ch1;
  // 径向收缩方形值域为圆
  f32 abs_x = abs_f32(x);
  f32 abs_y = abs_f32(y);
  f32 max_xy = (abs_x > abs_y) ? abs_x : abs_y;

  if (max_xy <= MATH_EPSILON) {
  } else {
    f32 r;
    arm_sqrt_f32(x * x + y * y, &r);
    f32 scale = max_xy / r;
    x *= scale;
    y *= scale;
  }

  // 解算并设置轮速
  f32 a, b, c, d;
  solve_motion(x, y, rot_coeff, &a, &b, &c, &d);

  m3508_pidv_set_target(0, a * MAX_SPEED);
  m3508_pidv_set_target(1, b * MAX_SPEED);
  m3508_pidv_set_target(2, c * MAX_SPEED);
  m3508_pidv_set_target(3, d * MAX_SPEED);
}
