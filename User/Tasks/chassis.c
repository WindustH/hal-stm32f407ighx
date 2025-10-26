#ifdef BOARD_CHASSIS
#include "Tasks/chassis.h"
#include "BSP/dwt.h"
#include "Drivers/BOARD_CAN_COM/driver.h"
#include "Tasks/PID_M3508/pidv.h"
#include "Tasks/PID_WHEEL/feedforward.h"
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
static chaMode cha_mode = CHA_NONE;
static u8 chassis_started = false;
static u32 dwt_cnt;
static f32 rot_speed = 5.0f;
static f32 gim_yaw = 0.0f;

void chassis_update() {
  if (!chassis_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  bComGimDat *bcgd = bc_cha_get_gim_data();
  f32 rc_yaw = bcgd->ch2;
  cha_mode = cha_mode_map[bcgd->s1];

  if (cha_mode == CHA_FREE) {
  } else if (cha_mode == CHA_FOLLOW) {
    // 更新云台的目标位置
    gim_yaw += dt * YAW_SPEED * rc_yaw;
    // 根据旋转速度增加位置环目标
    wheel_pidx_target_add(dt * YAW_SPEED * rc_yaw);
  } else if (cha_mode == CHA_ROT) {
    gim_yaw += dt * YAW_SPEED * rc_yaw;
    wheel_pidx_target_add(dt * rot_speed);
    // 拨杆 2 置于 2、3 时增加或减小转速
    if (bcgd->s2 == 2)
      rot_speed += ROD_ROT_ACCEL * dt;
    else if (bcgd->s2 == 3)
      rot_speed -= ROD_ROT_ACCEL * dt;
    // 限制转速
    rot_speed = clamp_f32(rot_speed, -MAX_ROT_SPEED, MAX_ROT_SPEED);
  }

  if (cha_mode != CHA_NONE) {
    // 解算和设置轮速
    chassis_update_wheel_speed();
  } else {
    m3508_pidv_set_target(0, 0.0f);
    m3508_pidv_set_target(1, 0.0f);
    m3508_pidv_set_target(2, 0.0f);
    m3508_pidv_set_target(3, 0.0f);
  }
}
void chassis_start() {
  dwt_cnt = DWT->CYCCNT;
  // 添加遥控器到前馈
  wheel_pidx_ff_add(&bc_cha_get_gim_data()->ch2, YAW_SPEED);
  chassis_started = true;
}

void chassis_set_rot(f32 rot) { rot_coeff = clamp_f32(rot, -4.0f, 4.0f); }

static void chassis_update_wheel_speed() {
  bComGimDat *bcgd = bc_cha_get_gim_data();
  f32 x = bcgd->ch0;
  f32 y = bcgd->ch1;
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
  // 根据云台底盘差角旋转向量
  f32 d_theta = wheel_pidx_get_target() - gim_yaw;
  f32 tmp_x = x;
  f32 tmp_y = y;
  x = tmp_x * arm_cos_f32(d_theta) - tmp_y * arm_sin_f32(d_theta);
  y = tmp_x * arm_sin_f32(d_theta) + tmp_y * arm_cos_f32(d_theta);
  // 解算并设置轮速
  f32 a, b, c, d;
  solve_motion(x, y, rot_coeff, &a, &b, &c, &d);

  m3508_pidv_set_target(0, a * MAX_SPEED);
  m3508_pidv_set_target(1, b * MAX_SPEED);
  m3508_pidv_set_target(2, c * MAX_SPEED);
  m3508_pidv_set_target(3, d * MAX_SPEED);
}

#endif
