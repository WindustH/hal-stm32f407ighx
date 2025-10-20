#ifdef BOARD_CHASSIS
#include "def.h"
#include "Tasks/chassis.h"
#include "Drivers/BOARD_CAN_COM/driver.h"
#include "Tasks/PID_M3508/pidv.h"
#include "Utils/motion.h"
#include "arm_math.h" // IWYU pragma: keep

#define MAX_SPEED 3500.0f
static volatile f32 rot_coeff = 0.0f;
void chassis_update_wheel_speed() {
  bComGimDat *bcgd = bc_cha_get_gim_data();
  f32 x = bcgd->ch0;
  f32 y = bcgd->ch1;

  f32 abs_x = abs_f32(x);
  f32 abs_y = abs_f32(y);
  f32 max_xy = (abs_x > abs_y) ? abs_x : abs_y;

  if (max_xy <= MATH_EPSILON) {
  } else {
      f32 r;
      arm_sqrt_f32(x*x + y*y,&r);
      f32 scale = max_xy / r;
      x *= scale;
      y *= scale;
  }

  f32 a, b, c, d;
  solve_motion(x, y, rot_coeff, &a, &b, &c, &d);

  m3508_pidv_set_target(0, a * MAX_SPEED);
  m3508_pidv_set_target(1, b * MAX_SPEED);
  m3508_pidv_set_target(2, c * MAX_SPEED);
  m3508_pidv_set_target(3, d * MAX_SPEED);
}

void chassis_set_rot(f32 rot) {
    rot_coeff = clamp_f32(rot, -4.0f, 4.0f);
}
#endif
