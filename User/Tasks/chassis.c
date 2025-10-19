#ifdef BOARD_CHASSIS
#include "Tasks/chassis.h"
#include "Drivers/BOARD_CAN_COM/driver.h"
#include "Tasks/PID_M3508/pidv.h"
#include "Utils/motion.h"
#include "arm_math.h" // IWYU pragma: keep
void chassis_update_wheel_speed() {
  bComGimDat *bcgd = bc_cha_get_gim_data();
  f32 a, b, c, d;
  solve_motion(bcgd->ch0, bcgd->ch1, 0.0f, &a, &b, &c, &d);
  //   f32 l;
  //   segment_length_in_square(bcgd->ch0, bcgd->ch1, &l);
  f32 r;
  arm_sqrt_f32(bcgd->ch0 * bcgd->ch0 + bcgd->ch1 * bcgd->ch1, &r);

  m3508_pidv_set_target(0, a * 5000.0f * r);
  m3508_pidv_set_target(1, b * 5000.0f * r);
  m3508_pidv_set_target(2, c * 5000.0f * r);
  m3508_pidv_set_target(3, d * 5000.0f * r);
}
#endif