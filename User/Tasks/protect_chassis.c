#include "protect_chassis.h"
#include "BSP/dwt.h"
#include "Drivers/BMI088/driver.h"
#include "Drivers/MOT_M3508/protect.h"
#include "type.h"

static u32 dwt_cnt = 0;
static f32 idle_time = 0.0f;
static u8 protect_started = false;
void chassis_protect_start() {
  dwt_cnt = DWT->CYCCNT;
  protect_started = true;
}
void chassis_protect_update_idle_time() {
  if (!protect_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  static u8 idled = false;
  idle_time += dt * 1000.0f;
  if (idle_time > 21.0f) {
    protect_m3508();
    bmi088_stop_update_pose();
    idled = true;
  } else if (idled) {
    idled = false;
    lift_protect_m3508();
    bmi088_start_update_pose();
  }
}
void chassis_protect_refresh_idle_time() { idle_time = 0.0f; }
