#include "protect_gimbal.h"
#include "BSP/dwt.h"
#include "Drivers/BMI088/driver.h"
#include "Drivers/MOT_DMJ4310/protect.h"
#include "Drivers/MOT_DMJ6006/protect.h"
#include "Drivers/MOT_M3508/protect.h"
#include "Tasks/PID_DMJ4310/pidv.h"
#include "Tasks/PID_DMJ4310/pidx.h"
#include "Tasks/PID_DMJ6006/pidv.h"
#include "Tasks/PID_DMJ6006/pidx.h"
#include "type.h"

static u32 dwt_cnt = 0;
static f32 idle_time = 0.0f;
static u8 protect_started = false;
void gimbal_protect_start() {
  dwt_cnt = DWT->CYCCNT;
  protect_started = true;
}
void gimbal_protect_update_idle_time() {
  if (!protect_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  static u8 idled = false;
  idle_time += dt * 1000.0f;
  if (idle_time > 21.0f) {
    protect_m3508();
    protect_dmj4310();
    protect_dmj6006();
    dmj6006_reset_pidv_stat();
    dmj6006_reset_pidx_stat();
    dmj4310_reset_pidv_stat();
    dmj4310_reset_pidx_stat();
    bmi088_stop_update_pose();
    idled = true;
  } else if (idled) {
    idled = false;
    lift_protect_dmj4310();
    lift_protect_m3508();
    lift_protect_dmj6006();
    bmi088_start_update_pose();
  }
}
void gimbal_protect_refresh_idle_time() { idle_time = 0.0f; }