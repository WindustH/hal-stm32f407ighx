#include "BSP/dwt.h"
#include "Drivers/MOT_M3508/protect.h"
#include "protect_gimbal.h"
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
  idle_time += dt * 1000.0f;
  if (idle_time > 14.0f) {
    protect_m3508();
  } else {
    lift_protect_m3508();
  }
}
void chassis_protect_refresh_idle_time() { idle_time = 0.0f; }