#include "pidv.h"
#include "BSP/dwt.h"
#include "Drivers/MOT_DMJ6006/driver.h"
#include "Utils/piecewise_pid.h"
#include "conf.h"
#include <string.h>

static volatile f32 *feedback;
static volatile pidStat dmj6006_pidv_stat = {0};
static volatile pwPidArg dmj6006_pidv_arg = {0};
static volatile pwIGainExtArg dmj6006_pidv_ig_arg = {0};
static u32 dwt_cnt;
static u8 dmj6006_pidv_started = false;
static const pwPidArg default_pid_arg = {.R = DMJ6006_PIDV_BIG_R,
                                         .r = DMJ6006_PIDV_R,
                                         .kp = DMJ6006_PIDV_KP,
                                         .ki = DMJ6006_PIDV_KI,
                                         .kd = DMJ6006_PIDV_KD,
                                         .kpr = DMJ6006_PIDV_KPR,
                                         .kir = DMJ6006_PIDV_KIR,
                                         .kdr = DMJ6006_PIDV_KDR,
                                         .ol = DMJ6006_PIDV_OL};
static const pwIGainExtArg default_pid_ig_arg = {.R = DMJ6006_PIDV_BIG_R,
                                                 .r = DMJ6006_PIDV_R,
                                                 .ig = DMJ6006_PIDV_IGAIN_K,
                                                 .igr = DMJ6006_PIDV_IGAIN_KR};

void dmj6006_pidv_setup(volatile f32 *fb) {
  feedback = fb;
  dmj6006_pidv_arg = default_pid_arg;
  dmj6006_pidv_ig_arg = default_pid_ig_arg;
}

void dmj6006_pidv_update() {
  if (!dmj6006_pidv_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  dmj6006_pidv_stat.dt = dt;
  // f32 output = pw_pid_compute(&dmj6006_pidv_stat, &dmj6006_pidv_arg,
  // *feedback);
  f32 output = pw_pid_with_pw_i_gain_compute(
      &dmj6006_pidv_stat, &dmj6006_pidv_arg, &dmj6006_pidv_ig_arg, *feedback);
  dmj6006_set_torque(output);
}

void dmj6006_pidv_set_target(f32 tgt) {
  if (!dmj6006_pidv_started)
    return;
  dmj6006_pidv_stat.target = tgt;
}

void dmj6006_pidv_start() {
  dwt_cnt = DWT->CYCCNT;
  memset((void *)&dmj6006_pidv_stat, 0, sizeof(dmj6006_pidv_stat));
  dmj6006_pidv_started = true;
}

void dmj6006_pidv_stop() { dmj6006_pidv_started = false; }

void dmj6006_reset_pidv_stat() {
  dmj6006_pidv_stat.p = 0.0f;
  dmj6006_pidv_stat.i = 0.0f;
  dmj6006_pidv_stat.d = 0.0f;
  dmj6006_pidv_stat.target = 0.0f;
}