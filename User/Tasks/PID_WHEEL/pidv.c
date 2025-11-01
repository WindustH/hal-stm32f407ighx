#include "pidv.h"
#include "BSP/dwt.h"
#include "Tasks/chassis.h"
#include "Utils/piecewise_pid.h"
#include "conf.h"
#include <string.h>

static volatile f32 *feedback;
static volatile pidStat wheel_pidv_stat = {0};
static volatile pwPidArg wheel_pidv_arg = {0};
static volatile pwIGainExtArg wheel_pidv_ig_arg = {0};
static u32 dwt_cnt;
static u8 wheel_pidv_started = false;
static const pwPidArg default_pid_arg = {.R = WHEEL_PIDV_BIG_R,
                                         .r = WHEEL_PIDV_R,
                                         .kp = WHEEL_PIDV_KP,
                                         .ki = WHEEL_PIDV_KI,
                                         .kd = WHEEL_PIDV_KD,
                                         .kpr = WHEEL_PIDV_KPR,
                                         .kir = WHEEL_PIDV_KIR,
                                         .kdr = WHEEL_PIDV_KDR,
                                         .ol = WHEEL_PIDV_OL};
static const pwIGainExtArg default_pid_ig_arg = {.R = WHEEL_PIDV_BIG_R,
                                                 .r = WHEEL_PIDV_R,
                                                 .ig = WHEEL_PIDV_IGAIN_K,
                                                 .igr = WHEEL_PIDV_IGAIN_KR};

void wheel_pidv_setup(volatile f32 *fb) {
  feedback = fb;
  wheel_pidv_arg = default_pid_arg;
  wheel_pidv_ig_arg = default_pid_ig_arg;
}

void wheel_pidv_update() {
  if (!wheel_pidv_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  wheel_pidv_stat.dt = dt;
  // f32 output = pw_pid_compute(&wheel_pidv_stat, &wheel_pidv_arg,
  // *feedback);
  f32 output = pw_pid_with_pw_i_gain_compute(&wheel_pidv_stat, &wheel_pidv_arg,
                                             &wheel_pidv_ig_arg, *feedback);
  chassis_set_rot(output);
}

void wheel_pidv_set_target(f32 tgt) {
  if (!wheel_pidv_started)
    return;
  wheel_pidv_stat.target = tgt;
}

void wheel_pidv_start() {
  dwt_cnt = DWT->CYCCNT;
  memset((void *)&wheel_pidv_stat, 0, sizeof(wheel_pidv_stat));
  wheel_pidv_started = true;
}

void wheel_pidv_stop() { wheel_pidv_started = false; }

void wheel_reset_pidv_stat() {
  wheel_pidv_stat.p = 0.0f;
  wheel_pidv_stat.i = 0.0f;
  wheel_pidv_stat.d = 0.0f;
  wheel_pidv_stat.target = 0.0f;
}
