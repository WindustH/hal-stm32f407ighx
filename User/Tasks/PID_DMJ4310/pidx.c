#include "pidx.h"
#include "BSP/dwt.h"
#include "Utils/piecewise_pid.h"
#include "conf.h"
#include "feedforward.h"
#include "pidv.h"
#include <string.h>

static volatile f32 *feedback;
static volatile pidStat dmj4310_pidx_stat = {0};
static volatile pwPidArg dmj4310_pidx_arg = {0};
static u32 dwt_cnt;
static u8 volatile dmj4310_pidx_started = false;
static const pwPidArg default_pid_arg = {.R = DMJ4310_PIDX_BIG_R,
                                         .r = DMJ4310_PIDX_R,
                                         .kp = DMJ4310_PIDX_KP,
                                         .ki = DMJ4310_PIDX_KI,
                                         .kd = DMJ4310_PIDX_KD,
                                         .kpr = DMJ4310_PIDX_KPR,
                                         .kir = DMJ4310_PIDX_KIR,
                                         .kdr = DMJ4310_PIDX_KDR,
                                         .ol = DMJ4310_PIDX_OL};
static volatile pwIGainExtArg dmj4310_pidx_ig_arg = {0};
static const pwIGainExtArg default_pid_ig_arg = {.R = DMJ4310_PIDX_BIG_R,
                                                 .r = DMJ4310_PIDX_R,
                                                 .ig = DMJ4310_PIDX_IGAIN_K,
                                                 .igr = DMJ4310_PIDX_IGAIN_KR};

void dmj4310_pidx_setup(volatile f32 *fb) {
  feedback = fb;
  dmj4310_pidx_arg = default_pid_arg;
  dmj4310_pidx_ig_arg = default_pid_ig_arg;
}

void dmj4310_pidx_update() {
  if (!dmj4310_pidx_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  dmj4310_pidx_stat.dt = dt;
  f32 output = pw_pid_with_pw_i_gain_compute(
      &dmj4310_pidx_stat, &dmj4310_pidx_arg, &dmj4310_pidx_ig_arg, *feedback);
  dmj4310_pidv_set_target(output + dmj4310_pidx_ff_sum());
}

void dmj4310_pidx_set_target(f32 tgt) {
  if (!dmj4310_pidx_started)
    return;
  dmj4310_pidx_stat.target = tgt;
}

void dmj4310_pidx_start() {
  dwt_cnt = DWT->CYCCNT;
  memset((void *)&dmj4310_pidx_stat, 0, sizeof(dmj4310_pidx_stat));
  dmj4310_pidx_started = true;
}

void dmj4310_pidx_stop() { dmj4310_pidx_started = false; }

void dmj4310_reset_pidx_stat() {
  dmj4310_pidx_stat.p = 0.0f;
  dmj4310_pidx_stat.i = 0.0f;
  dmj4310_pidx_stat.d = 0.0f;
  // dmj4310_pidx_stat.target = 0.0f;
}
void dmj4310_pidx_target_add(f32 d) {
  if (!dmj4310_pidx_started)
    return;
  dmj4310_pidx_stat.target =
      clamp_f32(dmj4310_pidx_stat.target + d, MIN_PITCH, MAX_PITCH);
}