#include "pidx.h"
#include "BSP/dwt.h"
#include "Utils/piecewise_pid.h"
#include "conf.h"
#include "pidv.h"
#include <string.h>

static volatile f32 *feedback;
static volatile pidStat dmj6006_pidx_stat = {0};
static volatile pwPidArg dmj6006_pidx_arg = {0};
static u32 dwt_cnt;
static u8 volatile dmj6006_pidx_started = false;
static f32 volatile feedforward_output = 0.0f;
static const pwPidArg default_pid_arg = {.R = DMJ6006_PIDX_BIG_R,
                                         .r = DMJ6006_PIDX_R,
                                         .kp = DMJ6006_PIDX_KP,
                                         .ki = DMJ6006_PIDX_KI,
                                         .kd = DMJ6006_PIDX_KD,
                                         .kpr = DMJ6006_PIDX_KPR,
                                         .kir = DMJ6006_PIDX_KIR,
                                         .kdr = DMJ6006_PIDX_KDR,
                                         .ol = DMJ6006_PIDX_OL};
static volatile pwIGainExtArg dmj6006_pidx_ig_arg = {0};
static const pwIGainExtArg default_pid_ig_arg = {.R = DMJ6006_PIDV_BIG_R,
                                                 .r = DMJ6006_PIDX_R,
                                                 .ig = DMJ6006_PIDX_IGAIN_K,
                                                 .igr = DMJ6006_PIDX_IGAIN_KR};
void dmj6006_pidx_setup(volatile f32 *fb) {
  feedback = fb;
  dmj6006_pidx_arg = default_pid_arg;
  dmj6006_pidx_ig_arg = default_pid_ig_arg;
}

void dmj6006_pidx_update() {
  if (!dmj6006_pidx_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  dmj6006_pidx_stat.dt = dt;
  f32 output = pw_pid_with_pw_i_gain_compute(
      &dmj6006_pidx_stat, &dmj6006_pidx_arg, &dmj6006_pidx_ig_arg, *feedback);
  dmj6006_pidv_set_target(output + feedforward_output);
}

void dmj6006_pidx_set_target(f32 tgt) {
  if (!dmj6006_pidx_started)
    return;
  dmj6006_pidx_stat.target = tgt;
}

void dmj6006_pidx_start() {
  dwt_cnt = DWT->CYCCNT;
  memset((void *)&dmj6006_pidx_stat, 0, sizeof(dmj6006_pidx_stat));
  dmj6006_pidx_started = true;
}

void dmj6006_pidx_stop() { dmj6006_pidx_started = false; }

void dmj6006_set_pidx_feedforward(f32 ff_o) { feedforward_output = ff_o; }

void dmj6006_reset_pidx_stat() {
  dmj6006_pidx_stat.p = 0.0f;
  dmj6006_pidx_stat.i = 0.0f;
  dmj6006_pidx_stat.d = 0.0f;
  dmj6006_pidx_stat.target = 0.0f;
}
