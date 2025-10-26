#ifdef BOARD_CHASSIS
#include "pidx.h"
#include "BSP/dwt.h"
#include "Utils/piecewise_pid.h"
#include "conf.h"
#include "feedforward.h"
#include "pidv.h"
#include <string.h>

static volatile f32 *feedback;
static volatile pidStat wheel_pidx_stat = {0};
static volatile pwPidArg wheel_pidx_arg = {0};
static u32 dwt_cnt;
static u8 volatile wheel_pidx_started = false;

static const pwPidArg default_pid_arg = {.R = WHEEL_PIDX_BIG_R,
                                         .r = WHEEL_PIDX_R,
                                         .kp = WHEEL_PIDX_KP,
                                         .ki = WHEEL_PIDX_KI,
                                         .kd = WHEEL_PIDX_KD,
                                         .kpr = WHEEL_PIDX_KPR,
                                         .kir = WHEEL_PIDX_KIR,
                                         .kdr = WHEEL_PIDX_KDR,
                                         .ol = WHEEL_PIDX_OL};
static volatile pwIGainExtArg wheel_pidx_ig_arg = {0};
static const pwIGainExtArg default_pid_ig_arg = {.R = WHEEL_PIDV_BIG_R,
                                                 .r = WHEEL_PIDX_R,
                                                 .ig = WHEEL_PIDX_IGAIN_K,
                                                 .igr = WHEEL_PIDX_IGAIN_KR};
void wheel_pidx_setup(volatile f32 *fb) {
  feedback = fb;
  wheel_pidx_arg = default_pid_arg;
  wheel_pidx_ig_arg = default_pid_ig_arg;
}

void wheel_pidx_update() {
  if (!wheel_pidx_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  wheel_pidx_stat.dt = dt;
  f32 output = pw_pid_with_pw_i_gain_compute(&wheel_pidx_stat, &wheel_pidx_arg,
                                             &wheel_pidx_ig_arg, *feedback);
  wheel_pidv_set_target(output + wheel_pidx_ff_sum());
}

void wheel_pidx_set_target(f32 tgt) {
  if (!wheel_pidx_started)
    return;
  wheel_pidx_stat.target = tgt;
}

void wheel_pidx_target_add(f32 d) {
  if (!wheel_pidx_started)
    return;
  wheel_pidx_stat.target += d;
}

void wheel_pidx_start() {
  dwt_cnt = DWT->CYCCNT;
  memset((void *)&wheel_pidx_stat, 0, sizeof(wheel_pidx_stat));
  wheel_pidx_started = true;
}

void wheel_pidx_stop() { wheel_pidx_started = false; }

void wheel_reset_pidx_stat() {
  wheel_pidx_stat.p = 0.0f;
  wheel_pidx_stat.i = 0.0f;
  wheel_pidx_stat.d = 0.0f;
  // wheel_pidx_stat.target = 0.0f;
}

f32 wheel_pidx_get_target() { return wheel_pidx_stat.target; }
#endif
