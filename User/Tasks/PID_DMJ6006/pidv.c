#include "pidv.h"
#include "BSP/dwt.h"
#include "Drivers/MOT_DMJ6006/driver.h"
#include "Utils/piecewise_pid.h"
#include "conf.h"
#include <string.h>

static volatile f32 *feedback;
static volatile pidStat pid_stat = {0};
static volatile pwPidArg pid_arg = {0};
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

void dmj6006_pidv_setup(volatile f32 *fb) {
  feedback = fb;
  pid_arg = default_pid_arg;
}

void dmj6006_pidv_update() {
  if (!dmj6006_pidv_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  pid_stat.dt = dt;
  f32 output = pw_pid_compute(&pid_stat, &pid_arg, *feedback);
  dmj6006_set_torque(output);
}

void dmj6006_pidv_set_target(f32 tgt) {
  if (!dmj6006_pidv_started)
    return;
  pid_stat.target = tgt;
}

void dmj6006_pidv_start() {
  dwt_cnt = DWT->CYCCNT;
  memset((void *)&pid_stat, 0, sizeof(pid_stat));
  dmj6006_pidv_started = true;
}

void dmj6006_pidv_stop() { dmj6006_pidv_started = false; }