#include "pidv.h"
#include "BSP/dwt.h"
#include "Drivers/MOT_DMJ4310/driver.h"
#include "Utils/piecewise_pid.h"
#include "conf.h"
#include <string.h>

static volatile f32 *feedback;
static volatile pidStat pid_stat = {0};
static volatile pwPidArg pid_arg = {0};
static u32 dwt_cnt;
static u8 started = false;
static const pwPidArg default_pid_arg = {.R = DMJ4310_PIDV_BIG_R,
                                         .r = DMJ4310_PIDV_R,
                                         .kp = DMJ4310_PIDV_KP,
                                         .ki = DMJ4310_PIDV_KI,
                                         .kd = DMJ4310_PIDV_KD,
                                         .kpr = DMJ4310_PIDV_KPR,
                                         .kir = DMJ4310_PIDV_KIR,
                                         .kdr = DMJ4310_PIDV_KDR,
                                         .ol = DMJ4310_PIDV_OL};

void dmj4310_pidv_setup(volatile f32 *fb) {
  feedback = fb;
  pid_arg = default_pid_arg;
}

void dmj4310_pidv_update() {
  if (!started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  pid_stat.dt = dt;
  f32 output = pw_pid_compute(&pid_stat, &pid_arg, *feedback);
  dmj4310_set_torque(output);
}

void dmj4310_pidv_set_target(f32 tgt) {
  if (!started)
    return;
  pid_stat.target = tgt;
}

void dmj4310_pidv_start() {
  dwt_cnt = DWT->CYCCNT;
  memset((void *)&pid_stat, 0, sizeof(pid_stat));
  started = true;
}

void dmj4310_pidv_stop() { started = false; }