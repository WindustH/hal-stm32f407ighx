#include "pidv.h"
#include "BSP/dwt.h"
#include "Drivers/MOT_M3508/driver.h"
#include "Utils/piecewise_pid.h"
#include "conf.h"
#include <string.h>

static volatile f32 **feedback;
static volatile pidStat m3508_pidv_stat[8] = {0};
static volatile pwPidArg m3508_pidv_arg = {0};
static u32 dwt_cnt;
static volatile u8 m3508_pidv_started = false;
static const pwPidArg default_pid_arg = {.R = M3508_PIDV_BIG_R,
                                         .r = M3508_PIDV_R,
                                         .kp = M3508_PIDV_KP,
                                         .ki = M3508_PIDV_KI,
                                         .kd = M3508_PIDV_KD,
                                         .kpr = M3508_PIDV_KPR,
                                         .kir = M3508_PIDV_KIR,
                                         .kdr = M3508_PIDV_KDR,
                                         .ol = M3508_PIDV_OL};

void m3508_pidv_setup(volatile f32 **fb) {
  feedback = fb;
  m3508_pidv_arg = default_pid_arg;
}

void m3508_pidv_update() {
  if (!m3508_pidv_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  for (u8 i = 0; i < 8; i++) {
    m3508_pidv_stat[i].dt = dt;
    f32 output =
        pw_pid_compute(&m3508_pidv_stat[i], &m3508_pidv_arg, *feedback[i]);
    m3508_set_current(i, output);
  }
}

void m3508_pidv_set_target(u8 id, f32 tgt) {
  if (!m3508_pidv_started)
    return;
  m3508_pidv_stat[id].target = tgt;
}

void m3508_pidv_start() {
  dwt_cnt = DWT->CYCCNT;
  memset((void *)&m3508_pidv_stat, 0, sizeof(m3508_pidv_stat));
  m3508_pidv_started = true;
}

void m3508_pidv_stop() { m3508_pidv_started = false; }