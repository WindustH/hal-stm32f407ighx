#include "pidv.h"
#include "BSP/dwt.h"
#include "Drivers/MOT_M3508/driver.h"
#include "Utils/piecewise_pid.h"
#include "conf.h"
#include <string.h>

static volatile f32 **feedback;
static volatile pidStat pid_stat[8] = {0};
static volatile pwPidArg pid_arg = {0};
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
  pid_arg = default_pid_arg;
}

void m3508_pidv_update() {
  extern uint8_t debug_point;
  debug_point++;
  if (!m3508_pidv_started)
    return;
  f32 dt = DWT_GetDeltaT(&dwt_cnt);
  for (u8 i = 0; i < 8; i++) {
    pid_stat[i].dt = dt;
    f32 output = pw_pid_compute(&pid_stat[i], &pid_arg, *feedback[i]);
    m3508_set_current(i, output);
  }
}

void m3508_pidv_set_target(u8 id, f32 tgt) {
  if (!m3508_pidv_started)
    return;
  pid_stat[id].target = tgt;
}

void m3508_pidv_start() {
  dwt_cnt = DWT->CYCCNT;
  memset((void *)&pid_stat, 0, sizeof(pid_stat));
  m3508_pidv_started = true;
}

void m3508_pidv_stop() { m3508_pidv_started = false; }