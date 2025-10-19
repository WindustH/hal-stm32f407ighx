#ifndef __USER_UTILS_PIECEWISE_PID__
#define __USER_UTILS_PIECEWISE_PID__
#include "type.h"
typedef struct {
  f32 dt;
  f32 p;
  f32 i;
  f32 d;
  f32 target;
  f32 prev_error;
} pidStat;

typedef struct {
  f32 r; // 使用第一套参数的控制域
  f32 R; // 过渡控制域大小
  f32 kpr;
  f32 kir;
  f32 kdr;
  f32 kp;
  f32 ki;
  f32 kd;
  f32 ol;
} pwPidArg;
typedef struct {
  f32 ig;
  f32 igr;
  f32 r;
  f32 R;
} pwIGainExtArg;

f32 pw_pid_compute(volatile pidStat *const pid, volatile pwPidArg *const arg,
                   const f32 feedback);

f32 pw_pid_with_pw_i_gain_compute(volatile pidStat *const stat,
                                  volatile pwPidArg *const arg,
                                  volatile pwIGainExtArg *const arg_pw_i_gain,
                                  const f32 feedback);

#endif /* __USER_UTILS_PIECEWISE_PID__ */
