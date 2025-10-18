#include "piecewise_pid.h"
static inline f32 smooth_interpolation(f32 k) {
  return 20 * k * k * k * k - 15 * k * k * k * k * k;
}
f32 pw_pid_compute(volatile pidStat *const stat, volatile pwPidArg *const arg,
                   const f32 feedback) {
  // Calculate error
  stat->p = stat->target - feedback;

  f32 kp, ki, kd;
  if (stat->p < arg->r && stat->p > -arg->r) {
    kp = arg->kpr;
    ki = arg->kir;
    kd = arg->kdr;
    //   } else if (pid->p < 4 * arg->r && pid->p > -4 * arg->r) {
  } else if (stat->p < arg->R && stat->p > -arg->R) {
    f32 k;
    if (stat->p > 0)
      k = (stat->p - arg->r);
    else
      k = -(stat->p + arg->r);
    k /= arg->R - arg->r;
    f32 c = smooth_interpolation(k);
    kp = arg->kpr * (1 - c) + arg->kp * c;
    ki = arg->kir * (1 - c) + arg->ki * c;
    kd = arg->kdr * (1 - c) + arg->kd * c;
  } else {
    kp = arg->kp;
    ki = arg->ki;
    kd = arg->kd;
  }
  // Proportional term
  f32 p_term = kp * stat->p;

  // Integral term with anti-windup
  stat->i += stat->p * stat->dt;

  // Clamp integral to prevent windup
  f32 max_integral = arg->ol / (arg->ki + 1e-6f);
  if (stat->i > max_integral)
    stat->i = max_integral;
  if (stat->i < -max_integral)
    stat->i = -max_integral;

  f32 i_term = ki * stat->i;

  // Derivative term
  stat->d = (stat->p - stat->prev_error) / stat->dt;
  f32 d_term = kd * stat->d;
  stat->prev_error = stat->p;

  f32 output = p_term + i_term + d_term;

  if (output > arg->ol)
    output = arg->ol;
  if (output < -arg->ol)
    output = -arg->ol;

  return output;
}