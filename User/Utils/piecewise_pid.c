#include "piecewise_pid.h"
static inline f32 smooth_interpolation(f32 k) {
  return 20 * k * k * k * k - 15 * k * k * k * k * k;
}
f32 pid_compute(volatile pidStat *const pid, volatile pwPidArg *const arg,
                const f32 feedback) {
  if (!arg->enabled)
    return 0.0f;

  // Calculate error
  pid->p = arg->target - feedback;

  f32 kp, ki, kd;
  if (pid->p < arg->r && pid->p > -arg->r) {
    kp = arg->kpr;
    ki = arg->kir;
    kd = arg->kdr;
    //   } else if (pid->p < 4 * arg->r && pid->p > -4 * arg->r) {
  } else if (pid->p < arg->R && pid->p > -arg->R) {
    f32 k;
    if (pid->p > 0)
      k = (pid->p - arg->r);
    else
      k = -(pid->p + arg->r);
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
  f32 p_term = kp * pid->p;

  // Integral term with anti-windup
  pid->i += pid->p * pid->dt;

  // Clamp integral to prevent windup
  f32 max_integral = arg->ol / (arg->ki + 1e-6f);
  if (pid->i > max_integral)
    pid->i = max_integral;
  if (pid->i < -max_integral)
    pid->i = -max_integral;

  f32 i_term = ki * pid->i;

  // Derivative term
  pid->d = (pid->p - pid->prev_error) / pid->dt;
  f32 d_term = kd * pid->d;
  pid->prev_error = pid->p;

  f32 output = p_term + i_term + d_term;

  if (output > arg->ol)
    output = arg->ol;
  if (output < -arg->ol)
    output = -arg->ol;

  return output;
}