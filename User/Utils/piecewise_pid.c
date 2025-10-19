#include "piecewise_pid.h"
static inline f32 smooth_interpolation(f32 t) {
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;
  f32 t2 = t * t;
  f32 t3 = t2 * t;
  f32 ct = 1.0f - t;
  f32 ct2 = ct * ct;
  f32 ct3 = ct2 * ct;
  return t3 / (t3 + ct3);
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

f32 pw_pid_with_pw_i_gain_compute(volatile pidStat *const stat,
                                  volatile pwPidArg *const arg,
                                  volatile pwIGainExtArg *const arg_pw_i_gain,
                                  const f32 feedback) {
  // Calculate error
  stat->p = stat->target - feedback;
  f32 e = stat->p;

  // ====== Compute kp and kd using original piecewise logic ======
  f32 kp, kd;
  if (e < arg->r && e > -arg->r) {
    kp = arg->kpr;
    kd = arg->kdr;
  } else if (e < arg->R && e > -arg->R) {
    f32 k;
    if (e > 0)
      k = (e - arg->r);
    else
      k = -(e + arg->r);
    k /= arg->R - arg->r;
    f32 c = smooth_interpolation(k);
    kp = arg->kpr * (1 - c) + arg->kp * c;
    kd = arg->kdr * (1 - c) + arg->kd * c;
  } else {
    kp = arg->kp;
    kd = arg->kd;
  }

  // ====== Compute effective integral gain based on |e| ======
  f32 abs_e = (e >= 0) ? e : -e;
  f32 ki_eff;

  if (abs_e < arg_pw_i_gain->r) {
    // Small error region: use enhanced integral gain
    ki_eff = arg_pw_i_gain->igr;
  } else if (abs_e < arg_pw_i_gain->R) {
    // Transition region
    f32 k = (abs_e - arg_pw_i_gain->r) / (arg_pw_i_gain->R - arg_pw_i_gain->r);
    f32 c = smooth_interpolation(k);
    ki_eff = arg_pw_i_gain->igr * (1 - c) + arg_pw_i_gain->ig * c;
  } else {
    // Large error region: use base integral gain
    ki_eff = arg_pw_i_gain->ig;
  }

  // ====== Proportional term ======
  f32 p_term = kp * e;

  // ====== Integral term: accumulate raw error, apply gain at output ======
  stat->i += e * stat->dt;

  // Anti-windup: clamp integral state based on worst-case (max ki_eff)
  // To be safe, use the larger of ig and igr as worst-case gain
  f32 max_ki_for_windup = (arg_pw_i_gain->igr > arg_pw_i_gain->ig)
                              ? arg_pw_i_gain->igr
                              : arg_pw_i_gain->ig;
  f32 max_integral = arg->ol / (max_ki_for_windup + 1e-6f);
  if (stat->i > max_integral)
    stat->i = max_integral;
  if (stat->i < -max_integral)
    stat->i = -max_integral;

  f32 i_term = ki_eff * stat->i;

  // ====== Derivative term ======
  stat->d = (e - stat->prev_error) / stat->dt;
  f32 d_term = kd * stat->d;
  stat->prev_error = e;

  // ====== Output and final clamping ======
  f32 output = p_term + i_term + d_term;
  if (output > arg->ol)
    output = arg->ol;
  if (output < -arg->ol)
    output = -arg->ol;

  return output;
}