/**
 * @file    bmi088d_pid.c
 * @brief   BMI088 Driver Library - PID Control Algorithms (Ported from
 * source-project)
 * @author  Extracted from RoboMaster INS Example
 * @version 1.1.0
 * @date    2025-10-14
 */

#include "bmi088d_pid.h"
#include "bmi088d_utils.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Internal improvement functions ported from source-project */
static void pid_trapezoid_integral(bmi088d_pid_t *pid, float dt);
static void pid_integral_limit(bmi088d_pid_t *pid);
static void pid_derivative_on_measurement(bmi088d_pid_t *pid, float dt);
static void pid_changing_integration_rate(bmi088d_pid_t *pid);
static void pid_output_filter(bmi088d_pid_t *pid, float dt);
static void pid_derivative_filter(bmi088d_pid_t *pid, float dt);
static void pid_output_limit(bmi088d_pid_t *pid);
static void pid_proportion_limit(bmi088d_pid_t *pid);
static void pid_error_handle(bmi088d_pid_t *pid);

int32_t bmi088d_pid_init(bmi088d_pid_t *pid, float max_output,
                         float integral_limit, float deadband, float kp,
                         float ki, float kd, float coef_a, float coef_b,
                         float output_lpf_rc, float derivative_lpf_rc,
                         uint8_t improvements) {
  if (!pid) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memset(pid, 0, sizeof(bmi088d_pid_t));

  pid->deadband = deadband;
  pid->integral_limit = integral_limit;
  pid->max_output = max_output;

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->coef_a = coef_a;
  pid->coef_b = coef_b;

  pid->output_lpf_rc = output_lpf_rc;
  pid->derivative_lpf_rc = derivative_lpf_rc;

  pid->improvements = improvements;
  pid->error_handler.error_count = 0;
  pid->error_handler.error_type = BMI088D_PID_ERROR_NONE;

  return BMI088D_SUCCESS;
}

void bmi088d_pid_deinit(bmi088d_pid_t *pid) {
  /* No OLS to deinitialize */
  (void)pid;
}

float bmi088d_pid_calculate(bmi088d_pid_t *pid, float measure, float ref,
                            float dt) {
  if (!pid || dt < 1e-6f) {
    return 0.0f;
  }

  if (pid->improvements & BMI088D_PID_IMPROVE_ERROR_HANDLE) {
    pid_error_handle(pid);
  }

  pid->measure = measure;
  pid->ref = ref;
  pid->error = pid->ref - pid->measure;

  if (pid->user_func1) {
    pid->user_func1(pid);
  }

  if (fabsf(pid->error) > pid->deadband) {
    float kp_term = pid->kp;
    float ki_term = pid->ki;
    float kd_term = pid->kd;

    pid->p_out = kp_term * pid->error;
    pid->i_term = ki_term * pid->error * dt;
    pid->d_out = kd_term * (pid->error - pid->last_error) / dt;

    if (pid->user_func2) {
      pid->user_func2(pid);
    }

    if (pid->improvements & BMI088D_PID_IMPROVE_TRAPEZOID_INTEGRAL) {
      pid_trapezoid_integral(pid, dt);
    }
    if (pid->improvements & BMI088D_PID_IMPROVE_CHANGING_INTEGRATION_RATE) {
      pid_changing_integration_rate(pid);
    }
    if (pid->improvements & BMI088D_PID_IMPROVE_DERIVATIVE_ON_MEASUREMENT) {
      pid_derivative_on_measurement(pid, dt);
    }
    if (pid->improvements & BMI088D_PID_IMPROVE_DERIVATIVE_FILTER) {
      pid_derivative_filter(pid, dt);
    }
    if (pid->improvements & BMI088D_PID_IMPROVE_INTEGRAL_LIMIT) {
      pid_integral_limit(pid);
    }

    pid->i_out += pid->i_term;
    pid->output = pid->p_out + pid->i_out + pid->d_out;

    if (pid->improvements & BMI088D_PID_IMPROVE_OUTPUT_FILTER) {
      pid_output_filter(pid, dt);
    }

    pid_output_limit(pid);
    pid_proportion_limit(pid);
  }

  pid->last_measure = pid->measure;
  pid->last_output = pid->output;
  pid->last_d_out = pid->d_out;
  pid->last_error = pid->error;
  pid->last_iterm = pid->i_term;

  return pid->output;
}

static void pid_trapezoid_integral(bmi088d_pid_t *pid, float dt) {
  pid->i_term = pid->ki * ((pid->error + pid->last_error) / 2.0f) * dt;
}

static void pid_changing_integration_rate(bmi088d_pid_t *pid) {
  if (pid->error * pid->i_out > 0) {
    if (fabsf(pid->error) <= pid->coef_b) {
      return;
    }
    if (fabsf(pid->error) <= (pid->coef_a + pid->coef_b)) {
      pid->i_term *=
          (pid->coef_a - fabsf(pid->error) + pid->coef_b) / pid->coef_a;
    } else {
      pid->i_term = 0;
    }
  }
}

static void pid_integral_limit(bmi088d_pid_t *pid) {
  float temp_i_out = pid->i_out + pid->i_term;
  float temp_output = pid->p_out + temp_i_out + pid->d_out;

  if (fabsf(temp_output) > pid->max_output) {
    if (pid->error * pid->i_out > 0) {
      pid->i_term = 0;
    }
  }

  if (temp_i_out > pid->integral_limit) {
    pid->i_term = 0;
    pid->i_out = pid->integral_limit;
  }
  if (temp_i_out < -pid->integral_limit) {
    pid->i_term = 0;
    pid->i_out = -pid->integral_limit;
  }
}

static void pid_derivative_on_measurement(bmi088d_pid_t *pid, float dt) {
  pid->d_out = pid->kd * (pid->last_measure - pid->measure) / dt;
}

static void pid_derivative_filter(bmi088d_pid_t *pid, float dt) {
  if (pid->derivative_lpf_rc > 0.0f) {
    pid->d_out = pid->d_out * dt / (pid->derivative_lpf_rc + dt) +
                 pid->last_d_out * pid->derivative_lpf_rc /
                     (pid->derivative_lpf_rc + dt);
  }
}

static void pid_output_filter(bmi088d_pid_t *pid, float dt) {
  if (pid->output_lpf_rc > 0.0f) {
    pid->output =
        pid->output * dt / (pid->output_lpf_rc + dt) +
        pid->last_output * pid->output_lpf_rc / (pid->output_lpf_rc + dt);
  }
}

static void pid_output_limit(bmi088d_pid_t *pid) {
  pid->output = bmi088d_abs_limit(pid->output, pid->max_output);
}

static void pid_proportion_limit(bmi088d_pid_t *pid) {
  pid->p_out = bmi088d_abs_limit(pid->p_out, pid->max_output);
}

static void pid_error_handle(bmi088d_pid_t *pid) {
  if (fabsf(pid->output) < pid->max_output * 0.001f ||
      fabsf(pid->ref) < 1e-5f) {
    return;
  }

  if ((fabsf(pid->ref - pid->measure) / fabsf(pid->ref)) > 0.95f) {
    pid->error_handler.error_count++;
  } else {
    pid->error_handler.error_count = 0;
  }

  if (pid->error_handler.error_count > 500) {
    pid->error_handler.error_type = BMI088D_PID_ERROR_MOTOR_BLOCKED;
  }
}
