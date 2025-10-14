/**
 * @file    bmi088d_pid.h
 * @brief   BMI088 Driver Library - PID Control Algorithms (Ported from
 * source-project)
 * @author  Extracted from RoboMaster INS Example
 * @version 1.1.0
 * @date    2025-10-14
 */

#ifndef BMI088D_PID_H
#define BMI088D_PID_H

#include "bmi088d_fuzzy.h"
#include "bmi088d_ols.h"
#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PID improvement flags from source-project */
typedef enum {
  BMI088D_PID_IMPROVE_NONE = 0x00,
  BMI088D_PID_IMPROVE_INTEGRAL_LIMIT = 0x01,
  BMI088D_PID_IMPROVE_DERIVATIVE_ON_MEASUREMENT = 0x02,
  BMI088D_PID_IMPROVE_TRAPEZOID_INTEGRAL = 0x04,
  BMI088D_PID_IMPROVE_OUTPUT_FILTER = 0x10,
  BMI088D_PID_IMPROVE_CHANGING_INTEGRATION_RATE = 0x20,
  BMI088D_PID_IMPROVE_DERIVATIVE_FILTER = 0x40,
  BMI088D_PID_IMPROVE_ERROR_HANDLE = 0x80,
} bmi088d_pid_improvement_t;

/* PID error types from source-project */
typedef enum {
  BMI088D_PID_ERROR_NONE = 0x00U,
  BMI088D_PID_ERROR_MOTOR_BLOCKED = 0x01U
} bmi088d_pid_error_type_t;

/* PID error handler structure from source-project */
typedef struct {
  uint64_t error_count;
  bmi088d_pid_error_type_t error_type;
} bmi088d_pid_error_handler_t;

/* Forward declaration for PID structure */
typedef struct bmi088d_pid_s bmi088d_pid_t;

/* PID controller structure, adapted from source-project */
struct bmi088d_pid_s {
  /* Core PID parameters */
  float ref;
  float kp;
  float ki;
  float kd;

  /* State variables */
  float measure;
  float last_measure;
  float error;
  float last_error;
  float last_iterm;

  /* Output components */
  float p_out;
  float i_out;
  float d_out;
  float i_term;

  float output;
  float last_output;
  float last_d_out;

  /* Configuration parameters */
  float max_output;
  float integral_limit;
  float deadband;
  float coef_a;        /* For Changing Integration Rate */
  float coef_b;        /* ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B */
  float output_lpf_rc; /* Output low-pass filter RC time constant */
  float derivative_lpf_rc; /* Derivative low-pass filter RC time constant */

  /* OLS for derivative calculation */
  uint16_t ols_order;
  bmi088d_ols_t ols;

  /* Fuzzy logic extension */
  bmi088d_fuzzy_rule_t *fuzzy_rule;

  /* Improvement flags */
  uint8_t improvements;

  /* Error handler */
  bmi088d_pid_error_handler_t error_handler;

  /* User callback functions */
  void (*user_func1)(bmi088d_pid_t *pid);
  void (*user_func2)(bmi088d_pid_t *pid);
};

/**
 * @brief Initialize PID controller (adapted from source-project)
 * @param[in,out] pid PID controller structure
 * @param[in] max_output Maximum output limit
 * @param[in] integral_limit Integral windup limit
 * @param[in] deadband Error deadband
 * @param[in] kp Proportional gain
 * @param[in] ki Integral gain
 * @param[in] kd Derivative gain
 * @param[in] coef_a Changing integration coefficient A
 * @param[in] coef_b Changing integration coefficient B
 * @param[in] output_lpf_rc Output low-pass filter time constant
 * @param[in] derivative_lpf_rc Derivative low-pass filter time constant
 * @param[in] ols_order Order for OLS derivative calculation (e.g., 10). If < 2,
 * standard diff is used.
 * @param[in] improvements Improvement flags
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_pid_init(bmi088d_pid_t *pid, float max_output,
                         float integral_limit, float deadband, float kp,
                         float ki, float kd, float coef_a, float coef_b,
                         float output_lpf_rc, float derivative_lpf_rc,
                         uint16_t ols_order, uint8_t improvements);

/**
 * @brief Deinitialize PID controller (frees OLS memory)
 * @param[in] pid PID controller structure
 */
void bmi088d_pid_deinit(bmi088d_pid_t *pid);

/**
 * @brief Calculate PID output (adapted from source-project)
 * @param[in,out] pid PID controller structure
 * @param[in] measure Measurement value
 * @param[in] ref Reference value
 * @param[in] dt Time step in seconds
 * @return PID output
 */
float bmi088d_pid_calculate(bmi088d_pid_t *pid, float measure, float ref,
                            float dt);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_PID_H */