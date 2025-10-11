/**
 * @file    bmi088d_pid.h
 * @brief   BMI088 Driver Library - PID Control Algorithms
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_PID_H
#define BMI088D_PID_H

#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PID improvement flags */
typedef enum {
    BMI088D_PID_NONE = 0x00,
    BMI088D_PID_INTEGRAL_LIMIT = 0x01,
    BMI088D_PID_DERIVATIVE_ON_MEASUREMENT = 0x02,
    BMI088D_PID_TRAPEZOID_INTEGRAL = 0x04,
    BMI088D_PID_PROPORTIONAL_ON_MEASUREMENT = 0x08,
    BMI088D_PID_OUTPUT_FILTER = 0x10,
    BMI088D_PID_CHANGING_INTEGRATION_RATE = 0x20,
    BMI088D_PID_DERIVATIVE_FILTER = 0x40,
    BMI088D_PID_ERROR_HANDLE = 0x80,
} bmi088d_pid_improvement_t;

/* Forward declaration for PID structure */
typedef struct bmi088d_pid_s bmi088d_pid_t;

/* PID controller structure */
struct bmi088d_pid_s {
    float ref;                    /* Reference value */
    float kp;                     /* Proportional gain */
    float ki;                     /* Integral gain */
    float kd;                     /* Derivative gain */

    float measure;                /* Measurement value */
    float last_measure;           /* Last measurement value */
    float error;                  /* Current error */
    float last_error;             /* Last error */

    float p_out;                  /* Proportional output */
    float i_out;                  /* Integral output */
    float d_out;                  /* Derivative output */
    float i_term;                 /* Integral term */

    float output;                 /* Controller output */
    float last_output;            /* Last output */
    float last_d_out;             /* Last derivative output */

    float max_output;             /* Maximum output limit */
    float integral_limit;         /* Integral windup limit */
    float deadband;               /* Error deadband */
    float control_period;         /* Control period */

    float coef_a;                 /* Changing integration coefficient A */
    float coef_b;                 /* Changing integration coefficient B */
    float output_lpf_rc;          /* Output low-pass filter time constant */
    float derivative_lpf_rc;      /* Derivative low-pass filter time constant */

    uint32_t dwt_cnt;             /* DWT timer counter */
    float dt;                     /* Time step */

    uint8_t improvements;         /* Improvement flags */

    void (*user_func1)(bmi088d_pid_t *pid);
    void (*user_func2)(bmi088d_pid_t *pid);
};

/**
 * @brief Initialize PID controller
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
 * @param[in] improvements Improvement flags
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_pid_init(bmi088d_pid_t *pid,
                        float max_output,
                        float integral_limit,
                        float deadband,
                        float kp,
                        float ki,
                        float kd,
                        float coef_a,
                        float coef_b,
                        float output_lpf_rc,
                        float derivative_lpf_rc,
                        uint8_t improvements);

/**
 * @brief Calculate PID output
 * @param[in,out] pid PID controller structure
 * @param[in] measure Measurement value
 * @param[in] ref Reference value
 * @return PID output
 */
float bmi088d_pid_calculate(bmi088d_pid_t *pid, float measure, float ref);

/**
 * @brief Reset PID controller
 * @param[in,out] pid PID controller structure
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_pid_reset(bmi088d_pid_t *pid);

/**
 * @brief Set PID gains
 * @param[in,out] pid PID controller structure
 * @param[in] kp Proportional gain
 * @param[in] ki Integral gain
 * @param[in] kd Derivative gain
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_pid_set_gains(bmi088d_pid_t *pid, float kp, float ki, float kd);

/**
 * @brief Get PID gains
 * @param[in] pid PID controller structure
 * @param[out] kp Proportional gain
 * @param[out] ki Integral gain
 * @param[out] kd Derivative gain
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_pid_get_gains(const bmi088d_pid_t *pid, float *kp, float *ki, float *kd);

/**
 * @brief Set PID limits
 * @param[in,out] pid PID controller structure
 * @param[in] max_output Maximum output limit
 * @param[in] integral_limit Integral windup limit
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_pid_set_limits(bmi088d_pid_t *pid, float max_output, float integral_limit);

/**
 * @brief Enable/disable PID improvements
 * @param[in,out] pid PID controller structure
 * @param[in] improvements Improvement flags to enable
 * @param[in] disable Improvement flags to disable
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_pid_set_improvements(bmi088d_pid_t *pid, uint8_t enable, uint8_t disable);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_PID_H */