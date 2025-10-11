/**
 * @file    bmi088d_pid.c
 * @brief   BMI088 Driver Library - PID Control Algorithms
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_pid.h"
#include "bmi088d_utils.h"
#include <string.h>
#include <math.h>

/* Internal improvement functions */
static void bmi088d_pid_trapezoid_integral(bmi088d_pid_t *pid);
static void bmi088d_pid_integral_limit(bmi088d_pid_t *pid);
static void bmi088d_pid_derivative_on_measurement(bmi088d_pid_t *pid);
static void bmi088d_pid_changing_integration_rate(bmi088d_pid_t *pid);
static void bmi088d_pid_output_filter(bmi088d_pid_t *pid);
static void bmi088d_pid_derivative_filter(bmi088d_pid_t *pid);
static void bmi088d_pid_output_limit(bmi088d_pid_t *pid);
static void bmi088d_pid_proportion_limit(bmi088d_pid_t *pid);
static void bmi088d_pid_error_handle(bmi088d_pid_t *pid);

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
                        uint8_t improvements)
{
    if (!pid) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Initialize basic parameters */
    pid->deadband = deadband;
    pid->integral_limit = integral_limit;
    pid->max_output = max_output;
    pid->ref = 0.0f;

    /* Initialize gains */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_term = 0.0f;

    /* Changing integration rate coefficients */
    pid->coef_a = coef_a;
    pid->coef_b = coef_b;

    /* Filter parameters */
    pid->output_lpf_rc = output_lpf_rc;
    pid->derivative_lpf_rc = derivative_lpf_rc;

    /* Time tracking */
    pid->dwt_cnt = 0;
    pid->dt = 0.0f;

    /* Improvement flags */
    pid->improvements = improvements;

    /* User functions */
    pid->user_func1 = NULL;
    pid->user_func2 = NULL;

    /* Reset state variables */
    pid->measure = 0.0f;
    pid->last_measure = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
    pid->last_output = 0.0f;
    pid->last_d_out = 0.0f;

    return BMI088D_SUCCESS;
}

float bmi088d_pid_calculate(bmi088d_pid_t *pid, float measure, float ref)
{
    if (!pid) {
        return 0.0f;
    }

    /* Error handling */
    if (pid->improvements & BMI088D_PID_ERROR_HANDLE) {
        bmi088d_pid_error_handle(pid);
    }

    /* Update time step */
    pid->dt = bmi088d_get_delta_t(&pid->dwt_cnt);

    /* Update measurement and reference */
    pid->measure = measure;
    pid->ref = ref;
    pid->error = pid->ref - pid->measure;

    /* Call user function 1 if defined */
    if (pid->user_func1) {
        pid->user_func1(pid);
    }

    /* Check deadband */
    if (fabsf(pid->error) > pid->deadband) {
        /* Standard PID calculation */
        pid->p_out = pid->kp * pid->error;
        pid->i_term = pid->ki * pid->error * pid->dt;
        pid->d_out = pid->kd * (pid->error - pid->last_error) / pid->dt;

        /* Call user function 2 if defined */
        if (pid->user_func2) {
            pid->user_func2(pid);
        }

        /* Apply improvements */
        if (pid->improvements & BMI088D_PID_TRAPEZOID_INTEGRAL) {
            bmi088d_pid_trapezoid_integral(pid);
        }

        if (pid->improvements & BMI088D_PID_CHANGING_INTEGRATION_RATE) {
            bmi088d_pid_changing_integration_rate(pid);
        }

        if (pid->improvements & BMI088D_PID_DERIVATIVE_ON_MEASUREMENT) {
            bmi088d_pid_derivative_on_measurement(pid);
        }

        if (pid->improvements & BMI088D_PID_DERIVATIVE_FILTER) {
            bmi088d_pid_derivative_filter(pid);
        }

        if (pid->improvements & BMI088D_PID_INTEGRAL_LIMIT) {
            bmi088d_pid_integral_limit(pid);
        }

        /* Update integral term */
        pid->i_out += pid->i_term;

        /* Calculate output */
        pid->output = pid->p_out + pid->i_out + pid->d_out;

        /* Apply output filter */
        if (pid->improvements & BMI088D_PID_OUTPUT_FILTER) {
            bmi088d_pid_output_filter(pid);
        }

        /* Apply output limit */
        bmi088d_pid_output_limit(pid);

        /* Apply proportion limit */
        bmi088d_pid_proportion_limit(pid);
    }

    /* Update state variables */
    pid->last_measure = pid->measure;
    pid->last_output = pid->output;
    pid->last_d_out = pid->d_out;
    pid->last_error = pid->error;

    return pid->output;
}

int32_t bmi088d_pid_reset(bmi088d_pid_t *pid)
{
    if (!pid) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Reset state variables */
    pid->measure = 0.0f;
    pid->last_measure = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
    pid->last_output = 0.0f;
    pid->last_d_out = 0.0f;
    pid->i_term = 0.0f;

    /* Reset time tracking */
    pid->dwt_cnt = 0;
    pid->dt = 0.0f;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_pid_set_gains(bmi088d_pid_t *pid, float kp, float ki, float kd)
{
    if (!pid) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_pid_get_gains(const bmi088d_pid_t *pid, float *kp, float *ki, float *kd)
{
    if (!pid || !kp || !ki || !kd) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    *kp = pid->kp;
    *ki = pid->ki;
    *kd = pid->kd;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_pid_set_limits(bmi088d_pid_t *pid, float max_output, float integral_limit)
{
    if (!pid) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    pid->max_output = max_output;
    pid->integral_limit = integral_limit;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_pid_set_improvements(bmi088d_pid_t *pid, uint8_t enable, uint8_t disable)
{
    if (!pid) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Enable specified improvements */
    pid->improvements |= enable;

    /* Disable specified improvements */
    pid->improvements &= ~disable;

    return BMI088D_SUCCESS;
}

/* Internal improvement functions */

static void bmi088d_pid_trapezoid_integral(bmi088d_pid_t *pid)
{
    /* Use trapezoidal integration for better accuracy */
    pid->i_term = pid->ki * ((pid->error + pid->last_error) / 2.0f) * pid->dt;
}

static void bmi088d_pid_integral_limit(bmi088d_pid_t *pid)
{
    float temp_i_out = pid->i_out + pid->i_term;
    float temp_output = pid->p_out + pid->i_out + pid->d_out;

    /* Anti-windup: stop integration if output is saturated */
    if (fabsf(temp_output) > pid->max_output) {
        if (pid->error * pid->i_out > 0.0f) {
            /* Integral is still increasing and output is saturated */
            pid->i_term = 0.0f;
        }
    }

    /* Apply integral limit */
    if (temp_i_out > pid->integral_limit) {
        pid->i_term = 0.0f;
        pid->i_out = pid->integral_limit;
    } else if (temp_i_out < -pid->integral_limit) {
        pid->i_term = 0.0f;
        pid->i_out = -pid->integral_limit;
    }
}

static void bmi088d_pid_derivative_on_measurement(bmi088d_pid_t *pid)
{
    /* Calculate derivative based on measurement instead of error */
    /* This reduces derivative kick when reference changes */
    pid->d_out = pid->kd * (pid->last_measure - pid->measure) / pid->dt;
}

static void bmi088d_pid_changing_integration_rate(bmi088d_pid_t *pid)
{
    /* Change integration rate based on error magnitude */
    if (pid->error * pid->i_out > 0.0f) {
        /* Integral is still increasing */
        if (fabsf(pid->error) <= pid->coef_b) {
            /* Full integration */
            return;
        }
        if (fabsf(pid->error) <= (pid->coef_a + pid->coef_b)) {
            /* Reduce integration rate */
            pid->i_term *= (pid->coef_a - fabsf(pid->error) + pid->coef_b) / pid->coef_a;
        } else {
            /* Stop integration */
            pid->i_term = 0.0f;
        }
    }
}

static void bmi088d_pid_derivative_filter(bmi088d_pid_t *pid)
{
    /* Apply low-pass filter to derivative term */
    pid->d_out = pid->d_out * pid->dt / (pid->derivative_lpf_rc + pid->dt) +
                 pid->last_d_out * pid->derivative_lpf_rc / (pid->derivative_lpf_rc + pid->dt);
}

static void bmi088d_pid_output_filter(bmi088d_pid_t *pid)
{
    /* Apply low-pass filter to output */
    pid->output = pid->output * pid->dt / (pid->output_lpf_rc + pid->dt) +
                  pid->last_output * pid->output_lpf_rc / (pid->output_lpf_rc + pid->dt);
}

static void bmi088d_pid_output_limit(bmi088d_pid_t *pid)
{
    /* Limit output to maximum value */
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }
}

static void bmi088d_pid_proportion_limit(bmi088d_pid_t *pid)
{
    /* Limit proportional term */
    if (pid->p_out > pid->max_output) {
        pid->p_out = pid->max_output;
    } else if (pid->p_out < -pid->max_output) {
        pid->p_out = -pid->max_output;
    }
}

static void bmi088d_pid_error_handle(bmi088d_pid_t *pid)
{
    /* Simplified error handling */
    /* In a full implementation, this would detect motor blocking or other issues */

    /* Skip if output is very small or reference is near zero */
    if (pid->output < pid->max_output * 0.001f || fabsf(pid->ref) < 0.0001f) {
        return;
    }

    /* Detect if error is persistently large */
    if ((fabsf(pid->ref - pid->measure) / fabsf(pid->ref)) > 0.95f) {
        /* Error is large relative to reference - could indicate motor blocking */
        /* In a full implementation, this would increment an error counter */
    }
}