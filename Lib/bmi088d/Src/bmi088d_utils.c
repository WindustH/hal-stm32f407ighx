/**
 * @file    bmi088d_utils.c
 * @brief   BMI088 Driver Library - Utility Functions
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_utils.h"
#include <math.h>

/* Mathematical utility functions */

float bmi088d_sqrt(float x)
{
    /* Simple square root using standard library */
    if (x <= 0.0f) {
        return 0.0f;
    }
    return sqrtf(x);
}

float bmi088d_abs_limit(float num, float limit)
{
    /* Limit absolute value */
    if (num > limit) {
        return limit;
    }
    if (num < -limit) {
        return -limit;
    }
    return num;
}

float bmi088d_sign(float value)
{
    /* Get sign of value */
    if (value >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

float bmi088d_deadband(float value, float min_value, float max_value)
{
    /* Apply deadband to value */
    if (value > max_value) {
        return value;
    }
    if (value < min_value) {
        return value;
    }
    return 0.0f;
}

float bmi088d_constrain(float value, float min_value, float max_value)
{
    /* Constrain value to range */
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

float bmi088d_loop_constrain(float input, float min_value, float max_value)
{
    /* Loop constrain value (for angles) */
    float range = max_value - min_value;

    while (input > max_value) {
        input -= range;
    }
    while (input < min_value) {
        input += range;
    }

    return input;
}

float bmi088d_theta_format(float angle)
{
    /* Format angle to -180 to 180 degrees */
    return bmi088d_loop_constrain(angle, -180.0f, 180.0f);
}

float bmi088d_rad_format(float angle)
{
    /* Format angle to -PI to PI radians */
    return bmi088d_loop_constrain(angle, -3.14159265359f, 3.14159265359f);
}

/* Vector operations */

float bmi088d_vec_magnitude(const bmi088d_vec3_t *vec)
{
    if (!vec) {
        return 0.0f;
    }

    return bmi088d_sqrt(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
}

int32_t bmi088d_vec_normalize(bmi088d_vec3_t *vec)
{
    if (!vec) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    float mag = bmi088d_vec_magnitude(vec);

    if (mag < 1e-12f) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    float inv_mag = 1.0f / mag;
    vec->x *= inv_mag;
    vec->y *= inv_mag;
    vec->z *= inv_mag;

    return BMI088D_SUCCESS;
}

float bmi088d_vec_dot(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2)
{
    if (!vec1 || !vec2) {
        return 0.0f;
    }

    return vec1->x * vec2->x + vec1->y * vec2->y + vec1->z * vec2->z;
}

int32_t bmi088d_vec_cross(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2, bmi088d_vec3_t *result)
{
    if (!vec1 || !vec2 || !result) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    result->x = vec1->y * vec2->z - vec1->z * vec2->y;
    result->y = vec1->z * vec2->x - vec1->x * vec2->z;
    result->z = vec1->x * vec2->y - vec1->y * vec2->x;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_vec_add(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2, bmi088d_vec3_t *result)
{
    if (!vec1 || !vec2 || !result) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    result->x = vec1->x + vec2->x;
    result->y = vec1->y + vec2->y;
    result->z = vec1->z + vec2->z;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_vec_subtract(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2, bmi088d_vec3_t *result)
{
    if (!vec1 || !vec2 || !result) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    result->x = vec1->x - vec2->x;
    result->y = vec1->y - vec2->y;
    result->z = vec1->z - vec2->z;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_vec_scale(const bmi088d_vec3_t *vec, float scale, bmi088d_vec3_t *result)
{
    if (!vec || !result) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    result->x = vec->x * scale;
    result->y = vec->y * scale;
    result->z = vec->z * scale;

    return BMI088D_SUCCESS;
}

/* Matrix operations (simplified for 3x3) */

int32_t bmi088d_mat_vec_multiply(const float mat[9], const bmi088d_vec3_t *vec, bmi088d_vec3_t *result)
{
    if (!mat || !vec || !result) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Matrix is row-major: mat[0-2] = first row, mat[3-5] = second row, mat[6-8] = third row */
    result->x = mat[0] * vec->x + mat[1] * vec->y + mat[2] * vec->z;
    result->y = mat[3] * vec->x + mat[4] * vec->y + mat[5] * vec->z;
    result->z = mat[6] * vec->x + mat[7] * vec->y + mat[8] * vec->z;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_mat_transpose(const float mat[9], float result[9])
{
    if (!mat || !result) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Transpose 3x3 matrix */
    result[0] = mat[0];
    result[1] = mat[3];
    result[2] = mat[6];

    result[3] = mat[1];
    result[4] = mat[4];
    result[5] = mat[7];

    result[6] = mat[2];
    result[7] = mat[5];
    result[8] = mat[8];

    return BMI088D_SUCCESS;
}

/* Time utilities */

float bmi088d_get_delta_t(uint32_t *last_cnt)
{
    /* Simplified time difference calculation */
    /* In a real implementation, this would use DWT counter or system timer */

    static uint32_t last_time = 0;
    uint32_t current_time;

    /* Get current time (placeholder) */
    current_time = 0; /* Would be replaced with actual timer reading */

    if (last_cnt) {
        current_time = *last_cnt;
    }

    float dt = (current_time - last_time) * 1e-6f; /* Assuming microseconds */

    if (dt < 0.0f || dt > 1.0f) {
        /* Invalid time step, use default */
        dt = 0.001f; /* 1ms default */
    }

    last_time = current_time;

    return dt;
}

void bmi088d_delay_s(float seconds)
{
    /* Simple delay function in seconds */
    /* This is a placeholder - would be implemented with system timer */
    uint32_t ms = (uint32_t)(seconds * 1000.0f);
    bmi088d_delay_ms(ms);
}

void bmi088d_delay_ms(uint32_t ms)
{
    /* Simple delay function in milliseconds */
    /* This is a placeholder - would be implemented with system timer */

    /* For now, use a busy loop (not recommended for production) */
    volatile uint32_t i;
    for (i = 0; i < ms * 1000; i++) {
        __asm__("nop");
    }
}

/* Additional utility functions */

float bmi088d_lerp(float a, float b, float t)
{
    /* Linear interpolation */
    return a + (b - a) * t;
}

float bmi088d_map(float x, float in_min, float in_max, float out_min, float out_max)
{
    /* Map value from one range to another */
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float bmi088d_deg_to_rad(float degrees)
{
    /* Convert degrees to radians */
    return degrees * 0.01745329252f;
}

float bmi088d_rad_to_deg(float radians)
{
    /* Convert radians to degrees */
    return radians * 57.295779513f;
}

int32_t bmi088d_is_finite(float value)
{
    /* Check if value is finite (not NaN or infinity) */
    return isfinite(value);
}

int32_t bmi088d_is_nan(float value)
{
    /* Check if value is NaN */
    return isnan(value);
}

int32_t bmi088d_is_inf(float value)
{
    /* Check if value is infinity */
    return isinf(value);
}