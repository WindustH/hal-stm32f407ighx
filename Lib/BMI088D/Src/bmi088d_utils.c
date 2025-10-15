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

float bmi088d_sqrt(float x) {
  /* Simple square root using standard library */
  if (x <= 0.0f) {
    return 0.0f;
  }
  return sqrtf(x);
}

float bmi088d_abs_limit(float num, float limit) {
  /* Limit absolute value */
  if (num > limit) {
    return limit;
  }
  if (num < -limit) {
    return -limit;
  }
  return num;
}

/* Note: Removed bmi088d_sign function - can be replaced with (x >= 0 ? 1 : -1)
 */

/* Note: Removed bmi088d_deadband function - implementation was incorrect and
 * unused. Standard deadband should be: if (fabs(value) < threshold) return 0;
 * else return value;
 */

float bmi088d_constrain(float value, float min_value, float max_value) {
  /* Constrain value to range */
  if (value > max_value) {
    return max_value;
  }
  if (value < min_value) {
    return min_value;
  }
  return value;
}

float bmi088d_loop_constrain(float input, float min_value, float max_value) {
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

/* Note: Removed format functions:
 * - bmi088d_format_deg: can be replaced with bmi088d_loop_constrain(angle,
 * -180, 180)
 * - bmi088d_format_rad: can be replaced with bmi088d_loop_constrain(angle, -PI,
 * PI)
 */

/* Vector operations */

float bmi088d_vec_magnitude(const bmi088d_vec3_t *vec) {
  if (!vec) {
    return 0.0f;
  }

  return bmi088d_sqrt(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
}

int32_t bmi088d_vec_normalize(bmi088d_vec3_t *vec) {
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

float bmi088d_vec_dot(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2) {
  if (!vec1 || !vec2) {
    return 0.0f;
  }

  return vec1->x * vec2->x + vec1->y * vec2->y + vec1->z * vec2->z;
}

/* Note: Removed unused vector operations:
 * - bmi088d_vec_cross
 * - bmi088d_vec_add
 * - bmi088d_vec_subtract
 * - bmi088d_vec_scale
 * These functions were not used in the BMI088D driver.
 */

/* Note: Removed unused matrix operations:
 * - bmi088d_mat_vec_multiply
 * - bmi088d_mat_transpose
 * These functions were not used in the BMI088D driver.
 */

/* Note: Removed unused matrix operations:
 * - bmi088d_mat_multiply_6x6_6x6
 * - bmi088d_mat_multiply_3x6_6x6
 * - bmi088d_mat_multiply_3x6_6x3
 * - bmi088d_mat_transpose_6x3
 * These functions were redundant with CMSIS DSP library functionality.
 */

/* Time utilities */

/* Note: bmi088d_get_delta_t, bmi088d_delay_s, and bmi088d_delay_ms functions
 * have been removed as they were unused in the BMI088D driver. The driver uses
 * hardware abstraction layer delay functions (bmi088d_hal_delay_ms/us) instead
 * and explicit time step parameters for PID control.
 */

/* Additional utility functions */
/* Note: Removed unused functions: lerp, map, deg_to_rad, rad_to_deg, is_finite,
 * is_nan, is_inf */