/**
 * @file    bmi088d_utils.h
 * @brief   BMI088 Driver Library - Utility Functions
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_UTILS_H
#define BMI088D_UTILS_H

#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mathematical utility functions */

/**
 * @brief Square root approximation
 * @param[in] x Input value
 * @return sqrt(x)
 */
float bmi088d_sqrt(float x);

/**
 * @brief Limit value to absolute range
 * @param[in] num Input value
 * @param[in] limit Absolute limit
 * @return Limited value
 */
float bmi088d_abs_limit(float num, float limit);

/* Note: Removed bmi088d_sign function - can be replaced with (x >= 0 ? 1 : -1)
 */

/* Note: Removed bmi088d_deadband function - implementation was incorrect and
 * unused. Standard deadband should be: if (fabs(value) < threshold) return 0;
 * else return value;
 */

/**
 * @brief Constrain value to range
 * @param[in] value Input value
 * @param[in] min_value Minimum value
 * @param[in] max_value Maximum value
 * @return Constrained value
 */
float bmi088d_constrain(float value, float min_value, float max_value);

/**
 * @brief Loop constrain value (for angles)
 * @param[in] input Input value
 * @param[in] min_value Minimum value
 * @param[in] max_value Maximum value
 * @return Loop-constrained value
 */
float bmi088d_loop_constrain(float input, float min_value, float max_value);

/* Note: Removed format functions:
 * - bmi088d_format_deg: can be replaced with bmi088d_loop_constrain(angle,
 * -180, 180)
 * - bmi088d_format_rad: can be replaced with bmi088d_loop_constrain(angle, -PI,
 * PI)
 */

/* Vector operations */

/**
 * @brief Calculate vector magnitude
 * @param[in] vec Input vector
 * @return Vector magnitude
 */
float bmi088d_vec_magnitude(const bmi088d_vec3_t *vec);

/**
 * @brief Normalize vector
 * @param[in,out] vec Vector to normalize
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_normalize(bmi088d_vec3_t *vec);

/**
 * @brief Calculate dot product of two vectors
 * @param[in] vec1 First vector
 * @param[in] vec2 Second vector
 * @return Dot product
 */
float bmi088d_vec_dot(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2);

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

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_UTILS_H */