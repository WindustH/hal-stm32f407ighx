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
 * @brief Fast inverse square root approximation
 * @param[in] x Input value
 * @return 1/sqrt(x)
 */
float bmi088d_inv_sqrt(float x);

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

/**
 * @brief Get sign of value
 * @param[in] value Input value
 * @return 1.0f if positive, -1.0f if negative
 */
float bmi088d_sign(float value);

/**
 * @brief Apply deadband to value
 * @param[in] value Input value
 * @param[in] min_value Minimum deadband value
 * @param[in] max_value Maximum deadband value
 * @return Value with deadband applied
 */
float bmi088d_deadband(float value, float min_value, float max_value);

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

/**
 * @brief Format angle to -180 to 180 degrees
 * @param[in] angle Input angle
 * @return Formatted angle
 */
float bmi088d_theta_format(float angle);

/**
 * @brief Format angle to -PI to PI radians
 * @param[in] angle Input angle
 * @return Formatted angle
 */
float bmi088d_rad_format(float angle);

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

/**
 * @brief Calculate cross product of two vectors
 * @param[in] vec1 First vector
 * @param[in] vec2 Second vector
 * @param[out] result Cross product
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_cross(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2, bmi088d_vec3_t *result);

/**
 * @brief Add two vectors
 * @param[in] vec1 First vector
 * @param[in] vec2 Second vector
 * @param[out] result Sum vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_add(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2, bmi088d_vec3_t *result);

/**
 * @brief Subtract two vectors
 * @param[in] vec1 First vector
 * @param[in] vec2 Second vector
 * @param[out] result Difference vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_subtract(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2, bmi088d_vec3_t *result);

/**
 * @brief Scale vector
 * @param[in] vec Input vector
 * @param[in] scale Scale factor
 * @param[out] result Scaled vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_scale(const bmi088d_vec3_t *vec, float scale, bmi088d_vec3_t *result);

/* Matrix operations (simplified for 3x3) */

/**
 * @brief Multiply 3x3 matrix by vector
 * @param[in] mat 3x3 matrix (row-major)
 * @param[in] vec Input vector
 * @param[out] result Result vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_mat_vec_multiply(const float mat[9], const bmi088d_vec3_t *vec, bmi088d_vec3_t *result);

/**
 * @brief Transpose 3x3 matrix
 * @param[in] mat Input matrix
 * @param[out] result Transposed matrix
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_mat_transpose(const float mat[9], float result[9]);

/* Time utilities */

/**
 * @brief Get time difference using DWT counter
 * @param[in] last_cnt Pointer to last DWT counter value
 * @return Time difference in seconds
 */
float bmi088d_get_delta_t(uint32_t *last_cnt);

/**
 * @brief Simple delay function
 * @param[in] seconds Delay time in seconds
 */
void bmi088d_delay_s(float seconds);

/**
 * @brief Simple delay function in milliseconds
 * @param[in] ms Delay time in milliseconds
 */
void bmi088d_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_UTILS_H */