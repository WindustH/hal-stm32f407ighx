/**
 * @file    bmi088d_quaternion.h
 * @brief   BMI088 Driver Library - Quaternion Operations
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_QUATERNION_H
#define BMI088D_QUATERNION_H

#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize quaternion to identity
 * @param[out] quat Quaternion to initialize
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_identity(bmi088d_quat_t *quat);

/**
 * @brief Normalize quaternion
 * @param[in,out] quat Quaternion to normalize
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_normalize(bmi088d_quat_t *quat);

/**
 * @brief Multiply two quaternions: result = q1 * q2
 * @param[in] q1 First quaternion
 * @param[in] q2 Second quaternion
 * @param[out] result Result quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_multiply(const bmi088d_quat_t *q1, const bmi088d_quat_t *q2, bmi088d_quat_t *result);

/**
 * @brief Get quaternion conjugate
 * @param[in] quat Input quaternion
 * @param[out] conjugate Conjugate quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_conjugate(const bmi088d_quat_t *quat, bmi088d_quat_t *conjugate);

/**
 * @brief Convert quaternion to Euler angles
 * @param[in] quat Input quaternion
 * @param[out] euler Euler angles in degrees
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_to_euler(const bmi088d_quat_t *quat, bmi088d_euler_t *euler);

/**
 * @brief Convert Euler angles to quaternion
 * @param[in] euler Euler angles in degrees
 * @param[out] quat Output quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_euler_to_quat(const bmi088d_euler_t *euler, bmi088d_quat_t *quat);

/**
 * @brief Rotate vector by quaternion
 * @param[in] quat Rotation quaternion
 * @param[in] vec Input vector
 * @param[out] result Rotated vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_rotate_vector(const bmi088d_quat_t *quat, const bmi088d_vec3_t *vec, bmi088d_vec3_t *result);

/**
 * @brief Get quaternion from axis-angle representation
 * @param[in] axis Rotation axis (normalized)
 * @param[in] angle Rotation angle in radians
 * @param[out] quat Output quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_from_axis_angle(const bmi088d_vec3_t *axis, float angle, bmi088d_quat_t *quat);

/**
 * @brief Get quaternion from gyroscope integration
 * @param[in] gyro Gyroscope data in rad/s
 * @param[in] dt Time step in seconds
 * @param[in,out] quat Current quaternion (updated in-place)
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_from_gyro(const bmi088d_vec3_t *gyro, float dt, bmi088d_quat_t *quat);

/**
 * @brief Calculate quaternion difference
 * @param[in] q1 First quaternion
 * @param[in] q2 Second quaternion
 * @param[out] diff Difference quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_difference(const bmi088d_quat_t *q1, const bmi088d_quat_t *q2, bmi088d_quat_t *diff);

/**
 * @brief Spherical linear interpolation between two quaternions
 * @param[in] q1 Start quaternion
 * @param[in] q2 End quaternion
 * @param[in] t Interpolation factor [0, 1]
 * @param[out] result Interpolated quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_slerp(const bmi088d_quat_t *q1, const bmi088d_quat_t *q2, float t, bmi088d_quat_t *result);

/**
 * @brief Get quaternion magnitude
 * @param[in] quat Input quaternion
 * @return Quaternion magnitude
 */
float bmi088d_quat_magnitude(const bmi088d_quat_t *quat);

/**
 * @brief Check if quaternion is valid (finite and normalized)
 * @param[in] quat Input quaternion
 * @return TRUE if valid, FALSE otherwise
 */
uint8_t bmi088d_quat_is_valid(const bmi088d_quat_t *quat);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_QUATERNION_H */