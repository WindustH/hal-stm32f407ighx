/**
 * @file    bmi088d_quaternion.c
 * @brief   BMI088 Driver Library - Quaternion Operations
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_quaternion.h"
#include "bmi088d_utils.h"
#include <math.h>

int32_t bmi088d_quat_identity(bmi088d_quat_t *quat)
{
    if (!quat) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    quat->w = 1.0f;
    quat->x = 0.0f;
    quat->y = 0.0f;
    quat->z = 0.0f;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_normalize(bmi088d_quat_t *quat)
{
    if (!quat) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    float norm = bmi088d_quat_magnitude(quat);

    if (norm < 1e-12f) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    float inv_norm = 1.0f / norm;
    quat->w *= inv_norm;
    quat->x *= inv_norm;
    quat->y *= inv_norm;
    quat->z *= inv_norm;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_multiply(const bmi088d_quat_t *q1, const bmi088d_quat_t *q2, bmi088d_quat_t *result)
{
    if (!q1 || !q2 || !result) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_conjugate(const bmi088d_quat_t *quat, bmi088d_quat_t *conjugate)
{
    if (!quat || !conjugate) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    conjugate->w = quat->w;
    conjugate->x = -quat->x;
    conjugate->y = -quat->y;
    conjugate->z = -quat->z;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_to_euler(const bmi088d_quat_t *quat, bmi088d_euler_t *euler)
{
    if (!quat || !euler) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    float w = quat->w;
    float x = quat->x;
    float y = quat->y;
    float z = quat->z;

    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp) * 57.295779513f;

    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(1.57079632679f, sinp) * 57.295779513f; /* 90 degrees */
    } else {
        euler->pitch = asinf(sinp) * 57.295779513f;
    }

    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp) * 57.295779513f;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_euler_to_quat(const bmi088d_euler_t *euler, bmi088d_quat_t *quat)
{
    if (!euler || !quat) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Convert degrees to radians */
    float roll_rad = euler->roll * 0.01745329252f;
    float pitch_rad = euler->pitch * 0.01745329252f;
    float yaw_rad = euler->yaw * 0.01745329252f;

    /* Calculate half angles */
    float cy = cosf(yaw_rad * 0.5f);
    float sy = sinf(yaw_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);

    quat->w = cr * cp * cy + sr * sp * sy;
    quat->x = sr * cp * cy - cr * sp * sy;
    quat->y = cr * sp * cy + sr * cp * sy;
    quat->z = cr * cp * sy - sr * sp * cy;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_rotate_vector(const bmi088d_quat_t *quat, const bmi088d_vec3_t *vec, bmi088d_vec3_t *result)
{
    if (!quat || !vec || !result) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Convert vector to pure quaternion */
    bmi088d_quat_t vec_quat = {0.0f, vec->x, vec->y, vec->z};
    bmi088d_quat_t quat_conj;
    bmi088d_quat_t temp;

    /* Calculate conjugate */
    bmi088d_quat_conjugate(quat, &quat_conj);

    /* Rotate vector: result = quat * vec_quat * quat_conj */
    bmi088d_quat_multiply(quat, &vec_quat, &temp);
    bmi088d_quat_multiply(&temp, &quat_conj, &vec_quat);

    /* Extract rotated vector */
    result->x = vec_quat.x;
    result->y = vec_quat.y;
    result->z = vec_quat.z;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_from_axis_angle(const bmi088d_vec3_t *axis, float angle, bmi088d_quat_t *quat)
{
    if (!axis || !quat) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Normalize axis */
    bmi088d_vec3_t axis_norm = *axis;
    bmi088d_vec_normalize(&axis_norm);

    /* Calculate half angle */
    float half_angle = angle * 0.5f;
    float sin_half = sinf(half_angle);

    quat->w = cosf(half_angle);
    quat->x = axis_norm.x * sin_half;
    quat->y = axis_norm.y * sin_half;
    quat->z = axis_norm.z * sin_half;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_from_gyro(const bmi088d_vec3_t *gyro, float dt, bmi088d_quat_t *quat)
{
    if (!gyro || !quat) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Calculate rotation vector magnitude */
    float gyro_norm = bmi088d_vec_magnitude(gyro);

    if (gyro_norm < 1e-12f) {
        /* No rotation - return identity */
        return BMI088D_SUCCESS;
    }

    /* Calculate rotation angle */
    float angle = gyro_norm * dt;

    /* Create normalized rotation axis */
    bmi088d_vec3_t axis;
    float inv_norm = 1.0f / gyro_norm;
    axis.x = gyro->x * inv_norm;
    axis.y = gyro->y * inv_norm;
    axis.z = gyro->z * inv_norm;

    /* Create rotation quaternion */
    bmi088d_quat_t rotation_quat;
    bmi088d_quat_from_axis_angle(&axis, angle, &rotation_quat);

    /* Apply rotation to current quaternion */
    bmi088d_quat_t result;
    bmi088d_quat_multiply(quat, &rotation_quat, &result);

    /* Copy result back */
    *quat = result;

    /* Normalize quaternion */
    bmi088d_quat_normalize(quat);

    return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_difference(const bmi088d_quat_t *q1, const bmi088d_quat_t *q2, bmi088d_quat_t *diff)
{
    if (!q1 || !q2 || !diff) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Calculate difference: diff = q2 * q1_conj */
    bmi088d_quat_t q1_conj;
    bmi088d_quat_conjugate(q1, &q1_conj);
    bmi088d_quat_multiply(q2, &q1_conj, diff);

    return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_slerp(const bmi088d_quat_t *q1, const bmi088d_quat_t *q2, float t, bmi088d_quat_t *result)
{
    if (!q1 || !q2 || !result) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Clamp t to [0, 1] */
    t = bmi088d_constrain(t, 0.0f, 1.0f);

    /* Calculate dot product */
    float dot = q1->w * q2->w + q1->x * q2->x + q1->y * q2->y + q1->z * q2->z;

    /* If the dot product is negative, the quaternions have opposite handedness */
    float scale1, scale2;
    if (dot < 0.0f) {
        dot = -dot;
        scale2 = -1.0f;
    } else {
        scale2 = 1.0f;
    }

    /* Check for very close quaternions */
    if (dot > 0.9995f) {
        /* Linear interpolation */
        scale1 = 1.0f - t;
        scale2 *= t;
    } else {
        /* Spherical interpolation */
        float theta_0 = acosf(dot);
        float theta = theta_0 * t;
        float sin_theta = sinf(theta);
        float sin_theta_0 = sinf(theta_0);

        scale1 = cosf(theta) - dot * sin_theta / sin_theta_0;
        scale2 = sin_theta / sin_theta_0;
    }

    /* Calculate interpolated quaternion */
    result->w = scale1 * q1->w + scale2 * q2->w;
    result->x = scale1 * q1->x + scale2 * q2->x;
    result->y = scale1 * q1->y + scale2 * q2->y;
    result->z = scale1 * q1->z + scale2 * q2->z;

    /* Normalize result */
    bmi088d_quat_normalize(result);

    return BMI088D_SUCCESS;
}

float bmi088d_quat_magnitude(const bmi088d_quat_t *quat)
{
    if (!quat) {
        return 0.0f;
    }

    return bmi088d_sqrt(quat->w * quat->w +
                       quat->x * quat->x +
                       quat->y * quat->y +
                       quat->z * quat->z);
}

uint8_t bmi088d_quat_is_valid(const bmi088d_quat_t *quat)
{
    if (!quat) {
        return 0;
    }

    /* Check for finite values */
    if (!isfinite(quat->w) || !isfinite(quat->x) ||
        !isfinite(quat->y) || !isfinite(quat->z)) {
        return 0;
    }

    /* Check magnitude (should be close to 1 for normalized quaternion) */
    float mag = bmi088d_quat_magnitude(quat);
    if (fabsf(mag - 1.0f) > 0.01f) {
        return 0;
    }

    return 1;
}