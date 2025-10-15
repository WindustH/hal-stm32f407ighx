/**
 * @file    bmi088d_quaternion.c
 * @brief   BMI088 Driver Library - Quaternion Operations
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_quaternion.h"
#include <math.h>

/* Use CMSIS DSP for quaternion operations */
#include "dsp/quaternion_math_functions.h"

int32_t bmi088d_quat_identity(bmi088d_quat_t *quat) {
  if (!quat) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  quat->w = 1.0f;
  quat->x = 0.0f;
  quat->y = 0.0f;
  quat->z = 0.0f;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_normalize(bmi088d_quat_t *quat) {
  if (!quat) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  float input_quat[4] = {quat->w, quat->x, quat->y, quat->z};
  float normalized_quat[4];

  arm_quaternion_normalize_f32(input_quat, normalized_quat, 1);

  quat->w = normalized_quat[0];
  quat->x = normalized_quat[1];
  quat->y = normalized_quat[2];
  quat->z = normalized_quat[3];

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_multiply(const bmi088d_quat_t *q1,
                              const bmi088d_quat_t *q2,
                              bmi088d_quat_t *result) {
  if (!q1 || !q2 || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  float qa[4] = {q1->w, q1->x, q1->y, q1->z};
  float qb[4] = {q2->w, q2->x, q2->y, q2->z};
  float qr[4];

  arm_quaternion_product_single_f32(qa, qb, qr);

  result->w = qr[0];
  result->x = qr[1];
  result->y = qr[2];
  result->z = qr[3];

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_conjugate(const bmi088d_quat_t *quat,
                               bmi088d_quat_t *conjugate) {
  if (!quat || !conjugate) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  float input_quat[4] = {quat->w, quat->x, quat->y, quat->z};
  float conjugate_quat[4];

  arm_quaternion_conjugate_f32(input_quat, conjugate_quat, 1);

  conjugate->w = conjugate_quat[0];
  conjugate->x = conjugate_quat[1];
  conjugate->y = conjugate_quat[2];
  conjugate->z = conjugate_quat[3];

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_to_euler(const bmi088d_quat_t *quat,
                              bmi088d_euler_t *euler) {
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
    euler->pitch =
        copysignf(1.57079632679f, sinp) * 57.295779513f; /* 90 degrees */
  } else {
    euler->pitch = asinf(sinp) * 57.295779513f;
  }

  /* Yaw (z-axis rotation) */
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  euler->yaw = atan2f(siny_cosp, cosy_cosp) * 57.295779513f;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_euler_to_quat(const bmi088d_euler_t *euler,
                              bmi088d_quat_t *quat) {
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

float bmi088d_quat_magnitude(const bmi088d_quat_t *quat) {
  if (!quat) {
    return 0.0f;
  }

  float input_quat[4] = {quat->w, quat->x, quat->y, quat->z};
  float norm;

  arm_quaternion_norm_f32(input_quat, &norm, 1);

  return norm;
}
