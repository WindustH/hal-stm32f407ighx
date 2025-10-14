/**
 * @file    bmi088d_ekf.h
 * @brief   BMI088 Driver Library - Extended Kalman Filter for Attitude
 * Estimation
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_EKF_H
#define BMI088D_EKF_H

#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* EKF configuration */
typedef struct {
  float process_noise_quat;   /* Quaternion process noise */
  float process_noise_bias;   /* Gyro bias process noise */
  float measurement_noise;    /* Accelerometer measurement noise */
  float lambda;               /* Fading coefficient */
  float acc_lpf_coef;         /* Acceleration low-pass filter coefficient */
  float chi_square_threshold; /* Chi-square test threshold */
  float gyro_norm_threshold;  /* Gyro norm threshold for stability */
  float acc_norm_min;         /* Minimum acceleration norm */
  float acc_norm_max;         /* Maximum acceleration norm */
} bmi088d_ekf_config_t;

/* EKF state */
typedef struct {
  bmi088d_filter_status_t status;
  bmi088d_quat_t quaternion;         /* Estimated quaternion */
  bmi088d_vec3_t gyro_bias;          /* Estimated gyroscope bias */
  bmi088d_vec3_t gyro;               /* Gyroscope data (bias compensated) */
  bmi088d_vec3_t accel;              /* Accelerometer data (filtered) */
  bmi088d_vec3_t orientation_cosine; /* Orientation cosine */
  bmi088d_euler_t euler;             /* Euler angles */
  float yaw_total_angle;     /* Total yaw angle (for continuous rotation) */
  float gyro_norm;           /* Gyroscope norm */
  float accel_norm;          /* Accelerometer norm */
  float adaptive_gain_scale; /* Adaptive gain scaling factor */
  float dt;                  /* Update period */
  float chi_square;          /* Chi-square test value */
  int16_t yaw_round_count;   /* Yaw round counter */
  float yaw_angle_last;      /* Last yaw angle */
} bmi088d_ekf_state_t;

/**
 * @brief Initialize Extended Kalman Filter
 * @param[in] config EKF configuration parameters
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_ekf_init(const bmi088d_ekf_config_t *config);

/**
 * @brief Update EKF with new sensor data
 * @param[in] gyro_x Gyroscope X-axis data (rad/s)
 * @param[in] gyro_y Gyroscope Y-axis data (rad/s)
 * @param[in] gyro_z Gyroscope Z-axis data (rad/s)
 * @param[in] accel_x Accelerometer X-axis data (m/s²)
 * @param[in] accel_y Accelerometer Y-axis data (m/s²)
 * @param[in] accel_z Accelerometer Z-axis data (m/s²)
 * @param[in] dt Time step (seconds)
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_ekf_update(float gyro_x, float gyro_y, float gyro_z,
                           float accel_x, float accel_y, float accel_z,
                           float dt);

/**
 * @brief Get current EKF state
 * @param[out] state EKF state structure
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_state(bmi088d_ekf_state_t *state);

/**
 * @brief Get estimated quaternion
 * @param[out] quat Estimated quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_quaternion(bmi088d_quat_t *quat);

/**
 * @brief Get estimated Euler angles
 * @param[out] euler Estimated Euler angles (degrees)
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_euler(bmi088d_euler_t *euler);

/**
 * @brief Get estimated gyroscope bias
 * @param[out] bias Estimated gyroscope bias (rad/s)
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_gyro_bias(bmi088d_vec3_t *bias);

/**
 * @brief Get filter status
 * @param[out] status Filter status structure
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_status(bmi088d_filter_status_t *status);

/**
 * @brief Reset EKF to initial state
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_reset(void);

/**
 * @brief Set EKF initial orientation
 * @param[in] quat Initial quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_set_initial_orientation(const bmi088d_quat_t *quat);

/**
 * @brief Get default EKF configuration
 * @param[out] config Default EKF configuration
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_default_config(bmi088d_ekf_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_EKF_H */