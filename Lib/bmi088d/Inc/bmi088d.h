/**
 * @file    bmi088d.h
 * @brief   BMI088 Driver Library - Main Header
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 *
 * @attention
 * This library provides BMI088 sensor driver algorithms including:
 * - Sensor data reading and calibration
 * - Extended Kalman Filter (EKF) for attitude estimation
 * - Quaternion-based orientation algorithms
 * - Temperature compensation algorithms
 * - Various data processing and filtering routines
 *
 * Dependencies: CMSIS, HAL, Standard C Library (GCC ARM)
 */

#ifndef BMI088D_H
#define BMI088D_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Library version */
#define BMI088D_VERSION_MAJOR 1
#define BMI088D_VERSION_MINOR 0
#define BMI088D_VERSION_PATCH 0

/* Common mathematical constants */
#ifndef PI
#define PI 3.14159265354f
#endif

#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif

/* Boolean type definitions */
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* Memory allocation macro */
#ifndef BMI088D_MALLOC
#ifdef _CMSIS_OS_H
#define BMI088D_MALLOC pvPortMalloc
#else
#define BMI088D_MALLOC malloc
#endif
#endif

/* Include submodules */
#include "bmi088d_types.h"
#include "bmi088d_reg.h"
#include "bmi088d_sensor.h"
#include "bmi088d_calibration.h"
#include "bmi088d_ekf.h"
#include "bmi088d_quaternion.h"
#include "bmi088d_pid.h"
#include "bmi088d_utils.h"
#include "bmi088d_hal.h"

/**
 * @brief Initialize the BMI088 driver library
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_init(void);

/**
 * @brief Initialize the BMI088 driver library with hardware interface
 * @param config Hardware configuration
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_init_with_hardware(const bmi088d_hw_config_t *config);

/**
 * @brief Deinitialize the BMI088 driver library
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_deinit(void);

/**
 * @brief Get library version information
 * @param[out] major Major version number
 * @param[out] minor Minor version number
 * @param[out] patch Patch version number
 */
void bmi088d_get_version(uint8_t *major, uint8_t *minor, uint8_t *patch);

/* Configuration functions */

/**
 * @brief Set library configuration
 * @param[in] config Configuration parameters
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_set_config(const bmi088d_config_t *config);

/**
 * @brief Get library configuration
 * @param[out] config Configuration parameters
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_get_config(bmi088d_config_t *config);

/* IMU data processing */

/**
 * @brief Read sensor data from BMI088
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_read_sensor_data(void);

/**
 * @brief Process raw sensor data
 * @param[in] accel_raw Raw accelerometer data
 * @param[in] gyro_raw Raw gyroscope data
 * @param[in] temp_raw Raw temperature data
 * @param[out] imu_data Processed IMU data
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_process_raw_data(const int16_t *accel_raw, const int16_t *gyro_raw,
                                int16_t temp_raw, bmi088d_imu_data_t *imu_data);

/**
 * @brief Get processed IMU data
 * @param[out] imu_data IMU data structure
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_get_imu_data(bmi088d_imu_data_t *imu_data);

/* High-level processing functions */

/**
 * @brief Update attitude estimation using EKF
 * @param[in] dt Time step in seconds
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_update_attitude(float dt);

/**
 * @brief Get current orientation as Euler angles
 * @param[out] attitude Euler angles structure
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_get_attitude(bmi088d_euler_t *attitude);

/**
 * @brief Get current orientation as quaternion
 * @param[out] orientation Quaternion structure
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_get_orientation(bmi088d_quat_t *orientation);

/* Utility functions for external use */

/**
 * @brief Perform sensor calibration
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_calibrate_sensors(void);

/**
 * @brief Set pre-calibrated sensor data
 * @param[in] gyro_offset Gyroscope offset values
 * @param[in] g_norm Gravity norm value
 * @param[in] temp_calibration Temperature during calibration
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_set_calibration_data(const float *gyro_offset, float g_norm, float temp_calibration);

/**
 * @brief Reset all filters to initial state
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_reset_filters(void);

/**
 * @brief Apply temperature compensation to sensor data
 * @param[in] current_temp Current temperature in Celsius
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_apply_temperature_compensation(float current_temp);

/* Temperature control functions */

/**
 * @brief Calculate PWM output for temperature control using PID
 * @param[in] current_temp Current temperature in Celsius
 * @param[in] target_temp Target temperature in Celsius
 * @param[in] kp Proportional gain
 * @param[in] ki Integral gain
 * @param[in] kd Derivative gain
 * @param[in] dt Time step in seconds
 * @param[in,out] integral_state Integral state for PID controller
 * @param[in,out] last_error Last error for PID controller
 * @return PWM output value (0-100%)
 */
float bmi088d_temp_control_pid(float current_temp, float target_temp,
                              float kp, float ki, float kd, float dt,
                              float *integral_state, float *last_error);

/* Status functions */

/**
 * @brief Check if library is initialized
 * @return 0 if initialized, error code if not
 */
int32_t bmi088d_is_initialized(void);

/**
 * @brief Get filter status
 * @param[out] status Filter status structure
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_get_filter_status(bmi088d_filter_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_H */