/**
 * @file    bmi088d_calibration.h
 * @brief   BMI088 Driver Library - Calibration Routines
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_CALIBRATION_H
#define BMI088D_CALIBRATION_H

#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize calibration module
 * @param[in] default_calib Default calibration data (can be NULL)
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_calib_init(const bmi088d_calib_data_t *default_calib);

/**
 * @brief Apply calibration to raw sensor data
 * @param[in,out] imu_data IMU data structure
 * @param[in] accel_raw Raw accelerometer data
 * @param[in] gyro_raw Raw gyroscope data
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_apply(bmi088d_imu_data_t *imu_data,
                            const int16_t *accel_raw, const int16_t *gyro_raw);

/**
 * @brief Set pre-calibrated offsets
 * @param[in,out] imu_data IMU data structure
 * @param[in] gyro_offset Gyroscope offset values
 * @param[in] g_norm Gravity norm value
 * @param[in] temp_calibration Temperature during calibration
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_set_offsets(bmi088d_imu_data_t *imu_data,
                                  const float *gyro_offset, float g_norm,
                                  float temp_calibration);

/**
 * @brief Get calibration data
 * @param[out] calib_data Calibration data structure
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_get_data(bmi088d_calib_data_t *calib_data);

/**
 * @brief Reset calibration to default values
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_reset(void);

/**
 * @brief Apply temperature compensation to IMU data
 * @param[in,out] imu_data IMU data structure
 * @param[in] current_temp Current temperature
 * @return BMI088D_SUCCESS on success
 */
int32_t
bmi088d_calib_apply_temperature_compensation(bmi088d_imu_data_t *imu_data,
                                             float current_temp);

/**
 * @brief Perform automatic calibration using sample data
 * @param[in,out] imu_data IMU data structure
 * @param[in] accel_samples Accelerometer sample data
 * @param[in] gyro_samples Gyroscope sample data
 * @param[in] num_samples Number of samples
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_auto_calibrate(bmi088d_imu_data_t *imu_data,
                                     const int16_t *accel_samples,
                                     const int16_t *gyro_samples,
                                     uint16_t num_samples);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_CALIBRATION_H */