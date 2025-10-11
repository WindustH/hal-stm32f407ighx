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

/* Calibration configuration */
typedef struct {
    uint16_t sample_count;        /* Number of samples for calibration */
    float max_timeout_s;          /* Maximum calibration timeout in seconds */
    float gyro_diff_threshold;    /* Gyroscope difference threshold */
    float g_norm_diff_threshold;  /* Gravity norm difference threshold */
    float g_norm_expected;        /* Expected gravity norm (9.81 m/s²) */
    float gyro_offset_threshold;  /* Gyroscope offset threshold */
} bmi088d_calib_config_t;

/* Calibration status */
typedef struct {
    uint8_t calibrated;
    uint8_t in_progress;
    uint16_t current_sample;
    float progress;
    bmi088d_error_t last_error;
} bmi088d_calib_status_t;

/**
 * @brief Initialize calibration module
 * @param[in] default_calib Default calibration data (can be NULL)
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_calib_init(const bmi088d_calib_data_t *default_calib);

/**
 * @brief Perform gyroscope and accelerometer calibration
 * @param[in,out] imu_data IMU data structure to store calibration results
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_calib_perform(bmi088d_imu_data_t *imu_data);

/**
 * @brief Start calibration process
 * @param[in,out] imu_data IMU data structure
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_calib_start(bmi088d_imu_data_t *imu_data);

/**
 * @brief Update calibration with new sample
 * @param[in,out] imu_data IMU data structure
 * @param[in] accel_raw Raw accelerometer data
 * @param[in] gyro_raw Raw gyroscope data
 * @return BMI088D_SUCCESS if calibration complete, BMI088D_ERROR_CALIBRATION if failed, BMI088D_SUCCESS if still in progress
 */
int32_t bmi088d_calib_update(bmi088d_imu_data_t *imu_data, const int16_t *accel_raw, const int16_t *gyro_raw);

/**
 * @brief Get calibration status
 * @param[out] status Calibration status structure
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_get_status(bmi088d_calib_status_t *status);

/**
 * @brief Apply calibration to raw sensor data
 * @param[in,out] imu_data IMU data structure
 * @param[in] accel_raw Raw accelerometer data
 * @param[in] gyro_raw Raw gyroscope data
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_apply(bmi088d_imu_data_t *imu_data, const int16_t *accel_raw, const int16_t *gyro_raw);

/**
 * @brief Set pre-calibrated offsets
 * @param[in,out] imu_data IMU data structure
 * @param[in] gyro_offset Gyroscope offset values
 * @param[in] g_norm Gravity norm value
 * @param[in] temp_calibration Temperature during calibration
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_set_offsets(bmi088d_imu_data_t *imu_data, const float *gyro_offset, float g_norm, float temp_calibration);

/**
 * @brief Get default calibration configuration
 * @param[out] config Default calibration configuration
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_get_default_config(bmi088d_calib_config_t *config);

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
int32_t bmi088d_calib_apply_temperature_compensation(bmi088d_imu_data_t *imu_data, float current_temp);

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