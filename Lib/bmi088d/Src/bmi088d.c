/**
 * @file    bmi088d.c
 * @brief   BMI088 Driver Library - Main Implementation
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d.h"
#include "bmi088d_calibration.h"
#include "bmi088d_ekf.h"
#include "bmi088d_sensor.h"

#include <stdlib.h>
#include <string.h>

/* Library state */
static struct {
    uint8_t initialized;
    bmi088d_imu_data_t imu_data;
    bmi088d_config_t config;
} bmi088d_state = {0};

int32_t bmi088d_init(void)
{
    if (bmi088d_state.initialized) {
        return BMI088D_SUCCESS;
    }

    /* Initialize default configuration */
    bmi088d_state.config.accel_sensitivity = BMI088_ACCEL_6G_SEN;
    bmi088d_state.config.gyro_sensitivity = BMI088_GYRO_2000_SEN;
    bmi088d_state.config.calibrate_on_init = 1;
    bmi088d_state.config.use_temperature_compensation = 1;

    /* Initialize IMU data structure */
    memset(&bmi088d_state.imu_data, 0, sizeof(bmi088d_state.imu_data));
    bmi088d_state.imu_data.accel_scale = 1.0f;
    bmi088d_state.imu_data.g_norm = 9.81f;

    /* Initialize submodules */
    bmi088d_calib_init(NULL);

    bmi088d_ekf_config_t ekf_config;
    bmi088d_ekf_get_default_config(&ekf_config);
    bmi088d_ekf_init(&ekf_config);

    bmi088d_state.initialized = 1;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_init_with_hardware(const bmi088d_hw_config_t *config)
{
    if (bmi088d_state.initialized) {
        return BMI088D_SUCCESS;
    }

    /* Initialize hardware driver */
    if (bmi088d_sensor_init(config) != BMI088D_SENSOR_OK) {
        return BMI088D_ERROR_SENSOR;
    }

    /* Initialize sensors */
    if (bmi088d_accel_init() != BMI088D_SENSOR_OK) {
        return BMI088D_ERROR_SENSOR;
    }

    if (bmi088d_gyro_init() != BMI088D_SENSOR_OK) {
        return BMI088D_ERROR_SENSOR;
    }

    /* Initialize default configuration */
    bmi088d_state.config.accel_sensitivity = BMI088_ACCEL_6G_SEN;
    bmi088d_state.config.gyro_sensitivity = BMI088_GYRO_2000_SEN;
    bmi088d_state.config.calibrate_on_init = 1;
    bmi088d_state.config.use_temperature_compensation = 1;

    /* Initialize IMU data structure */
    memset(&bmi088d_state.imu_data, 0, sizeof(bmi088d_state.imu_data));
    bmi088d_state.imu_data.accel_scale = 1.0f;
    bmi088d_state.imu_data.g_norm = 9.81f;

    /* Initialize submodules */
    bmi088d_calib_init(NULL);

    bmi088d_ekf_config_t ekf_config;
    bmi088d_ekf_get_default_config(&ekf_config);
    bmi088d_ekf_init(&ekf_config);

    bmi088d_state.initialized = 1;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_deinit(void)
{
    if (!bmi088d_state.initialized) {
        return BMI088D_SUCCESS;
    }

    /* Reset state */
    memset(&bmi088d_state, 0, sizeof(bmi088d_state));

    return BMI088D_SUCCESS;
}

void bmi088d_get_version(uint8_t *major, uint8_t *minor, uint8_t *patch)
{
    if (major) *major = BMI088D_VERSION_MAJOR;
    if (minor) *minor = BMI088D_VERSION_MINOR;
    if (patch) *patch = BMI088D_VERSION_PATCH;
}

/* Configuration functions */

int32_t bmi088d_set_config(const bmi088d_config_t *config)
{
    if (!config) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    memcpy(&bmi088d_state.config, config, sizeof(bmi088d_config_t));
    return BMI088D_SUCCESS;
}

int32_t bmi088d_get_config(bmi088d_config_t *config)
{
    if (!config) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    memcpy(config, &bmi088d_state.config, sizeof(bmi088d_config_t));
    return BMI088D_SUCCESS;
}

/* IMU data processing */

int32_t bmi088d_read_sensor_data(void)
{
    if (!bmi088d_state.initialized) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Read raw data from sensor */
    bmi088d_sensor_read_data(&bmi088d_state.imu_data);

    return BMI088D_SUCCESS;
}

int32_t bmi088d_process_raw_data(const int16_t *accel_raw, const int16_t *gyro_raw,
                                int16_t temp_raw, bmi088d_imu_data_t *imu_data)
{
    if (!accel_raw || !gyro_raw || !imu_data) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Apply calibration if available */
    bmi088d_calib_apply(imu_data, accel_raw, gyro_raw);

    /* Process temperature */
    if (temp_raw > 1023) {
        temp_raw -= 2048;
    }
    imu_data->temperature = temp_raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return BMI088D_SUCCESS;
}

int32_t bmi088d_get_imu_data(bmi088d_imu_data_t *imu_data)
{
    if (!imu_data) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    memcpy(imu_data, &bmi088d_state.imu_data, sizeof(bmi088d_imu_data_t));
    return BMI088D_SUCCESS;
}

/* High-level processing functions */

int32_t bmi088d_update_attitude(float dt)
{
    if (!bmi088d_state.initialized) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    /* Update EKF with current IMU data */
    return bmi088d_ekf_update(bmi088d_state.imu_data.gyro[0],
                             bmi088d_state.imu_data.gyro[1],
                             bmi088d_state.imu_data.gyro[2],
                             bmi088d_state.imu_data.accel[0],
                             bmi088d_state.imu_data.accel[1],
                             bmi088d_state.imu_data.accel[2],
                             dt);
}

int32_t bmi088d_get_attitude(bmi088d_euler_t *attitude)
{
    if (!attitude) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    return bmi088d_ekf_get_euler(attitude);
}

int32_t bmi088d_get_orientation(bmi088d_quat_t *orientation)
{
    if (!orientation) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    return bmi088d_ekf_get_quaternion(orientation);
}

/* Utility functions for external use */

int32_t bmi088d_calibrate_sensors(void)
{
    if (!bmi088d_state.initialized) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    return bmi088d_calib_perform(&bmi088d_state.imu_data);
}

int32_t bmi088d_set_calibration_data(const float *gyro_offset, float g_norm, float temp_calibration)
{
    if (!gyro_offset) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    return bmi088d_calib_set_offsets(&bmi088d_state.imu_data, gyro_offset, g_norm, temp_calibration);
}

int32_t bmi088d_apply_temperature_compensation(float current_temp)
{
    if (!bmi088d_state.initialized) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    return bmi088d_calib_apply_temperature_compensation(&bmi088d_state.imu_data, current_temp);
}

float bmi088d_temp_control_pid(float current_temp, float target_temp,
                              float kp, float ki, float kd, float dt,
                              float *integral_state, float *last_error)
{
    /* Calculate error */
    float error = target_temp - current_temp;

    /* Calculate integral */
    *integral_state += error * dt;

    /* Calculate derivative */
    float derivative = (error - *last_error) / dt;

    /* Calculate output */
    float output = kp * error + ki * (*integral_state) + kd * derivative;

    /* Limit output to 0-100% */
    if (output > 100.0f) {
        output = 100.0f;
    } else if (output < 0.0f) {
        output = 0.0f;
    }

    /* Update last error */
    *last_error = error;

    return output;
}

/* Status functions */

int32_t bmi088d_is_initialized(void)
{
    return bmi088d_state.initialized ? BMI088D_SUCCESS : BMI088D_ERROR_INVALID_PARAM;
}

int32_t bmi088d_get_filter_status(bmi088d_filter_status_t *status)
{
    if (!status) {
        return BMI088D_ERROR_INVALID_PARAM;
    }

    return bmi088d_ekf_get_status(status);
}