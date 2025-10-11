/**
 * @file    bmi088d_sensor.h
 * @brief   BMI088 Driver Library - Sensor Driver Interface
 * @author
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_SENSOR_H
#define BMI088D_SENSOR_H

#include "bmi088d_types.h"
#include "bmi088d_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Error codes for sensor driver
 */
typedef enum {
    BMI088D_SENSOR_OK = 0,
    BMI088D_SENSOR_ERROR_COMM = -1,
    BMI088D_SENSOR_ERROR_ID = -2,
    BMI088D_SENSOR_ERROR_CONFIG = -3,
    BMI088D_SENSOR_ERROR_SELF_TEST = -4
} bmi088d_sensor_error_t;

/**
 * @brief Initialize sensor driver
 * @param config Hardware configuration
 * @return Error code
 */
bmi088d_sensor_error_t bmi088d_sensor_init(const bmi088d_hw_config_t *config);

/**
 * @brief Initialize accelerometer
 * @return Error code
 */
bmi088d_sensor_error_t bmi088d_accel_init(void);

/**
 * @brief Initialize gyroscope
 * @return Error code
 */
bmi088d_sensor_error_t bmi088d_gyro_init(void);

/**
 * @brief Read sensor data
 * @param data IMU data structure
 */
void bmi088d_sensor_read_data(bmi088d_imu_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_SENSOR_H */