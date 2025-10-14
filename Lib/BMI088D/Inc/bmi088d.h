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

#ifndef __INC_BMI088D__
#define __INC_BMI088D__

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
#include "bmi088d_calibration.h"
#include "bmi088d_ekf.h"
#include "bmi088d_fuzzy.h"
#include "bmi088d_hal.h"
#include "bmi088d_ols.h"
#include "bmi088d_pid.h"
#include "bmi088d_quaternion.h"
#include "bmi088d_reg.h"
#include "bmi088d_sensor.h"
#include "bmi088d_types.h"
#include "bmi088d_utils.h"

/* Default target temperature from source project */
#define BMI088D_DEFAULT_TARGET_TEMP 40.0f

/* Main API functions */

/**
 * @brief Initialize the BMI088 driver library with hardware interface
 * @param config Hardware configuration
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_init(const bmi088d_hw_config_t *config);

/**
 * @brief Update IMU data and perform all processing
 * @param[out] imu_data IMU data structure to be filled
 * @param[in] dt Time step in seconds
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_update(bmi088d_imu_data_t *imu_data, float dt);

/**
 * @brief Control IMU temperature (heater PWM output)
 * @param[in] current_temp Current temperature in Celsius
 * @param[in] dt Time step in seconds
 * @return 0 on success, error code on failure
 */
int32_t bmi088d_temp_control(float current_temp, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __INC_BMI088D__ */