/**
 * @file    bmi088d_types.h
 * @brief   BMI088 Driver Library - Type Definitions
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_TYPES_H
#define BMI088D_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BMI088 sensor constants */
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

/* Error codes */
typedef enum {
  BMI088D_SUCCESS = 0,
  BMI088D_ERROR_INVALID_PARAM = -1,
  BMI088D_ERROR_MEMORY = -2,
  BMI088D_ERROR_CALIBRATION = -3,
  BMI088D_ERROR_FILTER = -4,
  BMI088D_ERROR_SENSOR = -5,
} bmi088d_error_t;

/* IMU data structure */
typedef struct {
  float accel[3];         /* Accelerometer data in m/s² */
  float gyro[3];          /* Gyroscope data in rad/s */
  float temperature;      /* Temperature in °C */
  float calibration_temp; /* Temperature during calibration */
  float accel_scale;      /* Accelerometer scale factor */
  float gyro_offset[3];   /* Gyroscope offset */
  float g_norm;           /* Gravity norm during calibration */
} bmi088d_imu_data_t;

/* Vector structure */
typedef struct {
  float x;
  float y;
  float z;
} bmi088d_vec3_t;

/* Quaternion structure */
typedef struct {
  float w;
  float x;
  float y;
  float z;
} bmi088d_quat_t;

/* Euler angles structure */
typedef struct {
  float roll;  /* Roll angle in degrees */
  float pitch; /* Pitch angle in degrees */
  float yaw;   /* Yaw angle in degrees */
} bmi088d_euler_t;

/* Sensor configuration */
typedef struct {
  float accel_sensitivity;              /* Accelerometer sensitivity */
  float gyro_sensitivity;               /* Gyroscope sensitivity */
  uint8_t calibrate_on_init;            /* Auto-calibrate on initialization */
  uint8_t use_temperature_compensation; /* Enable temperature compensation */
} bmi088d_config_t;

/* Calibration data structure */
typedef struct {
  float gyro_offset[3];   /* Gyroscope offset values */
  float g_norm;           /* Gravity norm during calibration */
  float temp_calibration; /* Temperature during calibration */
  float accel_scale;      /* Accelerometer scale factor */
} bmi088d_calib_data_t;

/* Filter status */
typedef struct {
  uint8_t initialized;
  uint8_t converged;
  uint8_t stable;
  uint64_t error_count;
  uint64_t update_count;
} bmi088d_filter_status_t;

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_TYPES_H */