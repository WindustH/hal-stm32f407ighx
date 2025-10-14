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

/* Temperature control PID */
static bmi088d_pid_t temp_ctrl_pid = {0};

int32_t bmi088d_init(const bmi088d_hw_config_t *config) {
  if (bmi088d_state.initialized) {
    return BMI088D_SUCCESS;
  }

  /* Initialize hardware driver and sensors */
  if (bmi088d_sensor_init(config) != BMI088D_SENSOR_OK) {
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

  /* Initialize temperature control PID with parameters from original source
   * project */
  bmi088d_pid_init(&temp_ctrl_pid, 2000.0f, /* max_output */
                   300.0f,                  /* integral_limit */
                   0.0f,                    /* deadband */
                   1000.0f,                 /* kp */
                   20.0f,                   /* ki */
                   0.0f,                    /* kd */
                   0.0f,                    /* coef_a */
                   0.0f,                    /* coef_b */
                   0.0f,                    /* output_lpf_rc */
                   0.0f,                    /* derivative_lpf_rc */
                   0,                       /* ols_order */
                   0);                      /* improvements */

  bmi088d_state.initialized = 1;
  return BMI088D_SUCCESS;
}

int32_t bmi088d_update(bmi088d_imu_data_t *imu_data, float dt) {
  if (!bmi088d_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  if (!imu_data) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Read raw sensor data */
  bmi088d_sensor_read_data(&bmi088d_state.imu_data);

  /* Update attitude estimation */
  bmi088d_ekf_update(
      bmi088d_state.imu_data.gyro[0], bmi088d_state.imu_data.gyro[1],
      bmi088d_state.imu_data.gyro[2], bmi088d_state.imu_data.accel[0],
      bmi088d_state.imu_data.accel[1], bmi088d_state.imu_data.accel[2], dt);

  /* Copy data to output structure */
  memcpy(imu_data, &bmi088d_state.imu_data, sizeof(bmi088d_imu_data_t));

  return BMI088D_SUCCESS;
}

int32_t bmi088d_temp_control(float current_temp, float dt) {
  if (!bmi088d_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Use the advanced PID controller with explicit time step and default target
   * temperature */
  float output = bmi088d_pid_calculate(&temp_ctrl_pid, current_temp,
                                       BMI088D_DEFAULT_TARGET_TEMP, dt);

  /* Limit output to 0-100% */
  if (output > 100.0f) {
    output = 100.0f;
  } else if (output < 0.0f) {
    output = 0.0f;
  }

  /* Get timer handle and channel from HAL */
  TIM_HandleTypeDef *htim = bmi088d_hal_get_timer();
  uint32_t tim_channel = bmi088d_hal_get_timer_channel();

  /* Check if timer is valid */
  if (htim == NULL) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Convert percentage to timer pulse value */
  /* Assuming timer period is 2000 (0-1999) */
  uint32_t pulse = (uint32_t)(output * 2000.0f / 100.0f);

  /* Update PWM duty cycle */
  __HAL_TIM_SET_COMPARE(htim, tim_channel, pulse);

  return BMI088D_SUCCESS;
}
