/**
 * @file    bmi088d_calibration.c
 * @brief   BMI088 Driver Library - Calibration Algorithms
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_calibration.h"
#include "bmi088d_utils.h"
#include <math.h>
#include <string.h>

/* Calibration state */
static struct {
  uint8_t initialized;
  bmi088d_calib_data_t calib_data;
} bmi088d_calib_state = {0};

/* Default calibration values */
static const float DEFAULT_GYRO_OFFSET[3] = {0.0f, 0.0f, 0.0f};
static const float DEFAULT_G_NORM = 9.81f;
static const float DEFAULT_TEMP_CALIBRATION = 40.0f;

int32_t bmi088d_calib_init(const bmi088d_calib_data_t *default_calib) {
  if (bmi088d_calib_state.initialized) {
    return BMI088D_SUCCESS;
  }

  /* Initialize with default or provided calibration data */
  if (default_calib) {
    memcpy(&bmi088d_calib_state.calib_data, default_calib,
           sizeof(bmi088d_calib_data_t));
  } else {
    /* Set default calibration */
    memcpy(bmi088d_calib_state.calib_data.gyro_offset, DEFAULT_GYRO_OFFSET,
           sizeof(DEFAULT_GYRO_OFFSET));
    bmi088d_calib_state.calib_data.g_norm = DEFAULT_G_NORM;
    bmi088d_calib_state.calib_data.temp_calibration = DEFAULT_TEMP_CALIBRATION;
    bmi088d_calib_state.calib_data.accel_scale = 1.0f;
  }

  bmi088d_calib_state.initialized = 1;
  return BMI088D_SUCCESS;
}

int32_t bmi088d_calib_apply(bmi088d_imu_data_t *imu_data,
                            const int16_t *accel_raw, const int16_t *gyro_raw) {
  if (!imu_data) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Apply gyroscope offset calibration */
  if (gyro_raw) {
    /* Convert raw gyro data and apply offset */
    /* Use default gyro sensitivity (2000 deg/s range) */
    float gyro_scale = BMI088_GYRO_2000_SEN;
    imu_data->gyro[0] = gyro_raw[0] * gyro_scale -
                        bmi088d_calib_state.calib_data.gyro_offset[0];
    imu_data->gyro[1] = gyro_raw[1] * gyro_scale -
                        bmi088d_calib_state.calib_data.gyro_offset[1];
    imu_data->gyro[2] = gyro_raw[2] * gyro_scale -
                        bmi088d_calib_state.calib_data.gyro_offset[2];
  }

  /* Apply accelerometer scale calibration */
  if (accel_raw) {
    imu_data->accel[0] = accel_raw[0] * imu_data->accel_scale *
                         bmi088d_calib_state.calib_data.accel_scale;
    imu_data->accel[1] = accel_raw[1] * imu_data->accel_scale *
                         bmi088d_calib_state.calib_data.accel_scale;
    imu_data->accel[2] = accel_raw[2] * imu_data->accel_scale *
                         bmi088d_calib_state.calib_data.accel_scale;
  }

  /* Apply gravity norm calibration */
  imu_data->g_norm = bmi088d_calib_state.calib_data.g_norm;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_calib_set_offsets(bmi088d_imu_data_t *imu_data,
                                  const float *gyro_offset, float g_norm,
                                  float temp_calibration) {
  if (!gyro_offset || !imu_data) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Set gyroscope offsets */
  memcpy(bmi088d_calib_state.calib_data.gyro_offset, gyro_offset,
         3 * sizeof(float));

  /* Set gravity norm */
  bmi088d_calib_state.calib_data.g_norm = g_norm;

  /* Set temperature calibration */
  bmi088d_calib_state.calib_data.temp_calibration = temp_calibration;

  /* Calculate accelerometer scale based on gravity norm */
  bmi088d_calib_state.calib_data.accel_scale = 9.81f / g_norm;

  /* Update IMU data */
  imu_data->g_norm = g_norm;
  imu_data->accel_scale = bmi088d_calib_state.calib_data.accel_scale;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_calib_get_data(bmi088d_calib_data_t *calib_data) {
  if (!calib_data) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memcpy(calib_data, &bmi088d_calib_state.calib_data,
         sizeof(bmi088d_calib_data_t));
  return BMI088D_SUCCESS;
}

int32_t bmi088d_calib_reset(void) {
  /* Reset to default calibration */
  memcpy(bmi088d_calib_state.calib_data.gyro_offset, DEFAULT_GYRO_OFFSET,
         sizeof(DEFAULT_GYRO_OFFSET));
  bmi088d_calib_state.calib_data.g_norm = DEFAULT_G_NORM;
  bmi088d_calib_state.calib_data.temp_calibration = DEFAULT_TEMP_CALIBRATION;
  bmi088d_calib_state.calib_data.accel_scale = 1.0f;

  return BMI088D_SUCCESS;
}

/* Temperature compensation functions */

int32_t
bmi088d_calib_apply_temperature_compensation(bmi088d_imu_data_t *imu_data,
                                             float current_temp) {
  if (!imu_data) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Simple temperature compensation model */
  float temp_diff =
      current_temp - bmi088d_calib_state.calib_data.temp_calibration;

  /* Apply temperature compensation to gyroscope (example coefficients) */
  float temp_comp_coef = 0.001f; /* Temperature compensation coefficient */

  imu_data->gyro[0] *= (1.0f + temp_comp_coef * temp_diff);
  imu_data->gyro[1] *= (1.0f + temp_comp_coef * temp_diff);
  imu_data->gyro[2] *= (1.0f + temp_comp_coef * temp_diff);

  return BMI088D_SUCCESS;
}

/* Advanced calibration algorithms extracted from original code */

int32_t bmi088d_calib_auto_calibrate(bmi088d_imu_data_t *imu_data,
                                     const int16_t *accel_samples,
                                     const int16_t *gyro_samples,
                                     uint16_t num_samples) {
  if (!imu_data || !accel_samples || !gyro_samples || num_samples == 0) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  float gyro_sum[3] = {0};
  float g_norm_sum = 0;
  float gyro_max[3] = {0}, gyro_min[3] = {0};
  float g_norm_max = 0, g_norm_min = 0;
  float gyro_diff[3], g_norm_diff;

  /* Calculate gyroscope offset and gravity norm from samples */
  for (uint16_t i = 0; i < num_samples; i++) {
    const int16_t *accel = &accel_samples[i * 3];
    const int16_t *gyro = &gyro_samples[i * 3];

    /* Accumulate gyroscope data */
    /* Use default gyro sensitivity (2000 deg/s range) */
    float gyro_scale = BMI088_GYRO_2000_SEN;
    float gyro_x = gyro[0] * gyro_scale;
    float gyro_y = gyro[1] * gyro_scale;
    float gyro_z = gyro[2] * gyro_scale;

    gyro_sum[0] += gyro_x;
    gyro_sum[1] += gyro_y;
    gyro_sum[2] += gyro_z;

    /* Calculate gravity norm from accelerometer */
    float accel_x = accel[0] * imu_data->accel_scale;
    float accel_y = accel[1] * imu_data->accel_scale;
    float accel_z = accel[2] * imu_data->accel_scale;

    float g_norm =
        bmi088d_sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    g_norm_sum += g_norm;

    /* Track min/max for stability checking */
    if (i == 0) {
      g_norm_max = g_norm_min = g_norm;
      gyro_max[0] = gyro_min[0] = gyro_x;
      gyro_max[1] = gyro_min[1] = gyro_y;
      gyro_max[2] = gyro_min[2] = gyro_z;
    } else {
      if (g_norm > g_norm_max)
        g_norm_max = g_norm;
      if (g_norm < g_norm_min)
        g_norm_min = g_norm;
      if (gyro_x > gyro_max[0])
        gyro_max[0] = gyro_x;
      if (gyro_x < gyro_min[0])
        gyro_min[0] = gyro_x;
      if (gyro_y > gyro_max[1])
        gyro_max[1] = gyro_y;
      if (gyro_y < gyro_min[1])
        gyro_min[1] = gyro_y;
      if (gyro_z > gyro_max[2])
        gyro_max[2] = gyro_z;
      if (gyro_z < gyro_min[2])
        gyro_min[2] = gyro_z;
    }
  }

  /* Calculate differences */
  g_norm_diff = g_norm_max - g_norm_min;
  for (int i = 0; i < 3; i++) {
    gyro_diff[i] = gyro_max[i] - gyro_min[i];
  }

  /* Check stability - if motion is detected, calibration is not reliable */
  if (g_norm_diff > 0.5f || gyro_diff[0] > 0.15f || gyro_diff[1] > 0.15f ||
      gyro_diff[2] > 0.15f) {
    return BMI088D_ERROR_CALIBRATION;
  }

  /* Calculate averages */
  bmi088d_calib_state.calib_data.gyro_offset[0] = gyro_sum[0] / num_samples;
  bmi088d_calib_state.calib_data.gyro_offset[1] = gyro_sum[1] / num_samples;
  bmi088d_calib_state.calib_data.gyro_offset[2] = gyro_sum[2] / num_samples;
  bmi088d_calib_state.calib_data.g_norm = g_norm_sum / num_samples;

  /* Check gravity norm */
  if (fabsf(bmi088d_calib_state.calib_data.g_norm - 9.8f) > 0.5f) {
    return BMI088D_ERROR_CALIBRATION;
  }

  /* Check gyro bias */
  if (fabsf(bmi088d_calib_state.calib_data.gyro_offset[0]) > 0.01f ||
      fabsf(bmi088d_calib_state.calib_data.gyro_offset[1]) > 0.01f ||
      fabsf(bmi088d_calib_state.calib_data.gyro_offset[2]) > 0.01f) {
    return BMI088D_ERROR_CALIBRATION;
  }

  /* Calculate accelerometer scale */
  bmi088d_calib_state.calib_data.accel_scale =
      9.81f / bmi088d_calib_state.calib_data.g_norm;

  /* Update IMU data */
  imu_data->g_norm = bmi088d_calib_state.calib_data.g_norm;
  imu_data->accel_scale = bmi088d_calib_state.calib_data.accel_scale;

  return BMI088D_SUCCESS;
}