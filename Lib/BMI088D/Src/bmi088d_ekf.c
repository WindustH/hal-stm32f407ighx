/**
 * @file    bmi088d_ekf.c
 * @brief   BMI088 Driver Library - Extended Kalman Filter for Attitude
 * Estimation
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_ekf.h"
#include "bmi088d_quaternion.h"
#include "bmi088d_utils.h"
#include <math.h>
#include <string.h>

/* Use CMSIS DSP for matrix operations if available */
#ifdef STM32F407xx
#define BMI088D_USE_CMSIS_DSP
#endif

/* EKF state */
static struct {
  uint8_t initialized;
  bmi088d_ekf_state_t state;
  bmi088d_ekf_config_t config;
} bmi088d_ekf_state = {0};

/* Default EKF configuration */
static const bmi088d_ekf_config_t DEFAULT_EKF_CONFIG = {
    .process_noise_quat = 10.0f,
    .process_noise_bias = 0.001f,
    .measurement_noise = 1000000.0f,
    .lambda = 0.9996f,
    .acc_lpf_coef = 0.0f,
    .chi_square_threshold = 1e-8f,
    .gyro_norm_threshold = 0.3f,
    .acc_norm_min = 9.3f,
    .acc_norm_max = 10.3f};

/* EKF matrices */
static const float EKF_F_MATRIX[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};

static float EKF_P_MATRIX[36] = {
    100000, 0.1, 0.1,    0.1, 0.1, 0.1, 0.1, 100000, 0.1, 0.1,    0.1, 0.1,
    0.1,    0.1, 100000, 0.1, 0.1, 0.1, 0.1, 0.1,    0.1, 100000, 0.1, 0.1,
    0.1,    0.1, 0.1,    0.1, 100, 0.1, 0.1, 0.1,    0.1, 0.1,    0.1, 100};

/* Internal functions */
static void bmi088d_ekf_linearize_f_matrix(float *F, const bmi088d_quat_t *quat,
                                           const bmi088d_vec3_t *gyro,
                                           float dt);
static void bmi088d_ekf_set_h_matrix(float *H, const bmi088d_quat_t *quat);
static void bmi088d_ekf_update_quaternion(bmi088d_quat_t *quat,
                                          const bmi088d_vec3_t *gyro, float dt);
static void bmi088d_ekf_calculate_euler(bmi088d_quat_t *quat,
                                        bmi088d_euler_t *euler);
static void bmi088d_ekf_apply_fading_filter(float *P);
static void bmi088d_ekf_apply_chi_square_test(const float *innovation,
                                              const float *S_inv,
                                              float *chi_square);

int32_t bmi088d_ekf_init(const bmi088d_ekf_config_t *config) {
  if (bmi088d_ekf_state.initialized) {
    return BMI088D_SUCCESS;
  }

  /* Initialize configuration */
  if (config) {
    memcpy(&bmi088d_ekf_state.config, config, sizeof(bmi088d_ekf_config_t));
  } else {
    memcpy(&bmi088d_ekf_state.config, &DEFAULT_EKF_CONFIG,
           sizeof(bmi088d_ekf_config_t));
  }

  /* Initialize state */
  memset(&bmi088d_ekf_state.state, 0, sizeof(bmi088d_ekf_state_t));

  /* Initialize quaternion to identity */
  bmi088d_quat_identity(&bmi088d_ekf_state.state.quaternion);

  /* Initialize status */
  bmi088d_ekf_state.state.status.converged = 0;
  bmi088d_ekf_state.state.status.stable = 0;
  bmi088d_ekf_state.state.status.error_count = 0;
  bmi088d_ekf_state.state.status.update_count = 0;

  /* Initialize matrices */
  /* P matrix is already initialized statically */

  bmi088d_ekf_state.initialized = 1;
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_update(float gyro_x, float gyro_y, float gyro_z,
                           float accel_x, float accel_y, float accel_z,
                           float dt) {
  if (!bmi088d_ekf_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  bmi088d_ekf_state_t *state = &bmi088d_ekf_state.state;
  bmi088d_ekf_config_t *config = &bmi088d_ekf_state.config;

  /* Update time step */
  state->dt = dt;

  /* Apply gyroscope bias compensation */
  state->gyro.x = gyro_x - state->gyro_bias.x;
  state->gyro.y = gyro_y - state->gyro_bias.y;
  state->gyro.z = gyro_z - state->gyro_bias.z;

  /* Apply accelerometer low-pass filter */
  if (state->status.update_count == 0) {
    /* First update - initialize filter */
    state->accel.x = accel_x;
    state->accel.y = accel_y;
    state->accel.z = accel_z;
  } else {
    /* Apply LPF */
    float alpha = config->acc_lpf_coef / (dt + config->acc_lpf_coef);
    float beta = dt / (dt + config->acc_lpf_coef);

    state->accel.x = state->accel.x * alpha + accel_x * beta;
    state->accel.y = state->accel.y * alpha + accel_y * beta;
    state->accel.z = state->accel.z * alpha + accel_z * beta;
  }

  /* Calculate sensor norms */
  state->gyro_norm = bmi088d_vec_magnitude(&state->gyro);
  state->accel_norm = bmi088d_vec_magnitude(&state->accel);

  /* Check stability conditions */
  if (state->gyro_norm < config->gyro_norm_threshold &&
      state->accel_norm > config->acc_norm_min &&
      state->accel_norm < config->acc_norm_max) {
    state->status.stable = 1;
  } else {
    state->status.stable = 0;
  }

  /* Prediction step */
  /* Linearize state transition matrix F */
  float F[36];
  bmi088d_ekf_linearize_f_matrix(F, &state->quaternion, &state->gyro, dt);

  /* Predict state (quaternion integration) */
  bmi088d_ekf_update_quaternion(&state->quaternion, &state->gyro, dt);

  /* Apply fading filter to prevent over-convergence */
  bmi088d_ekf_apply_fading_filter(EKF_P_MATRIX);

  /* Measurement update - only if stable */
  if (state->status.stable) {
    /* Normalize accelerometer measurement */
    bmi088d_vec3_t accel_normed = state->accel;
    bmi088d_vec_normalize(&accel_normed);

    /* Calculate predicted gravity vector from quaternion */
    bmi088d_vec3_t predicted_gravity;
    predicted_gravity.x = 2 * (state->quaternion.x * state->quaternion.z -
                               state->quaternion.w * state->quaternion.y);
    predicted_gravity.y = 2 * (state->quaternion.w * state->quaternion.x +
                               state->quaternion.y * state->quaternion.z);
    predicted_gravity.z = state->quaternion.w * state->quaternion.w -
                          state->quaternion.x * state->quaternion.x -
                          state->quaternion.y * state->quaternion.y +
                          state->quaternion.z * state->quaternion.z;

    /* Calculate innovation */
    float innovation[3];
    innovation[0] = accel_normed.x - predicted_gravity.x;
    innovation[1] = accel_normed.y - predicted_gravity.y;
    innovation[2] = accel_normed.z - predicted_gravity.z;

    /* Simple innovation covariance calculation */
    float S_inv[9] = {1.0f / config->measurement_noise, 0, 0, 0,
                      1.0f / config->measurement_noise, 0, 0, 0,
                      1.0f / config->measurement_noise};

    /* Apply chi-square test */
    float chi_square;
    bmi088d_ekf_apply_chi_square_test(innovation, S_inv, &chi_square);

    /* Update based on chi-square test result */
    if (chi_square < config->chi_square_threshold) {
      /* Innovation is acceptable, proceed with update */
      /* Calculate Kalman gain (simplified) */
      float K[18]; /* 6x3 matrix */
      for (int i = 0; i < 18; i++) {
        K[i] = 0.1f; /* Simplified gain */
      }

      /* Update state */
      state->quaternion.w +=
          K[0] * innovation[0] + K[1] * innovation[1] + K[2] * innovation[2];
      state->quaternion.x +=
          K[3] * innovation[0] + K[4] * innovation[1] + K[5] * innovation[2];
      state->quaternion.y +=
          K[6] * innovation[0] + K[7] * innovation[1] + K[8] * innovation[2];
      state->quaternion.z +=
          K[9] * innovation[0] + K[10] * innovation[1] + K[11] * innovation[2];
      state->gyro_bias.x +=
          K[12] * innovation[0] + K[13] * innovation[1] + K[14] * innovation[2];
      state->gyro_bias.y +=
          K[15] * innovation[0] + K[16] * innovation[1] + K[17] * innovation[2];

      /* Normalize quaternion */
      bmi088d_quat_normalize(&state->quaternion);
    }
  }

  /* Calculate Euler angles */
  bmi088d_ekf_calculate_euler(&state->quaternion, &state->euler);

  /* Update yaw total angle (for continuous rotation) */
  if (state->euler.yaw - state->yaw_angle_last > 180.0f) {
    state->yaw_round_count--;
  } else if (state->euler.yaw - state->yaw_angle_last < -180.0f) {
    state->yaw_round_count++;
  }
  state->yaw_total_angle = 360.0f * state->yaw_round_count + state->euler.yaw;
  state->yaw_angle_last = state->euler.yaw;

  /* Increment update counter */
  state->status.update_count++;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_state(bmi088d_ekf_state_t *state) {
  if (!state || !bmi088d_ekf_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memcpy(state, &bmi088d_ekf_state.state, sizeof(bmi088d_ekf_state_t));
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_quaternion(bmi088d_quat_t *quat) {
  if (!quat || !bmi088d_ekf_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memcpy(quat, &bmi088d_ekf_state.state.quaternion, sizeof(bmi088d_quat_t));
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_euler(bmi088d_euler_t *euler) {
  if (!euler || !bmi088d_ekf_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memcpy(euler, &bmi088d_ekf_state.state.euler, sizeof(bmi088d_euler_t));
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_gyro_bias(bmi088d_vec3_t *bias) {
  if (!bias || !bmi088d_ekf_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memcpy(bias, &bmi088d_ekf_state.state.gyro_bias, sizeof(bmi088d_vec3_t));
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_status(bmi088d_filter_status_t *status) {
  if (!status || !bmi088d_ekf_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memcpy(status, &bmi088d_ekf_state.state.status,
         sizeof(bmi088d_filter_status_t));
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_reset(void) {
  if (!bmi088d_ekf_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Reset state */
  memset(&bmi088d_ekf_state.state, 0, sizeof(bmi088d_ekf_state_t));

  /* Reset quaternion to identity */
  bmi088d_quat_identity(&bmi088d_ekf_state.state.quaternion);

  /* Reset matrices */
  /* P matrix is already initialized statically */

  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_set_initial_orientation(const bmi088d_quat_t *quat) {
  if (!quat || !bmi088d_ekf_state.initialized) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memcpy(&bmi088d_ekf_state.state.quaternion, quat, sizeof(bmi088d_quat_t));
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_default_config(bmi088d_ekf_config_t *config) {
  if (!config) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memcpy(config, &DEFAULT_EKF_CONFIG, sizeof(bmi088d_ekf_config_t));
  return BMI088D_SUCCESS;
}

/* Internal implementation functions */

static void bmi088d_ekf_linearize_f_matrix(float *F, const bmi088d_quat_t *quat,
                                           const bmi088d_vec3_t *gyro,
                                           float dt) {
  /* Linearize the state transition matrix F */
  /* This is a simplified version of the original algorithm */

  float half_gx_dt = 0.5f * gyro->x * dt;
  float half_gy_dt = 0.5f * gyro->y * dt;
  float half_gz_dt = 0.5f * gyro->z * dt;

  /* Copy base identity matrix */
  memcpy(F, EKF_F_MATRIX, sizeof(EKF_F_MATRIX));

  /* Update F matrix elements for quaternion propagation */
  F[1] = -half_gx_dt;
  F[2] = -half_gy_dt;
  F[3] = -half_gz_dt;

  F[6] = half_gx_dt;
  F[8] = half_gz_dt;
  F[9] = -half_gy_dt;

  F[12] = half_gy_dt;
  F[13] = -half_gz_dt;
  F[15] = half_gx_dt;

  F[18] = half_gz_dt;
  F[19] = half_gy_dt;
  F[20] = -half_gx_dt;

  /* Update bias-related terms */
  F[4] = quat->x * dt / 2;
  F[5] = quat->y * dt / 2;

  F[10] = -quat->w * dt / 2;
  F[11] = quat->z * dt / 2;

  F[16] = -quat->z * dt / 2;
  F[17] = -quat->w * dt / 2;

  F[22] = quat->y * dt / 2;
  F[23] = -quat->x * dt / 2;
}

static void bmi088d_ekf_set_h_matrix(float *H, const bmi088d_quat_t *quat) {
  /* Set the observation matrix H */
  float double_w = 2 * quat->w;
  float double_x = 2 * quat->x;
  float double_y = 2 * quat->y;
  float double_z = 2 * quat->z;

  /* Initialize H to zero */
  memset(H, 0, 18 * sizeof(float));

  /* Set H matrix elements */
  H[0] = -double_y;
  H[1] = double_z;
  H[2] = -double_w;
  H[3] = double_x;

  H[6] = double_x;
  H[7] = double_w;
  H[8] = double_z;
  H[9] = double_y;

  H[12] = double_w;
  H[13] = -double_x;
  H[14] = -double_y;
  H[15] = double_z;
}

static void bmi088d_ekf_update_quaternion(bmi088d_quat_t *quat,
                                          const bmi088d_vec3_t *gyro,
                                          float dt) {
  /* Update quaternion using gyroscope integration */
  /* This is a simplified version - in the full EKF, this would be part of the
   * prediction step */

  float half_gx_dt = 0.5f * gyro->x * dt;
  float half_gy_dt = 0.5f * gyro->y * dt;
  float half_gz_dt = 0.5f * gyro->z * dt;

  /* Quaternion derivative */
  bmi088d_quat_t dq;
  dq.w = -half_gx_dt * quat->x - half_gy_dt * quat->y - half_gz_dt * quat->z;
  dq.x = half_gx_dt * quat->w + half_gz_dt * quat->y - half_gy_dt * quat->z;
  dq.y = half_gy_dt * quat->w - half_gz_dt * quat->x + half_gx_dt * quat->z;
  dq.z = half_gz_dt * quat->w + half_gy_dt * quat->x - half_gx_dt * quat->y;

  /* Update quaternion */
  quat->w += dq.w;
  quat->x += dq.x;
  quat->y += dq.y;
  quat->z += dq.z;

  /* Normalize quaternion */
  bmi088d_quat_normalize(quat);
}

static void bmi088d_ekf_calculate_euler(bmi088d_quat_t *quat,
                                        bmi088d_euler_t *euler) {
  /* Convert quaternion to Euler angles */
  float w = quat->w;
  float x = quat->x;
  float y = quat->y;
  float z = quat->z;

  /* Yaw (z-axis rotation) */
  euler->yaw = atan2f(2.0f * (w * z + x * y), 2.0f * (w * w + x * x) - 1.0f) *
               57.295779513f;

  /* Pitch (y-axis rotation) */
  euler->pitch = atan2f(2.0f * (w * x + y * z), 2.0f * (w * w + z * z) - 1.0f) *
                 57.295779513f;

  /* Roll (x-axis rotation) */
  euler->roll = asinf(-2.0f * (x * z - w * y)) * 57.295779513f;
}

static void bmi088d_ekf_apply_fading_filter(float *P) {
  /* Apply fading filter to prevent over-convergence */
  /* This is a simplified version */

  float lambda = bmi088d_ekf_state.config.lambda;

  /* Apply fading to bias terms */
  P[28] /= lambda; /* Gyro bias x variance */
  P[35] /= lambda; /* Gyro bias y variance */

  /* Limit variance to prevent divergence */
  if (P[28] > 10000.0f) {
    P[28] = 10000.0f;
  }
  if (P[35] > 10000.0f) {
    P[35] = 10000.0f;
  }
}

static void bmi088d_ekf_apply_chi_square_test(const float *innovation,
                                              const float *S_inv,
                                              float *chi_square) {
  /* Simplified chi-square test implementation */
  /* In the full implementation, this would calculate the innovation covariance
   */

  /* For now, use a simplified approach */
  *chi_square = innovation[0] * innovation[0] * S_inv[0] +
                innovation[1] * innovation[1] * S_inv[4] +
                innovation[2] * innovation[2] * S_inv[8];
}