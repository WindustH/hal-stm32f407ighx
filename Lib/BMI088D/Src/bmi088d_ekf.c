/**
 * @file    bmi088d_ekf.c
 * @brief   BMI088 Driver Library - Extended Kalman Filter for Attitude
 * Estimation (Corrected to match source-project)
 * @author  Extracted and corrected from RoboMaster INS Example
 * @version 1.0.1
 * @date    2025-10-15
 */

#include "bmi088d_ekf.h"
#include "bmi088d_quaternion.h"
#include "bmi088d_utils.h"
#include <math.h>
#include <string.h>

/* Use CMSIS DSP for matrix operations */
#include "arm_math.h" // IWYU pragma: keep

/* EKF state */
static bmi088d_ekf_state_t ekf_state = {0};
static bmi088d_ekf_config_t ekf_config = {0};

/* Matrices */
#define STATE_SIZE 6
#define MEAS_SIZE 3

static float F[STATE_SIZE * STATE_SIZE];
static float P[STATE_SIZE * STATE_SIZE];
static float Q[STATE_SIZE * STATE_SIZE];
static float R[MEAS_SIZE * MEAS_SIZE];
static float H[MEAS_SIZE * STATE_SIZE];
static float K[STATE_SIZE * MEAS_SIZE];
static float x[STATE_SIZE];
static float z[MEAS_SIZE];
static float x_minus[STATE_SIZE];
static float P_minus[STATE_SIZE * STATE_SIZE];
static float S[MEAS_SIZE * MEAS_SIZE];
static float HT[STATE_SIZE * MEAS_SIZE];
static float temp_vec1[MEAS_SIZE];
static float temp_mat[STATE_SIZE * STATE_SIZE];
static float temp_mat1[STATE_SIZE * STATE_SIZE];

static float invSqrt(float x);
static void ekf_linearize_F(float dt);
static void ekf_set_H(void);
static void ekf_predict(float dt);
static void ekf_update(void);
static void ekf_apply_chi_square_test(void);
static void ekf_apply_fading_filter(void);

/* Default configuration */
static const bmi088d_ekf_config_t DEFAULT_CONFIG = {
    .process_noise_quat = 10.0f,
    .process_noise_bias = 0.001f,
    .measurement_noise = 10000000.0f,
    .lambda = 0.9996f,
    .acc_lpf_coef = 0.0f,
    .chi_square_threshold = 1e-8f,
    .gyro_norm_threshold = 0.3f,
    .acc_norm_min = 9.3f,
    .acc_norm_max = 10.3f};

int32_t bmi088d_ekf_init(const bmi088d_ekf_config_t *config) {
  if (config) {
    ekf_config = *config;
  } else {
    ekf_config = DEFAULT_CONFIG;
  }

  memset(&ekf_state, 0, sizeof(ekf_state));
  ekf_state.initialized = 1;
  ekf_state.converged = 0;
  ekf_state.stable = 0;
  ekf_state.error_count = 0;
  ekf_state.update_count = 0;
  ekf_state.chi_square_threshold = ekf_config.chi_square_threshold;
  ekf_state.lambda = ekf_config.lambda;
  ekf_state.acc_lpf_coef = ekf_config.acc_lpf_coef;
  ekf_state.Q1 = ekf_config.process_noise_quat;
  ekf_state.Q2 = ekf_config.process_noise_bias;
  ekf_state.R = ekf_config.measurement_noise;

  // Initialize identity F
  memset(F, 0, sizeof(F));
  for (int i = 0; i < STATE_SIZE; i++)
    F[i * STATE_SIZE + i] = 1.0f;

  // Initialize P
  memset(P, 0, sizeof(P));
  P[0] = 100000;
  P[7] = 100000;
  P[14] = 100000;
  P[21] = 100000;
  P[28] = 100;
  P[35] = 100;
  for (int i = 0; i < STATE_SIZE; i++) {
    for (int j = 0; j < STATE_SIZE; j++) {
      if (i != j)
        P[i * STATE_SIZE + j] = 0.1f;
    }
  }

  // Initialize state
  x[0] = 1.0f;
  x[1] = 0;
  x[2] = 0;
  x[3] = 0;
  x[4] = 0;
  x[5] = 0;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_update(float gyro_x, float gyro_y, float gyro_z,
                           float accel_x, float accel_y, float accel_z,
                           float dt) {
  if (!ekf_state.initialized)
    return BMI088D_ERROR_INVALID_PARAM;

  ekf_state.dt = dt;

  // Bias-compensated gyro
  ekf_state.gyro.x = gyro_x - ekf_state.gyro_bias.x;
  ekf_state.gyro.y = gyro_y - ekf_state.gyro_bias.y;
  ekf_state.gyro.z = gyro_z - ekf_state.gyro_bias.z;

  // Accelerometer LPF
  if (ekf_state.update_count == 0) {
    ekf_state.accel.x = accel_x;
    ekf_state.accel.y = accel_y;
    ekf_state.accel.z = accel_z;
  } else {
    float alpha = ekf_config.acc_lpf_coef / (dt + ekf_config.acc_lpf_coef);
    float beta = dt / (dt + ekf_config.acc_lpf_coef);
    ekf_state.accel.x = ekf_state.accel.x * alpha + accel_x * beta;
    ekf_state.accel.y = ekf_state.accel.y * alpha + accel_y * beta;
    ekf_state.accel.z = ekf_state.accel.z * alpha + accel_z * beta;
  }

  // Norms
  ekf_state.gyro_norm = bmi088d_vec_magnitude(&ekf_state.gyro);
  ekf_state.accel_norm = bmi088d_vec_magnitude(&ekf_state.accel);

  // Stability check
  if (ekf_state.gyro_norm < ekf_config.gyro_norm_threshold &&
      ekf_state.accel_norm > ekf_config.acc_norm_min &&
      ekf_state.accel_norm < ekf_config.acc_norm_max) {
    ekf_state.stable = 1;
  } else {
    ekf_state.stable = 0;
  }

  // Normalize accel for measurement
  float inv_norm = invSqrt(ekf_state.accel.x * ekf_state.accel.x +
                           ekf_state.accel.y * ekf_state.accel.y +
                           ekf_state.accel.z * ekf_state.accel.z);
  z[0] = ekf_state.accel.x * inv_norm;
  z[1] = ekf_state.accel.y * inv_norm;
  z[2] = ekf_state.accel.z * inv_norm;

  // Set current state
  x[0] = ekf_state.quaternion.w;
  x[1] = ekf_state.quaternion.x;
  x[2] = ekf_state.quaternion.y;
  x[3] = ekf_state.quaternion.z;
  x[4] = ekf_state.gyro_bias.x;
  x[5] = ekf_state.gyro_bias.y;

  // Set Q and R
  memset(Q, 0, sizeof(Q));
  Q[0] = ekf_state.Q1 * dt;
  Q[7] = ekf_state.Q1 * dt;
  Q[14] = ekf_state.Q1 * dt;
  Q[21] = ekf_state.Q1 * dt;
  Q[28] = ekf_state.Q2 * dt;
  Q[35] = ekf_state.Q2 * dt;

  memset(R, 0, sizeof(R));
  R[0] = ekf_state.R;
  R[4] = ekf_state.R;
  R[8] = ekf_state.R;

  // Predict
  ekf_linearize_F(dt);
  ekf_predict(dt);

  // Update if stable or converged
  if (ekf_state.stable || ekf_state.converged) {
    ekf_set_H();
    ekf_apply_chi_square_test();
    ekf_update();
  } else {
    // No update: just copy predicted state
    memcpy(x, x_minus, sizeof(x));
    memcpy(P, P_minus, sizeof(P));
  }

  // Extract results
  ekf_state.quaternion.w = x[0];
  ekf_state.quaternion.x = x[1];
  ekf_state.quaternion.y = x[2];
  ekf_state.quaternion.z = x[3];
  ekf_state.gyro_bias.x = x[4];
  ekf_state.gyro_bias.y = x[5];
  ekf_state.gyro_bias.z = 0; // Z bias not estimated

  // Normalize quaternion
  bmi088d_quat_normalize(&ekf_state.quaternion);

  // Euler angles
  bmi088d_quat_to_euler(&ekf_state.quaternion, &ekf_state.euler);

  // Yaw total
  if (ekf_state.euler.yaw - ekf_state.yaw_angle_last > 180.0f) {
    ekf_state.yaw_round_count--;
  } else if (ekf_state.euler.yaw - ekf_state.yaw_angle_last < -180.0f) {
    ekf_state.yaw_round_count++;
  }
  ekf_state.yaw_total_angle =
      360.0f * ekf_state.yaw_round_count + ekf_state.euler.yaw;
  ekf_state.yaw_angle_last = ekf_state.euler.yaw;

  ekf_state.update_count++;
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_state(bmi088d_ekf_state_t *state) {
  if (!state)
    return BMI088D_ERROR_INVALID_PARAM;
  *state = ekf_state;
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_quaternion(bmi088d_quat_t *quat) {
  if (!quat)
    return BMI088D_ERROR_INVALID_PARAM;
  *quat = ekf_state.quaternion;
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_euler(bmi088d_euler_t *euler) {
  if (!euler)
    return BMI088D_ERROR_INVALID_PARAM;
  *euler = ekf_state.euler;
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_get_gyro_bias(bmi088d_vec3_t *bias) {
  if (!bias)
    return BMI088D_ERROR_INVALID_PARAM;
  *bias = ekf_state.gyro_bias;
  return BMI088D_SUCCESS;
}

int32_t bmi088d_ekf_reset(void) { return bmi088d_ekf_init(NULL); }

int32_t bmi088d_ekf_get_default_config(bmi088d_ekf_config_t *config) {
  if (!config)
    return BMI088D_ERROR_INVALID_PARAM;
  *config = DEFAULT_CONFIG;
  return BMI088D_SUCCESS;
}

/* Internal functions */

static void ekf_linearize_F(float dt) {
  float q0 = x_minus[0], q1 = x_minus[1], q2 = x_minus[2], q3 = x_minus[3];
  float gx = ekf_state.gyro.x, gy = ekf_state.gyro.y, gz = ekf_state.gyro.z;

  float half_gx_dt = 0.5f * gx * dt;
  float half_gy_dt = 0.5f * gy * dt;
  float half_gz_dt = 0.5f * gz * dt;

  // Reset F to identity for upper-left 4x4 and lower-right 2x2
  memset(F, 0, sizeof(F));
  for (int i = 0; i < 4; i++)
    F[i * STATE_SIZE + i] = 1.0f;
  F[28] = 1.0f;
  F[35] = 1.0f;

  // Upper-left 4x4 (quaternion propagation)
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

  // Upper-right 4x2 (bias effect)
  F[4] = q1 * dt / 2.0f;
  F[5] = q2 * dt / 2.0f;
  F[10] = -q0 * dt / 2.0f;
  F[11] = q3 * dt / 2.0f;
  F[16] = -q3 * dt / 2.0f;
  F[17] = -q0 * dt / 2.0f;
  F[22] = q2 * dt / 2.0f;
  F[23] = -q1 * dt / 2.0f;
}

static void ekf_set_H(void) {
  float q0 = x_minus[0], q1 = x_minus[1], q2 = x_minus[2], q3 = x_minus[3];
  float dq0 = 2 * q0, dq1 = 2 * q1, dq2 = 2 * q2, dq3 = 2 * q3;

  memset(H, 0, sizeof(H));

  H[0] = -dq2;
  H[1] = dq3;
  H[2] = -dq0;
  H[3] = dq1;
  H[6] = dq1;
  H[7] = dq0;
  H[8] = dq3;
  H[9] = dq2;
  H[12] = dq0;
  H[13] = -dq1;
  H[14] = -dq2;
  H[15] = dq3;
  // Last two columns (bias) are zero by default
}

static void ekf_predict(float dt __attribute__((unused))) {
  // x_minus = F * x
  arm_matrix_instance_f32 F_mat, x_mat, x_minus_mat;
  arm_mat_init_f32(&F_mat, STATE_SIZE, STATE_SIZE, F);
  arm_mat_init_f32(&x_mat, STATE_SIZE, 1, x);
  arm_mat_init_f32(&x_minus_mat, STATE_SIZE, 1, x_minus);
  arm_mat_mult_f32(&F_mat, &x_mat, &x_minus_mat);

  // Normalize quaternion part of x_minus
  float q_norm = invSqrt(x_minus[0] * x_minus[0] + x_minus[1] * x_minus[1] +
                         x_minus[2] * x_minus[2] + x_minus[3] * x_minus[3]);
  for (int i = 0; i < 4; i++)
    x_minus[i] *= q_norm;

  // P_minus = F * P * F^T + Q
  arm_matrix_instance_f32 P_mat, FT_mat, temp1, temp2, Q_mat, P_minus_mat;
  arm_mat_init_f32(&F_mat, STATE_SIZE, STATE_SIZE, F);
  arm_mat_init_f32(&P_mat, STATE_SIZE, STATE_SIZE, P);
  arm_mat_init_f32(&FT_mat, STATE_SIZE, STATE_SIZE, temp_mat);
  arm_mat_init_f32(&temp1, STATE_SIZE, STATE_SIZE, temp_mat);
  arm_mat_init_f32(&temp2, STATE_SIZE, STATE_SIZE, temp_mat1);
  arm_mat_init_f32(&Q_mat, STATE_SIZE, STATE_SIZE, Q);
  arm_mat_init_f32(&P_minus_mat, STATE_SIZE, STATE_SIZE, P_minus);

  arm_mat_trans_f32(&F_mat, &FT_mat);
  arm_mat_mult_f32(&F_mat, &P_mat, &temp1);
  arm_mat_mult_f32(&temp1, &FT_mat, &temp2);
  arm_mat_add_f32(&temp2, &Q_mat, &P_minus_mat);

  // Apply fading filter and limit
  ekf_apply_fading_filter();
}

static void ekf_update(void) {
  arm_matrix_instance_f32 H_mat, P_minus_mat, HT_mat, R_mat, S_mat, S_inv_mat;
  arm_matrix_instance_f32 K_mat, temp1, temp2, z_mat, x_mat, x_minus_mat;

  arm_mat_init_f32(&H_mat, MEAS_SIZE, STATE_SIZE, H);
  arm_mat_init_f32(&P_minus_mat, STATE_SIZE, STATE_SIZE, P_minus);
  arm_mat_init_f32(&HT_mat, STATE_SIZE, MEAS_SIZE, HT);
  arm_mat_init_f32(&R_mat, MEAS_SIZE, MEAS_SIZE, R);
  arm_mat_init_f32(&S_mat, MEAS_SIZE, MEAS_SIZE, S);
  arm_mat_init_f32(&S_inv_mat, MEAS_SIZE, MEAS_SIZE, temp_mat1);
  arm_mat_init_f32(&K_mat, STATE_SIZE, MEAS_SIZE, K);
  arm_mat_init_f32(&temp1, MEAS_SIZE, STATE_SIZE, temp_mat);
  arm_mat_init_f32(&temp2, MEAS_SIZE, MEAS_SIZE, temp_mat1);
  arm_mat_init_f32(&z_mat, MEAS_SIZE, 1, z);
  arm_mat_init_f32(&x_minus_mat, STATE_SIZE, 1, x_minus);
  arm_mat_init_f32(&x_mat, STATE_SIZE, 1, x);

  // h = H(x_minus) -> predicted measurement
  float h[3];
  h[0] = 2 * (x_minus[1] * x_minus[3] - x_minus[0] * x_minus[2]);
  h[1] = 2 * (x_minus[0] * x_minus[1] + x_minus[2] * x_minus[3]);
  h[2] = x_minus[0] * x_minus[0] - x_minus[1] * x_minus[1] -
         x_minus[2] * x_minus[2] + x_minus[3] * x_minus[3];
  arm_matrix_instance_f32 h_mat_inst;
  arm_mat_init_f32(&h_mat_inst, MEAS_SIZE, 1, h);

  // innovation = z - h
  arm_matrix_instance_f32 innov_mat_inst;
  arm_mat_init_f32(&innov_mat_inst, MEAS_SIZE, 1, temp_vec1);
  arm_mat_sub_f32(&z_mat, &h_mat_inst, &innov_mat_inst);

  // Orientation cosine
  ekf_state.orientation_cosine.x = acosf(fabsf(h[0]));
  ekf_state.orientation_cosine.y = acosf(fabsf(h[1]));
  ekf_state.orientation_cosine.z = acosf(fabsf(h[2]));

  // S = H * P_minus * H^T + R
  arm_mat_trans_f32(&H_mat, &HT_mat);
  arm_mat_mult_f32(&H_mat, &P_minus_mat, &temp1);
  arm_mat_mult_f32(&temp1, &HT_mat, &temp2);
  arm_mat_add_f32(&temp2, &R_mat, &S_mat);

  // S_inv = inv(S)
  if (arm_mat_inverse_f32(&S_mat, &S_inv_mat) != ARM_MATH_SUCCESS) {
    // Skip update on singular matrix
    memcpy(x, x_minus, sizeof(x));
    memcpy(P, P_minus, sizeof(P));
    return;
  }

  // K = P_minus * H^T * S_inv
  arm_mat_mult_f32(&P_minus_mat, &HT_mat, &temp1);
  arm_mat_mult_f32(&temp1, &S_inv_mat, &K_mat);

  // Adaptive gain
  float adaptive_scale = ekf_state.adaptive_gain_scale;

  // Apply orientation cosine weighting to bias rows
  for (int i = 4; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == 4) {
        K[i * 3 + j] *= ekf_state.orientation_cosine.x / 1.5707963f;
      } else if (i == 5) {
        K[i * 3 + j] *= ekf_state.orientation_cosine.y / 1.5707963f;
      }
    }
  }

  // Apply adaptive scale
  for (int i = 0; i < STATE_SIZE * MEAS_SIZE; i++) {
    K[i] *= adaptive_scale;
  }

  // x = x_minus + K * innovation
  arm_mat_mult_f32(&K_mat, &innov_mat_inst, &temp1);
  // Zero yaw correction
  temp1.pData[3] = 0;
  // Bias correction limit
  if (ekf_state.converged) {
    for (int i = 4; i < 6; i++) {
      if (temp1.pData[i] > 1e-2f * ekf_state.dt)
        temp1.pData[i] = 1e-2f * ekf_state.dt;
      if (temp1.pData[i] < -1e-2f * ekf_state.dt)
        temp1.pData[i] = -1e-2f * ekf_state.dt;
    }
  }
  arm_mat_add_f32(&x_minus_mat, &temp1, &x_mat);

  // P = (I - K*H) * P_minus
  arm_mat_mult_f32(&K_mat, &H_mat, &temp1);

  // Create identity matrix
  arm_matrix_instance_f32 I_mat;
  arm_mat_init_f32(&I_mat, STATE_SIZE, STATE_SIZE, temp_mat);
  memset(temp_mat, 0, sizeof(float) * STATE_SIZE * STATE_SIZE);
  for (int i = 0; i < STATE_SIZE; i++) {
    temp_mat[i * STATE_SIZE + i] = 1.0f;
  }

  // I - K*H
  arm_matrix_instance_f32 I_KH;
  arm_mat_init_f32(&I_KH, STATE_SIZE, STATE_SIZE, temp_mat1);
  arm_mat_sub_f32(&I_mat, &temp1, &I_KH);

  arm_matrix_instance_f32 P_mat;
  arm_mat_init_f32(&P_mat, STATE_SIZE, STATE_SIZE, P);
  arm_mat_mult_f32(&I_KH, &P_minus_mat, &P_mat);
}

static void ekf_apply_chi_square_test(void) {
  arm_matrix_instance_f32 H_mat, P_minus_mat, HT_mat, R_mat, S_mat, S_inv_mat;
  arm_matrix_instance_f32 innov_mat;
  arm_matrix_instance_f32 temp1, temp2;

  arm_mat_init_f32(&H_mat, MEAS_SIZE, STATE_SIZE, H);
  arm_mat_init_f32(&P_minus_mat, STATE_SIZE, STATE_SIZE, P_minus);
  arm_mat_init_f32(&HT_mat, STATE_SIZE, MEAS_SIZE, HT);
  arm_mat_init_f32(&R_mat, MEAS_SIZE, MEAS_SIZE, R);
  arm_mat_init_f32(&S_mat, MEAS_SIZE, MEAS_SIZE, S);
  arm_mat_init_f32(&S_inv_mat, MEAS_SIZE, MEAS_SIZE, temp_mat1);

  // h = H(x_minus)
  float h[3];
  h[0] = 2 * (x_minus[1] * x_minus[3] - x_minus[0] * x_minus[2]);
  h[1] = 2 * (x_minus[0] * x_minus[1] + x_minus[2] * x_minus[3]);
  h[2] = x_minus[0] * x_minus[0] - x_minus[1] * x_minus[1] -
         x_minus[2] * x_minus[2] + x_minus[3] * x_minus[3];

  // innovation = z - h
  float innov[3] = {z[0] - h[0], z[1] - h[1], z[2] - h[2]};
  arm_mat_init_f32(&innov_mat, MEAS_SIZE, 1, innov);

  // S = H * P_minus * H^T + R
  arm_mat_trans_f32(&H_mat, &HT_mat);
  arm_mat_mult_f32(&H_mat, &P_minus_mat, &temp1);
  arm_mat_mult_f32(&temp1, &HT_mat, &temp2);
  arm_mat_add_f32(&temp2, &R_mat, &S_mat);

  // S_inv = inv(S)
  if (arm_mat_inverse_f32(&S_mat, &S_inv_mat) != ARM_MATH_SUCCESS) {
    ekf_state.chi_square = 1e6f;
    return;
  }

  // chi_square = innov^T * S_inv * innov
  arm_mat_mult_f32(&S_inv_mat, &innov_mat, &temp1);
  float chi_sq;
  arm_dot_prod_f32(innov, temp1.pData, MEAS_SIZE, &chi_sq);
  ekf_state.chi_square = chi_sq;

  if (chi_sq < 0.5f * ekf_state.chi_square_threshold) {
    ekf_state.converged = 1;
  }

  if (chi_sq > ekf_state.chi_square_threshold && ekf_state.converged) {
    if (ekf_state.stable) {
      ekf_state.error_count++;
    } else {
      ekf_state.error_count = 0;
    }

    if (ekf_state.error_count > 50) {
      ekf_state.converged = 0;
      // Skip P update
    } else {
      // Skip update
      memcpy(x, x_minus, sizeof(x));
      memcpy(P, P_minus, sizeof(P));
      return;
    }
  }

  // Adaptive gain scale
  if (chi_sq > 0.1f * ekf_state.chi_square_threshold && ekf_state.converged) {
    ekf_state.adaptive_gain_scale = (ekf_state.chi_square_threshold - chi_sq) /
                                    (0.9f * ekf_state.chi_square_threshold);
  } else {
    ekf_state.adaptive_gain_scale = 1.0f;
  }
  ekf_state.error_count = 0;
}

static void ekf_apply_fading_filter(void) {
  P_minus[28] /= ekf_state.lambda;
  P_minus[35] /= ekf_state.lambda;
  if (P_minus[28] > 10000)
    P_minus[28] = 10000;
  if (P_minus[35] > 10000)
    P_minus[35] = 10000;
}

static float invSqrt(float x) {
  if (x <= 0.0f) {
    return 0.0f;
  }

  // Use standard library sqrtf for better portability and no aliasing issues
  return 1.0f / sqrtf(x);
}