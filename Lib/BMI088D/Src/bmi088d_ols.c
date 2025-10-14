/**
 * @file    bmi088d_ols.c
 * @brief   BMI088 Driver Library - Ordinary Least Squares (OLS) Module
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-14
 */

#include "bmi088d_ols.h"
#include "bmi088d.h" // For BMI088D_MALLOC
#include <stdlib.h>  // For malloc and free
#include <string.h>

// Note: In the original source project, OLS_Init used `user_malloc`, which
// could be `pvPortMalloc` (FreeRTOS) or `malloc`. We are using a macro
// `BMI088D_MALLOC` defined in `bmi088d.h` to keep it consistent.

int32_t bmi088d_ols_init(bmi088d_ols_t *ols, uint16_t order) {
  if (!ols || order == 0) {
    return -1;
  }

  ols->order = order;
  ols->count = 0;

  // Allocate memory for historical data and calculation arrays
  ols->x_his = (float *)BMI088D_MALLOC(sizeof(float) * order);
  ols->y_his = (float *)BMI088D_MALLOC(sizeof(float) * order);
  ols->x_sum = (float *)BMI088D_MALLOC(sizeof(float) * (2 * order - 1));
  ols->y_sum = (float *)BMI088D_MALLOC(sizeof(float) * order);
  ols->xy_sum = (float *)BMI088D_MALLOC(sizeof(float) * order);
  ols->x_sqr_sum = (float *)BMI088D_MALLOC(sizeof(float) * order);
  ols->beta = (float *)BMI088D_MALLOC(sizeof(float) * order);

  if (!ols->x_his || !ols->y_his || !ols->x_sum || !ols->y_sum ||
      !ols->xy_sum || !ols->x_sqr_sum || !ols->beta) {
    // Memory allocation failed, free any allocated blocks
    bmi088d_ols_deinit(ols);
    return -1;
  }

  // Initialize all allocated memory to zero
  memset(ols->x_his, 0, sizeof(float) * order);
  memset(ols->y_his, 0, sizeof(float) * order);
  memset(ols->x_sum, 0, sizeof(float) * (2 * order - 1));
  memset(ols->y_sum, 0, sizeof(float) * order);
  memset(ols->xy_sum, 0, sizeof(float) * order);
  memset(ols->x_sqr_sum, 0, sizeof(float) * order);
  memset(ols->beta, 0, sizeof(float) * order);

  return 0;
}

void bmi088d_ols_deinit(bmi088d_ols_t *ols) {
  if (!ols)
    return;
  // The original code did not have a deinit function, which can lead to memory
  // leaks. We add one here for completeness.
  if (ols->x_his) {
    free(ols->x_his);
    ols->x_his = NULL;
  }
  if (ols->y_his) {
    free(ols->y_his);
    ols->y_his = NULL;
  }
  if (ols->x_sum) {
    free(ols->x_sum);
    ols->x_sum = NULL;
  }
  if (ols->y_sum) {
    free(ols->y_sum);
    ols->y_sum = NULL;
  }
  if (ols->xy_sum) {
    free(ols->xy_sum);
    ols->xy_sum = NULL;
  }
  if (ols->x_sqr_sum) {
    free(ols->x_sqr_sum);
    ols->x_sqr_sum = NULL;
  }
  if (ols->beta) {
    free(ols->beta);
    ols->beta = NULL;
  }
}

float bmi088d_ols_derivative(bmi088d_ols_t *ols, float dt, float y) {
  if (!ols || ols->order < 2) { // OLS for derivative needs at least 2nd order
    return 0.0f;
  }

  // Shift history data
  float first_x = ols->x_his[0];
  for (uint16_t i = 0; i < ols->order - 1; ++i) {
    ols->x_his[i] = ols->x_his[i + 1] - first_x;
    ols->y_his[i] = ols->y_his[i + 1];
  }

  // Add new data point
  ols->x_his[ols->order - 1] = ols->x_his[ols->order - 2] + dt;
  ols->y_his[ols->order - 1] = y;

  if (ols->count < ols->order) {
    ols->count++;
  }

  // Recalculate sums for linear regression (1st order polynomial fit)
  float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
  uint16_t start_index = ols->order - ols->count;

  for (uint16_t i = start_index; i < ols->order; ++i) {
    sum_x += ols->x_his[i];
    sum_y += ols->y_his[i];
    sum_xy += ols->x_his[i] * ols->y_his[i];
    sum_x2 += ols->x_his[i] * ols->x_his[i];
  }

  // Calculate slope (k) of the fitted line, which is the derivative
  float denominator = (ols->count * sum_x2 - sum_x * sum_x);
  if (denominator > 1e-6f || denominator < -1e-6f) { // Avoid division by zero
    ols->beta[1] = (ols->count * sum_xy - sum_x * sum_y) / denominator;
  } else {
    ols->beta[1] = 0.0f;
  }

  return ols->beta[1]; // beta[1] is the slope (derivative)
}
