/**
 * @file    bmi088d_ols.h
 * @brief   BMI088 Driver Library - Ordinary Least Squares (OLS) Module
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-14
 */

#ifndef BMI088D_OLS_H
#define BMI088D_OLS_H

#include <stdint.h>
#include <stdlib.h>  // For malloc and free

#ifdef __cplusplus
extern "C" {
#endif

/* OLS structure */
typedef struct {
  float *x_his;
  float *y_his;
  float *x_sum;
  float *y_sum;
  float *xy_sum;
  float *x_sqr_sum;
  float *beta;
  uint16_t order;
  uint16_t count;
} bmi088d_ols_t;

/**
 * @brief Initialize the OLS estimator
 * @param[in,out] ols OLS structure instance
 * @param[in] order Order of the polynomial
 * @return 0 on success, -1 on failure (memory allocation)
 */
int32_t bmi088d_ols_init(bmi088d_ols_t *ols, uint16_t order);

/**
 * @brief Calculate the derivative using OLS
 * @param[in,out] ols OLS structure instance
 * @param[in] dt Time delta
 * @param[in] y Current value
 * @return Calculated derivative
 */
float bmi088d_ols_derivative(bmi088d_ols_t *ols, float dt, float y);

/**
 * @brief Deinitialize the OLS estimator (free memory)
 * @param[in] ols OLS structure instance
 */
void bmi088d_ols_deinit(bmi088d_ols_t *ols);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_OLS_H */
