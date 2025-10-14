# Source Code Collection

Root path: `/run/media/windy/iris/Downloads/workspace/hal-stm32f407ighx/Lib/BMI088D`

## `Inc/bmi088d.h`

```cpp
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
```

## `Inc/bmi088d_all.h`

```cpp
/**
 * @file    bmi088d_all.h
 * @brief   BMI088 Driver Library - Consolidated Header
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 *
 * @attention
 * This is a consolidated header file that includes all BMI088D functionality
 * in a single header for simplified usage.
 */

#ifndef BMI088D_ALL_H
#define BMI088D_ALL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include all BMI088D components */
#include "bmi088d.h"
#include "bmi088d_calibration.h"
#include "bmi088d_ekf.h"
#include "bmi088d_hal.h"
#include "bmi088d_pid.h"
#include "bmi088d_quaternion.h"
#include "bmi088d_reg.h"
#include "bmi088d_sensor.h"
#include "bmi088d_types.h"
#include "bmi088d_utils.h"

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_ALL_H */
```

## `Inc/bmi088d_calibration.h`

```cpp
/**
 * @file    bmi088d_calibration.h
 * @brief   BMI088 Driver Library - Calibration Routines
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_CALIBRATION_H
#define BMI088D_CALIBRATION_H

#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Calibration configuration */
typedef struct {
  uint16_t sample_count;       /* Number of samples for calibration */
  float max_timeout_s;         /* Maximum calibration timeout in seconds */
  float gyro_diff_threshold;   /* Gyroscope difference threshold */
  float g_norm_diff_threshold; /* Gravity norm difference threshold */
  float g_norm_expected;       /* Expected gravity norm (9.81 m/s²) */
  float gyro_offset_threshold; /* Gyroscope offset threshold */
} bmi088d_calib_config_t;

/* Calibration status */
typedef struct {
  uint8_t calibrated;
  uint8_t in_progress;
  uint16_t current_sample;
  float progress;
  bmi088d_error_t last_error;
} bmi088d_calib_status_t;

/**
 * @brief Initialize calibration module
 * @param[in] default_calib Default calibration data (can be NULL)
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_calib_init(const bmi088d_calib_data_t *default_calib);

/**
 * @brief Start calibration process
 * @param[in,out] imu_data IMU data structure
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_calib_start(bmi088d_imu_data_t *imu_data);

/**
 * @brief Update calibration with new sample
 * @param[in,out] imu_data IMU data structure
 * @param[in] accel_raw Raw accelerometer data
 * @param[in] gyro_raw Raw gyroscope data
 * @return BMI088D_SUCCESS if calibration complete, BMI088D_ERROR_CALIBRATION if
 * failed, BMI088D_SUCCESS if still in progress
 */
int32_t bmi088d_calib_update(bmi088d_imu_data_t *imu_data,
                             const int16_t *accel_raw, const int16_t *gyro_raw);

/**
 * @brief Get calibration status
 * @param[out] status Calibration status structure
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_get_status(bmi088d_calib_status_t *status);

/**
 * @brief Apply calibration to raw sensor data
 * @param[in,out] imu_data IMU data structure
 * @param[in] accel_raw Raw accelerometer data
 * @param[in] gyro_raw Raw gyroscope data
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_apply(bmi088d_imu_data_t *imu_data,
                            const int16_t *accel_raw, const int16_t *gyro_raw);

/**
 * @brief Set pre-calibrated offsets
 * @param[in,out] imu_data IMU data structure
 * @param[in] gyro_offset Gyroscope offset values
 * @param[in] g_norm Gravity norm value
 * @param[in] temp_calibration Temperature during calibration
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_set_offsets(bmi088d_imu_data_t *imu_data,
                                  const float *gyro_offset, float g_norm,
                                  float temp_calibration);

/**
 * @brief Get default calibration configuration
 * @param[out] config Default calibration configuration
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_get_default_config(bmi088d_calib_config_t *config);

/**
 * @brief Get calibration data
 * @param[out] calib_data Calibration data structure
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_get_data(bmi088d_calib_data_t *calib_data);

/**
 * @brief Reset calibration to default values
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_reset(void);

/**
 * @brief Apply temperature compensation to IMU data
 * @param[in,out] imu_data IMU data structure
 * @param[in] current_temp Current temperature
 * @return BMI088D_SUCCESS on success
 */
int32_t
bmi088d_calib_apply_temperature_compensation(bmi088d_imu_data_t *imu_data,
                                             float current_temp);

/**
 * @brief Perform automatic calibration using sample data
 * @param[in,out] imu_data IMU data structure
 * @param[in] accel_samples Accelerometer sample data
 * @param[in] gyro_samples Gyroscope sample data
 * @param[in] num_samples Number of samples
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_calib_auto_calibrate(bmi088d_imu_data_t *imu_data,
                                     const int16_t *accel_samples,
                                     const int16_t *gyro_samples,
                                     uint16_t num_samples);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_CALIBRATION_H */
```

## `Inc/bmi088d_ekf.h`

```cpp
/**
 * @file    bmi088d_ekf.h
 * @brief   BMI088 Driver Library - Extended Kalman Filter for Attitude
 * Estimation
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_EKF_H
#define BMI088D_EKF_H

#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* EKF configuration */
typedef struct {
  float process_noise_quat;   /* Quaternion process noise */
  float process_noise_bias;   /* Gyro bias process noise */
  float measurement_noise;    /* Accelerometer measurement noise */
  float lambda;               /* Fading coefficient */
  float acc_lpf_coef;         /* Acceleration low-pass filter coefficient */
  float chi_square_threshold; /* Chi-square test threshold */
  float gyro_norm_threshold;  /* Gyro norm threshold for stability */
  float acc_norm_min;         /* Minimum acceleration norm */
  float acc_norm_max;         /* Maximum acceleration norm */
} bmi088d_ekf_config_t;

/* EKF state */
typedef struct {
  bmi088d_filter_status_t status;
  bmi088d_quat_t quaternion;         /* Estimated quaternion */
  bmi088d_vec3_t gyro_bias;          /* Estimated gyroscope bias */
  bmi088d_vec3_t gyro;               /* Gyroscope data (bias compensated) */
  bmi088d_vec3_t accel;              /* Accelerometer data (filtered) */
  bmi088d_vec3_t orientation_cosine; /* Orientation cosine */
  bmi088d_euler_t euler;             /* Euler angles */
  float yaw_total_angle;     /* Total yaw angle (for continuous rotation) */
  float gyro_norm;           /* Gyroscope norm */
  float accel_norm;          /* Accelerometer norm */
  float adaptive_gain_scale; /* Adaptive gain scaling factor */
  float dt;                  /* Update period */
  float chi_square;          /* Chi-square test value */
  int16_t yaw_round_count;   /* Yaw round counter */
  float yaw_angle_last;      /* Last yaw angle */
} bmi088d_ekf_state_t;

/**
 * @brief Initialize Extended Kalman Filter
 * @param[in] config EKF configuration parameters
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_ekf_init(const bmi088d_ekf_config_t *config);

/**
 * @brief Update EKF with new sensor data
 * @param[in] gyro_x Gyroscope X-axis data (rad/s)
 * @param[in] gyro_y Gyroscope Y-axis data (rad/s)
 * @param[in] gyro_z Gyroscope Z-axis data (rad/s)
 * @param[in] accel_x Accelerometer X-axis data (m/s²)
 * @param[in] accel_y Accelerometer Y-axis data (m/s²)
 * @param[in] accel_z Accelerometer Z-axis data (m/s²)
 * @param[in] dt Time step (seconds)
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_ekf_update(float gyro_x, float gyro_y, float gyro_z,
                           float accel_x, float accel_y, float accel_z,
                           float dt);

/**
 * @brief Get current EKF state
 * @param[out] state EKF state structure
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_state(bmi088d_ekf_state_t *state);

/**
 * @brief Get estimated quaternion
 * @param[out] quat Estimated quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_quaternion(bmi088d_quat_t *quat);

/**
 * @brief Get estimated Euler angles
 * @param[out] euler Estimated Euler angles (degrees)
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_euler(bmi088d_euler_t *euler);

/**
 * @brief Get estimated gyroscope bias
 * @param[out] bias Estimated gyroscope bias (rad/s)
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_gyro_bias(bmi088d_vec3_t *bias);

/**
 * @brief Get filter status
 * @param[out] status Filter status structure
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_status(bmi088d_filter_status_t *status);

/**
 * @brief Reset EKF to initial state
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_reset(void);

/**
 * @brief Set EKF initial orientation
 * @param[in] quat Initial quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_set_initial_orientation(const bmi088d_quat_t *quat);

/**
 * @brief Get default EKF configuration
 * @param[out] config Default EKF configuration
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_ekf_get_default_config(bmi088d_ekf_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_EKF_H */
```

## `Inc/bmi088d_fuzzy.h`

```cpp
/**
 * @file    bmi088d_fuzzy.h
 * @brief   BMI088 Driver Library - Fuzzy Logic for PID
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-14
 */

#ifndef BMI088D_FUZZY_H
#define BMI088D_FUZZY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fuzzy logic linguistic variables */
#define BMI088D_FUZZY_NB -3.0f
#define BMI088D_FUZZY_NM -2.0f
#define BMI088D_FUZZY_NS -1.0f
#define BMI088D_FUZZY_ZE  0.0f
#define BMI088D_FUZZY_PS  1.0f
#define BMI088D_FUZZY_PM  2.0f
#define BMI088D_FUZZY_PB  3.0f

/* Fuzzy rule structure */
typedef struct {
    float kp_fuzzy;
    float ki_fuzzy;
    float kd_fuzzy;

    const float (*rule_kp)[7];
    const float (*rule_ki)[7];
    const float (*rule_kd)[7];

    float e_step;
    float ec_step;

    float e;
    float ec;
    float e_last;
} bmi088d_fuzzy_rule_t;

/**
 * @brief Initialize the fuzzy logic rule set for PID controller
 * @param[out] rule Pointer to the fuzzy rule structure
 * @param[in] fuzzy_rule_kp Custom Kp rule matrix (7x7), or NULL for default
 * @param[in] fuzzy_rule_ki Custom Ki rule matrix (7x7), or NULL for default
 * @param[in] fuzzy_rule_kd Custom Kd rule matrix (7x7), or NULL for default
 * @param[in] e_step Step size for error quantization
 * @param[in] ec_step Step size for error change quantization
 */
void bmi088d_fuzzy_rule_init(bmi088d_fuzzy_rule_t *rule,
                             const float (*fuzzy_rule_kp)[7],
                             const float (*fuzzy_rule_ki)[7],
                             const float (*fuzzy_rule_kd)[7],
                             float e_step,
                             float ec_step);

/**
 * @brief Execute the fuzzy logic inference to get PID gain adjustments
 * @param[in,out] rule Pointer to the fuzzy rule structure
 * @param[in] measure Current measured value
 * @param[in] ref Target reference value
 * @param[in] dt Time delta
 */
void bmi088d_fuzzy_rule_implement(bmi088d_fuzzy_rule_t *rule, float measure, float ref, float dt);


#ifdef __cplusplus
}
#endif

#endif /* BMI088D_FUZZY_H */

```

## `Inc/bmi088d_hal.h`

```cpp
/**
 * @file    bmi088d_hal.h
 * @brief   BMI088 Driver Library - Hardware Abstraction Layer
 * @author
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_HAL_H
#define BMI088D_HAL_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Hardware configuration structure
 */
typedef struct {
  SPI_HandleTypeDef *hspi;     /**< SPI handle */
  GPIO_TypeDef *accel_cs_port; /**< Accelerometer CS port */
  uint16_t accel_cs_pin;       /**< Accelerometer CS pin */
  GPIO_TypeDef *gyro_cs_port;  /**< Gyroscope CS port */
  uint16_t gyro_cs_pin;        /**< Gyroscope CS pin */
  TIM_HandleTypeDef *htim;     /**< Timer handle for PWM output */
  uint32_t tim_channel;        /**< Timer channel for PWM output */
} bmi088d_hw_config_t;

/**
 * @brief Initialize hardware abstraction layer
 * @param config Hardware configuration
 */
void bmi088d_hal_init(const bmi088d_hw_config_t *config);

/**
 * @brief Get timer handle
 * @return Timer handle
 */
TIM_HandleTypeDef *bmi088d_hal_get_timer(void);

/**
 * @brief Get timer channel
 * @return Timer channel
 */
uint32_t bmi088d_hal_get_timer_channel(void);

/**
 * @brief Delay in milliseconds
 * @param ms Delay in milliseconds
 */
void bmi088d_hal_delay_ms(uint32_t ms);

/**
 * @brief Delay in microseconds
 * @param us Delay in microseconds
 */
void bmi088d_hal_delay_us(uint32_t us);

/**
 * @brief Set accelerometer chip select low
 */
void bmi088d_hal_accel_cs_low(void);

/**
 * @brief Set accelerometer chip select high
 */
void bmi088d_hal_accel_cs_high(void);

/**
 * @brief Set gyroscope chip select low
 */
void bmi088d_hal_gyro_cs_low(void);

/**
 * @brief Set gyroscope chip select high
 */
void bmi088d_hal_gyro_cs_high(void);

/**
 * @brief SPI read/write byte
 * @param data Data to write
 * @return Data read
 */
uint8_t bmi088d_hal_spi_read_write(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_HAL_H */
```

## `Inc/bmi088d_ols.h`

```cpp
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

```

## `Inc/bmi088d_pid.h`

```cpp
/**
 * @file    bmi088d_pid.h
 * @brief   BMI088 Driver Library - PID Control Algorithms (Ported from
 * source-project)
 * @author  Extracted from RoboMaster INS Example
 * @version 1.1.0
 * @date    2025-10-14
 */

#ifndef BMI088D_PID_H
#define BMI088D_PID_H

#include "bmi088d_fuzzy.h"
#include "bmi088d_ols.h"
#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PID improvement flags from source-project */
typedef enum {
  BMI088D_PID_IMPROVE_NONE = 0x00,
  BMI088D_PID_IMPROVE_INTEGRAL_LIMIT = 0x01,
  BMI088D_PID_IMPROVE_DERIVATIVE_ON_MEASUREMENT = 0x02,
  BMI088D_PID_IMPROVE_TRAPEZOID_INTEGRAL = 0x04,
  BMI088D_PID_IMPROVE_OUTPUT_FILTER = 0x10,
  BMI088D_PID_IMPROVE_CHANGING_INTEGRATION_RATE = 0x20,
  BMI088D_PID_IMPROVE_DERIVATIVE_FILTER = 0x40,
  BMI088D_PID_IMPROVE_ERROR_HANDLE = 0x80,
} bmi088d_pid_improvement_t;

/* PID error types from source-project */
typedef enum {
  BMI088D_PID_ERROR_NONE = 0x00U,
  BMI088D_PID_ERROR_MOTOR_BLOCKED = 0x01U
} bmi088d_pid_error_type_t;

/* PID error handler structure from source-project */
typedef struct {
  uint64_t error_count;
  bmi088d_pid_error_type_t error_type;
} bmi088d_pid_error_handler_t;

/* Forward declaration for PID structure */
typedef struct bmi088d_pid_s bmi088d_pid_t;

/* PID controller structure, adapted from source-project */
struct bmi088d_pid_s {
  /* Core PID parameters */
  float ref;
  float kp;
  float ki;
  float kd;

  /* State variables */
  float measure;
  float last_measure;
  float error;
  float last_error;
  float last_iterm;

  /* Output components */
  float p_out;
  float i_out;
  float d_out;
  float i_term;

  float output;
  float last_output;
  float last_d_out;

  /* Configuration parameters */
  float max_output;
  float integral_limit;
  float deadband;
  float coef_a;        /* For Changing Integration Rate */
  float coef_b;        /* ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B */
  float output_lpf_rc; /* Output low-pass filter RC time constant */
  float derivative_lpf_rc; /* Derivative low-pass filter RC time constant */

  /* OLS for derivative calculation */
  uint16_t ols_order;
  bmi088d_ols_t ols;

  /* Fuzzy logic extension */
  bmi088d_fuzzy_rule_t *fuzzy_rule;

  /* Improvement flags */
  uint8_t improvements;

  /* Error handler */
  bmi088d_pid_error_handler_t error_handler;

  /* User callback functions */
  void (*user_func1)(bmi088d_pid_t *pid);
  void (*user_func2)(bmi088d_pid_t *pid);
};

/**
 * @brief Initialize PID controller (adapted from source-project)
 * @param[in,out] pid PID controller structure
 * @param[in] max_output Maximum output limit
 * @param[in] integral_limit Integral windup limit
 * @param[in] deadband Error deadband
 * @param[in] kp Proportional gain
 * @param[in] ki Integral gain
 * @param[in] kd Derivative gain
 * @param[in] coef_a Changing integration coefficient A
 * @param[in] coef_b Changing integration coefficient B
 * @param[in] output_lpf_rc Output low-pass filter time constant
 * @param[in] derivative_lpf_rc Derivative low-pass filter time constant
 * @param[in] ols_order Order for OLS derivative calculation (e.g., 10). If < 2,
 * standard diff is used.
 * @param[in] improvements Improvement flags
 * @return BMI088D_SUCCESS on success, error code on failure
 */
int32_t bmi088d_pid_init(bmi088d_pid_t *pid, float max_output,
                         float integral_limit, float deadband, float kp,
                         float ki, float kd, float coef_a, float coef_b,
                         float output_lpf_rc, float derivative_lpf_rc,
                         uint16_t ols_order, uint8_t improvements);

/**
 * @brief Deinitialize PID controller (frees OLS memory)
 * @param[in] pid PID controller structure
 */
void bmi088d_pid_deinit(bmi088d_pid_t *pid);

/**
 * @brief Calculate PID output (adapted from source-project)
 * @param[in,out] pid PID controller structure
 * @param[in] measure Measurement value
 * @param[in] ref Reference value
 * @param[in] dt Time step in seconds
 * @return PID output
 */
float bmi088d_pid_calculate(bmi088d_pid_t *pid, float measure, float ref,
                            float dt);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_PID_H */
```

## `Inc/bmi088d_quaternion.h`

```cpp
/**
 * @file    bmi088d_quaternion.h
 * @brief   BMI088 Driver Library - Quaternion Operations
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_QUATERNION_H
#define BMI088D_QUATERNION_H

#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize quaternion to identity
 * @param[out] quat Quaternion to initialize
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_identity(bmi088d_quat_t *quat);

/**
 * @brief Normalize quaternion
 * @param[in,out] quat Quaternion to normalize
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_normalize(bmi088d_quat_t *quat);

/**
 * @brief Multiply two quaternions: result = q1 * q2
 * @param[in] q1 First quaternion
 * @param[in] q2 Second quaternion
 * @param[out] result Result quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_multiply(const bmi088d_quat_t *q1,
                              const bmi088d_quat_t *q2, bmi088d_quat_t *result);

/**
 * @brief Get quaternion conjugate
 * @param[in] quat Input quaternion
 * @param[out] conjugate Conjugate quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_conjugate(const bmi088d_quat_t *quat,
                               bmi088d_quat_t *conjugate);

/**
 * @brief Convert quaternion to Euler angles
 * @param[in] quat Input quaternion
 * @param[out] euler Euler angles in degrees
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_to_euler(const bmi088d_quat_t *quat,
                              bmi088d_euler_t *euler);

/**
 * @brief Convert Euler angles to quaternion
 * @param[in] euler Euler angles in degrees
 * @param[out] quat Output quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_euler_to_quat(const bmi088d_euler_t *euler,
                              bmi088d_quat_t *quat);

/**
 * @brief Rotate vector by quaternion
 * @param[in] quat Rotation quaternion
 * @param[in] vec Input vector
 * @param[out] result Rotated vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_rotate_vector(const bmi088d_quat_t *quat,
                                   const bmi088d_vec3_t *vec,
                                   bmi088d_vec3_t *result);

/**
 * @brief Get quaternion from axis-angle representation
 * @param[in] axis Rotation axis (normalized)
 * @param[in] angle Rotation angle in radians
 * @param[out] quat Output quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_from_axis_angle(const bmi088d_vec3_t *axis, float angle,
                                     bmi088d_quat_t *quat);

/**
 * @brief Get quaternion from gyroscope integration
 * @param[in] gyro Gyroscope data in rad/s
 * @param[in] dt Time step in seconds
 * @param[in,out] quat Current quaternion (updated in-place)
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_from_gyro(const bmi088d_vec3_t *gyro, float dt,
                               bmi088d_quat_t *quat);

/**
 * @brief Calculate quaternion difference
 * @param[in] q1 First quaternion
 * @param[in] q2 Second quaternion
 * @param[out] diff Difference quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_difference(const bmi088d_quat_t *q1,
                                const bmi088d_quat_t *q2, bmi088d_quat_t *diff);

/**
 * @brief Spherical linear interpolation between two quaternions
 * @param[in] q1 Start quaternion
 * @param[in] q2 End quaternion
 * @param[in] t Interpolation factor [0, 1]
 * @param[out] result Interpolated quaternion
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_quat_slerp(const bmi088d_quat_t *q1, const bmi088d_quat_t *q2,
                           float t, bmi088d_quat_t *result);

/**
 * @brief Get quaternion magnitude
 * @param[in] quat Input quaternion
 * @return Quaternion magnitude
 */
float bmi088d_quat_magnitude(const bmi088d_quat_t *quat);

/**
 * @brief Check if quaternion is valid (finite and normalized)
 * @param[in] quat Input quaternion
 * @return TRUE if valid, FALSE otherwise
 */
uint8_t bmi088d_quat_is_valid(const bmi088d_quat_t *quat);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_QUATERNION_H */
```

## `Inc/bmi088d_reg.h`

```cpp
/**
 * @file    bmi088d_reg.h
 * @brief   BMI088 Driver Library - Register Definitions
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_REG_H
#define BMI088D_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Accelerometer registers */
#define BMI088_ACC_CHIP_ID 0x00 // the register is "Who am I"
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
#define BMI088_FATAL_ERROR_SHFITS 0x0
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

#define BMI088_ACC_STATUS 0x03
#define BMI088_ACCEL_DRDY_SHFITS 0x7
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

#define BMI088_ACCEL_XOUT_L 0x12
#define BMI088_ACCEL_XOUT_M 0x13
#define BMI088_ACCEL_YOUT_L 0x14
#define BMI088_ACCEL_YOUT_M 0x15
#define BMI088_ACCEL_ZOUT_L 0x16
#define BMI088_ACCEL_ZOUT_M 0x17

#define BMI088_SENSORTIME_DATA_L 0x18
#define BMI088_SENSORTIME_DATA_M 0x19
#define BMI088_SENSORTIME_DATA_H 0x1A

#define BMI088_ACC_INT_STAT_1 0x1D
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

#define BMI088_TEMP_M 0x22
#define BMI088_TEMP_L 0x23

#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_CONF_MUST_Set 0x80
#define BMI088_ACC_BWP_SHFITS 0x4
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)

#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)

#define BMI088_ACC_RANGE 0x41
#define BMI088_ACC_RANGE_SHFITS 0x0
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)

#define BMI088_INT1_IO_CTRL 0x53
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)

#define BMI088_INT2_IO_CTRL 0x54
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)

#define BMI088_INT_MAP_DATA 0x58
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6
#define BMI088_ACC_INT2_DRDY_INTERRUPT                                         \
  (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS)
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
#define BMI088_ACC_INT1_DRDY_INTERRUPT                                         \
  (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)

#define BMI088_ACC_SELF_TEST 0x6D
#define BMI088_ACC_SELF_TEST_OFF 0x00
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09

#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00

#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_ENABLE_ACC_OFF 0x00
#define BMI088_ACC_ENABLE_ACC_ON 0x04

#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6

/* Gyroscope registers */
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

#define BMI088_GYRO_X_L 0x02
#define BMI088_GYRO_X_H 0x03
#define BMI088_GYRO_Y_L 0x04
#define BMI088_GYRO_Y_H 0x05
#define BMI088_GYRO_Z_L 0x06
#define BMI088_GYRO_Z_H 0x07

#define BMI088_GYRO_INT_STAT_1 0x0A
#define BMI088_GYRO_DYDR_SHFITS 0x7
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)

#define BMI088_GYRO_BANDWIDTH 0x10
// the first num means Output data rate, the second num means bandwidth
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80
#define BMI088_GYRO_2000_532_HZ 0x00
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02
#define BMI088_GYRO_400_47_HZ 0x03
#define BMI088_GYRO_200_23_HZ 0x04
#define BMI088_GYRO_100_12_HZ 0x05
#define BMI088_GYRO_200_64_HZ 0x06
#define BMI088_GYRO_100_32_HZ 0x07

#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SUSPEND_MODE 0x80
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20

#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

#define BMI088_GYRO_CTRL 0x15
#define BMI088_DRDY_OFF 0x00
#define BMI088_DRDY_ON 0x80

#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)

#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18
#define BMI088_GYRO_DRDY_IO_OFF 0x00
#define BMI088_GYRO_DRDY_IO_INT3 0x01
#define BMI088_GYRO_DRDY_IO_INT4 0x80
#define BMI088_GYRO_DRDY_IO_BOTH                                               \
  (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4)

#define BMI088_GYRO_SELF_TEST 0x3C
#define BMI088_GYRO_RATE_OK_SHFITS 0x4
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_REG_H */
```

## `Inc/bmi088d_sensor.h`

```cpp
/**
 * @file    bmi088d_sensor.h
 * @brief   BMI088 Driver Library - Sensor Driver Interface
 * @author
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_SENSOR_H
#define BMI088D_SENSOR_H

#include "bmi088d_hal.h"
#include "bmi088d_types.h"

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
 * @brief Read sensor data
 * @param data IMU data structure
 */
void bmi088d_sensor_read_data(bmi088d_imu_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_SENSOR_H */
```

## `Inc/bmi088d_types.h`

```cpp
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
```

## `Inc/bmi088d_utils.h`

```cpp
/**
 * @file    bmi088d_utils.h
 * @brief   BMI088 Driver Library - Utility Functions
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#ifndef BMI088D_UTILS_H
#define BMI088D_UTILS_H

#include "bmi088d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mathematical utility functions */

/**
 * @brief Fast inverse square root approximation
 * @param[in] x Input value
 * @return 1/sqrt(x)
 */
float bmi088d_inv_sqrt(float x);

/**
 * @brief Square root approximation
 * @param[in] x Input value
 * @return sqrt(x)
 */
float bmi088d_sqrt(float x);

/**
 * @brief Limit value to absolute range
 * @param[in] num Input value
 * @param[in] limit Absolute limit
 * @return Limited value
 */
float bmi088d_abs_limit(float num, float limit);

/**
 * @brief Get sign of value
 * @param[in] value Input value
 * @return 1.0f if positive, -1.0f if negative
 */
float bmi088d_sign(float value);

/**
 * @brief Apply deadband to value
 * @param[in] value Input value
 * @param[in] min_value Minimum deadband value
 * @param[in] max_value Maximum deadband value
 * @return Value with deadband applied
 */
float bmi088d_deadband(float value, float min_value, float max_value);

/**
 * @brief Constrain value to range
 * @param[in] value Input value
 * @param[in] min_value Minimum value
 * @param[in] max_value Maximum value
 * @return Constrained value
 */
float bmi088d_constrain(float value, float min_value, float max_value);

/**
 * @brief Loop constrain value (for angles)
 * @param[in] input Input value
 * @param[in] min_value Minimum value
 * @param[in] max_value Maximum value
 * @return Loop-constrained value
 */
float bmi088d_loop_constrain(float input, float min_value, float max_value);

/**
 * @brief Format angle to -180 to 180 degrees
 * @param[in] angle Input angle
 * @return Formatted angle
 */
float bmi088d_format_deg(float angle);

/**
 * @brief Format angle to -PI to PI radians
 * @param[in] angle Input angle
 * @return Formatted angle
 */
float bmi088d_format_rad(float angle);

/* Vector operations */

/**
 * @brief Calculate vector magnitude
 * @param[in] vec Input vector
 * @return Vector magnitude
 */
float bmi088d_vec_magnitude(const bmi088d_vec3_t *vec);

/**
 * @brief Normalize vector
 * @param[in,out] vec Vector to normalize
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_normalize(bmi088d_vec3_t *vec);

/**
 * @brief Calculate dot product of two vectors
 * @param[in] vec1 First vector
 * @param[in] vec2 Second vector
 * @return Dot product
 */
float bmi088d_vec_dot(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2);

/**
 * @brief Calculate cross product of two vectors
 * @param[in] vec1 First vector
 * @param[in] vec2 Second vector
 * @param[out] result Cross product
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_cross(const bmi088d_vec3_t *vec1,
                          const bmi088d_vec3_t *vec2, bmi088d_vec3_t *result);

/**
 * @brief Add two vectors
 * @param[in] vec1 First vector
 * @param[in] vec2 Second vector
 * @param[out] result Sum vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_add(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2,
                        bmi088d_vec3_t *result);

/**
 * @brief Subtract two vectors
 * @param[in] vec1 First vector
 * @param[in] vec2 Second vector
 * @param[out] result Difference vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_subtract(const bmi088d_vec3_t *vec1,
                             const bmi088d_vec3_t *vec2,
                             bmi088d_vec3_t *result);

/**
 * @brief Scale vector
 * @param[in] vec Input vector
 * @param[in] scale Scale factor
 * @param[out] result Scaled vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_vec_scale(const bmi088d_vec3_t *vec, float scale,
                          bmi088d_vec3_t *result);

/* Matrix operations (simplified for 3x3) */

/**
 * @brief Multiply 3x3 matrix by vector
 * @param[in] mat 3x3 matrix (row-major)
 * @param[in] vec Input vector
 * @param[out] result Result vector
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_mat_vec_multiply(const float mat[9], const bmi088d_vec3_t *vec,
                                 bmi088d_vec3_t *result);

/**
 * @brief Transpose 3x3 matrix
 * @param[in] mat Input matrix
 * @param[out] result Transposed matrix
 * @return BMI088D_SUCCESS on success
 */
int32_t bmi088d_mat_transpose(const float mat[9], float result[9]);

/* Time utilities */

/* Note: bmi088d_get_delta_t, bmi088d_delay_s, and bmi088d_delay_ms functions
 * have been removed as they were unused in the BMI088D driver. The driver uses
 * hardware abstraction layer delay functions (bmi088d_hal_delay_ms/us) instead
 * and explicit time step parameters for PID control.
 */

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_UTILS_H */
```

## `Src/bmi088d.c`

```cpp
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

```

## `Src/bmi088d_calibration.c`

```cpp
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
  float gyro_max[3], gyro_min[3];
  float g_norm_max, g_norm_min;
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
```

## `Src/bmi088d_ekf.c`

```cpp
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
```

## `Src/bmi088d_fuzzy.c`

```cpp
/**
 * @file    bmi088d_fuzzy.c
 * @brief   BMI088 Driver Library - Fuzzy Logic for PID
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-14
 */

#include "bmi088d_fuzzy.h"
#include <stddef.h> // For NULL

/* Default Fuzzy Rule matrices from source project */
static const float FUZZY_RULE_KP_DEFAULT[7][7] = {
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS},
    {BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS},
    {BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM},
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM},
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PB},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PB, BMI088D_FUZZY_PB}};

static const float FUZZY_RULE_KI_DEFAULT[7][7] = {
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS},
    {BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM},
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PB},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PB, BMI088D_FUZZY_PB},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PB, BMI088D_FUZZY_PB}};

static const float FUZZY_RULE_KD_DEFAULT[7][7] = {
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PB, BMI088D_FUZZY_PB,
     BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS},
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE,
     BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PB},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PB}};

void bmi088d_fuzzy_rule_init(bmi088d_fuzzy_rule_t *rule,
                             const float (*fuzzy_rule_kp)[7],
                             const float (*fuzzy_rule_ki)[7],
                             const float (*fuzzy_rule_kd)[7], float e_step,
                             float ec_step) {
  if (!rule)
    return;

  rule->rule_kp =
      (fuzzy_rule_kp == NULL) ? FUZZY_RULE_KP_DEFAULT : fuzzy_rule_kp;
  rule->rule_ki =
      (fuzzy_rule_ki == NULL) ? FUZZY_RULE_KI_DEFAULT : fuzzy_rule_ki;
  rule->rule_kd =
      (fuzzy_rule_kd == NULL) ? FUZZY_RULE_KD_DEFAULT : fuzzy_rule_kd;

  rule->e_step = (e_step < 1e-5f) ? 1.0f : e_step;
  rule->ec_step = (ec_step < 1e-5f) ? 1.0f : ec_step;

  rule->e = 0.0f;
  rule->ec = 0.0f;
  rule->e_last = 0.0f;
  rule->kp_fuzzy = 0.0f;
  rule->ki_fuzzy = 0.0f;
  rule->kd_fuzzy = 0.0f;
}

void bmi088d_fuzzy_rule_implement(bmi088d_fuzzy_rule_t *rule, float measure,
                                  float ref, float dt) {
  if (!rule || dt < 1e-6f)
    return;

  float e_left_membership, ec_left_membership;
  float e_right_membership, ec_right_membership;
  int e_left_idx, ec_left_idx;
  int e_right_idx, ec_right_idx;

  rule->e = ref - measure;
  rule->ec = (rule->e - rule->e_last) / dt;
  rule->e_last = rule->e;

  // Determine indices for error (e)
  if (rule->e >= 3 * rule->e_step) {
    e_left_idx = 6;
    e_right_idx = 6;
  } else if (rule->e <= -3 * rule->e_step) {
    e_left_idx = 0;
    e_right_idx = 0;
  } else {
    e_left_idx = (rule->e >= 0) ? ((int)(rule->e / rule->e_step) + 3)
                                : ((int)(rule->e / rule->e_step) + 2);
    e_right_idx = (rule->e >= 0) ? (e_left_idx + 1) : (e_left_idx + 1);
  }

  // Determine indices for error change (ec)
  if (rule->ec >= 3 * rule->ec_step) {
    ec_left_idx = 6;
    ec_right_idx = 6;
  } else if (rule->ec <= -3 * rule->ec_step) {
    ec_left_idx = 0;
    ec_right_idx = 0;
  } else {
    ec_left_idx = (rule->ec >= 0) ? ((int)(rule->ec / rule->ec_step) + 3)
                                  : ((int)(rule->ec / rule->ec_step) + 2);
    ec_right_idx = (rule->ec >= 0) ? (ec_left_idx + 1) : (ec_left_idx + 1);
  }

  // Calculate membership degrees for e
  e_left_membership = (rule->e >= 3 * rule->e_step)
                          ? 0.0f
                          : ((rule->e <= -3 * rule->e_step)
                                 ? 1.0f
                                 : (e_right_idx - rule->e / rule->e_step - 3));
  e_right_membership = (rule->e >= 3 * rule->e_step)
                           ? 1.0f
                           : ((rule->e <= -3 * rule->e_step)
                                  ? 0.0f
                                  : (rule->e / rule->e_step - e_left_idx + 3));

  // Calculate membership degrees for ec
  ec_left_membership =
      (rule->ec >= 3 * rule->ec_step)
          ? 0.0f
          : ((rule->ec <= -3 * rule->ec_step)
                 ? 1.0f
                 : (ec_right_idx - rule->ec / rule->ec_step - 3));
  ec_right_membership =
      (rule->ec >= 3 * rule->ec_step)
          ? 1.0f
          : ((rule->ec <= -3 * rule->ec_step)
                 ? 0.0f
                 : (rule->ec / rule->ec_step - ec_left_idx + 3));

  // Defuzzification using weighted average method
  rule->kp_fuzzy = e_left_membership * ec_left_membership *
                       rule->rule_kp[e_left_idx][ec_left_idx] +
                   e_left_membership * ec_right_membership *
                       rule->rule_kp[e_right_idx][ec_left_idx] +
                   e_right_membership * ec_left_membership *
                       rule->rule_kp[e_left_idx][ec_right_idx] +
                   e_right_membership * ec_right_membership *
                       rule->rule_kp[e_right_idx][ec_right_idx];

  rule->ki_fuzzy = e_left_membership * ec_left_membership *
                       rule->rule_ki[e_left_idx][ec_left_idx] +
                   e_left_membership * ec_right_membership *
                       rule->rule_ki[e_right_idx][ec_left_idx] +
                   e_right_membership * ec_left_membership *
                       rule->rule_ki[e_left_idx][ec_right_idx] +
                   e_right_membership * ec_right_membership *
                       rule->rule_ki[e_right_idx][ec_right_idx];

  rule->kd_fuzzy = e_left_membership * ec_left_membership *
                       rule->rule_kd[e_left_idx][ec_left_idx] +
                   e_left_membership * ec_right_membership *
                       rule->rule_kd[e_right_idx][ec_left_idx] +
                   e_right_membership * ec_left_membership *
                       rule->rule_kd[e_left_idx][ec_right_idx] +
                   e_right_membership * ec_right_membership *
                       rule->rule_kd[e_right_idx][ec_right_idx];
}

```

## `Src/bmi088d_hal.c`

```cpp
/**
 * @file    bmi088d_hal.c
 * @brief   BMI088 Driver Library - Hardware Abstraction Layer Implementation
 * @author
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_hal.h"
#include "stm32f4xx_hal.h"

/* Private variables */
static bmi088d_hw_config_t bmi088d_hw_config;

/**
 * @brief Initialize hardware abstraction layer
 * @param config Hardware configuration
 */
void bmi088d_hal_init(const bmi088d_hw_config_t *config) {
  if (config != NULL) {
    bmi088d_hw_config = *config;
  }
}

/**
 * @brief Get timer handle
 * @return Timer handle
 */
TIM_HandleTypeDef *bmi088d_hal_get_timer(void) {
  return bmi088d_hw_config.htim;
}

/**
 * @brief Get timer channel
 * @return Timer channel
 */
uint32_t bmi088d_hal_get_timer_channel(void) {
  return bmi088d_hw_config.tim_channel;
}

/**
 * @brief Delay in milliseconds
 * @param ms Delay in milliseconds
 */
void bmi088d_hal_delay_ms(uint32_t ms) { HAL_Delay(ms); }

/**
 * @brief Delay in microseconds
 * @param us Delay in microseconds
 */
void bmi088d_hal_delay_us(uint32_t us) {
  // For small delays, use DWT if available, otherwise use a simple loop
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000);

  while ((DWT->CYCCNT - start) < ticks) {
    __NOP();
  }
}

/**
 * @brief Set accelerometer chip select low
 */
void bmi088d_hal_accel_cs_low(void) {
  HAL_GPIO_WritePin(bmi088d_hw_config.accel_cs_port,
                    bmi088d_hw_config.accel_cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief Set accelerometer chip select high
 */
void bmi088d_hal_accel_cs_high(void) {
  HAL_GPIO_WritePin(bmi088d_hw_config.accel_cs_port,
                    bmi088d_hw_config.accel_cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Set gyroscope chip select low
 */
void bmi088d_hal_gyro_cs_low(void) {
  HAL_GPIO_WritePin(bmi088d_hw_config.gyro_cs_port,
                    bmi088d_hw_config.gyro_cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief Set gyroscope chip select high
 */
void bmi088d_hal_gyro_cs_high(void) {
  HAL_GPIO_WritePin(bmi088d_hw_config.gyro_cs_port,
                    bmi088d_hw_config.gyro_cs_pin, GPIO_PIN_SET);
}

/**
 * @brief SPI read/write byte
 * @param data Data to write
 * @return Data read
 */
uint8_t bmi088d_hal_spi_read_write(uint8_t data) {
  uint8_t rx_data;
  HAL_SPI_TransmitReceive(bmi088d_hw_config.hspi, &data, &rx_data, 1,
                          HAL_MAX_DELAY);
  return rx_data;
}
```

## `Src/bmi088d_ols.c`

```cpp
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

```

## `Src/bmi088d_pid.c`

```cpp
/**
 * @file    bmi088d_pid.c
 * @brief   BMI088 Driver Library - PID Control Algorithms (Ported from
 * source-project)
 * @author  Extracted from RoboMaster INS Example
 * @version 1.1.0
 * @date    2025-10-14
 */

#include "bmi088d_pid.h"
#include "bmi088d_utils.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h> // For malloc and free
#include <string.h>

/* Internal improvement functions ported from source-project */
static void pid_trapezoid_integral(bmi088d_pid_t *pid, float dt);
static void pid_integral_limit(bmi088d_pid_t *pid);
static void pid_derivative_on_measurement(bmi088d_pid_t *pid, float dt);
static void pid_changing_integration_rate(bmi088d_pid_t *pid);
static void pid_output_filter(bmi088d_pid_t *pid, float dt);
static void pid_derivative_filter(bmi088d_pid_t *pid, float dt);
static void pid_output_limit(bmi088d_pid_t *pid);
static void pid_proportion_limit(bmi088d_pid_t *pid);
static void pid_error_handle(bmi088d_pid_t *pid);

int32_t bmi088d_pid_init(bmi088d_pid_t *pid, float max_output,
                         float integral_limit, float deadband, float kp,
                         float ki, float kd, float coef_a, float coef_b,
                         float output_lpf_rc, float derivative_lpf_rc,
                         uint16_t ols_order, uint8_t improvements) {
  if (!pid) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  memset(pid, 0, sizeof(bmi088d_pid_t));

  pid->deadband = deadband;
  pid->integral_limit = integral_limit;
  pid->max_output = max_output;

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->coef_a = coef_a;
  pid->coef_b = coef_b;

  pid->output_lpf_rc = output_lpf_rc;
  pid->derivative_lpf_rc = derivative_lpf_rc;

  pid->ols_order = ols_order;
  if (ols_order > 0) {
    if (bmi088d_ols_init(&pid->ols, ols_order) != 0) {
      return BMI088D_ERROR_MEMORY;
    }
  }

  pid->improvements = improvements;
  pid->error_handler.error_count = 0;
  pid->error_handler.error_type = BMI088D_PID_ERROR_NONE;

  return BMI088D_SUCCESS;
}

void bmi088d_pid_deinit(bmi088d_pid_t *pid) {
  if (pid && pid->ols_order > 0) {
    bmi088d_ols_deinit(&pid->ols);
  }
}

float bmi088d_pid_calculate(bmi088d_pid_t *pid, float measure, float ref,
                            float dt) {
  if (!pid || dt < 1e-6f) {
    return 0.0f;
  }

  if (pid->improvements & BMI088D_PID_IMPROVE_ERROR_HANDLE) {
    pid_error_handle(pid);
  }

  pid->measure = measure;
  pid->ref = ref;
  pid->error = pid->ref - pid->measure;

  if (pid->user_func1) {
    pid->user_func1(pid);
  }

  if (fabsf(pid->error) > pid->deadband) {
    float kp_term = pid->kp;
    float ki_term = pid->ki;
    float kd_term = pid->kd;

    if (pid->fuzzy_rule) {
      bmi088d_fuzzy_rule_implement(pid->fuzzy_rule, measure, ref, dt);
      kp_term += pid->fuzzy_rule->kp_fuzzy;
      ki_term += pid->fuzzy_rule->ki_fuzzy;
      kd_term += pid->fuzzy_rule->kd_fuzzy;
    }

    pid->p_out = kp_term * pid->error;
    pid->i_term = ki_term * pid->error * dt;

    if (pid->ols_order >= 2) {
      pid->d_out = kd_term * bmi088d_ols_derivative(&pid->ols, dt, pid->error);
    } else {
      pid->d_out = kd_term * (pid->error - pid->last_error) / dt;
    }

    if (pid->user_func2) {
      pid->user_func2(pid);
    }

    if (pid->improvements & BMI088D_PID_IMPROVE_TRAPEZOID_INTEGRAL) {
      pid_trapezoid_integral(pid, dt);
    }
    if (pid->improvements & BMI088D_PID_IMPROVE_CHANGING_INTEGRATION_RATE) {
      pid_changing_integration_rate(pid);
    }
    if (pid->improvements & BMI088D_PID_IMPROVE_DERIVATIVE_ON_MEASUREMENT) {
      pid_derivative_on_measurement(pid, dt);
    }
    if (pid->improvements & BMI088D_PID_IMPROVE_DERIVATIVE_FILTER) {
      pid_derivative_filter(pid, dt);
    }
    if (pid->improvements & BMI088D_PID_IMPROVE_INTEGRAL_LIMIT) {
      pid_integral_limit(pid);
    }

    pid->i_out += pid->i_term;
    pid->output = pid->p_out + pid->i_out + pid->d_out;

    if (pid->improvements & BMI088D_PID_IMPROVE_OUTPUT_FILTER) {
      pid_output_filter(pid, dt);
    }

    pid_output_limit(pid);
    pid_proportion_limit(pid);
  }

  pid->last_measure = pid->measure;
  pid->last_output = pid->output;
  pid->last_d_out = pid->d_out;
  pid->last_error = pid->error;
  pid->last_iterm = pid->i_term;

  return pid->output;
}

static void pid_trapezoid_integral(bmi088d_pid_t *pid, float dt) {
  float ki_term = pid->ki;
  if (pid->fuzzy_rule) {
    ki_term += pid->fuzzy_rule->ki_fuzzy;
  }
  pid->i_term = ki_term * ((pid->error + pid->last_error) / 2.0f) * dt;
}

static void pid_changing_integration_rate(bmi088d_pid_t *pid) {
  if (pid->error * pid->i_out > 0) {
    if (fabsf(pid->error) <= pid->coef_b) {
      return;
    }
    if (fabsf(pid->error) <= (pid->coef_a + pid->coef_b)) {
      pid->i_term *=
          (pid->coef_a - fabsf(pid->error) + pid->coef_b) / pid->coef_a;
    } else {
      pid->i_term = 0;
    }
  }
}

static void pid_integral_limit(bmi088d_pid_t *pid) {
  float temp_i_out = pid->i_out + pid->i_term;
  float temp_output = pid->p_out + temp_i_out + pid->d_out;

  if (fabsf(temp_output) > pid->max_output) {
    if (pid->error * pid->i_out > 0) {
      pid->i_term = 0;
    }
  }

  if (temp_i_out > pid->integral_limit) {
    pid->i_term = 0;
    pid->i_out = pid->integral_limit;
  }
  if (temp_i_out < -pid->integral_limit) {
    pid->i_term = 0;
    pid->i_out = -pid->integral_limit;
  }
}

static void pid_derivative_on_measurement(bmi088d_pid_t *pid, float dt) {
  float kd_term = pid->kd;
  if (pid->fuzzy_rule) {
    kd_term += pid->fuzzy_rule->kd_fuzzy;
  }
  if (pid->ols_order >= 2) {
    pid->d_out = kd_term * bmi088d_ols_derivative(&pid->ols, dt, -pid->measure);
  } else {
    pid->d_out = kd_term * (pid->last_measure - pid->measure) / dt;
  }
}

static void pid_derivative_filter(bmi088d_pid_t *pid, float dt) {
  if (pid->derivative_lpf_rc > 0.0f) {
    pid->d_out = pid->d_out * dt / (pid->derivative_lpf_rc + dt) +
                 pid->last_d_out * pid->derivative_lpf_rc /
                     (pid->derivative_lpf_rc + dt);
  }
}

static void pid_output_filter(bmi088d_pid_t *pid, float dt) {
  if (pid->output_lpf_rc > 0.0f) {
    pid->output =
        pid->output * dt / (pid->output_lpf_rc + dt) +
        pid->last_output * pid->output_lpf_rc / (pid->output_lpf_rc + dt);
  }
}

static void pid_output_limit(bmi088d_pid_t *pid) {
  pid->output = bmi088d_abs_limit(pid->output, pid->max_output);
}

static void pid_proportion_limit(bmi088d_pid_t *pid) {
  pid->p_out = bmi088d_abs_limit(pid->p_out, pid->max_output);
}

static void pid_error_handle(bmi088d_pid_t *pid) {
  if (fabsf(pid->output) < pid->max_output * 0.001f ||
      fabsf(pid->ref) < 1e-5f) {
    return;
  }

  if ((fabsf(pid->ref - pid->measure) / fabsf(pid->ref)) > 0.95f) {
    pid->error_handler.error_count++;
  } else {
    pid->error_handler.error_count = 0;
  }

  if (pid->error_handler.error_count > 500) {
    pid->error_handler.error_type = BMI088D_PID_ERROR_MOTOR_BLOCKED;
  }
}

```

## `Src/bmi088d_quaternion.c`

```cpp
/**
 * @file    bmi088d_quaternion.c
 * @brief   BMI088 Driver Library - Quaternion Operations
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_quaternion.h"
#include "bmi088d_utils.h"
#include <math.h>

int32_t bmi088d_quat_identity(bmi088d_quat_t *quat) {
  if (!quat) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  quat->w = 1.0f;
  quat->x = 0.0f;
  quat->y = 0.0f;
  quat->z = 0.0f;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_normalize(bmi088d_quat_t *quat) {
  if (!quat) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  float norm = bmi088d_quat_magnitude(quat);

  if (norm < 1e-12f) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  float inv_norm = 1.0f / norm;
  quat->w *= inv_norm;
  quat->x *= inv_norm;
  quat->y *= inv_norm;
  quat->z *= inv_norm;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_multiply(const bmi088d_quat_t *q1,
                              const bmi088d_quat_t *q2,
                              bmi088d_quat_t *result) {
  if (!q1 || !q2 || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
  result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_conjugate(const bmi088d_quat_t *quat,
                               bmi088d_quat_t *conjugate) {
  if (!quat || !conjugate) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  conjugate->w = quat->w;
  conjugate->x = -quat->x;
  conjugate->y = -quat->y;
  conjugate->z = -quat->z;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_to_euler(const bmi088d_quat_t *quat,
                              bmi088d_euler_t *euler) {
  if (!quat || !euler) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  float w = quat->w;
  float x = quat->x;
  float y = quat->y;
  float z = quat->z;

  /* Roll (x-axis rotation) */
  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  euler->roll = atan2f(sinr_cosp, cosr_cosp) * 57.295779513f;

  /* Pitch (y-axis rotation) */
  float sinp = 2.0f * (w * y - z * x);
  if (fabsf(sinp) >= 1.0f) {
    euler->pitch =
        copysignf(1.57079632679f, sinp) * 57.295779513f; /* 90 degrees */
  } else {
    euler->pitch = asinf(sinp) * 57.295779513f;
  }

  /* Yaw (z-axis rotation) */
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  euler->yaw = atan2f(siny_cosp, cosy_cosp) * 57.295779513f;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_euler_to_quat(const bmi088d_euler_t *euler,
                              bmi088d_quat_t *quat) {
  if (!euler || !quat) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Convert degrees to radians */
  float roll_rad = euler->roll * 0.01745329252f;
  float pitch_rad = euler->pitch * 0.01745329252f;
  float yaw_rad = euler->yaw * 0.01745329252f;

  /* Calculate half angles */
  float cy = cosf(yaw_rad * 0.5f);
  float sy = sinf(yaw_rad * 0.5f);
  float cp = cosf(pitch_rad * 0.5f);
  float sp = sinf(pitch_rad * 0.5f);
  float cr = cosf(roll_rad * 0.5f);
  float sr = sinf(roll_rad * 0.5f);

  quat->w = cr * cp * cy + sr * sp * sy;
  quat->x = sr * cp * cy - cr * sp * sy;
  quat->y = cr * sp * cy + sr * cp * sy;
  quat->z = cr * cp * sy - sr * sp * cy;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_rotate_vector(const bmi088d_quat_t *quat,
                                   const bmi088d_vec3_t *vec,
                                   bmi088d_vec3_t *result) {
  if (!quat || !vec || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Convert vector to pure quaternion */
  bmi088d_quat_t vec_quat = {0.0f, vec->x, vec->y, vec->z};
  bmi088d_quat_t quat_conj;
  bmi088d_quat_t temp;

  /* Calculate conjugate */
  bmi088d_quat_conjugate(quat, &quat_conj);

  /* Rotate vector: result = quat * vec_quat * quat_conj */
  bmi088d_quat_multiply(quat, &vec_quat, &temp);
  bmi088d_quat_multiply(&temp, &quat_conj, &vec_quat);

  /* Extract rotated vector */
  result->x = vec_quat.x;
  result->y = vec_quat.y;
  result->z = vec_quat.z;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_from_axis_angle(const bmi088d_vec3_t *axis, float angle,
                                     bmi088d_quat_t *quat) {
  if (!axis || !quat) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Normalize axis */
  bmi088d_vec3_t axis_norm = *axis;
  bmi088d_vec_normalize(&axis_norm);

  /* Calculate half angle */
  float half_angle = angle * 0.5f;
  float sin_half = sinf(half_angle);

  quat->w = cosf(half_angle);
  quat->x = axis_norm.x * sin_half;
  quat->y = axis_norm.y * sin_half;
  quat->z = axis_norm.z * sin_half;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_from_gyro(const bmi088d_vec3_t *gyro, float dt,
                               bmi088d_quat_t *quat) {
  if (!gyro || !quat) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Calculate rotation vector magnitude */
  float gyro_norm = bmi088d_vec_magnitude(gyro);

  if (gyro_norm < 1e-12f) {
    /* No rotation - return identity */
    return BMI088D_SUCCESS;
  }

  /* Calculate rotation angle */
  float angle = gyro_norm * dt;

  /* Create normalized rotation axis */
  bmi088d_vec3_t axis;
  float inv_norm = 1.0f / gyro_norm;
  axis.x = gyro->x * inv_norm;
  axis.y = gyro->y * inv_norm;
  axis.z = gyro->z * inv_norm;

  /* Create rotation quaternion */
  bmi088d_quat_t rotation_quat;
  bmi088d_quat_from_axis_angle(&axis, angle, &rotation_quat);

  /* Apply rotation to current quaternion */
  bmi088d_quat_t result;
  bmi088d_quat_multiply(quat, &rotation_quat, &result);

  /* Copy result back */
  *quat = result;

  /* Normalize quaternion */
  bmi088d_quat_normalize(quat);

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_difference(const bmi088d_quat_t *q1,
                                const bmi088d_quat_t *q2,
                                bmi088d_quat_t *diff) {
  if (!q1 || !q2 || !diff) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Calculate difference: diff = q2 * q1_conj */
  bmi088d_quat_t q1_conj;
  bmi088d_quat_conjugate(q1, &q1_conj);
  bmi088d_quat_multiply(q2, &q1_conj, diff);

  return BMI088D_SUCCESS;
}

int32_t bmi088d_quat_slerp(const bmi088d_quat_t *q1, const bmi088d_quat_t *q2,
                           float t, bmi088d_quat_t *result) {
  if (!q1 || !q2 || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Clamp t to [0, 1] */
  t = bmi088d_constrain(t, 0.0f, 1.0f);

  /* Calculate dot product */
  float dot = q1->w * q2->w + q1->x * q2->x + q1->y * q2->y + q1->z * q2->z;

  /* If the dot product is negative, the quaternions have opposite handedness */
  float scale1, scale2;
  if (dot < 0.0f) {
    dot = -dot;
    scale2 = -1.0f;
  } else {
    scale2 = 1.0f;
  }

  /* Check for very close quaternions */
  if (dot > 0.9995f) {
    /* Linear interpolation */
    scale1 = 1.0f - t;
    scale2 *= t;
  } else {
    /* Spherical interpolation */
    float theta_0 = acosf(dot);
    float theta = theta_0 * t;
    float sin_theta = sinf(theta);
    float sin_theta_0 = sinf(theta_0);

    scale1 = cosf(theta) - dot * sin_theta / sin_theta_0;
    scale2 = sin_theta / sin_theta_0;
  }

  /* Calculate interpolated quaternion */
  result->w = scale1 * q1->w + scale2 * q2->w;
  result->x = scale1 * q1->x + scale2 * q2->x;
  result->y = scale1 * q1->y + scale2 * q2->y;
  result->z = scale1 * q1->z + scale2 * q2->z;

  /* Normalize result */
  bmi088d_quat_normalize(result);

  return BMI088D_SUCCESS;
}

float bmi088d_quat_magnitude(const bmi088d_quat_t *quat) {
  if (!quat) {
    return 0.0f;
  }

  return bmi088d_sqrt(quat->w * quat->w + quat->x * quat->x +
                      quat->y * quat->y + quat->z * quat->z);
}

uint8_t bmi088d_quat_is_valid(const bmi088d_quat_t *quat) {
  if (!quat) {
    return 0;
  }

  /* Check for finite values */
  if (!isfinite(quat->w) || !isfinite(quat->x) || !isfinite(quat->y) ||
      !isfinite(quat->z)) {
    return 0;
  }

  /* Check magnitude (should be close to 1 for normalized quaternion) */
  float mag = bmi088d_quat_magnitude(quat);
  if (fabsf(mag - 1.0f) > 0.01f) {
    return 0;
  }

  return 1;
}
```

## `Src/bmi088d_sensor.c`

```cpp
/**
 * @file    bmi088d_sensor.c
 * @brief   BMI088 Driver Library - Sensor Driver Implementation
 * @author
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_sensor.h"
#include "bmi088d_hal.h"
#include "bmi088d_reg.h"

/* Private function prototypes */
static void bmi088d_accel_write_single_reg(uint8_t reg, uint8_t data);
static void bmi088d_accel_read_single_reg(uint8_t reg, uint8_t *return_data);
static void bmi088d_accel_read_multi_reg(uint8_t reg, uint8_t *buf,
                                         uint8_t len);
static void bmi088d_gyro_write_single_reg(uint8_t reg, uint8_t data);
static void bmi088d_gyro_read_single_reg(uint8_t reg, uint8_t *return_data);
static void bmi088d_gyro_read_multi_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/* Accelerometer write register data */
static const uint8_t write_bmi088_accel_reg_data[][2] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE},
    {BMI088_ACC_CONF,
     BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP |
                              BMI088_ACC_INT1_GPIO_LOW},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT}};

#define BMI088_WRITE_ACCEL_REG_NUM                                             \
  (sizeof(write_bmi088_accel_reg_data) / sizeof(write_bmi088_accel_reg_data[0]))

/* Gyroscope write register data */
static const uint8_t write_bmi088_gyro_reg_data[][2] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000},
    {BMI088_GYRO_BANDWIDTH,
     BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON},
    {BMI088_GYRO_INT3_INT4_IO_CONF,
     BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3}};

#define BMI088_WRITE_GYRO_REG_NUM                                              \
  (sizeof(write_bmi088_gyro_reg_data) / sizeof(write_bmi088_gyro_reg_data[0]))

static bmi088d_sensor_error_t bmi088d_accel_init(void);
static bmi088d_sensor_error_t bmi088d_gyro_init(void);

/**
 * @brief Initialize sensor driver
 * @param config Hardware configuration
 * @return Error code
 */
bmi088d_sensor_error_t bmi088d_sensor_init(const bmi088d_hw_config_t *config) {
  bmi088d_sensor_error_t err = BMI088D_SENSOR_OK;

  // Initialize hardware abstraction layer
  bmi088d_hal_init(config);

  // Initialize accelerometer
  err = bmi088d_accel_init();
  if (err != BMI088D_SENSOR_OK) {
    return err;
  }

  // Initialize gyroscope
  err = bmi088d_gyro_init();
  if (err != BMI088D_SENSOR_OK) {
    return err;
  }

  return BMI088D_SENSOR_OK;
}

/**
 * @brief Initialize accelerometer
 * @return Error code
 */
static bmi088d_sensor_error_t bmi088d_accel_init(void) {
  uint8_t res = 0;

  // Check communication
  bmi088d_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  bmi088d_hal_delay_ms(1);
  bmi088d_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  bmi088d_hal_delay_ms(1);

  // Check the "who am I"
  if (res != BMI088_ACC_CHIP_ID_VALUE)
    return BMI088D_SENSOR_ERROR_ID;

  // Accelerometer software reset
  bmi088d_accel_write_single_reg(BMI088_ACC_SOFTRESET,
                                 BMI088_ACC_SOFTRESET_VALUE);
  bmi088d_hal_delay_ms(80); // BMI088_LONG_DELAY_TIME

  // Check communication is normal after reset
  bmi088d_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  bmi088d_hal_delay_ms(1);
  bmi088d_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  bmi088d_hal_delay_ms(1);

  // Check the "who am I" again
  if (res != BMI088_ACC_CHIP_ID_VALUE)
    return BMI088D_SENSOR_ERROR_ID;

  // Set accelerometer sensor config
  for (uint8_t i = 0; i < BMI088_WRITE_ACCEL_REG_NUM; i++) {
    bmi088d_accel_write_single_reg(write_bmi088_accel_reg_data[i][0],
                                   write_bmi088_accel_reg_data[i][1]);
    bmi088d_hal_delay_ms(1);
    bmi088d_accel_read_single_reg(write_bmi088_accel_reg_data[i][0], &res);
    bmi088d_hal_delay_ms(1);

    if (res != write_bmi088_accel_reg_data[i][1]) {
      return BMI088D_SENSOR_ERROR_CONFIG;
    }
  }

  return BMI088D_SENSOR_OK;
}

/**
 * @brief Initialize gyroscope
 * @return Error code
 */
static bmi088d_sensor_error_t bmi088d_gyro_init(void) {
  uint8_t res = 0;

  // Check communication
  bmi088d_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  bmi088d_hal_delay_ms(1);
  bmi088d_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  bmi088d_hal_delay_ms(1);

  // Check the "who am I"
  if (res != BMI088_GYRO_CHIP_ID_VALUE)
    return BMI088D_SENSOR_ERROR_ID;

  // Gyroscope software reset
  bmi088d_gyro_write_single_reg(BMI088_GYRO_SOFTRESET,
                                BMI088_GYRO_SOFTRESET_VALUE);
  bmi088d_hal_delay_ms(80); // BMI088_LONG_DELAY_TIME

  // Check communication is normal after reset
  bmi088d_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  bmi088d_hal_delay_ms(1);
  bmi088d_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  bmi088d_hal_delay_ms(1);

  // Check the "who am I" again
  if (res != BMI088_GYRO_CHIP_ID_VALUE)
    return BMI088D_SENSOR_ERROR_ID;

  // Set gyroscope sensor config
  for (uint8_t i = 0; i < BMI088_WRITE_GYRO_REG_NUM; i++) {
    bmi088d_gyro_write_single_reg(write_bmi088_gyro_reg_data[i][0],
                                  write_bmi088_gyro_reg_data[i][1]);
    bmi088d_hal_delay_ms(1);
    bmi088d_gyro_read_single_reg(write_bmi088_gyro_reg_data[i][0], &res);
    bmi088d_hal_delay_ms(1);

    if (res != write_bmi088_gyro_reg_data[i][1]) {
      return BMI088D_SENSOR_ERROR_CONFIG;
    }
  }

  return BMI088D_SENSOR_OK;
}

/**
 * @brief Read IMU data
 * @param data IMU data structure
 */
void bmi088d_sensor_read_data(bmi088d_imu_data_t *data) {
  static uint8_t buf[8] = {0};
  static int16_t raw_temp;

  // Read accelerometer data
  bmi088d_accel_read_multi_reg(BMI088_ACCEL_XOUT_L, buf, 6);

  raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
  data->accel[0] = raw_temp * BMI088_ACCEL_6G_SEN;

  raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
  data->accel[1] = raw_temp * BMI088_ACCEL_6G_SEN;

  raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
  data->accel[2] = raw_temp * BMI088_ACCEL_6G_SEN;

  // Read gyroscope data
  bmi088d_gyro_read_multi_reg(BMI088_GYRO_X_L, buf, 6);

  raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
  data->gyro[0] = raw_temp * BMI088_GYRO_2000_SEN;

  raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
  data->gyro[1] = raw_temp * BMI088_GYRO_2000_SEN;

  raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
  data->gyro[2] = raw_temp * BMI088_GYRO_2000_SEN;

  // Read temperature data
  bmi088d_accel_read_multi_reg(BMI088_TEMP_M, buf, 2);

  raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

  if (raw_temp > 1023) {
    raw_temp -= 2048;
  }

  data->temperature = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/* Private functions */

/**
 * @brief Write single register for accelerometer
 * @param reg Register address
 * @param data Data to write
 */
static void bmi088d_accel_write_single_reg(uint8_t reg, uint8_t data) {
  bmi088d_hal_accel_cs_low();
  bmi088d_hal_spi_read_write(reg);
  bmi088d_hal_spi_read_write(data);
  bmi088d_hal_accel_cs_high();
}

/**
 * @brief Read single register for accelerometer
 * @param reg Register address
 * @param return_data Pointer to store read data
 */
static void bmi088d_accel_read_single_reg(uint8_t reg, uint8_t *return_data) {
  bmi088d_hal_accel_cs_low();
  bmi088d_hal_spi_read_write(reg | 0x80);
  *return_data = bmi088d_hal_spi_read_write(0x55);
  bmi088d_hal_accel_cs_high();
}

/**
 * @brief Read multiple registers for accelerometer
 * @param reg Register address
 * @param buf Buffer to store read data
 * @param len Number of bytes to read
 */
static void bmi088d_accel_read_multi_reg(uint8_t reg, uint8_t *buf,
                                         uint8_t len) {
  bmi088d_hal_accel_cs_low();
  bmi088d_hal_spi_read_write(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = bmi088d_hal_spi_read_write(0x55);
  }
  bmi088d_hal_accel_cs_high();
}

/**
 * @brief Write single register for gyroscope
 * @param reg Register address
 * @param data Data to write
 */
static void bmi088d_gyro_write_single_reg(uint8_t reg, uint8_t data) {
  bmi088d_hal_gyro_cs_low();
  bmi088d_hal_spi_read_write(reg);
  bmi088d_hal_spi_read_write(data);
  bmi088d_hal_gyro_cs_high();
}

/**
 * @brief Read single register for gyroscope
 * @param reg Register address
 * @param return_data Pointer to store read data
 */
static void bmi088d_gyro_read_single_reg(uint8_t reg, uint8_t *return_data) {
  bmi088d_hal_gyro_cs_low();
  bmi088d_hal_spi_read_write(reg | 0x80);
  *return_data = bmi088d_hal_spi_read_write(0x55);
  bmi088d_hal_gyro_cs_high();
}

/**
 * @brief Read multiple registers for gyroscope
 * @param reg Register address
 * @param buf Buffer to store read data
 * @param len Number of bytes to read
 */
static void bmi088d_gyro_read_multi_reg(uint8_t reg, uint8_t *buf,
                                        uint8_t len) {
  bmi088d_hal_gyro_cs_low();
  bmi088d_hal_spi_read_write(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = bmi088d_hal_spi_read_write(0x55);
  }
  bmi088d_hal_gyro_cs_high();
}

```

## `Src/bmi088d_utils.c`

```cpp
/**
 * @file    bmi088d_utils.c
 * @brief   BMI088 Driver Library - Utility Functions
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_utils.h"
#include <math.h>

/* Mathematical utility functions */

float bmi088d_sqrt(float x) {
  /* Simple square root using standard library */
  if (x <= 0.0f) {
    return 0.0f;
  }
  return sqrtf(x);
}

float bmi088d_abs_limit(float num, float limit) {
  /* Limit absolute value */
  if (num > limit) {
    return limit;
  }
  if (num < -limit) {
    return -limit;
  }
  return num;
}

float bmi088d_sign(float value) {
  /* Get sign of value */
  if (value >= 0.0f) {
    return 1.0f;
  } else {
    return -1.0f;
  }
}

float bmi088d_deadband(float value, float min_value, float max_value) {
  /* Apply deadband to value */
  if (value > max_value) {
    return value;
  }
  if (value < min_value) {
    return value;
  }
  return 0.0f;
}

float bmi088d_constrain(float value, float min_value, float max_value) {
  /* Constrain value to range */
  if (value > max_value) {
    return max_value;
  }
  if (value < min_value) {
    return min_value;
  }
  return value;
}

float bmi088d_loop_constrain(float input, float min_value, float max_value) {
  /* Loop constrain value (for angles) */
  float range = max_value - min_value;

  while (input > max_value) {
    input -= range;
  }
  while (input < min_value) {
    input += range;
  }

  return input;
}

float bmi088d_format_deg(float angle) {
  /* Format angle to -180 to 180 degrees */
  return bmi088d_loop_constrain(angle, -180.0f, 180.0f);
}

float bmi088d_format_rad(float angle) {
  /* Format angle to -PI to PI radians */
  return bmi088d_loop_constrain(angle, -3.14159265359f, 3.14159265359f);
}

/* Vector operations */

float bmi088d_vec_magnitude(const bmi088d_vec3_t *vec) {
  if (!vec) {
    return 0.0f;
  }

  return bmi088d_sqrt(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
}

int32_t bmi088d_vec_normalize(bmi088d_vec3_t *vec) {
  if (!vec) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  float mag = bmi088d_vec_magnitude(vec);

  if (mag < 1e-12f) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  float inv_mag = 1.0f / mag;
  vec->x *= inv_mag;
  vec->y *= inv_mag;
  vec->z *= inv_mag;

  return BMI088D_SUCCESS;
}

float bmi088d_vec_dot(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2) {
  if (!vec1 || !vec2) {
    return 0.0f;
  }

  return vec1->x * vec2->x + vec1->y * vec2->y + vec1->z * vec2->z;
}

int32_t bmi088d_vec_cross(const bmi088d_vec3_t *vec1,
                          const bmi088d_vec3_t *vec2, bmi088d_vec3_t *result) {
  if (!vec1 || !vec2 || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  result->x = vec1->y * vec2->z - vec1->z * vec2->y;
  result->y = vec1->z * vec2->x - vec1->x * vec2->z;
  result->z = vec1->x * vec2->y - vec1->y * vec2->x;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_vec_add(const bmi088d_vec3_t *vec1, const bmi088d_vec3_t *vec2,
                        bmi088d_vec3_t *result) {
  if (!vec1 || !vec2 || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  result->x = vec1->x + vec2->x;
  result->y = vec1->y + vec2->y;
  result->z = vec1->z + vec2->z;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_vec_subtract(const bmi088d_vec3_t *vec1,
                             const bmi088d_vec3_t *vec2,
                             bmi088d_vec3_t *result) {
  if (!vec1 || !vec2 || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  result->x = vec1->x - vec2->x;
  result->y = vec1->y - vec2->y;
  result->z = vec1->z - vec2->z;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_vec_scale(const bmi088d_vec3_t *vec, float scale,
                          bmi088d_vec3_t *result) {
  if (!vec || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  result->x = vec->x * scale;
  result->y = vec->y * scale;
  result->z = vec->z * scale;

  return BMI088D_SUCCESS;
}

/* Matrix operations (simplified for 3x3) */

int32_t bmi088d_mat_vec_multiply(const float mat[9], const bmi088d_vec3_t *vec,
                                 bmi088d_vec3_t *result) {
  if (!mat || !vec || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Matrix is row-major: mat[0-2] = first row, mat[3-5] = second row, mat[6-8]
   * = third row */
  result->x = mat[0] * vec->x + mat[1] * vec->y + mat[2] * vec->z;
  result->y = mat[3] * vec->x + mat[4] * vec->y + mat[5] * vec->z;
  result->z = mat[6] * vec->x + mat[7] * vec->y + mat[8] * vec->z;

  return BMI088D_SUCCESS;
}

int32_t bmi088d_mat_transpose(const float mat[9], float result[9]) {
  if (!mat || !result) {
    return BMI088D_ERROR_INVALID_PARAM;
  }

  /* Transpose 3x3 matrix */
  result[0] = mat[0];
  result[1] = mat[3];
  result[2] = mat[6];

  result[3] = mat[1];
  result[4] = mat[4];
  result[5] = mat[7];

  result[6] = mat[2];
  result[7] = mat[5];
  result[8] = mat[8];

  return BMI088D_SUCCESS;
}

/* Time utilities */

/* Note: bmi088d_get_delta_t, bmi088d_delay_s, and bmi088d_delay_ms functions
 * have been removed as they were unused in the BMI088D driver. The driver uses
 * hardware abstraction layer delay functions (bmi088d_hal_delay_ms/us) instead
 * and explicit time step parameters for PID control.
 */

/* Additional utility functions */

float bmi088d_lerp(float a, float b, float t) {
  /* Linear interpolation */
  return a + (b - a) * t;
}

float bmi088d_map(float x, float in_min, float in_max, float out_min,
                  float out_max) {
  /* Map value from one range to another */
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float bmi088d_deg_to_rad(float degrees) {
  /* Convert degrees to radians */
  return degrees * 0.01745329252f;
}

float bmi088d_rad_to_deg(float radians) {
  /* Convert radians to degrees */
  return radians * 57.295779513f;
}

int32_t bmi088d_is_finite(float value) {
  /* Check if value is finite (not NaN or infinity) */
  return isfinite(value);
}

int32_t bmi088d_is_nan(float value) {
  /* Check if value is NaN */
  return isnan(value);
}

int32_t bmi088d_is_inf(float value) {
  /* Check if value is infinity */
  return isinf(value);
}
```

## `example.c`

```cpp
/**
 * @file    example.c
 * @brief   Example usage of BMI088 Driver Library
 * @author  
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d.h"
#include "main.h" // For SPI handle and pin definitions

/* Private variables */
static SPI_HandleTypeDef *bmi088_spi_handle = NULL;
static uint32_t last_time = 0;
static uint32_t dwt_counter = 0; // For precise time measurement

/* Pin definitions for BMI088 */
/* 
 * 6 轴 IMU (BMI088)       |             |       |
 *                         | TIM10_CH1   | PF6   |
 *                         | INT1_Accel  | PC4   |
 *                         | INT1_Gyro   | PC5   |
 *                         | CS1_Accel   | PA4   |
 *                         | CS1_Gyro    | PB0   |
 *                         | SPI1_CLK    | PB3   |
 *                         | SPI1_MOSI   | PA7   |
 *                         | SPI1_MISO   | PB4   |
 */

/**
 * @brief System initialization
 */
void SystemInit_Custom(void)
{
    // Initialize HAL
    HAL_Init();
    
    // System clock configuration would go here
    // SystemClock_Config();
    
    // Initialize peripherals
    // MX_GPIO_Init();
    // MX_SPI1_Init();
    // MX_TIM10_Init();
    
    // Initialize DWT counter for precise timing (if available)
    // CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // DWT->CYCCNT = 0;
    // DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Get precise time delta using DWT counter
 * @return Time delta in seconds
 */
float get_time_delta(void)
{
    // In a real implementation, this would use DWT counter or system timer
    // For this example, we'll return a fixed time step
    return 0.001f; // 1ms = 0.001 seconds
}

/**
 * @brief Initialize BMI088 sensor
 * @return 0 on success, error code on failure
 */
int32_t bmi088_sensor_init(void)
{
    int32_t result;
    
    // Configure hardware
    bmi088d_hw_config_t hw_config;
    hw_config.hspi = bmi088_spi_handle;
    hw_config.accel_cs_port = GPIOA;
    hw_config.accel_cs_pin = GPIO_PIN_4;
    hw_config.gyro_cs_port = GPIOB;
    hw_config.gyro_cs_pin = GPIO_PIN_0;
    hw_config.htim = &htim10;  // Timer for PWM output
    hw_config.tim_channel = TIM_CHANNEL_1;  // Timer channel
    
    // Initialize the BMI088 driver library with hardware interface
    result = bmi088d_init(&hw_config);
    if (result != BMI088D_SUCCESS) {
        return result;
    }
    
    // Perform initial calibration
    // Note: Calibration is now handled internally during update
    
    return BMI088D_SUCCESS;
}

/**
 * @brief Main application function
 */
int main(void)
{
    bmi088d_imu_data_t imu_data;
    bmi088d_euler_t attitude;
    bmi088d_quat_t orientation;
    float dt;
    
    // System initialization
    SystemInit_Custom();
    
    // Get SPI handle (assuming it's hspi1 from your project)
    extern SPI_HandleTypeDef hspi1;
    bmi088_spi_handle = &hspi1;
    
    // Initialize GPIO for BMI088
    bmi088d_gpio_init();
    
    // Initialize IMU heater
    imu_heater_init();
    
    // Initialize BMI088 sensor
    if (bmi088_sensor_init() != BMI088D_SUCCESS) {
        // Handle initialization error
        while (1) {
            // Error loop
        }
    }
    
    // Main loop
    while (1) {
        // Calculate time delta for this iteration
        dt = get_time_delta();
        
        // Read sensor data and update attitude estimation
        if (bmi088d_update(&imu_data, dt) == BMI088D_SUCCESS) {
            // Get orientation as Euler angles
            bmi088d_ekf_get_euler(&attitude);
            
            // Get orientation as quaternion
            bmi088d_ekf_get_quaternion(&orientation);
            
            // Temperature control using simplified API (uses default 40°C target)
            bmi088d_temp_control(imu_data.temperature, dt);
            
            // Use the data as needed
            // For example, send to UART for debugging:
            /*
            printf("Accel: X=%f, Y=%f, Z=%f\r\n", imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
            printf("Gyro: X=%f, Y=%f, Z=%f\r\n", imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
            printf("Temperature: %f°C\r\n", imu_data.temperature);
            printf("Attitude: Roll=%f, Pitch=%f, Yaw=%f\r\n", attitude.roll, attitude.pitch, attitude.yaw);
            */
        }
        
        // Delay for next iteration (adjust as needed)
        HAL_Delay(1); // 1ms delay for 1kHz update rate
    }
}

/**
 * @brief Configure GPIO for BMI088
 */
void bmi088d_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // Configure CS1_ACCEL pin (PA4)
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Configure CS1_GYRO pin (PB0)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Set CS pins high (deselect sensors)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
 * @brief Configure TIM10 for IMU heating
 */
void imu_heater_init(void)
{
    // Start TIM10
    extern TIM_HandleTypeDef htim10;
    HAL_TIM_Base_Start(&htim10);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    
    // Set initial PWM duty cycle to 0%
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
}
```

