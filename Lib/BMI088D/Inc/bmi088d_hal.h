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