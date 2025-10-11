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
void bmi088d_hal_init(const bmi088d_hw_config_t *config)
{
    if (config != NULL) {
        bmi088d_hw_config = *config;
    }
}

/**
 * @brief Delay in milliseconds
 * @param ms Delay in milliseconds
 */
void bmi088d_hal_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief Delay in microseconds
 * @param us Delay in microseconds
 */
void bmi088d_hal_delay_us(uint32_t us)
{
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
void bmi088d_hal_accel_cs_low(void)
{
    HAL_GPIO_WritePin(bmi088d_hw_config.accel_cs_port, bmi088d_hw_config.accel_cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief Set accelerometer chip select high
 */
void bmi088d_hal_accel_cs_high(void)
{
    HAL_GPIO_WritePin(bmi088d_hw_config.accel_cs_port, bmi088d_hw_config.accel_cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Set gyroscope chip select low
 */
void bmi088d_hal_gyro_cs_low(void)
{
    HAL_GPIO_WritePin(bmi088d_hw_config.gyro_cs_port, bmi088d_hw_config.gyro_cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief Set gyroscope chip select high
 */
void bmi088d_hal_gyro_cs_high(void)
{
    HAL_GPIO_WritePin(bmi088d_hw_config.gyro_cs_port, bmi088d_hw_config.gyro_cs_pin, GPIO_PIN_SET);
}

/**
 * @brief SPI read/write byte
 * @param data Data to write
 * @return Data read
 */
uint8_t bmi088d_hal_spi_read_write(uint8_t data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(bmi088d_hw_config.hspi, &data, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}