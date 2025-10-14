# BMI088D Driver Library

## Overview

The BMI088D driver library provides a complete interface for the Bosch BMI088 IMU sensor, including sensor data reading, attitude estimation, and temperature control functionality.

## Features

- Complete BMI088 sensor initialization and configuration
- IMU data reading (accelerometer, gyroscope, temperature)
- Attitude estimation using Extended Kalman Filter (EKF)
- Temperature control using PID controller (default target: 40°C)
- Hardware abstraction layer for easy porting

## Simplified API

The library now provides a clean, minimal API with only three main functions:

### 1. Initialization
```c
int32_t bmi088d_init(const bmi088d_hw_config_t *config);
```

### 2. Data Update
```c
int32_t bmi088d_update(bmi088d_imu_data_t *imu_data, float dt);
```

### 3. Temperature Control
```c
int32_t bmi088d_temp_control(float current_temp, float dt);
```

## Complete Workflow Example

This example demonstrates the complete workflow for using the BMI088D library:

```c
#include "bmi088d.h"
#include "main.h"

// Hardware handles
static SPI_HandleTypeDef *bmi088_spi_handle = NULL;

/**
 * @brief Initialize BMI088 sensor
 */
int32_t bmi088_sensor_init(void)
{
    // Configure hardware
    bmi088d_hw_config_t hw_config;
    hw_config.hspi = bmi088_spi_handle;
    hw_config.accel_cs_port = GPIOA;
    hw_config.accel_cs_pin = GPIO_PIN_4;
    hw_config.gyro_cs_port = GPIOB;
    hw_config.gyro_cs_pin = GPIO_PIN_0;
    hw_config.htim = &htim10;           // Timer for PWM
    hw_config.tim_channel = TIM_CHANNEL_1; // Timer channel
    
    // Initialize the BMI088 driver
    return bmi088d_init(&hw_config);
}

/**
 * @brief Main application loop
 */
void main_loop(void)
{
    bmi088d_imu_data_t imu_data;
    float dt;
    
    while (1) {
        // Calculate time delta
        dt = get_time_delta(); // Should return time in seconds
        
        // Read sensor data and update attitude estimation
        if (bmi088d_update(&imu_data, dt) == BMI088D_SUCCESS) {
            // Process data as needed
            // ...
            
            // Automatic temperature control (uses default 40°C target)
            bmi088d_temp_control(imu_data.temperature, dt);
        }
        
        // Loop delay
        HAL_Delay(1); // 1ms delay
    }
}
```

## Hardware Configuration

The library uses a hardware abstraction layer for easy porting:

```c
typedef struct {
    SPI_HandleTypeDef *hspi;        // SPI handle
    GPIO_TypeDef *accel_cs_port;    // Accelerometer CS port
    uint16_t accel_cs_pin;          // Accelerometer CS pin
    GPIO_TypeDef *gyro_cs_port;     // Gyroscope CS port
    uint16_t gyro_cs_pin;           // Gyroscope CS pin
    TIM_HandleTypeDef *htim;        // Timer handle for PWM output
    uint32_t tim_channel;           // Timer channel for PWM output
} bmi088d_hw_config_t;
```

## Temperature Control

The temperature control uses the original PID parameters from the source project:
- Target Temperature: 40°C (default from source project)
- Kp: 1000
- Ki: 20
- Kd: 0
- Max Output: 2000
- Integral Limit: 300

## Time Units

All time-related parameters use seconds as the unit:
- Time delta (`dt`) parameter in all functions
- Filter time constants
- Delay functions

## Error Handling

All functions return error codes:
- `BMI088D_SUCCESS` (0) - Operation successful
- `BMI088D_ERROR_INVALID_PARAM` (-1) - Invalid parameter
- `BMI088D_ERROR_SENSOR` (-5) - Sensor communication error

## Dependencies

- STM32 HAL library (or equivalent for your platform)
- Math library (libm) for floating-point operations
- SPI communication interface
- GPIO control for chip select lines
- Timer for PWM output

## Porting to Other Platforms

To port the library to other platforms:

1. Implement the hardware abstraction functions in `bmi088d_hal.c`
2. Adapt the SPI communication functions
3. Implement the delay functions
4. Adjust GPIO control for chip select lines
5. Modify timer functions for precise timing

## License

This library is based on the RoboMaster INS example code and is provided for educational and research purposes.