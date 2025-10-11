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
    
    // Initialize the BMI088 driver library with hardware interface
    result = bmi088d_init_with_hardware(&hw_config);
    if (result != BMI088D_SUCCESS) {
        return result;
    }
    
    // Perform initial calibration
    result = bmi088d_calibrate_sensors();
    if (result != BMI088D_SUCCESS) {
        return result;
    }
    
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
    bmi088d_filter_status_t filter_status;
    float dt;
    
    // System initialization
    SystemInit_Custom();
    
    // Get SPI handle (assuming it's hspi1 from your project)
    extern SPI_HandleTypeDef hspi1;
    bmi088_spi_handle = &hspi1;
    
    // Initialize BMI088 sensor
    if (bmi088_sensor_init() != BMI088D_SUCCESS) {
        // Handle initialization error
        while (1) {
            // Error loop
        }
    }
    
    // Main loop
    while (1) {
        // Calculate time delta
        dt = 0.001f; // Assuming 1kHz update rate
        
        // Read sensor data
        if (bmi088d_read_sensor_data() == BMI088D_SUCCESS) {
            // Get processed IMU data
            bmi088d_get_imu_data(&imu_data);
            
            // Update attitude estimation
            bmi088d_update_attitude(dt);
            
            // Get orientation as Euler angles
            bmi088d_get_attitude(&attitude);
            
            // Get orientation as quaternion
            bmi088d_get_orientation(&orientation);
            
            // Get filter status
            bmi088d_get_filter_status(&filter_status);
            
            // Use the data as needed
            // For example, send to UART for debugging:
            /*
            printf("Accel: X=%f, Y=%f, Z=%f\r\n", imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
            printf("Gyro: X=%f, Y=%f, Z=%f\r\n", imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
            printf("Temperature: %f\r\n", imu_data.temperature);
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
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    // Start TIM10
    HAL_TIM_Base_Start(&htim10);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    
    // Set PWM duty cycle (adjust as needed for target temperature)
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1000; // 50% duty cycle (2000-1 = period)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1);
}