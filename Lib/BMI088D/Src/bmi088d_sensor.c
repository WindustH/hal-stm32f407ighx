/**
 * @file    bmi088d_sensor.c
 * @brief   BMI088 Driver Library - Sensor Driver Implementation
 * @author
 * @version 1.0.0
 * @date    2025-10-11
 */

#include "bmi088d_sensor.h"
#include "bmi088d_calibration.h"
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
  do {
    bmi088d_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  } while (res != BMI088_ACC_CHIP_ID_VALUE);

  // Set accelerometer sensor config
  for (uint8_t i = 0; i < BMI088_WRITE_ACCEL_REG_NUM; i++) {
    bmi088d_accel_write_single_reg(write_bmi088_accel_reg_data[i][0],
                                   write_bmi088_accel_reg_data[i][1]);
    bmi088d_hal_delay_ms(1);
    // bmi088d_accel_read_single_reg(write_bmi088_accel_reg_data[i][0], &res);
    // bmi088d_hal_delay_ms(1);

    // if (res != write_bmi088_accel_reg_data[i][1]) {
    //   return BMI088D_SENSOR_ERROR_CONFIG;
    // }
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
  do {
    bmi088d_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  } while (res != BMI088_GYRO_CHIP_ID_VALUE);


  // Set gyroscope sensor config
  for (uint8_t i = 0; i < BMI088_WRITE_GYRO_REG_NUM; i++) {
    bmi088d_gyro_write_single_reg(write_bmi088_gyro_reg_data[i][0],
                                  write_bmi088_gyro_reg_data[i][1]);
    bmi088d_hal_delay_ms(1);
    // bmi088d_gyro_read_single_reg(write_bmi088_gyro_reg_data[i][0], &res);
    // bmi088d_hal_delay_ms(1);

    // if (res != write_bmi088_gyro_reg_data[i][1]) {
    //   return BMI088D_SENSOR_ERROR_CONFIG;
    // }
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
  static int16_t accel_raw[3], gyro_raw[3];

  // Read accelerometer data
  bmi088d_accel_read_multi_reg(BMI088_ACCEL_XOUT_L, buf, 6);

  accel_raw[0] = (int16_t)((buf[1]) << 8) | buf[0];
  accel_raw[1] = (int16_t)((buf[3]) << 8) | buf[2];
  accel_raw[2] = (int16_t)((buf[5]) << 8) | buf[4];

  // Read gyroscope data
  bmi088d_gyro_read_multi_reg(BMI088_GYRO_X_L, buf, 6);

  gyro_raw[0] = (int16_t)((buf[1]) << 8) | buf[0];
  gyro_raw[1] = (int16_t)((buf[3]) << 8) | buf[2];
  gyro_raw[2] = (int16_t)((buf[5]) << 8) | buf[4];

  // Read temperature data
  bmi088d_accel_read_multi_reg(BMI088_TEMP_M, buf, 2);

  raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

  if (raw_temp > 1023) {
    raw_temp -= 2048;
  }

  data->temperature = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

  // Apply calibration to raw sensor data
  bmi088d_calib_apply(data, accel_raw, gyro_raw);
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
