#include "driver.h"
#include "BSP/dwt.h"

static bmi088d_imu_data_t imu_data = {0};
static u32 pose_update_dwt_cnt;
static u32 temp_ctrl_dwt_cnt;
static u16 accel_it_pin_;
static u16 gyro_it_pin_;
static u8 started = false;

void setup_bmi088(SPI_HandleTypeDef *hspi, GPIO_TypeDef *accel_cs_port,
                  u16 accel_cs_pin, GPIO_TypeDef *gyro_cs_port, u16 gyro_cs_pin,
                  TIM_HandleTypeDef *htim, u32 tim_channel, u16 gyro_it_pin,
                  u16 accel_it_pin) {
  bmi088d_hw_config_t config;
  config.tim_channel = tim_channel;
  config.accel_cs_pin = accel_cs_pin;
  config.accel_cs_port = accel_cs_port;
  config.gyro_cs_port = gyro_cs_port;
  config.gyro_cs_pin = gyro_cs_pin;
  config.htim = htim;
  config.hspi = hspi;
  bmi088d_init(&config);
  accel_it_pin_ = accel_it_pin;
  gyro_it_pin_ = gyro_it_pin;
}

void bmi088_update_pose(u16 pin) {
  if (!started)
    return;
  if (pin == gyro_it_pin_ || pin == accel_it_pin_) {
    f32 dt = DWT_GetDeltaT(&pose_update_dwt_cnt);
    bmi088d_update(&imu_data, dt);
  }
}

void bmi088_temp_ctrl() {
  if (!started)
    return;
  f32 dt = DWT_GetDeltaT(&temp_ctrl_dwt_cnt);
  bmi088d_temp_control(imu_data.temperature, dt);
}

void start_bmi088() {
  pose_update_dwt_cnt = DWT->CYCCNT;
  temp_ctrl_dwt_cnt = DWT->CYCCNT;
  started = true;
}
bmi088d_imu_data_t *bmi088_get_pose() { return &imu_data; }