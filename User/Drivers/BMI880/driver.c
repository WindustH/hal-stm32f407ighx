#include "driver.h"
#include "BSP/dwt.h"
#include "main.h"

static volatile bmi088d_imu_data_t imu_data = {0};
static volatile poseT pose = {0};
static u32 pose_update_dwt_cnt;
static u32 temp_ctrl_dwt_cnt;
static u16 ac_it_pin;
static u16 gy_it_pin;
static volatile u8 bmi088_started = false;

void bmi088_setup(SPI_HandleTypeDef *hspi, GPIO_TypeDef *accel_cs_port,
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
  if (bmi088d_init(&config) != BMI088D_SUCCESS) {
    Error_Handler();
  }
  ac_it_pin = accel_it_pin;
  gy_it_pin = gyro_it_pin;
}

void bmi088_update_pose(u16 pin) {
  if (!bmi088_started)
    return;
  if (pin == gy_it_pin || pin == ac_it_pin) {
    f32 dt = DWT_GetDeltaT(&pose_update_dwt_cnt);
    bmi088d_update((bmi088d_imu_data_t *)&imu_data, dt);
    pose.x += imu_data.accel[0] * dt;
    pose.y += imu_data.accel[1] * dt;
    pose.z += imu_data.accel[2] * dt;
    pose.roll += imu_data.gyro[0] * dt;
    pose.pitch += imu_data.gyro[1] * dt;
    pose.yaw += imu_data.gyro[2] * dt;
  }
}

void bmi088_temp_ctrl() {
  if (!bmi088_started)
    return;
  f32 dt = DWT_GetDeltaT(&temp_ctrl_dwt_cnt);
  bmi088d_temp_control(imu_data.temperature, dt);
}

void bmi088_start() {
  pose_update_dwt_cnt = DWT->CYCCNT;
  temp_ctrl_dwt_cnt = DWT->CYCCNT;
  bmi088_started = true;
}
volatile bmi088d_imu_data_t *bmi088_get_imu_data() { return &imu_data; }