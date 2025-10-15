#ifndef __USER_DRIVERS_BMI880_DRIVER__
#define __USER_DRIVERS_BMI880_DRIVER__

#include "bmi088d.h"       // IWYU pragma: keep
#include "type.h"

void setup_bmi088(SPI_HandleTypeDef *hspi, GPIO_TypeDef *accel_cs_port,
                  u16 accel_cs_pin, GPIO_TypeDef *gyro_cs_port, u16 gyro_cs_pin,
                  TIM_HandleTypeDef *htim, u32 tim_channel, u16 gyro_it_pin,
                  u16 accel_it_pin);
void bmi088_update_pose(u16 pin);
void bmi088_temp_ctrl();
bmi088d_imu_data_t *bmi088_get_pose();
#endif /* __USER_DRIVERS_BMI880_DRIVER__ */
