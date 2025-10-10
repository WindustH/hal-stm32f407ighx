#ifndef BMI088_ALGORITHM_H
#define BMI088_ALGORITHM_H

#include "stdint.h"
#include "math.h"

// 常量定义
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

// 数据结构定义
typedef struct {
    float Accel[3];
    float Gyro[3];
    float TempWhenCali;
    float Temperature;
    float AccelScale;
    float GyroOffset[3];
    float gNorm;
} IMU_Data_t;

// 错误代码定义
enum {
    BMI088_NO_ERROR = 0x00,
    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

// 函数声明
void BMI088_Init_Algorithm(IMU_Data_t *bmi088, uint8_t calibrate);
void BMI088_Calibrate_Offset(IMU_Data_t *bmi088, uint16_t cali_times);
void BMI088_Read_Algorithm(IMU_Data_t *bmi088, uint8_t *accel_data, uint8_t *gyro_data, uint8_t *temp_data);
void BMI088_Set_Accel_Sensitivity(float sensitivity);
void BMI088_Set_Gyro_Sensitivity(float sensitivity);
uint8_t BMI088_Verify_Calibration(IMU_Data_t *bmi088, float g_norm_diff, float *gyro_diff);

#endif // BMI088_ALGORITHM_H