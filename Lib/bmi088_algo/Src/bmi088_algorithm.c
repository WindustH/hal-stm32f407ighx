#include "bmi088_algorithm.h"
#include "math.h"

// 全局变量
float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

// 设置加速度计灵敏度
void BMI088_Set_Accel_Sensitivity(float sensitivity) {
    BMI088_ACCEL_SEN = sensitivity;
}

// 设置陀螺仪灵敏度
void BMI088_Set_Gyro_Sensitivity(float sensitivity) {
    BMI088_GYRO_SEN = sensitivity;
}

// 初始化算法
void BMI088_Init_Algorithm(IMU_Data_t *bmi088, uint8_t calibrate) {
    // 初始化数据结构
    for (int i = 0; i < 3; i++) {
        bmi088->Accel[i] = 0.0f;
        bmi088->Gyro[i] = 0.0f;
        bmi088->GyroOffset[i] = 0.0f;
    }
    bmi088->Temperature = 0.0f;
    bmi088->TempWhenCali = 0.0f;
    bmi088->AccelScale = 1.0f;
    bmi088->gNorm = 9.81f;

    // 如果需要校准，则执行校准
    if (calibrate) {
        // 校准函数将在其他函数中实现
    }
}

// 高级校准偏移量
void BMI088_Calibrate_Offset(IMU_Data_t *bmi088, uint16_t cali_times) {
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    float g_norm_sum = 0.0f;
    float g_norm_temp;
    float gyro_max[3], gyro_min[3];
    float g_norm_max, g_norm_min;
    float gyro_diff[3], g_norm_diff;

    // 初始化最大值和最小值
    for (int i = 0; i < 3; i++) {
        gyro_max[i] = -1000.0f;
        gyro_min[i] = 1000.0f;
    }
    g_norm_max = -1000.0f;
    g_norm_min = 1000.0f;

    // 读取多个样本并计算统计值
    for (uint16_t i = 0; i < cali_times; i++) {
        // 计算加速度计数据的模
        g_norm_temp = sqrtf(bmi088->Accel[0] * bmi088->Accel[0] +
                            bmi088->Accel[1] * bmi088->Accel[1] +
                            bmi088->Accel[2] * bmi088->Accel[2]);
        g_norm_sum += g_norm_temp;

        // 累加陀螺仪数据
        gyro_sum[0] += bmi088->Gyro[0];
        gyro_sum[1] += bmi088->Gyro[1];
        gyro_sum[2] += bmi088->Gyro[2];

        // 更新最大值和最小值
        if (g_norm_temp > g_norm_max) g_norm_max = g_norm_temp;
        if (g_norm_temp < g_norm_min) g_norm_min = g_norm_temp;

        for (int j = 0; j < 3; j++) {
            if (bmi088->Gyro[j] > gyro_max[j]) gyro_max[j] = bmi088->Gyro[j];
            if (bmi088->Gyro[j] < gyro_min[j]) gyro_min[j] = bmi088->Gyro[j];
        }
    }

    // 计算平均值作为偏移量
    bmi088->gNorm = g_norm_sum / (float)cali_times;
    bmi088->GyroOffset[0] = gyro_sum[0] / (float)cali_times;
    bmi088->GyroOffset[1] = gyro_sum[1] / (float)cali_times;
    bmi088->GyroOffset[2] = gyro_sum[2] / (float)cali_times;

    // 计算差值
    g_norm_diff = g_norm_max - g_norm_min;
    for (int j = 0; j < 3; j++) {
        gyro_diff[j] = gyro_max[j] - gyro_min[j];
    }

    // 计算加速度计比例因子
    bmi088->AccelScale = 9.81f / bmi088->gNorm;
}

// 读取和处理算法
void BMI088_Read_Algorithm(IMU_Data_t *bmi088, uint8_t *accel_data, uint8_t *gyro_data, uint8_t *temp_data) {
    int16_t bmi088_raw_temp;

    // 处理加速度计数据
    bmi088_raw_temp = (int16_t)((accel_data[1]) << 8) | accel_data[0];
    bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((accel_data[3]) << 8) | accel_data[2];
    bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((accel_data[5]) << 8) | accel_data[4];
    bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    // 处理陀螺仪数据
    bmi088_raw_temp = (int16_t)((gyro_data[3]) << 8) | gyro_data[2];
    bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[0];
    bmi088_raw_temp = (int16_t)((gyro_data[5]) << 8) | gyro_data[4];
    bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[1];
    bmi088_raw_temp = (int16_t)((gyro_data[7]) << 8) | gyro_data[6];
    bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[2];

    // 处理温度数据
    bmi088_raw_temp = (int16_t)((temp_data[0] << 3) | (temp_data[1] >> 5));
    if (bmi088_raw_temp > 1023) {
        bmi088_raw_temp -= 2048;
    }
    bmi088->Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    // 应用加速度计比例因子
    bmi088->Accel[0] *= bmi088->AccelScale;
    bmi088->Accel[1] *= bmi088->AccelScale;
    bmi088->Accel[2] *= bmi088->AccelScale;
}

// 验证校准结果
uint8_t BMI088_Verify_Calibration(IMU_Data_t *bmi088, float g_norm_diff, float *gyro_diff) {
    // 检查校准结果是否在合理范围内
    if (g_norm_diff > 0.5f ||
        gyro_diff[0] > 0.15f ||
        gyro_diff[1] > 0.15f ||
        gyro_diff[2] > 0.15f ||
        fabsf(bmi088->GyroOffset[0]) > 0.01f ||
        fabsf(bmi088->GyroOffset[1]) > 0.01f ||
        fabsf(bmi088->GyroOffset[2]) > 0.01f) {
        return 0; // 校准失败
    }
    return 1; // 校准成功
}