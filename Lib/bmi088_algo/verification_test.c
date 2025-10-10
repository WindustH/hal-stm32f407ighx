#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "bmi088_algorithm.h"
#include "bmi088_quaternion_ekf.h"

// 模拟传感器数据
typedef struct {
    uint8_t accel_data[6];
    uint8_t gyro_data[8];
    uint8_t temp_data[2];
} SensorData_t;

// 生成模拟传感器数据
void generate_sensor_data(SensorData_t *data, float accel_x, float accel_y, float accel_z,
                         float gyro_x, float gyro_y, float gyro_z, float temp) {
    // 模拟加速度计数据 (假设6G量程)
    int16_t accel_raw_x = (int16_t)(accel_x / BMI088_ACCEL_6G_SEN);
    int16_t accel_raw_y = (int16_t)(accel_y / BMI088_ACCEL_6G_SEN);
    int16_t accel_raw_z = (int16_t)(accel_z / BMI088_ACCEL_6G_SEN);

    data->accel_data[0] = (uint8_t)(accel_raw_x & 0xFF);
    data->accel_data[1] = (uint8_t)((accel_raw_x >> 8) & 0xFF);
    data->accel_data[2] = (uint8_t)(accel_raw_y & 0xFF);
    data->accel_data[3] = (uint8_t)((accel_raw_y >> 8) & 0xFF);
    data->accel_data[4] = (uint8_t)(accel_raw_z & 0xFF);
    data->accel_data[5] = (uint8_t)((accel_raw_z >> 8) & 0xFF);

    // 模拟陀螺仪数据 (假设2000 deg/s量程)
    int16_t gyro_raw_x = (int16_t)(gyro_x / BMI088_GYRO_2000_SEN);
    int16_t gyro_raw_y = (int16_t)(gyro_y / BMI088_GYRO_2000_SEN);
    int16_t gyro_raw_z = (int16_t)(gyro_z / BMI088_GYRO_2000_SEN);

    // 陀螺仪数据从索引2开始，索引0是芯片ID
    data->gyro_data[0] = 0x0F; // 芯片ID
    data->gyro_data[1] = 0x00; // 保留
    data->gyro_data[2] = (uint8_t)(gyro_raw_x & 0xFF);
    data->gyro_data[3] = (uint8_t)((gyro_raw_x >> 8) & 0xFF);
    data->gyro_data[4] = (uint8_t)(gyro_raw_y & 0xFF);
    data->gyro_data[5] = (uint8_t)((gyro_raw_y >> 8) & 0xFF);
    data->gyro_data[6] = (uint8_t)(gyro_raw_z & 0xFF);
    data->gyro_data[7] = (uint8_t)((gyro_raw_z >> 8) & 0xFF);

    // 模拟温度数据
    int16_t temp_raw = (int16_t)((temp - BMI088_TEMP_OFFSET) / BMI088_TEMP_FACTOR);
    data->temp_data[0] = (uint8_t)((temp_raw >> 3) & 0xFF);
    data->temp_data[1] = (uint8_t)((temp_raw << 5) & 0xFF);
}

int main() {
    printf("BMI088 Algorithm Library Verification Test\n");
    printf("========================================\n");

    // 初始化IMU数据结构
    IMU_Data_t imu_data;
    BMI088_Init_Algorithm(&imu_data, 0); // 不进行校准

    // 初始化Quaternion EKF
    IMU_QuaternionEKF_Init(10.0f, 0.001f, 1000000.0f, 0.9996f, 0.0f);

    // 测试数据：静止状态下的传感器读数
    SensorData_t sensor_data;
    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 9.81f;  // 静止时只有重力加速度
    float gyro_x = 0.01f, gyro_y = -0.005f, gyro_z = 0.002f; // 微小的陀螺仪漂移
    float temp = 25.0f; // 25摄氏度
    float dt = 0.01f;   // 10ms更新周期

    printf("Test 1: Sensor Data Processing\n");
    printf("-----------------------------\n");

    // 生成模拟传感器数据
    generate_sensor_data(&sensor_data, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp);

    // 处理传感器数据
    BMI088_Read_Algorithm(&imu_data, sensor_data.accel_data, sensor_data.gyro_data, sensor_data.temp_data);

    printf("Processed Accelerometer Data:\n");
    printf("  X: %f m/s²\n", imu_data.Accel[0]);
    printf("  Y: %f m/s²\n", imu_data.Accel[1]);
    printf("  Z: %f m/s²\n", imu_data.Accel[2]);
    printf("Processed Gyroscope Data:\n");
    printf("  X: %f rad/s\n", imu_data.Gyro[0]);
    printf("  Y: %f rad/s\n", imu_data.Gyro[1]);
    printf("  Z: %f rad/s\n", imu_data.Gyro[2]);
    printf("Processed Temperature: %f °C\n", imu_data.Temperature);

    printf("\nTest 2: Quaternion EKF Update\n");
    printf("---------------------------\n");

    // 更新Quaternion EKF
    IMU_QuaternionEKF_Update(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, dt);

    printf("Quaternion EKF Results:\n");
    printf("  Quaternion: [%f, %f, %f, %f]\n", QEKF_INS.q[0], QEKF_INS.q[1], QEKF_INS.q[2], QEKF_INS.q[3]);
    printf("  Roll: %f radians (%f degrees)\n", QEKF_INS.Roll, QEKF_INS.Roll * 180.0f / M_PI);
    printf("  Pitch: %f radians (%f degrees)\n", QEKF_INS.Pitch, QEKF_INS.Pitch * 180.0f / M_PI);
    printf("  Yaw: %f radians (%f degrees)\n", QEKF_INS.Yaw, QEKF_INS.Yaw * 180.0f / M_PI);
    printf("  Gyro Bias X: %f rad/s\n", QEKF_INS.GyroBias[0]);
    printf("  Gyro Bias Y: %f rad/s\n", QEKF_INS.GyroBias[1]);

    printf("\nTest 3: Calibration Function\n");
    printf("--------------------------\n");

    // 测试校准函数
    // 模拟一些传感器读数用于校准
    for (int i = 0; i < 100; i++) {
        // 添加一些噪声
        float noise = ((float)rand() / RAND_MAX - 0.5f) * 0.1f;
        imu_data.Accel[0] = noise;
        imu_data.Accel[1] = noise;
        imu_data.Accel[2] = 9.81f + noise;
        imu_data.Gyro[0] = 0.01f + noise * 0.01f;
        imu_data.Gyro[1] = -0.005f + noise * 0.01f;
        imu_data.Gyro[2] = 0.002f + noise * 0.01f;

        // 累加用于校准
        if (i == 0) {
            imu_data.GyroOffset[0] = imu_data.Gyro[0];
            imu_data.GyroOffset[1] = imu_data.Gyro[1];
            imu_data.GyroOffset[2] = imu_data.Gyro[2];
            imu_data.gNorm = sqrtf(imu_data.Accel[0] * imu_data.Accel[0] +
                                  imu_data.Accel[1] * imu_data.Accel[1] +
                                  imu_data.Accel[2] * imu_data.Accel[2]);
        } else {
            imu_data.GyroOffset[0] += imu_data.Gyro[0];
            imu_data.GyroOffset[1] += imu_data.Gyro[1];
            imu_data.GyroOffset[2] += imu_data.Gyro[2];
            imu_data.gNorm += sqrtf(imu_data.Accel[0] * imu_data.Accel[0] +
                                   imu_data.Accel[1] * imu_data.Accel[1] +
                                   imu_data.Accel[2] * imu_data.Accel[2]);
        }
    }

    // 执行校准
    BMI088_Calibrate_Offset(&imu_data, 100);

    printf("Calibration Results:\n");
    printf("  Gyro Offset X: %f rad/s\n", imu_data.GyroOffset[0]);
    printf("  Gyro Offset Y: %f rad/s\n", imu_data.GyroOffset[1]);
    printf("  Gyro Offset Z: %f rad/s\n", imu_data.GyroOffset[2]);
    printf("  Gravity Norm: %f m/s²\n", imu_data.gNorm);
    printf("  Accel Scale: %f\n", imu_data.AccelScale);

    printf("\nVerification Test Completed Successfully!\n");

    return 0;
}