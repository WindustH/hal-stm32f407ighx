#include <stdio.h>
#include <stdlib.h>
#include "bmi088_quaternion_ekf.h"

int main() {
    printf("Testing BMI088 Quaternion EKF Implementation\n");

    // 初始化四元数EKF
    printf("Initializing Quaternion EKF...\n");
    IMU_QuaternionEKF_Init(10.0f, 0.001f, 1000000.0f, 0.9996f, 0.0f);

    // 检查初始化是否成功
    if (QEKF_INS.Initialized) {
        printf("Quaternion EKF initialized successfully.\n");
    } else {
        printf("Failed to initialize Quaternion EKF.\n");
        return -1;
    }

    // 测试一些基本的矩阵操作
    printf("Testing matrix operations...\n");

    // 创建测试矩阵
    float dataA[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    float dataB[4] = {2.0f, 3.0f, 4.0f, 5.0f};
    float dataC[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    mat matA, matB, matC;
    Matrix_Init(&matA, 2, 2, dataA);
    Matrix_Init(&matB, 2, 2, dataB);
    Matrix_Init(&matC, 2, 2, dataC);

    // 测试矩阵加法
    if (Matrix_Add(&matA, &matB, &matC) == 0) {
        printf("Matrix addition successful.\n");
        printf("Result: [%f, %f; %f, %f]\n", matC.data[0], matC.data[1], matC.data[2], matC.data[3]);
    } else {
        printf("Matrix addition failed.\n");
    }

    // 测试矩阵乘法
    if (Matrix_Multiply(&matA, &matB, &matC) == 0) {
        printf("Matrix multiplication successful.\n");
        printf("Result: [%f, %f; %f, %f]\n", matC.data[0], matC.data[1], matC.data[2], matC.data[3]);
    } else {
        printf("Matrix multiplication failed.\n");
    }

    // 测试卡尔曼滤波器初始化
    printf("Testing Kalman Filter initialization...\n");
    KalmanFilter_t testKF;
    Kalman_Filter_Init(&testKF, 4, 0, 3);

    if (testKF.xhatSize == 4 && testKF.zSize == 3) {
        printf("Kalman Filter initialized successfully.\n");
    } else {
        printf("Kalman Filter initialization failed.\n");
    }

    // 测试四元数EKF更新
    printf("Testing Quaternion EKF update...\n");
    float gx = 0.1f, gy = 0.2f, gz = 0.3f;  // 陀螺仪数据 (rad/s)
    float ax = 0.0f, ay = 0.0f, az = 9.8f;   // 加速度计数据 (m/s²)
    float dt = 0.01f;  // 时间步长 (s)

    IMU_QuaternionEKF_Update(gx, gy, gz, ax, ay, az, dt);

    printf("Quaternion EKF update completed.\n");
    printf("Estimated Roll: %f\n", QEKF_INS.Roll);
    printf("Estimated Pitch: %f\n", QEKF_INS.Pitch);
    printf("Estimated Yaw: %f\n", QEKF_INS.Yaw);

    printf("Test completed successfully.\n");

    return 0;
}