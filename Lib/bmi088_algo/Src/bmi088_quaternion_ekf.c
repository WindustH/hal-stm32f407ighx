#include "bmi088_quaternion_ekf.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"

// 外部变量定义
QEKF_INS_t QEKF_INS;
const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};

float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];

// 静态函数声明
static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief 快速平方根倒数计算
 * @param x 输入值
 * @return 平方根倒数
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
    QEKF_INS.Initialized = 1;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 1e-8;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;

    if (lambda > 1)
    {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;
    QEKF_INS.accLPFcoef = lpf;

    // 初始化矩阵维度信息
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

    // 姿态初始化
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = 1;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = 1;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s²
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;

    if (!QEKF_INS.Initialized)
    {
        IMU_QuaternionEKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    }

    QEKF_INS.dt = dt;

    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // set F
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (QEKF_INS.UpdateCount == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    }
    else
    {
        QEKF_INS.Accel[0] = QEKF_INS.accLPFcoef * QEKF_INS.Accel[0] + (1 - QEKF_INS.accLPFcoef) * ax;
        QEKF_INS.Accel[1] = QEKF_INS.accLPFcoef * QEKF_INS.Accel[1] + (1 - QEKF_INS.accLPFcoef) * ay;
        QEKF_INS.Accel[2] = QEKF_INS.accLPFcoef * QEKF_INS.Accel[2] + (1 - QEKF_INS.accLPFcoef) * az;
    }

    // 执行卡尔曼滤波步骤
    Kalman_Filter_Measure(&QEKF_INS.IMU_QuaternionEKF);
    Kalman_Filter_xhatMinusUpdate(&QEKF_INS.IMU_QuaternionEKF);
    Kalman_Filter_PminusUpdate(&QEKF_INS.IMU_QuaternionEKF);
    Kalman_Filter_SetK(&QEKF_INS.IMU_QuaternionEKF);
    Kalman_Filter_xhatUpdate(&QEKF_INS.IMU_QuaternionEKF);
    Kalman_Filter_P_Update(&QEKF_INS.IMU_QuaternionEKF);

    QEKF_INS.UpdateCount++;
}

// 自定义观测函数
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    // 观测步骤 - 根据加速度计测量更新观测向量
    // 这里将加速度计测量值转换为姿态观测
    // 对于IMU，通常使用重力向量作为观测

    // 归一化加速度计测量值
    float accelNorm = sqrtf(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] +
                           QEKF_INS.Accel[1] * QEKF_INS.Accel[1] +
                           QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);

    if (accelNorm > 0.001f) {
        // 归一化加速度计数据
        kf->z_data[0] = QEKF_INS.Accel[0] / accelNorm;
        kf->z_data[1] = QEKF_INS.Accel[1] / accelNorm;
        kf->z_data[2] = QEKF_INS.Accel[2] / accelNorm;
    }
}

// 自定义线性化和渐消函数
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    // 线性化和渐消步骤
    // 1. 四元数归一化
    float quatNorm = sqrtf(kf->xhat_data[0] * kf->xhat_data[0] +
                          kf->xhat_data[1] * kf->xhat_data[1] +
                          kf->xhat_data[2] * kf->xhat_data[2] +
                          kf->xhat_data[3] * kf->xhat_data[3]);

    if (quatNorm > 0.001f) {
        kf->xhat_data[0] /= quatNorm;
        kf->xhat_data[1] /= quatNorm;
        kf->xhat_data[2] /= quatNorm;
        kf->xhat_data[3] /= quatNorm;
    }

    // 2. 应用渐消因子（如果需要）
    if (QEKF_INS.lambda < 1.0f) {
        for (uint16_t i = 0; i < kf->P.rows * kf->P.cols; i++) {
            kf->P_data[i] *= QEKF_INS.lambda;
        }
    }

    // 3. 更新转置矩阵
    Matrix_Transpose(&kf->F, &kf->FT);
}

// 自定义设置H矩阵函数
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    // 设置观测矩阵H
    // 对于IMU姿态估计，H矩阵将四元数转换为重力向量观测

    // 从状态向量中提取四元数
    float q0 = kf->xhatminus_data[0];
    float q1 = kf->xhatminus_data[1];
    float q2 = kf->xhatminus_data[2];
    float q3 = kf->xhatminus_data[3];

    // 构建H矩阵，将四元数转换为重力向量
    // H矩阵的维度是3x6（观测维度x状态维度）
    // 前4个状态是四元数，后2个状态是陀螺仪零偏

    // d(gx)/d(q) = 2 * [q1, -q0, -q3, q2]
    kf->H_data[0] = 2.0f * q1;
    kf->H_data[1] = -2.0f * q0;
    kf->H_data[2] = -2.0f * q3;
    kf->H_data[3] = 2.0f * q2;
    kf->H_data[4] = 0.0f;
    kf->H_data[5] = 0.0f;

    // d(gy)/d(q) = 2 * [q2, q3, -q0, -q1]
    kf->H_data[6] = 2.0f * q2;
    kf->H_data[7] = 2.0f * q3;
    kf->H_data[8] = -2.0f * q0;
    kf->H_data[9] = -2.0f * q1;
    kf->H_data[10] = 0.0f;
    kf->H_data[11] = 0.0f;

    // d(gz)/d(q) = 2 * [q3, -q2, q1, -q0]
    kf->H_data[12] = 2.0f * q3;
    kf->H_data[13] = -2.0f * q2;
    kf->H_data[14] = 2.0f * q1;
    kf->H_data[15] = -2.0f * q0;
    kf->H_data[16] = 0.0f;
    kf->H_data[17] = 0.0f;

    // 更新转置矩阵
    Matrix_Transpose(&kf->H, &kf->HT);
}

// 自定义状态更新函数
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    // 自定义后验估计更新
    // 调用标准的卡尔曼更新
    Kalman_Filter_xhatUpdate(kf);

    // 四元数归一化约束
    float quatNorm = sqrtf(kf->xhat_data[0] * kf->xhat_data[0] +
                          kf->xhat_data[1] * kf->xhat_data[1] +
                          kf->xhat_data[2] * kf->xhat_data[2] +
                          kf->xhat_data[3] * kf->xhat_data[3]);

    if (quatNorm > 0.001f) {
        kf->xhat_data[0] /= quatNorm;
        kf->xhat_data[1] /= quatNorm;
        kf->xhat_data[2] /= quatNorm;
        kf->xhat_data[3] /= quatNorm;
    }

    // 更新陀螺仪零偏估计
    QEKF_INS.GyroBias[0] = kf->xhat_data[4];
    QEKF_INS.GyroBias[1] = kf->xhat_data[5];

    // 更新四元数
    QEKF_INS.q[0] = kf->xhat_data[0];
    QEKF_INS.q[1] = kf->xhat_data[1];
    QEKF_INS.q[2] = kf->xhat_data[2];
    QEKF_INS.q[3] = kf->xhat_data[3];

    // 转换为欧拉角
    float q0 = QEKF_INS.q[0];
    float q1 = QEKF_INS.q[1];
    float q2 = QEKF_INS.q[2];
    float q3 = QEKF_INS.q[3];

    QEKF_INS.Roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    QEKF_INS.Pitch = asinf(2.0f * (q0 * q2 - q1 * q3));
    QEKF_INS.Yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

// 卡尔曼滤波器初始化
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    // 分配内存
    kf->xhat_data = (float *)malloc(xhatSize * sizeof(float));
    kf->xhatminus_data = (float *)malloc(xhatSize * sizeof(float));
    kf->u_data = (float *)malloc(uSize * sizeof(float));
    kf->z_data = (float *)malloc(zSize * sizeof(float));
    kf->P_data = (float *)malloc(xhatSize * xhatSize * sizeof(float));
    kf->Pminus_data = (float *)malloc(xhatSize * xhatSize * sizeof(float));
    kf->F_data = (float *)malloc(xhatSize * xhatSize * sizeof(float));
    kf->FT_data = (float *)malloc(xhatSize * xhatSize * sizeof(float));
    kf->B_data = (float *)malloc(xhatSize * uSize * sizeof(float));
    kf->H_data = (float *)malloc(zSize * xhatSize * sizeof(float));
    kf->HT_data = (float *)malloc(xhatSize * zSize * sizeof(float));
    kf->Q_data = (float *)malloc(xhatSize * xhatSize * sizeof(float));
    kf->R_data = (float *)malloc(zSize * zSize * sizeof(float));
    kf->K_data = (float *)malloc(xhatSize * zSize * sizeof(float));
    kf->S_data = (float *)malloc(zSize * zSize * sizeof(float));
    kf->temp_matrix_data = (float *)malloc(xhatSize * xhatSize * sizeof(float));
    kf->temp_matrix_data1 = (float *)malloc(xhatSize * xhatSize * sizeof(float));
    kf->temp_vector_data = (float *)malloc(xhatSize * sizeof(float));
    kf->temp_vector_data1 = (float *)malloc(xhatSize * sizeof(float));

    // 初始化矩阵
    Matrix_Init(&kf->xhat, xhatSize, 1, kf->xhat_data);
    Matrix_Init(&kf->xhatminus, xhatSize, 1, kf->xhatminus_data);
    Matrix_Init(&kf->u, uSize, 1, kf->u_data);
    Matrix_Init(&kf->z, zSize, 1, kf->z_data);
    Matrix_Init(&kf->P, xhatSize, xhatSize, kf->P_data);
    Matrix_Init(&kf->Pminus, xhatSize, xhatSize, kf->Pminus_data);
    Matrix_Init(&kf->F, xhatSize, xhatSize, kf->F_data);
    Matrix_Init(&kf->FT, xhatSize, xhatSize, kf->FT_data);
    Matrix_Init(&kf->B, xhatSize, uSize, kf->B_data);
    Matrix_Init(&kf->H, zSize, xhatSize, kf->H_data);
    Matrix_Init(&kf->HT, xhatSize, zSize, kf->HT_data);
    Matrix_Init(&kf->Q, xhatSize, xhatSize, kf->Q_data);
    Matrix_Init(&kf->R, zSize, zSize, kf->R_data);
    Matrix_Init(&kf->K, xhatSize, zSize, kf->K_data);
    Matrix_Init(&kf->S, zSize, zSize, kf->S_data);
    Matrix_Init(&kf->temp_matrix, xhatSize, xhatSize, kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, xhatSize, xhatSize, kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, xhatSize, 1, kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, xhatSize, 1, kf->temp_vector_data1);
}

// 矩阵操作函数实现
void Matrix_Init(mat *S, uint16_t row, uint16_t col, float *data)
{
    S->rows = row;
    S->cols = col;
    S->data = data;
}

int Matrix_Add(mat *pSrcA, mat *pSrcB, mat *pDst)
{
    // 检查矩阵维度是否匹配
    if (pSrcA->rows != pSrcB->rows || pSrcA->cols != pSrcB->cols ||
        pSrcA->rows != pDst->rows || pSrcA->cols != pDst->cols) {
        return -1; // 维度不匹配
    }

    uint16_t size = pSrcA->rows * pSrcA->cols;
    for (uint16_t i = 0; i < size; i++) {
        pDst->data[i] = pSrcA->data[i] + pSrcB->data[i];
    }
    return 0;
}

int Matrix_Subtract(mat *pSrcA, mat *pSrcB, mat *pDst)
{
    // 检查矩阵维度是否匹配
    if (pSrcA->rows != pSrcB->rows || pSrcA->cols != pSrcB->cols ||
        pSrcA->rows != pDst->rows || pSrcA->cols != pDst->cols) {
        return -1; // 维度不匹配
    }

    uint16_t size = pSrcA->rows * pSrcA->cols;
    for (uint16_t i = 0; i < size; i++) {
        pDst->data[i] = pSrcA->data[i] - pSrcB->data[i];
    }
    return 0;
}

int Matrix_Multiply(mat *pSrcA, mat *pSrcB, mat *pDst)
{
    // 检查矩阵维度是否匹配
    if (pSrcA->cols != pSrcB->rows || pSrcA->rows != pDst->rows || pSrcB->cols != pDst->cols) {
        return -1; // 维度不匹配
    }

    // 初始化结果矩阵
    uint16_t size = pDst->rows * pDst->cols;
    for (uint16_t i = 0; i < size; i++) {
        pDst->data[i] = 0.0f;
    }

    // 矩阵乘法计算
    for (uint16_t i = 0; i < pSrcA->rows; i++) {
        for (uint16_t j = 0; j < pSrcB->cols; j++) {
            for (uint16_t k = 0; k < pSrcA->cols; k++) {
                pDst->data[i * pDst->cols + j] +=
                    pSrcA->data[i * pSrcA->cols + k] * pSrcB->data[k * pSrcB->cols + j];
            }
        }
    }
    return 0;
}

int Matrix_Transpose(mat *pSrc, mat *pDst)
{
    // 检查矩阵维度是否匹配
    if (pSrc->rows != pDst->cols || pSrc->cols != pDst->rows) {
        return -1; // 维度不匹配
    }

    for (uint16_t i = 0; i < pSrc->rows; i++) {
        for (uint16_t j = 0; j < pSrc->cols; j++) {
            pDst->data[j * pDst->cols + i] = pSrc->data[i * pSrc->cols + j];
        }
    }
    return 0;
}

int Matrix_Inverse(mat *pSrc, mat *pDst)
{
    // 检查矩阵是否为方阵
    if (pSrc->rows != pSrc->cols || pDst->rows != pDst->cols || pSrc->rows != pDst->rows) {
        return -1; // 不是方阵或维度不匹配
    }

    uint16_t n = pSrc->rows;

    // 创建增广矩阵 [A|I]
    float *augmented = (float *)malloc(n * 2 * n * sizeof(float));
    if (augmented == NULL) {
        return -2; // 内存分配失败
    }

    // 初始化增广矩阵
    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            augmented[i * 2 * n + j] = pSrc->data[i * n + j]; // 复制原矩阵
            augmented[i * 2 * n + n + j] = (i == j) ? 1.0f : 0.0f; // 单位矩阵
        }
    }

    // 高斯-约旦消元法
    for (uint16_t i = 0; i < n; i++) {
        // 寻找主元
        float pivot = augmented[i * 2 * n + i];
        if (fabsf(pivot) < 1e-10f) {
            free(augmented);
            return -3; // 矩阵奇异
        }

        // 将主元行归一化
        for (uint16_t j = 0; j < 2 * n; j++) {
            augmented[i * 2 * n + j] /= pivot;
        }

        // 消元
        for (uint16_t k = 0; k < n; k++) {
            if (k != i) {
                float factor = augmented[k * 2 * n + i];
                for (uint16_t j = 0; j < 2 * n; j++) {
                    augmented[k * 2 * n + j] -= factor * augmented[i * 2 * n + j];
                }
            }
        }
    }

    // 复制结果到目标矩阵
    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            pDst->data[i * n + j] = augmented[i * 2 * n + n + j];
        }
    }

    free(augmented);
    return 0;
}

// 其他卡尔曼滤波步骤实现
void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // 测量步骤 - 将测量向量复制到kf->z
    // 在实际应用中，这里会从传感器读取数据
}

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    // 先验估计更新: xhatminus = F * xhat + B * u
    Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
    if (kf->uSize > 0) {
        mat temp;
        Matrix_Init(&temp, kf->xhatSize, 1, kf->temp_vector_data);
        Matrix_Multiply(&kf->B, &kf->u, &temp);
        Matrix_Add(&kf->xhatminus, &temp, &kf->xhatminus);
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    // 先验协方差更新: Pminus = F * P * F' + Q
    mat temp;
    Matrix_Init(&temp, kf->xhatSize, kf->xhatSize, kf->temp_matrix_data);

    // temp = F * P
    Matrix_Multiply(&kf->F, &kf->P, &temp);

    // Pminus = temp * F'
    Matrix_Multiply(&temp, &kf->FT, &kf->Pminus);

    // Pminus = Pminus + Q
    Matrix_Add(&kf->Pminus, &kf->Q, &kf->Pminus);
}

void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    // 计算卡尔曼增益: K = Pminus * H' * inv(H * Pminus * H' + R)
    mat temp1, temp2, temp3;
    Matrix_Init(&temp1, kf->zSize, kf->xhatSize, kf->temp_matrix_data);
    Matrix_Init(&temp2, kf->zSize, kf->zSize, kf->temp_matrix_data1);
    Matrix_Init(&temp3, kf->xhatSize, kf->zSize, kf->temp_matrix_data + kf->zSize * kf->zSize);

    // temp1 = H * Pminus
    Matrix_Multiply(&kf->H, &kf->Pminus, &temp1);

    // temp2 = temp1 * H' = H * Pminus * H'
    Matrix_Multiply(&temp1, &kf->HT, &temp2);

    // temp2 = temp2 + R = H * Pminus * H' + R
    Matrix_Add(&temp2, &kf->R, &temp2);

    // temp3 = inv(H * Pminus * H' + R)
    Matrix_Inverse(&temp2, &temp3);

    // temp1 = Pminus * H'
    Matrix_Multiply(&kf->Pminus, &kf->HT, &temp1);

    // K = temp1 * inv(H * Pminus * H' + R) = Pminus * H' * inv(H * Pminus * H' + R)
    Matrix_Multiply(&temp1, &temp3, &kf->K);
}

void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    // 后验估计更新: xhat = xhatminus + K * (z - H * xhatminus)
    mat temp1, temp2;
    Matrix_Init(&temp1, kf->zSize, 1, kf->temp_vector_data);
    Matrix_Init(&temp2, kf->xhatSize, 1, kf->temp_vector_data1);

    // temp1 = H * xhatminus
    Matrix_Multiply(&kf->H, &kf->xhatminus, &temp1);

    // temp2 = z - temp1 = z - H * xhatminus
    Matrix_Subtract(&kf->z, &temp1, &temp2);

    // temp1 = K * temp2 = K * (z - H * xhatminus)
    Matrix_Multiply(&kf->K, &temp2, &temp1);

    // xhat = xhatminus + temp1 = xhatminus + K * (z - H * xhatminus)
    Matrix_Add(&kf->xhatminus, &temp1, &kf->xhat);
}

void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    // 后验协方差更新: P = (I - K * H) * Pminus
    mat temp1, temp2, I;
    Matrix_Init(&temp1, kf->xhatSize, kf->zSize, kf->temp_matrix_data);
    Matrix_Init(&temp2, kf->xhatSize, kf->xhatSize, kf->temp_matrix_data1);

    // 创建单位矩阵I
    for (uint16_t i = 0; i < kf->xhatSize * kf->xhatSize; i++) {
        kf->temp_matrix_data[i] = 0.0f;
    }
    for (uint16_t i = 0; i < kf->xhatSize; i++) {
        kf->temp_matrix_data[i * kf->xhatSize + i] = 1.0f;
    }
    Matrix_Init(&I, kf->xhatSize, kf->xhatSize, kf->temp_matrix_data);

    // temp1 = K * H
    Matrix_Multiply(&kf->K, &kf->H, &temp1);

    // temp2 = I - temp1 = I - K * H
    Matrix_Subtract(&I, &temp1, &temp2);

    // P = temp2 * Pminus = (I - K * H) * Pminus
    Matrix_Multiply(&temp2, &kf->Pminus, &kf->P);
}

float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 实现完整的卡尔曼滤波更新步骤
    if (!kf->SkipEq1) Kalman_Filter_Measure(kf);
    if (!kf->SkipEq2) Kalman_Filter_xhatMinusUpdate(kf);
    if (!kf->SkipEq3) Kalman_Filter_PminusUpdate(kf);
    if (!kf->SkipEq4) Kalman_Filter_SetK(kf);
    if (!kf->SkipEq5) Kalman_Filter_xhatUpdate(kf);
    Kalman_Filter_P_Update(kf);

    return kf->FilteredValue;
}