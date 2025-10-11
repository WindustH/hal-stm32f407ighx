#ifndef __USER_DRIVER_MOTOR_M3508_CONF__
#define __USER_DRIVER_MOTOR_M3508_CONF__

// M3508电机π值
#define M3508_PI 4096.0f

// 缩放因子
#define M3508_X_SCALE 7.669904e-4f // PI / M3508_PI
#define M3508_V_SCALE 1.0f         // 速度缩放因子
#define M3508_I_SCALE 1.0f         // 电流缩放因子

#endif /* __USER_DRIVER_MOTOR_M3508_CONF__ */
