#ifndef __USER_DRIVER_MOTOR_DMJ4310_CONF__
#define __USER_DRIVER_MOTOR_DMJ4310_CONF__

// DMJ4310电机π值
#define DMJ4310_PI 32768.0f
// CAN ID - MIT模式命令帧
#define DMJ4310_CAN_ID 0x002U
// 反馈帧ID - 来自电机
#define DMJ4310_MASTER_ID 0x003U
// 缩放因子
#define DMJ4310_X_SCALE 9.587380e-5f // PI / DMJ4310_PI
#define DMJ4310_V_SCALE 1.0f         // 速度缩放因子
#define DMJ4310_TOR_SCALE 1.0f       // 扭矩缩放因子

#endif /* __USER_DRIVER_MOTOR_DMJ4310_CONF__ */
