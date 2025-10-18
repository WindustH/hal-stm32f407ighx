#ifndef __USER_DRIVERS_MOT_DMJ6006_CONF__
#define __USER_DRIVERS_MOT_DMJ6006_CONF__

// DMJ6006电机π值
#define DMJ6006_PI 32768.0f
// 缩放因子
#define DMJ6006_X_SCALE 9.587380e-5f // PI / DMJ6006_PI
#define DMJ6006_V_SCALE 1.0f         // 速度缩放因子
#define DMJ6006_TOR_SCALE 1.0f       // 扭矩缩放因子
#define DMJ6006_MAX_TRQ 12.0f
#define DMJ6006_MIN_TRQ -12.0f

#endif /* __USER_DRIVERS_MOT_DMJ6006_CONF__ */
