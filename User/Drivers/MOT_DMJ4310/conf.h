#ifndef __USER_DRIVERS_MOT_DMJ4310_CONF__
#define __USER_DRIVERS_MOT_DMJ4310_CONF__

// DMJ4310电机π值
#define DMJ4310_PI 32768.0f
// 缩放因子
#define DMJ4310_X_SCALE 9.587380e-5f // PI / DMJ4310_PI
#define DMJ4310_V_SCALE 1.0f         // 速度缩放因子
#define DMJ4310_TOR_SCALE 1.0f       // 扭矩缩放因子
#define DMJ4310_MAX_TRQ 7.0f
#define DMJ4310_MIN_TRQ -7.0f

#endif /* __USER_DRIVERS_MOT_DMJ4310_CONF__ */
