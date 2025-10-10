#ifndef __USER_DRIVER_MOTOR_DMJ4310_CONF__
#define __USER_DRIVER_MOTOR_DMJ4310_CONF__

// CAN IDs for Pitch-axis DM4310 motor
#define DMJ4310_PI 32768.0f
#define DMJ4310_PITCH_CAN_ID 0x02U       // MIT-mode command frame
#define DMJ4310_PITCH_FEEDBACK_ID 0x206U // Feedback frame (from motor)

// Scaling factors
#define DMJ4310_X_SCALE 9.587380e-5f // 1.0f / DMJ4310_PI * PI
#define DMJ4310_V_SCALE 1.0f
#define DMJ4310_TOR_SCALE 1.0f

#endif /* __USER_DRIVER_MOTOR_DMJ4310_CONF__ */
