#ifndef __USER_DRIVERS_RC_DR16_PROTOCOL__
#define __USER_DRIVERS_RC_DR16_PROTOCOL__

#include "type.h"

/* ----------------------- 遥控器通道范围 ------------------------------- */
#define RC_CH_VALUE_MIN ((u16)364)     // 通道值最小值
#define RC_CH_VALUE_OFFSET ((u16)1024) // 通道值偏移量（中位）
#define RC_CH_VALUE_MAX ((u16)1684)    // 通道值最大值

/* ----------------------- 开关状态 ---------------------------------- */
#define RC_SW_UP ((u8)1)   // 开关向上
#define RC_SW_MID ((u8)3)  // 开关居中
#define RC_SW_DOWN ((u8)2) // 开关向下

/* ----------------------- 键盘按键掩码 ----------------------------- */
#define KEY_PRESSED_OFFSET_W (0x01U << 0)     // W键
#define KEY_PRESSED_OFFSET_S (0x01U << 1)     // S键
#define KEY_PRESSED_OFFSET_A (0x01U << 2)     // A键
#define KEY_PRESSED_OFFSET_D (0x01U << 3)     // D键
#define KEY_PRESSED_OFFSET_Q (0x01U << 4)     // Q键
#define KEY_PRESSED_OFFSET_E (0x01U << 5)     // E键
#define KEY_PRESSED_OFFSET_SHIFT (0x01U << 6) // SHIFT键
#define KEY_PRESSED_OFFSET_CTRL (0x01U << 7)  // CTRL键

/* ----------------------- 遥控器控制数据结构 ---------------------- */
typedef struct {
  struct {
    u16 ch0; // 通道0
    u16 ch1; // 通道1
    u16 ch2; // 通道2
    u16 ch3; // 通道3
    u8 s1;   // 开关1
    u8 s2;   // 开关2
  } rc;

  struct {
    i16 x;      // 鼠标X轴
    i16 y;      // 鼠标Y轴
    i16 z;      // 鼠标Z轴(滚轮)
    u8 press_l; // 左键按下状态 (0/1)
    u8 press_r; // 右键按下状态 (0/1)
  } mouse;

  struct {
    u16 v; // 按键值
  } key;
} rcCtrl_dr16;

/**
 * @brief 将原始18字节数据解析为结构化的遥控器控制数据
 * @param raw_data 指向18字节缓冲区的指针（不能为空）
 * @param out      指向输出结构体的指针（不能为空）
 */
void rc_ctrl_msg_parse_dr16(const volatile u8 *raw_data,
                            volatile rcCtrl_dr16 *out);

#endif /* __USER_DRIVERS_RC_DR16_PROTOCOL__ */