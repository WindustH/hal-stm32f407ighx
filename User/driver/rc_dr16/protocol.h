#ifndef __USER_DRIVER_RC_DR16_PROTOCOL__
#define __USER_DRIVER_RC_DR16_PROTOCOL__

#include "type.h"
/* ----------------------- RC Channel Range ------------------------------- */
#define RC_CH_VALUE_MIN ((u16)364)
#define RC_CH_VALUE_OFFSET ((u16)1024)
#define RC_CH_VALUE_MAX ((u16)1684)

/* ----------------------- Switch States ---------------------------------- */
#define RC_SW_UP ((u8)1)
#define RC_SW_MID ((u8)3)
#define RC_SW_DOWN ((u8)2)

/* ----------------------- Keyboard Key Masks ----------------------------- */
#define KEY_PRESSED_OFFSET_W (0x01U << 0)
#define KEY_PRESSED_OFFSET_S (0x01U << 1)
#define KEY_PRESSED_OFFSET_A (0x01U << 2)
#define KEY_PRESSED_OFFSET_D (0x01U << 3)
#define KEY_PRESSED_OFFSET_Q (0x01U << 4)
#define KEY_PRESSED_OFFSET_E (0x01U << 5)
#define KEY_PRESSED_OFFSET_SHIFT (0x01U << 6)
#define KEY_PRESSED_OFFSET_CTRL (0x01U << 7)

/* ----------------------- RC Control Data Structure ---------------------- */
typedef struct {
  struct {
    u16 ch0;
    u16 ch1;
    u16 ch2;
    u16 ch3;
    u8 s1;
    u8 s2;
  } rc;

  struct {
    i16 x;
    i16 y;
    i16 z;
    u8 press_l;
    u8 press_r;
  } mouse;

  struct {
    u16 v;
  } key;
} rcCtrl_dr16;

/**
 * @brief Parse raw 18-byte SBUS-like data into structured RC control data.
 * @param raw_data Pointer to 18-byte buffer (must not be NULL).
 * @param out      Pointer to output structure (must not be NULL).
 */
void rc_ctrl_msg_parse_dr16(const volatile u8 *raw_data, rcCtrl_dr16 *out);

#endif /* __USER_DRIVER_RC_DR16_PROTOCOL__ */
