#ifndef __USER_DRIVERS_RC_DR16_CB__
#define __USER_DRIVERS_RC_DR16_CB__

#include "type.h"

#define DR16_CB_MAX_CNT 8
typedef void (*dr16Cb)(void);
typedef struct {
  dr16Cb procs[DR16_CB_MAX_CNT];
  u32 state;
} dr16CbList;
u8 dr16_cb_add(dr16Cb p);
void dr16_cb_remove(u8 idx);
void dr16_cb_call();

#endif /* __USER_DRIVERS_RC_DR16_CB__ */
