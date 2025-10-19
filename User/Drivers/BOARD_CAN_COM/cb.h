#ifndef __USER_BSP_CB__
#define __USER_BSP_CB__

#include "type.h"

#define BCC_CB_MAX_CNT 32
typedef void (*cbT)(void);
typedef struct {
  cbT procs[BCC_CB_MAX_CNT];
  u32 state;
} bccCbList;
u8 bcc_cb_add(cbT p);
void bcc_cb_remove(u8 idx);
void bcc_cb_call();
#endif /* __USER_BSP_CB__ */
