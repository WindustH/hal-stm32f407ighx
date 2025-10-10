#ifndef __USER_BSP_CRON__
#define __USER_BSP_CRON__

#include "type.h"

typedef void (*proc)(void);

u8 bsp_cron_job_add(proc p);

void bsp_cron_job_remove(u8 idx);

#endif /* __USER_BSP_CRON__ */
