#ifndef __USER_BSP_CRON__
#define __USER_BSP_CRON__

#include "type.h"

#define CRON_JOB_MAX_CNT 32
typedef void (*cronJob)(void);
typedef struct {
  cronJob procs[CRON_JOB_MAX_CNT];
  u32 state;
} cronJobList;
u8 bsp_cron_job_add(cronJob p);
void bsp_cron_job_remove(u8 idx);

#endif /* __USER_BSP_CRON__ */
