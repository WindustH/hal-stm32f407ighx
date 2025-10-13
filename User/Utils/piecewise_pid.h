#ifndef __USER_UTILS_PIECEWISE_PID__
#define __USER_UTILS_PIECEWISE_PID__
#include "type.h"
f32 pid_compute(volatile pidStat *const pid, volatile pwPidArg *const arg,
                const f32 feedback);

#endif /* __USER_UTILS_PIECEWISE_PID__ */
