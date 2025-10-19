#ifndef __USER_TASKS_PID_M3508_CONF__
#define __USER_TASKS_PID_M3508_CONF__

#ifdef BOARD_CHASSIS
#define M3508_PIDV_KP 3000.0f
#define M3508_PIDV_KI 0.0f
#define M3508_PIDV_KD 0.0f
#define M3508_PIDV_KPR 10.0f
#define M3508_PIDV_KIR 0.0f
#define M3508_PIDV_KDR 0.0f
#define M3508_PIDV_R 400.0f
#define M3508_PIDV_BIG_R 1600.0f
#define M3508_PIDV_OL 16384.0f

#define M3508_PIDX_KP 0.0f
#define M3508_PIDX_KI 0.0f
#define M3508_PIDX_KD 0.0f
#define M3508_PIDX_KPR 0.0f
#define M3508_PIDX_KIR 0.0f
#define M3508_PIDX_KDR 0.0f
#define M3508_PIDX_R 4.0f
#define M3508_PIDX_BIG_R 16.0f
#define M3508_PIDX_OL 4800.0f
#endif
#ifdef BOARD_GIMBAL
#define M3508_PIDV_KP 3000.0f
#define M3508_PIDV_KI 0.0f
#define M3508_PIDV_KD 0.0f
#define M3508_PIDV_KPR 20.0f
#define M3508_PIDV_KIR 0.0f
#define M3508_PIDV_KDR 0.0f
#define M3508_PIDV_R 400.0f
#define M3508_PIDV_BIG_R 1600.0f
#define M3508_PIDV_OL 16384.0f

#define M3508_PIDX_KP 2000.0f
#define M3508_PIDX_KI 0.0f
#define M3508_PIDX_KD 0.0f
#define M3508_PIDX_KPR 100.0f
#define M3508_PIDX_KIR 0.0f
#define M3508_PIDX_KDR 0.0f
#define M3508_PIDX_R 8.0f      // 1.5 * PI
#define M3508_PIDX_BIG_R 24.0f // 6 * PI
#define M3508_PIDX_OL 500.0f
#endif
#endif /* __USER_TASKS_PID_M3508_CONF__ */
