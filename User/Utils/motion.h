#ifndef __USER_UTILS_MOTION__
#define __USER_UTILS_MOTION__
#include "type.h"
void solve_motion(f32 Vx, f32 Vy, f32 omega, f32 *A, f32 *B, f32 *C, f32 *D);
void segment_length_in_square(f32 x0, f32 y0, f32 *res);
#endif /* __USER_UTILS_MOTION__ */
