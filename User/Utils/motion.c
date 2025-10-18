#include "motion.h"
#include <math.h>
void solve_motion(f32 Vx, f32 Vy, f32 omega, f32 *A, f32 *B, f32 *C, f32 *D) {
  // Step 1: Compute the four critical points
  f32 p[4] = {0.0, (Vx + omega) / 2.0, (Vx - Vy) / 2.0, (omega - Vy) / 2.0};

  // Step 2: Sort to find median
  // Simple sorting for 4 elements
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (p[i] > p[j]) {
        f32 temp = p[i];
        p[i] = p[j];
        p[j] = temp;
      }
    }
  }

  // Median for even number of points: average of middle two
  f32 a = (p[1] + p[2]) / 2.0;

  // Step 3: Compute b, c, d using the general solution
  f32 b = (Vx + omega) / 2.0 - a;
  f32 c = a - (Vx - Vy) / 2.0;
  f32 d = (omega - Vy) / 2.0 - a;

  // Step 4: Compute m = max(|a|, |b|, |c|, |d|)
  f32 m = fabs(a);
  if (fabs(b) > m)
    m = fabs(b);
  if (fabs(c) > m)
    m = fabs(c);
  if (fabs(d) > m)
    m = fabs(d);

  // Step 5: Normalize
  if (m == 0.0) {
    *A = *B = *C = *D = 0.0;
  } else {
    *A = a / m;
    *B = b / m;
    *C = c / m;
    *D = d / m;
  }
}