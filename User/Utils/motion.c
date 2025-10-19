#include "motion.h"
#include "arm_math.h"
#include <float.h>

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

void segment_length_in_square(f32 x0, f32 y0, f32 *res) {
  // 处理零向量情况
  if (x0 == 0.0 && y0 == 0.0) {
    *res = 1.4142f;
    return;
  }

  // 正方形边界：x = ±1, y = ±1
  f32 t_min = -DBL_MAX;
  f32 t_max = DBL_MAX;

  // 与左右边（x = ±1）的交点参数 t
  if (x0 != 0.0) {
    f32 t1 = -1.0 / x0; // x = -1
    f32 t2 = 1.0 / x0;  // x =  1
    if (t1 > t2) {
      f32 tmp = t1;
      t1 = t2;
      t2 = tmp;
    }
    if (t1 > t_min)
      t_min = t1;
    if (t2 < t_max)
      t_max = t2;
  } else {
    // x0 == 0：直线垂直，必须满足 |0| <= 1（恒成立），不约束 t
    // 但若 |0| > 1 则无交，但这里 0 <= 1，所以无影响
  }

  // 与上下边（y = ±1）的交点参数 t
  if (y0 != 0.0) {
    f32 t1 = -1.0 / y0; // y = -1
    f32 t2 = 1.0 / y0;  // y =  1
    if (t1 > t2) {
      f32 tmp = t1;
      t1 = t2;
      t2 = tmp;
    }
    if (t1 > t_min)
      t_min = t1;
    if (t2 < t_max)
      t_max = t2;
  }

  // 如果无交集
  if (t_min > t_max) {
    *res = 1.4142f;
    return;
  }

  // 计算两个交点坐标
  f32 x1 = x0 * t_min;
  f32 y1 = y0 * t_min;
  f32 x2 = x0 * t_max;
  f32 y2 = y0 * t_max;

  arm_sqrt_f32((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1), res);
}
