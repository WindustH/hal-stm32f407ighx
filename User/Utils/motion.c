#include "motion.h"
#include "arm_math.h" // IWYU pragma: keep

void solve_motion(f32 Vx, f32 Vy, f32 omega, f32 *A, f32 *B, f32 *C, f32 *D) {
  // Step 1: Pure rotation vector v1 = [ω, ω, ω, ω]
  f32 v1[4] = {omega, omega, omega, omega};

  // Step 2: Pure translation vector v2 (for 45° omni wheels)
  f32 v2[4] = {(Vx - Vy) * 0.5f, (Vx + Vy) * 0.5f, (-Vx + Vy) * 0.5f,
               (-Vx - Vy) * 0.5f};

  // Step 3: Compute ||v2|| (which equals sqrt(Vx^2 + Vy^2) ∈ [0,1])
  f32 v2_norm_sq =
      v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2] + v2[3] * v2[3];

  if (v2_norm_sq < MATH_EPSILON) {
    // No translation: output clamped pure rotation
    *A = clamp_f32(v1[0], -1.0f, 1.0f);
    *B = clamp_f32(v1[1], -1.0f, 1.0f);
    *C = clamp_f32(v1[2], -1.0f, 1.0f);
    *D = clamp_f32(v1[3], -1.0f, 1.0f);
    return;
  }

  f32 v2_norm;
  arm_sqrt_f32(v2_norm_sq, &v2_norm); // v2_norm ∈ (0, 1]

  // Step 4: Unit direction e = v2 / ||v2||
  f32 inv_norm = 1.0f / v2_norm;
  f32 e[4] = {v2[0] * inv_norm, v2[1] * inv_norm, v2[2] * inv_norm,
              v2[3] * inv_norm};

  // Step 5: Find smallest t >= 0 such that v1[i] + t * e[i] ∈ [-1, 1] for all i
  f32 t_hit = 1e6f; // will be the minimal valid t

  for (int i = 0; i < 4; i++) {
    if (e[i] < MATH_EPSILON && e[i] > -MATH_EPSILON) {
      // Ray parallel to axis i: check if current v1[i] is within limits
      if (v1[i] < -1.0f || v1[i] > 1.0f) {
        t_hit = 0.0f; // already out of bounds → no translation allowed
        break;
      }
      continue;
    }

    // Compute t where p_i(t) = ±1
    f32 t1 = (-1.0f - v1[i]) / e[i];
    f32 t2 = (1.0f - v1[i]) / e[i];

    // Consider only t >= 0
    if (t1 >= 0.0f && t1 < t_hit)
      t_hit = t1;
    if (t2 >= 0.0f && t2 < t_hit)
      t_hit = t2;
  }

  // If no constraint found (e.g., v1 inside and e=0), t_hit remains large →
  // clamp to 0
  if (t_hit > 1e5f)
    t_hit = 0.0f;

  // l = t_hit (since e is unit vector, distance = t)
  f32 l = t_hit;

  // Step 6: Final output = v1 + e * l * ||v2||
  f32 scale = l * v2_norm;
  f32 out[4] = {v1[0] + e[0] * scale, v1[1] + e[1] * scale,
                v1[2] + e[2] * scale, v1[3] + e[3] * scale};

  // Final clamp to [-1, 1] (safety against numerical error)
  *A = clamp_f32(out[0], -1.0f, 1.0f);
  *B = clamp_f32(out[1], -1.0f, 1.0f);
  *C = clamp_f32(out[2], -1.0f, 1.0f);
  *D = clamp_f32(out[3], -1.0f, 1.0f);
}