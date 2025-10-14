/**
 * @file    bmi088d_fuzzy.c
 * @brief   BMI088 Driver Library - Fuzzy Logic for PID
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-14
 */

#include "bmi088d_fuzzy.h"
#include <stddef.h> // For NULL

/* Default Fuzzy Rule matrices from source project */
static const float FUZZY_RULE_KP_DEFAULT[7][7] = {
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS},
    {BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS},
    {BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM},
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM},
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PB},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PB, BMI088D_FUZZY_PB}};

static const float FUZZY_RULE_KI_DEFAULT[7][7] = {
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS},
    {BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM},
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PB},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PB, BMI088D_FUZZY_PB},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PB, BMI088D_FUZZY_PB}};

static const float FUZZY_RULE_KD_DEFAULT[7][7] = {
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PB, BMI088D_FUZZY_PB,
     BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PS},
    {BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PB, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PM, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE,
     BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE, BMI088D_FUZZY_ZE},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PS,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PB},
    {BMI088D_FUZZY_PB, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM, BMI088D_FUZZY_PM,
     BMI088D_FUZZY_PS, BMI088D_FUZZY_PS, BMI088D_FUZZY_PB}};

void bmi088d_fuzzy_rule_init(bmi088d_fuzzy_rule_t *rule,
                             const float (*fuzzy_rule_kp)[7],
                             const float (*fuzzy_rule_ki)[7],
                             const float (*fuzzy_rule_kd)[7], float e_step,
                             float ec_step) {
  if (!rule)
    return;

  rule->rule_kp =
      (fuzzy_rule_kp == NULL) ? FUZZY_RULE_KP_DEFAULT : fuzzy_rule_kp;
  rule->rule_ki =
      (fuzzy_rule_ki == NULL) ? FUZZY_RULE_KI_DEFAULT : fuzzy_rule_ki;
  rule->rule_kd =
      (fuzzy_rule_kd == NULL) ? FUZZY_RULE_KD_DEFAULT : fuzzy_rule_kd;

  rule->e_step = (e_step < 1e-5f) ? 1.0f : e_step;
  rule->ec_step = (ec_step < 1e-5f) ? 1.0f : ec_step;

  rule->e = 0.0f;
  rule->ec = 0.0f;
  rule->e_last = 0.0f;
  rule->kp_fuzzy = 0.0f;
  rule->ki_fuzzy = 0.0f;
  rule->kd_fuzzy = 0.0f;
}

void bmi088d_fuzzy_rule_implement(bmi088d_fuzzy_rule_t *rule, float measure,
                                  float ref, float dt) {
  if (!rule || dt < 1e-6f)
    return;

  float e_left_membership, ec_left_membership;
  float e_right_membership, ec_right_membership;
  int e_left_idx, ec_left_idx;
  int e_right_idx, ec_right_idx;

  rule->e = ref - measure;
  rule->ec = (rule->e - rule->e_last) / dt;
  rule->e_last = rule->e;

  // Determine indices for error (e)
  if (rule->e >= 3 * rule->e_step) {
    e_left_idx = 6;
    e_right_idx = 6;
  } else if (rule->e <= -3 * rule->e_step) {
    e_left_idx = 0;
    e_right_idx = 0;
  } else {
    e_left_idx = (rule->e >= 0) ? ((int)(rule->e / rule->e_step) + 3)
                                : ((int)(rule->e / rule->e_step) + 2);
    e_right_idx = (rule->e >= 0) ? (e_left_idx + 1) : (e_left_idx + 1);
  }

  // Determine indices for error change (ec)
  if (rule->ec >= 3 * rule->ec_step) {
    ec_left_idx = 6;
    ec_right_idx = 6;
  } else if (rule->ec <= -3 * rule->ec_step) {
    ec_left_idx = 0;
    ec_right_idx = 0;
  } else {
    ec_left_idx = (rule->ec >= 0) ? ((int)(rule->ec / rule->ec_step) + 3)
                                  : ((int)(rule->ec / rule->ec_step) + 2);
    ec_right_idx = (rule->ec >= 0) ? (ec_left_idx + 1) : (ec_left_idx + 1);
  }

  // Calculate membership degrees for e
  e_left_membership = (rule->e >= 3 * rule->e_step)
                          ? 0.0f
                          : ((rule->e <= -3 * rule->e_step)
                                 ? 1.0f
                                 : (e_right_idx - rule->e / rule->e_step - 3));
  e_right_membership = (rule->e >= 3 * rule->e_step)
                           ? 1.0f
                           : ((rule->e <= -3 * rule->e_step)
                                  ? 0.0f
                                  : (rule->e / rule->e_step - e_left_idx + 3));

  // Calculate membership degrees for ec
  ec_left_membership =
      (rule->ec >= 3 * rule->ec_step)
          ? 0.0f
          : ((rule->ec <= -3 * rule->ec_step)
                 ? 1.0f
                 : (ec_right_idx - rule->ec / rule->ec_step - 3));
  ec_right_membership =
      (rule->ec >= 3 * rule->ec_step)
          ? 1.0f
          : ((rule->ec <= -3 * rule->ec_step)
                 ? 0.0f
                 : (rule->ec / rule->ec_step - ec_left_idx + 3));

  // Defuzzification using weighted average method
  rule->kp_fuzzy = e_left_membership * ec_left_membership *
                       rule->rule_kp[e_left_idx][ec_left_idx] +
                   e_left_membership * ec_right_membership *
                       rule->rule_kp[e_right_idx][ec_left_idx] +
                   e_right_membership * ec_left_membership *
                       rule->rule_kp[e_left_idx][ec_right_idx] +
                   e_right_membership * ec_right_membership *
                       rule->rule_kp[e_right_idx][ec_right_idx];

  rule->ki_fuzzy = e_left_membership * ec_left_membership *
                       rule->rule_ki[e_left_idx][ec_left_idx] +
                   e_left_membership * ec_right_membership *
                       rule->rule_ki[e_right_idx][ec_left_idx] +
                   e_right_membership * ec_left_membership *
                       rule->rule_ki[e_left_idx][ec_right_idx] +
                   e_right_membership * ec_right_membership *
                       rule->rule_ki[e_right_idx][ec_right_idx];

  rule->kd_fuzzy = e_left_membership * ec_left_membership *
                       rule->rule_kd[e_left_idx][ec_left_idx] +
                   e_left_membership * ec_right_membership *
                       rule->rule_kd[e_right_idx][ec_left_idx] +
                   e_right_membership * ec_left_membership *
                       rule->rule_kd[e_left_idx][ec_right_idx] +
                   e_right_membership * ec_right_membership *
                       rule->rule_kd[e_right_idx][ec_right_idx];
}
