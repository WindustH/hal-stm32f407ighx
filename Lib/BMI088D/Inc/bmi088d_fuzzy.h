/**
 * @file    bmi088d_fuzzy.h
 * @brief   BMI088 Driver Library - Fuzzy Logic for PID
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-14
 */

#ifndef BMI088D_FUZZY_H
#define BMI088D_FUZZY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fuzzy logic linguistic variables */
#define BMI088D_FUZZY_NB -3.0f
#define BMI088D_FUZZY_NM -2.0f
#define BMI088D_FUZZY_NS -1.0f
#define BMI088D_FUZZY_ZE  0.0f
#define BMI088D_FUZZY_PS  1.0f
#define BMI088D_FUZZY_PM  2.0f
#define BMI088D_FUZZY_PB  3.0f

/* Fuzzy rule structure */
typedef struct {
    float kp_fuzzy;
    float ki_fuzzy;
    float kd_fuzzy;

    const float (*rule_kp)[7];
    const float (*rule_ki)[7];
    const float (*rule_kd)[7];

    float e_step;
    float ec_step;

    float e;
    float ec;
    float e_last;
} bmi088d_fuzzy_rule_t;

/**
 * @brief Initialize the fuzzy logic rule set for PID controller
 * @param[out] rule Pointer to the fuzzy rule structure
 * @param[in] fuzzy_rule_kp Custom Kp rule matrix (7x7), or NULL for default
 * @param[in] fuzzy_rule_ki Custom Ki rule matrix (7x7), or NULL for default
 * @param[in] fuzzy_rule_kd Custom Kd rule matrix (7x7), or NULL for default
 * @param[in] e_step Step size for error quantization
 * @param[in] ec_step Step size for error change quantization
 */
void bmi088d_fuzzy_rule_init(bmi088d_fuzzy_rule_t *rule,
                             const float (*fuzzy_rule_kp)[7],
                             const float (*fuzzy_rule_ki)[7],
                             const float (*fuzzy_rule_kd)[7],
                             float e_step,
                             float ec_step);

/**
 * @brief Execute the fuzzy logic inference to get PID gain adjustments
 * @param[in,out] rule Pointer to the fuzzy rule structure
 * @param[in] measure Current measured value
 * @param[in] ref Target reference value
 * @param[in] dt Time delta
 */
void bmi088d_fuzzy_rule_implement(bmi088d_fuzzy_rule_t *rule, float measure, float ref, float dt);


#ifdef __cplusplus
}
#endif

#endif /* BMI088D_FUZZY_H */
