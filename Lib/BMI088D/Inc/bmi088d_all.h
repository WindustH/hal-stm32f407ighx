/**
 * @file    bmi088d_all.h
 * @brief   BMI088 Driver Library - Consolidated Header
 * @author  Extracted from RoboMaster INS Example
 * @version 1.0.0
 * @date    2025-10-11
 *
 * @attention
 * This is a consolidated header file that includes all BMI088D functionality
 * in a single header for simplified usage.
 */

#ifndef BMI088D_ALL_H
#define BMI088D_ALL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include all BMI088D components */
#include "bmi088d.h"
#include "bmi088d_calibration.h"
#include "bmi088d_ekf.h"
#include "bmi088d_hal.h"
#include "bmi088d_pid.h"
#include "bmi088d_quaternion.h"
#include "bmi088d_reg.h"
#include "bmi088d_sensor.h"
#include "bmi088d_types.h"
#include "bmi088d_utils.h"

#ifdef __cplusplus
}
#endif

#endif /* BMI088D_ALL_H */