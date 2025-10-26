#ifndef __USER_DRIVERS_ALL__
#define __USER_DRIVERS_ALL__
#include "Drivers/BMI088/driver.h"        // IWYU pragma: keep
#include "Drivers/BOARD_CAN_COM/driver.h" // IWYU pragma: keep
#include "Drivers/MOT_M3508/driver.h"     // IWYU pragma: keep
#ifdef BOARD_GIMBAL
#include "Drivers/MOT_DMJ4310/driver.h" // IWYU pragma: keep
#include "Drivers/MOT_DMJ6006/driver.h" // IWYU pragma: keep
#include "Drivers/RC_DR16/driver.h"     // IWYU pragma: keep
#endif
#ifdef BOARD_CHASSIS
#endif
#endif /* __USER_DRIVERS_ALL__ */
