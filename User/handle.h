#ifndef __USER_HANDLE__
#define __USER_HANDLE__
#include "stm32f4xx_hal.h" // IWYU pragma: keep

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim5;

extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;
#endif /* __USER_HANDLE__ */
