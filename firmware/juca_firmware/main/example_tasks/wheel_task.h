/**
 * @file    wheel_task.h
 * @brief   FreeRTOS task definition for demonstrate motorized wheel control execution routines.
 *
 * @date    Jan 2025
 * @author  Matheus
 */

#ifndef MAIN_WHEEL_TASK_H_
#define MAIN_WHEEL_TASK_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//#define WHEEL_EXAMPLE_SINGLE_BASIC
//#define WHEEL_EXAMPLE_TWO_WHEELS
#define WHEEL_EXAMPLE_ENCODER_TEST

/**
 * @brief FreeRTOS task entry point for wheel control operations.
 *
 * @param[in] arg Task entry arguments (unused).
 */
portTASK_FUNCTION(wheel_ctrl_task, arg);

#endif /* MAIN_WHEEL_TASK_H_ */