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

//#define SINGLE_BASIC_WHEEL_CTRL_TASK
//#define TWO_WHEELS_CTRL_TASK
#define ENCODER_TEST_TASK

/**
 * @brief FreeRTOS task entry point for wheel control operations.
 *
 * @param[in] arg Task entry arguments (unused).
 */
portTASK_FUNCTION(WheelCtrl, arg);

#endif /* MAIN_WHEEL_TASK_H_ */