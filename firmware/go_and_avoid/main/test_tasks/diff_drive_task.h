/**
 * @file    wheel_task.h
 * @brief   FreeRTOS task definition for demonstrate motorized wheel control execution routines.
 *
 * @date    Jan 2025
 * @author  Matheus
 */

#ifndef DIFF_DRIVE_TASK_H_
#define DIFF_DRIVE_TASK_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief FreeRTOS task entry point for differential drive control operations.
 *
 * @param[in] arg Task entry arguments (unused).
 */
portTASK_FUNCTION(DiffDriveCtrl, arg);

#endif /* DIFF_DRIVE_TASK_H_ */