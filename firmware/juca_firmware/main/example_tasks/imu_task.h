/**
 * @file    imu_task.h
 * @brief   Example task for the MPU6050 IMU (orientation quaternion from the DMP).
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief FreeRTOS task entry point: calibrates the IMU and prints its orientation quaternion.
 *
 * @param[in] arg Task argument (unused).
 */
portTASK_FUNCTION(imu_task, arg);
