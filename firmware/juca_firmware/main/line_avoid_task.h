/**
 * @file    line_avoid_task.h
 * @brief   FreeRTOS task definition for the line follower with obstacle avoidance example.
 *
 * @date    Nov 2026
 * @author  Matheus
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief FreeRTOS task entry point: follows a line with the infrared sensors and goes around the
 *        obstacles seen by the ultrasonic sensors, driving the robot through the `diff_drive` component.
 *
 * @param[in] arg Task entry arguments (unused).
 */
portTASK_FUNCTION(line_avoid_task, arg);
