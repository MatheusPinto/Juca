/**
 * @file    threeeyes_task.h
 * @brief   Example task for the three HC-SR04 ultrasonic sensors (ThreeEyes component).
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief FreeRTOS task entry point: prints the nearest obstacle seen by the three sensors, twice per second.
 *
 * @param[in] arg Task argument (unused).
 */
portTASK_FUNCTION(threeeyes_task, arg);
