/**
 * @file    ir_line_task.h
 * @brief   Example task for the 5 infrared line sensors (ir_line component).
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief FreeRTOS task entry point: prints where the line is under the sensor bar, twice per second.
 *
 * @param[in] arg Task argument (unused).
 */
portTASK_FUNCTION(ir_line_task, arg);
