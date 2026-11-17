/**
 * @file    juca_port.h
 * @brief   Glue between the ESP-IDF style component sources and the Arduino core.
 *
 * @details Including the Arduino HAL here does two things for the C files of this library:
 *          - it declares `analogRead()`, used by wheel.c for the current sensors;
 *          - it redirects the ESP-IDF log macros (ESP_LOGI, ESP_LOGW, ESP_LOGE) to the Arduino ones,
 *            so the messages of the library follow "Tools > Core Debug Level" in the Arduino IDE
 *            (they are hidden at the default level, "None").
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include "esp32-hal.h"
