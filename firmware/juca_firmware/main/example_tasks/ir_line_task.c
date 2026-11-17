/**
 * @file    ir_line_task.c
 * @brief   Example: read the 5 infrared line sensors and print where the line is.
 *
 * @details Twice per second the task reads the sensor bar and prints which sensors see the line and
 *          the name of the pattern. Nothing moves: slide the robot by hand over the line and watch
 *          the log. See README_ir_line.md in this folder.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ir_line.h"
#include "ir_line_task.h"
#include "juca_board.h"

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG = "ir_line_task";

/** @brief Time between two readings. */
#define READ_PERIOD_MS    500

/**
 * @brief Gives a name to a sensor mask.
 *
 * @param[in] mask Mask from @ref ir_line_read_mask.
 * @return Name of the pattern, or "other" for a combination that a single line does not produce
 *         (sensors that are not neighbours, or three or four sensors at once).
 */
static const char *pattern_name(uint8_t mask)
{
    switch (mask)
    {
    case IR_LINE_PATTERN_NONE:            return "no line";
    case IR_LINE_PATTERN_FAR_LEFT:        return "far left";
    case IR_LINE_PATTERN_LEFT_FAR_LEFT:   return "left / far left";
    case IR_LINE_PATTERN_LEFT:            return "left";
    case IR_LINE_PATTERN_LEFT_MIDDLE:     return "left / middle";
    case IR_LINE_PATTERN_MIDDLE:          return "middle";
    case IR_LINE_PATTERN_RIGHT_MIDDLE:    return "middle / right";
    case IR_LINE_PATTERN_RIGHT:           return "right";
    case IR_LINE_PATTERN_RIGHT_FAR_RIGHT: return "right / far right";
    case IR_LINE_PATTERN_FAR_RIGHT:       return "far right";
    case IR_LINE_PATTERN_ALL:             return "all sensors";
    default:                              return "other";
    }
}

portTASK_FUNCTION(ir_line_task, arg)
{
    (void)arg;

    /* Pins and polarity come from the board header; the component itself knows no pin. */
    const ir_line_config_t config = {
        .sensor_gpios = {
            JUCA_PIN_IR_LINE_FAR_LEFT,
            JUCA_PIN_IR_LINE_LEFT,
            JUCA_PIN_IR_LINE_MIDDLE,
            JUCA_PIN_IR_LINE_RIGHT,
            JUCA_PIN_IR_LINE_FAR_RIGHT,
        },
        .line_level = ! JUCA_IR_LINE_LEVEL_ON_LINE,
    };
    esp_err_t init_status = ir_line_init(&config);
    if (init_status != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize the line sensors: %s", esp_err_to_name(init_status));
        vTaskDelete(NULL);
        return;
    }

    for ( ; ; )
    {
        uint8_t mask = 0;
        ir_line_read_mask(&mask);

        /* Draw the bar as the robot sees it: far left sensor first, '#' = over the line. */
        char bar[IR_LINE_SENSOR_COUNT + 1];
        for (int i = 0; i < IR_LINE_SENSOR_COUNT; i++)
        {
            bar[i] = (mask & (1U << i)) ? '#' : '.';
        }
        bar[IR_LINE_SENSOR_COUNT] = '\0';

        ESP_LOGI(TAG, "[%s] %s", bar, pattern_name(mask));

        vTaskDelay(pdMS_TO_TICKS(READ_PERIOD_MS));
    }
}
