/**
 * @file    threeeyes_task.c
 * @brief   Example: read the three HC-SR04 sensors and print the nearest obstacle.
 *
 * @details Twice per second the task fires the three sensors together, waits for the first echo and
 *          prints which sensor answered first and at what distance. Nothing moves. See
 *          README_threeeyes.md in this folder.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "threeeyes.h"
#include "threeeyes_task.h"
#include "juca_board.h"

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG = "threeeyes_task";

/** @brief Longest wait for an echo. A sensor with nothing in range answers after about 38 ms, so 60 ms
 *         only expires when no sensor answers at all (unplugged or unpowered). */
#define ECHO_TIMEOUT_MS      60

/** @brief Time between two measurements. */
#define MEASURE_PERIOD_MS    500

/** @brief Echoes longer than this distance are the sensor's "nothing in range" answer (about 650 cm),
 *         not a real object. 400 cm is the maximum range of the HC-SR04. */
#define MAX_RANGE_CM         400.0f

portTASK_FUNCTION(threeeyes_task, arg)
{
    (void)arg;

    /* Pins and MCPWM group come from the board header; the component itself knows no pin. */
    const threeeyes_config_t config = {
        .trig_gpio = JUCA_PIN_HCSR04_TRIG,
        .echo_left_gpio = JUCA_PIN_HCSR04_ECHO_LEFT,
        .echo_middle_gpio = JUCA_PIN_HCSR04_ECHO_MIDDLE,
        .echo_right_gpio = JUCA_PIN_HCSR04_ECHO_RIGHT,
        .mcpwm_group_id = JUCA_MCPWM_GROUP_HCSR04,
    };
    esp_err_t init_status = threeeyes_init(&config);
    if (init_status != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize the sensors: %s", esp_err_to_name(init_status));
        vTaskDelete(NULL);
        return;
    }

    static const char *const sensor_names[] = {"left", "middle", "right"};

    for ( ; ; )
    {
        if (threeeyes_trigger_and_wait(pdMS_TO_TICKS(ECHO_TIMEOUT_MS)) != ESP_OK)
        {
            ESP_LOGW(TAG, "No echo within %d ms: check the wiring and the 5 V supply of the sensors.", ECHO_TIMEOUT_MS);
        }
        else
        {
            threeeyes_value_t values[3];
            threeeyes_read_values(&values[0], &values[1], &values[2]);

            /* Among the sensors that answered, keep the one with the shortest echo. */
            int nearest_index = -1;
            for (int i = 0; i < 3; i++)
            {
                if (values[i].is_updated && (nearest_index < 0 || values[i].tof_ticks < values[nearest_index].tof_ticks))
                {
                    nearest_index = i;
                }
            }

            float distance_cm = (nearest_index >= 0) ? threeeyes_convert_ticks_to_cm(values[nearest_index].tof_ticks) : 0.0f;
            if (nearest_index < 0 || distance_cm > MAX_RANGE_CM)
            {
                ESP_LOGI(TAG, "Nothing in range.");
            }
            else
            {
                ESP_LOGI(TAG, "Nearest object: %s sensor, %.1f cm", sensor_names[nearest_index], distance_cm);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MEASURE_PERIOD_MS));
    }
}
