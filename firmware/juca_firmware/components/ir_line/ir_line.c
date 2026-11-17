/**
 * @file    ir_line.c
 * @brief   Implementation of the ir_line component (5 digital infrared line sensors on GPIO).
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#include "ir_line.h"
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG = "ir_line";

/** @brief Copy of the configuration given to @ref ir_line_init. */
static ir_line_config_t s_config;

/** @brief true after a successful @ref ir_line_init. */
static bool s_is_initialized = false;

esp_err_t ir_line_init(const ir_line_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Null configuration pointer provided");
    ESP_RETURN_ON_FALSE(config->line_level <= 1, ESP_ERR_INVALID_ARG, TAG, "line_level must be 0 or 1");

    /* Build the pin mask that gpio_config() expects: one bit per GPIO number. */
    uint64_t pin_bit_mask = 0;
    for (int i = 0; i < IR_LINE_SENSOR_COUNT; i++)
    {
        ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(config->sensor_gpios[i]), ESP_ERR_INVALID_ARG, TAG, "Sensor %d: invalid GPIO", i);
        pin_bit_mask |= 1ULL << config->sensor_gpios[i];
    }

    /* The sensor modules drive their output actively, so no pull resistor is needed. */
    const gpio_config_t gpio_configuration = {
        .pin_bit_mask = pin_bit_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&gpio_configuration), TAG, "Failed to configure the sensor pins");

    s_config = *config;
    s_is_initialized = true;
    return ESP_OK;
}

esp_err_t ir_line_read_mask(uint8_t *out_mask)
{
    ESP_RETURN_ON_FALSE(out_mask, ESP_ERR_INVALID_ARG, TAG, "Output pointer is NULL");
    *out_mask = 0;
    ESP_RETURN_ON_FALSE(s_is_initialized, ESP_ERR_INVALID_STATE, TAG, "Not initialized");

    uint8_t mask = 0;
    for (int i = 0; i < IR_LINE_SENSOR_COUNT; i++)
    {
        if ((uint32_t)gpio_get_level(s_config.sensor_gpios[i]) == s_config.line_level)
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    *out_mask = mask;
    return ESP_OK;
}
