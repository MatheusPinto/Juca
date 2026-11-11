/**
 * @file    wheel.h
 * @brief   snake_case handle-based public interface for individual motorized wheel control.
 *
 * @details Provides a modular abstraction layer for controlling an individual motorized wheel 
 *          assembly on ESP-IDF targets. Each instance manages:
 *           - A Brushed DC (BDC) motor driven via MCPWM peripheral (Mandatory).
 *           - A quadrature encoder decoded via Pulse Counter (PCNT) peripheral (Optional: set pins to GPIO_NUM_NC to disable).
 *           - An ADC channel for motor current sensing (Optional: set adc_handle to NULL to disable).
 *
 * @note    All hardware parameters are dynamically configured per instance during creation 
 *          via the @ref wheel_config_t structure.
 *
 * @date    Sep 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/pcnt_periph.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *                       DEFAULT PWM CONFIGURATION
 * ==========================================================================*/

/** 
 * @brief Maximum PWM duty cycle value measured in timer ticks.
 * @details Calculated as (CONFIG_WHEEL_TIMER_RESOLUTION_HZ / CONFIG_WHEEL_PWM_FREQ_HZ).
 *          For 10 MHz / 25 kHz, the upper limit is 400 ticks.
 */
#define WHEEL_PWM_DUTY_TICK_MAX       (CONFIG_WHEEL_TIMER_RESOLUTION_HZ / CONFIG_WHEEL_PWM_FREQ_HZ)

/* ============================================================================
 *                     ENCODER PCNT LIMIT DEFINITIONS
 * ==========================================================================*/

/** 
 * @brief Lower pulse-count limit used by the PCNT hardware unit (Symmetric to #CONFIG_WHEEL_ENCODER_PCNT_HIGH_LIMIT).
 */
#define WHEEL_ENCODER_PCNT_LOW_LIMIT    (-CONFIG_WHEEL_ENCODER_PCNT_HIGH_LIMIT)

/* ============================================================================
 *                              DATA TYPES
 * ==========================================================================*/

/**
 * @brief Opaque handle representing an instantiated motorized wheel context.
 */
typedef struct wheel_t *wheel_handle_t;

/**
 * @brief Configuration structure for initializing an individual wheel instance.
 */
typedef struct {
    /* --- MCPWM Motor Driver Properties (Mandatory) --- */
    gpio_num_t pwm_a_gpio;            /**< Primary MCPWM output GPIO pin connected to H-Bridge driver. */
    gpio_num_t pwm_b_gpio;            /**< Secondary MCPWM output GPIO pin connected to H-Bridge driver. */
    uint32_t mcpwm_group_id;          /**< MCPWM hardware group ID on ESP32 (0 or 1). */
    int32_t max_power_ticks;          /**< Max input power magnitude allowed, in PWM duty ticks (must be <= WHEEL_PWM_DUTY_TICK_MAX). */

    /* --- PCNT Quadrature Encoder Pins (Optional) --- */
    gpio_num_t encoder_a_gpio;        /**< Quadrature Channel A GPIO pin (Set to GPIO_NUM_NC if unused). */
    gpio_num_t encoder_b_gpio;        /**< Quadrature Channel B GPIO pin (Set to GPIO_NUM_NC if unused). */

    /* --- ADC Current Sensing Properties (Optional) --- */
    adc_oneshot_unit_handle_t adc_handle; /**< Pre-initialized ADC unit handle managed by host application (NULL if unused). */
    adc_channel_t adc_channel;        /**< ADC channel mapped to current sensor for this wheel. */
} wheel_config_t;

/* ============================================================================
 *                              PUBLIC API
 * ==========================================================================*/

/**
 * @brief Instantiate and configure an individual motorized wheel instance.
 *
 * @details Performs the following sequence:
 *          1. Validates configured max power limits (before anything is allocated).
 *          2. Allocates heap memory for the wheel handle context.
 *          3. Configures and enables the BDC motor driver attached to the MCPWM peripheral.
 *          4. Conditionally configures the PCNT unit for 4x quadrature decoding if valid encoder pins are provided.
 *          5. Conditionally links the ADC channel for motor current monitoring.
 *          6. Applies active braking as an initial safety state.
 *
 * @param[in]  config    Pointer to wheel configuration structure.
 * @param[out] out_wheel Pointer to store initialized wheel handle.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Wheel successfully initialized.
 *         - ESP_ERR_INVALID_ARG: Invalid arguments (null pointers, or max_power_ticks outside [0, WHEEL_PWM_DUTY_TICK_MAX]).
 *         - ESP_ERR_NO_MEM: Memory allocation failed on heap.
 *         - Drivers error codes returned by ESP-IDF MCPWM or PCNT modules.
 */
esp_err_t wheel_create(const wheel_config_t *config, wheel_handle_t *out_wheel);

/**
 * @brief Set motor power and direction using a unified signed value.
 *
 * @details 
 *  - Positive values (`power_ticks > 0`): Command FORWARD rotation.
 *  - Negative values (`power_ticks < 0`): Command REVERSE rotation.
 *  - Zero (`power_ticks == 0`): Triggers active motor braking.
 * 
 * Magnitude is automatically bounded to `[-max_power_ticks, +max_power_ticks]`
 * configured during wheel instance creation.
 *
 * @param[in] wheel Target wheel handle.
 * @param[in] power_ticks Signed power command, in PWM duty ticks.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Command applied to PWM output successfully.
 *         - ESP_ERR_INVALID_ARG: Provided wheel handle is NULL.
 */
esp_err_t wheel_set_power(wheel_handle_t wheel, int32_t power_ticks);

/**
 * @brief Actively brake the motor driver outputs.
 *
 * @details Forces motor driver outputs into braking state (short-circuiting motor terminals 
 *          via H-bridge) and resets current power state.
 *
 * @param[in] wheel Target wheel handle.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Active brake applied successfully.
 *         - ESP_ERR_INVALID_ARG: Provided wheel handle is NULL.
 */
esp_err_t wheel_brake(wheel_handle_t wheel);

/**
 * @brief Read accumulated pulse counter count from quadrature encoder.
 *
 * @param[in]  wheel     Target wheel handle.
 * @param[out] out_count Pointer to store pulse count.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Pulse count retrieved successfully.
 *         - ESP_ERR_INVALID_ARG: Null pointer passed for handle or output.
 *         - ESP_ERR_INVALID_STATE: Encoder feature disabled on this wheel instance.
 */
esp_err_t wheel_get_encoder_count(wheel_handle_t wheel, int *out_count);

/**
 * @brief Reset the encoder accumulator count back to zero.
 *
 * @param[in] wheel Target wheel handle.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Count reset to zero successfully.
 *         - ESP_ERR_INVALID_ARG: Provided wheel handle is NULL.
 *         - ESP_ERR_INVALID_STATE: Encoder feature disabled on this wheel instance.
 */
esp_err_t wheel_clear_encoder_count(wheel_handle_t wheel);

/**
 * @brief Retrieve raw ADC current reading from motor sensor.
 *
 * @param[in]  wheel       Target wheel handle.
 * @param[out] out_raw_adc Pointer to store raw ADC value.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Raw ADC value retrieved successfully.
 *         - ESP_ERR_INVALID_ARG: Null pointer passed for handle or output.
 *         - ESP_ERR_INVALID_STATE: ADC current sensor feature disabled on this wheel instance.
 */
esp_err_t wheel_get_current_raw(wheel_handle_t wheel, int *out_raw_adc);

/**
 * @brief Retrieve the power limit configured for this wheel.
 *
 * @details This is the magnitude to which @ref wheel_set_power clamps every command. Callers that
 *          need a full-power command to actually be reached (e.g. calibration routines) must not ask
 *          for more than this value.
 *
 * @param[in]  wheel                Target wheel handle.
 * @param[out] out_max_power_ticks  Pointer to store the limit, in PWM duty ticks (0 .. WHEEL_PWM_DUTY_TICK_MAX).
 *
 * @return esp_err_t
 *         - ESP_OK: Limit retrieved successfully.
 *         - ESP_ERR_INVALID_ARG: Null pointer passed for handle or output.
 */
esp_err_t wheel_get_max_power_ticks(wheel_handle_t wheel, uint32_t *out_max_power_ticks);

/**
 * @brief Destroy wheel instance and release allocated hardware and system resources.
 *
 * @details Stops BDC motor driver, disables and deletes PCNT unit/channels (if allocated),
 *          and frees allocated heap memory. The shared ADC handle is NOT destroyed as it 
 *          belongs to the host application.
 *
 * @param[in] wheel Target wheel handle to delete.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Resources freed successfully (or handle was already NULL).
 */
esp_err_t wheel_delete(wheel_handle_t wheel);

#ifdef __cplusplus
}
#endif
