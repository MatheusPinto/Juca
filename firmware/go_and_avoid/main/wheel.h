/**
 * @file    wheel.h
 * @brief   PascalCase handle-based public interface for individual motorized wheel control.
 *
 * @details Provides a modular abstraction layer for controlling an individual motorized wheel 
 *          assembly on ESP-IDF targets. Each instance manages:
 *           - A Brushed DC (BDC) motor driven via MCPWM peripheral (Mandatory).
 *           - A quadrature encoder decoded via Pulse Counter (PCNT) peripheral (Optional: set pins to GPIO_NUM_NC to disable).
 *           - An ADC channel for motor current sensing (Optional: set adc_handle to NULL to disable).
 *
 * @note    All hardware parameters are dynamically configured per instance during creation 
 *          via the @ref wheelConfig_t structure.
 *
 * @date    Sep 2026
 * @author  Matheus Leitezke Pinto
 */

#ifndef MAIN_WHEEL_H_
#define MAIN_WHEEL_H_

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
 * @brief MCPWM timer resolution in Hz (10 MHz = 0.1 us resolution per tick).
 */
#define WHEEL_TIMER_RESOLUTION_HZ   10000000

/** 
 * @brief Motor PWM switching frequency in Hz (25 kHz ultrasonic drive to eliminate audible noise).
 */
#define WHEEL_PWM_FREQ_HZ           25000

/** 
 * @brief Maximum PWM duty cycle value measured in timer ticks.
 * @details Calculated as (WHEEL_TIMER_RESOLUTION_HZ / WHEEL_PWM_FREQ_HZ).
 *          For 10 MHz / 25 kHz, the upper limit is 400 ticks.
 */
#define WHEEL_PWM_DUTY_TICK_MAX       (WHEEL_TIMER_RESOLUTION_HZ / WHEEL_PWM_FREQ_HZ)

/* ============================================================================
 *                     ENCODER PCNT LIMIT DEFINITIONS
 * ==========================================================================*/

/**
 * @brief Upper pulse-count limit used by the PCNT hardware unit before wrapping.
 *
 * @details Maximizing PCNT limits is critical for performance and driver validity:
 *          - Driver Requirement: Mandatory requirement by ESP-IDF driver (high_limit > 0 and low_limit < 0).
 *          - ISR Overhead Reduction: When accum_count is enabled, reaching these thresholds 
 *            dispatches an internal driver ISR to update a 64-bit hardware accumulator. 
 *            Maximizing limits minimizes ISR trigger frequency, reducing CPU overhead.
 *          - Overflow Prevention: Prevents hardware 16-bit counter saturation between polling cycles.
 */
#define WHEEL_ENCODER_PCNT_HIGH_LIMIT   7000

/** 
 * @brief Lower pulse-count limit used by the PCNT hardware unit (Symmetric to #WHEEL_ENCODER_PCNT_HIGH_LIMIT).
 */
#define WHEEL_ENCODER_PCNT_LOW_LIMIT    -WHEEL_ENCODER_PCNT_HIGH_LIMIT

/* ============================================================================
 *                              DATA TYPES
 * ==========================================================================*/

/**
 * @brief Opaque handle representing an instantiated motorized wheel context.
 */
typedef struct wheel_t *wheelHandle_t;

/**
 * @brief Configuration structure for initializing an individual wheel instance.
 */
typedef struct {
    /* --- MCPWM Motor Driver Properties (Mandatory) --- */
    gpio_num_t pwm_a_gpio;            /**< Primary MCPWM output GPIO pin connected to H-Bridge driver. */
    gpio_num_t pwm_b_gpio;            /**< Secondary MCPWM output GPIO pin connected to H-Bridge driver. */
    uint32_t mcpwm_group_id;          /**< MCPWM hardware group ID on ESP32 (0 or 1). */
    int32_t max_power_limit;          /**< Max input power magnitude allowed (Must be <= WHEEL_PWM_DUTY_TICK_MAX). */

    /* --- PCNT Quadrature Encoder Pins (Optional) --- */
    gpio_num_t encoder_a_gpio;        /**< Quadrature Channel A GPIO pin (Set to GPIO_NUM_NC if unused). */
    gpio_num_t encoder_b_gpio;        /**< Quadrature Channel B GPIO pin (Set to GPIO_NUM_NC if unused). */

    /* --- ADC Current Sensing Properties (Optional) --- */
    adc_oneshot_unit_handle_t adc_handle; /**< Pre-initialized ADC unit handle managed by host application (NULL if unused). */
    adc_channel_t adc_channel;        /**< ADC channel mapped to current sensor for this wheel. */
} wheelConfig_t;

/* ============================================================================
 *                              PUBLIC API
 * ==========================================================================*/

/**
 * @brief Instantiate and configure an individual motorized wheel instance.
 *
 * @details Performs the following sequence:
 *          1. Allocates heap memory for the wheel handle context.
 *          2. Validates configured max power limits.
 *          3. Configures and enables the BDC motor driver attached to the MCPWM peripheral.
 *          4. Conditionally configures the PCNT unit for 4x quadrature decoding if valid encoder pins are provided.
 *          5. Conditionally links the ADC channel for motor current monitoring.
 *          6. Applies active braking as an initial safety state.
 *
 * @param[in]  config    Pointer to wheel configuration structure.
 * @param[out] ret_wheel Pointer to store initialized wheel handle.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Wheel successfully initialized.
 *         - ESP_ERR_INVALID_ARG: Invalid arguments (null pointers or max_power_limit out of bounds).
 *         - ESP_ERR_NO_MEM: Memory allocation failed on heap.
 *         - Drivers error codes returned by ESP-IDF MCPWM or PCNT modules.
 */
esp_err_t Wheel_Create(const wheelConfig_t *config, wheelHandle_t *ret_wheel);

/**
 * @brief Set motor power and direction using a unified signed value.
 *
 * @details 
 *  - Positive values (`power > 0`): Command FORWARD rotation.
 *  - Negative values (`power < 0`): Command REVERSE rotation.
 *  - Zero (`power == 0`): Triggers active motor braking.
 * 
 * Magnitude is automatically bounded to `[-max_power_limit, +max_power_limit]`
 * configured during wheel instance creation.
 *
 * @param[in] wheel Target wheel handle.
 * @param[in] power Signed power command.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Command applied to PWM output successfully.
 *         - ESP_ERR_INVALID_ARG: Provided wheel handle is NULL.
 */
esp_err_t Wheel_SetPower(wheelHandle_t wheel, int32_t power);

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
esp_err_t Wheel_Brake(wheelHandle_t wheel);

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
esp_err_t Wheel_GetEncoderCount(wheelHandle_t wheel, int *out_count);

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
esp_err_t Wheel_ClearEncoderCount(wheelHandle_t wheel);

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
esp_err_t Wheel_GetCurrentRaw(wheelHandle_t wheel, int *out_raw_adc);

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
esp_err_t Wheel_Delete(wheelHandle_t wheel);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_WHEEL_H_ */