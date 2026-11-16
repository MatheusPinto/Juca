/**
 * @file    threeeyes.h
 * @brief   Public interface of the ThreeEyes component: three HC-SR04 ultrasonic sensors read in parallel.
 *
 * @details The three sensors share ONE trigger pin, so they all fire at the same instant. Each echo pin
 *          goes to its own MCPWM capture channel, which timestamps the rising and the falling edge of the
 *          echo pulse in hardware. The width of that pulse is the time of flight (ToF) of the sound, there
 *          and back.
 *
 *          The component is a singleton (one set of three sensors) and knows nothing about the board: the
 *          pins and the MCPWM group arrive through @ref threeeyes_config_t.
 *
 * @note    Only one task may use the measurement functions: @ref threeeyes_trigger_and_wait uses the task
 *          notification (index 0) of the calling task to wait for the echoes.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *                                 CONSTANTS
 * ==========================================================================*/

/**
 * @brief Round-trip time of sound per centimeter of distance, in microseconds.
 *
 * @details Sound travels at about 343 m/s (air, 20 degrees C), so 1 cm there and back (2 cm) takes
 *          2 cm / 34300 cm/s = 58.3 us. The HC-SR04 datasheet rounds it to 58.
 */
#define THREEEYES_US_PER_CM    58.0f

/* ============================================================================
 *                              DATA TYPES
 * ==========================================================================*/

/**
 * @brief Configuration of the three sensors. Every field is mandatory.
 */
typedef struct {
    gpio_num_t trig_gpio;         /**< Output pin wired to the TRIG input of ALL three sensors. */
    gpio_num_t echo_left_gpio;    /**< Input pin wired to the ECHO output of the left sensor. */
    gpio_num_t echo_middle_gpio;  /**< Input pin wired to the ECHO output of the middle sensor. */
    gpio_num_t echo_right_gpio;   /**< Input pin wired to the ECHO output of the right sensor. */
    int mcpwm_group_id;           /**< MCPWM group that provides the capture timer (0 or 1 on the ESP32-S3). It may be the same group that drives the motors: the capture timer is independent of the PWM timers. */
} threeeyes_config_t;

/**
 * @brief Result of one sensor for the last trigger.
 */
typedef struct {
    uint32_t tof_ticks;  /**< Width of the last echo pulse, in capture timer ticks. Convert with @ref threeeyes_convert_ticks_to_cm. Only meaningful when `is_updated` is true. */
    bool is_updated;     /**< true if this sensor's echo ended before @ref threeeyes_trigger_and_wait returned. */
} threeeyes_value_t;

/* ============================================================================
 *                              PUBLIC API
 * ==========================================================================*/

/**
 * @brief Configures the trigger pin and the three MCPWM capture channels.
 *
 * @details Creates the capture timer, one capture channel per echo pin (both edges), registers the edge
 *          callback, configures the trigger pin as an output at low level and starts the timer. If any
 *          step fails, everything created so far is released before returning.
 *
 * @param[in] config Pins and MCPWM group. Must not be NULL.
 *
 * @return esp_err_t
 *         - ESP_OK: Sensors ready.
 *         - ESP_ERR_INVALID_ARG: `config` is NULL.
 *         - ESP_ERR_INVALID_STATE: Already initialized.
 *         - Other: error of the MCPWM or GPIO driver (nothing is left allocated).
 */
esp_err_t threeeyes_init(const threeeyes_config_t *config);

/**
 * @brief Fires the three sensors and blocks the calling task until the FIRST echo ends, or the timeout expires.
 *
 * @details Sends a 10 us pulse on the trigger pin and waits. The function returns as soon as one sensor
 *          finishes its echo: that is the sensor with the NEAREST object. Sensors whose echo is still in
 *          flight at that moment are reported with `is_updated == false` by @ref threeeyes_read_values,
 *          even if they do see an object farther away.
 *
 *          A sensor with nothing in range still answers, with a long echo of about 38 ms. So with at
 *          least one sensor connected this function returns within about 38 ms.
 *
 * @param[in] timeout_ticks Longest wait, in RTOS ticks. Use at least 60 ms (the HC-SR04 measurement cycle).
 *
 * @return esp_err_t
 *         - ESP_OK: At least one sensor answered.
 *         - ESP_ERR_TIMEOUT: No echo arrived (sensors unplugged or unpowered). All `is_updated` flags are false.
 *         - ESP_ERR_INVALID_STATE: @ref threeeyes_init was not called.
 */
esp_err_t threeeyes_trigger_and_wait(TickType_t timeout_ticks);

/**
 * @brief Copies the result of the last trigger for the three sensors.
 *
 * @param[out] left   Result of the left sensor. Must not be NULL.
 * @param[out] middle Result of the middle sensor. Must not be NULL.
 * @param[out] right  Result of the right sensor. Must not be NULL.
 *
 * @return esp_err_t
 *         - ESP_OK: Values copied.
 *         - ESP_ERR_INVALID_ARG: A NULL pointer.
 */
esp_err_t threeeyes_read_values(threeeyes_value_t *left, threeeyes_value_t *middle, threeeyes_value_t *right);

/**
 * @brief Converts an echo width from capture timer ticks to a distance in centimeters.
 *
 * @details distance_cm = (tof_ticks / timer_resolution_hz) * 1e6 / @ref THREEEYES_US_PER_CM.
 *          On the ESP32-S3 the capture timer runs at 80 MHz, so one tick is 12.5 ns (0.0002 cm).
 *          An echo of about 38 ms (about 650 cm) means "nothing in range", not an object at 6.5 m.
 *
 * @param[in] tof_ticks Echo width from @ref threeeyes_value_t.
 * @return float Distance in centimeters, or 0.0f before @ref threeeyes_init.
 */
float threeeyes_convert_ticks_to_cm(uint32_t tof_ticks);

/**
 * @brief Stops capturing the echo of the left sensor (it then never reports `is_updated`).
 * @return esp_err_t ESP_OK, ESP_ERR_INVALID_STATE (not initialized or already disabled), or an MCPWM driver error.
 */
esp_err_t threeeyes_disable_left(void);

/**
 * @brief Stops capturing the echo of the middle sensor (it then never reports `is_updated`).
 * @return esp_err_t ESP_OK, ESP_ERR_INVALID_STATE (not initialized or already disabled), or an MCPWM driver error.
 */
esp_err_t threeeyes_disable_middle(void);

/**
 * @brief Stops capturing the echo of the right sensor (it then never reports `is_updated`).
 * @return esp_err_t ESP_OK, ESP_ERR_INVALID_STATE (not initialized or already disabled), or an MCPWM driver error.
 */
esp_err_t threeeyes_disable_right(void);

#ifdef __cplusplus
}
#endif
