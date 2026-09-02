/**
 * @file    wheel.h
 * @brief   PascalCase handle-based public interface for individual motorized wheel control.
 *
 * Provides a modular API for controlling an individual motorized wheel assembly containing:
 *  - Brushed DC (BDC) motor driven by MCPWM (Mandatory)
 *  - Quadrature encoder decoded via Pulse Counter (PCNT) peripheral (Optional: set pins to GPIO_NUM_NC to disable)
 *  - ADC channel for motor current sensing (Optional: set adc_handle to NULL to disable)
 *
 * All hardware parameters are dynamically configured per instance during creation via Wheel_Config_t.
 *
 * @date    Jan 2025
 * @author  Matheus
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
 *                       DEFAULT FALLBACK MACROS
 * ==========================================================================*/

/** Default MCPWM timer resolution in Hz if unspecified (10 MHz = 0.1 us resolution). */
#define WHEEL_TIMER_RESOLUTION_HZ   10000000

/** Default PWM switching frequency in Hz if unspecified (25 kHz ultrasonic drive). */
#define WHEEL_PWM_FREQ_HZ           25000

/** Maximum value that can be set for the PWM duty cycle, in timer ticks. */
#define WHEEL_PWM_DUTY_TICK_MAX       (WHEEL_TIMER_RESOLUTION_HZ / WHEEL_PWM_FREQ_HZ)

/* ============================================================================
 *                              DATA TYPES
 * ==========================================================================*/

/**
 * @brief Opaque handle representing an instantiated motorized wheel context.
 */
typedef struct Wheel_t *Wheel_Handle_t;

/**
 * @brief Operational direction modes for an individual wheel.
 */
typedef enum {
    WHEEL_STOP = 0, /**< Motor braked / disabled. */
    WHEEL_FORWARD,  /**< Motor driven in forward rotation. */
    WHEEL_REVERSE   /**< Motor driven in reverse rotation. */
} Wheel_Dir_t;

/**
 * @brief Configuration structure for initializing an individual wheel instance.
 */
typedef struct {
    /* --- MCPWM Motor Driver Properties (Mandatory) --- */
    gpio_num_t pwm_a_gpio;            /**< Primary MCPWM output GPIO pin. */
    gpio_num_t pwm_b_gpio;            /**< Secondary MCPWM output GPIO pin. */
    uint32_t mcpwm_group_id;          /**< MCPWM hardware group ID (0 or 1). */
    uint32_t mcpwm_resolution_hz;     /**< Timer resolution in Hz (0 defaults to 10 MHz). */
    uint32_t pwm_freq_hz;             /**< Switching frequency in Hz (0 defaults to 25 kHz). */
    int32_t max_power_limit;          /**< Max power input magnitude allowed (0 defaults to max duty ticks). */

    /* --- PCNT Quadrature Encoder Pins & Boundaries (Optional) --- */
    gpio_num_t encoder_a_gpio;        /**< Quadrature Channel A GPIO pin (Set to GPIO_NUM_NC if unused). */
    gpio_num_t encoder_b_gpio;        /**< Quadrature Channel B GPIO pin (Set to GPIO_NUM_NC if unused). */
    int16_t pcnt_high_limit;          /**< Upper hardware counter bound (0 defaults to INT16_MAX). */
    int16_t pcnt_low_limit;           /**< Lower hardware counter bound (0 defaults to INT16_MIN). */

    /* --- ADC Current Sensing Properties (Optional) --- */
    adc_oneshot_unit_handle_t adc_handle; /**< Pre-initialized ADC unit handle (NULL if unused). */
    adc_channel_t adc_channel;        /**< ADC channel mapped to current sensor. */
} Wheel_Config_t;

/* ============================================================================
 *                              PUBLIC API
 * ==========================================================================*/

/**
 * @brief Instantiate and configure an individual motorized wheel instance.
 *
 * Allocates internal context, configures custom MCPWM resolution and frequency,
 * sets up optional PCNT quadrature decoding, and links optional ADC current sensing.
 *
 * @param[in]  config    Pointer to wheel configuration structure.
 * @param[out] ret_wheel Pointer to store initialized wheel handle.
 * @return esp_err_t     ESP_OK on success, or appropriate ESP-IDF driver error code.
 */
esp_err_t Wheel_New(const Wheel_Config_t *config, Wheel_Handle_t *ret_wheel);

/**
 * @brief Set explicit rotation direction and raw PWM duty cycle ticks.
 *
 * Values exceeding the instance's calculated max duty ticks are automatically clamped.
 *
 * @param wheel    Target wheel handle.
 * @param dir      Desired direction (WHEEL_FORWARD, WHEEL_REVERSE, or WHEEL_STOP).
 * @param pwm_duty Duty cycle in timer ticks.
 * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t Wheel_SetDutyCycle(Wheel_Handle_t wheel, Wheel_Dir_t dir, uint32_t pwm_duty);

/**
 * @brief Set motor power and direction using a unified signed value.
 *
 * Positive values select forward direction; negative values select reverse.
 * Magnitude represents requested power bounded to [-max_power_limit, +max_power_limit].
 *
 * @param wheel  Target wheel handle.
 * @param power  Signed power command.
 * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t Wheel_SetPower(Wheel_Handle_t wheel, int32_t power);

/**
 * @brief Actively brake the motor driver outputs.
 *
 * @param wheel Target wheel handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t Wheel_Brake(Wheel_Handle_t wheel);

/**
 * @brief Read current pulse counter value accumulated from the quadrature encoder.
 *
 * @param[in]  wheel     Target wheel handle.
 * @param[out] out_count Pointer to store pulse count.
 * @return esp_err_t     ESP_OK on success, or ESP_ERR_INVALID_STATE if encoder is disabled.
 */
esp_err_t Wheel_GetEncoderCount(Wheel_Handle_t wheel, int *out_count);

/**
 * @brief Reset the encoder accumulator count back to zero.
 *
 * @param wheel Target wheel handle.
 * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_STATE if encoder is disabled.
 */
esp_err_t Wheel_ClearEncoderCount(Wheel_Handle_t wheel);

/**
 * @brief Retrieve raw ADC current reading from motor sensor.
 *
 * @param[in]  wheel       Target wheel handle.
 * @param[out] out_raw_adc Pointer to store raw ADC value.
 * @return esp_err_t       ESP_OK on success, or ESP_ERR_INVALID_STATE if ADC is disabled.
 */
esp_err_t Wheel_GetCurrentRaw(Wheel_Handle_t wheel, int *out_raw_adc);

/**
 * @brief Destroy wheel instance and release allocated hardware and system resources.
 *
 * @param wheel Target wheel handle to delete.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t Wheel_Del(Wheel_Handle_t wheel);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_WHEEL_H_ */