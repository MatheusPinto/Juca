/**
 * @file    wheel.h
 * @brief   Public interface of the differential drive wheel control module.
 *
 * Declares the configuration macros, data types and public API used to
 * initialize and drive the left/right DC motors, read their quadrature
 * encoders and monitor their current consumption.
 *
 * @date    Jan 9, 2025
 * @author  Matheus
 */

#ifndef MAIN_WHEEL_H_
#define MAIN_WHEEL_H_

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* ============================================================================
 *                          MCPWM / PWM CONFIGURATION
 * ==========================================================================*/

/** MCPWM timer resolution, in Hz (10 MHz -> 1 tick = 0.1 us). */
#define WHEEL_TIMER_RESOLUTION_HZ 10000000
/** PWM switching frequency applied to the motor drivers, in Hz. */
#define WHEEL_PWM_FREQ_HZ             25000
/** Maximum value that can be set for the PWM duty cycle, in timer ticks. */
#define WHEEL_PWM_DUTY_TICK_MAX       (WHEEL_TIMER_RESOLUTION_HZ / WHEEL_PWM_FREQ_HZ)

/* ============================================================================
 *                          MOTOR DRIVER GPIO PINS
 * ==========================================================================*/

/** Left motor MCPWM output pin A. */
#define WHEEL_LEFT_PWM_GPIO_A          GPIO_NUM_12
/** Left motor MCPWM output pin B. */
#define WHEEL_LEFT_PWM_GPIO_B          GPIO_NUM_13
/** Right motor MCPWM output pin A. */
#define WHEEL_RIGHT_PWM_GPIO_A         GPIO_NUM_11
/** Right motor MCPWM output pin B. */
#define WHEEL_RIGHT_PWM_GPIO_B         GPIO_NUM_10

#define WHEEL_POWER_MAX WHEEL_PWM_DUTY_TICK_MAX

/* ============================================================================
 *                          ENCODER GPIO PINS
 * ==========================================================================*/

/** Left encoder quadrature channel A pin. */
#define WHEEL_ENCODER_LEFT_GPIO_A        GPIO_NUM_7
/** Left encoder quadrature channel B pin. */
#define WHEEL_ENCODER_LEFT_GPIO_B        GPIO_NUM_6
/** Right encoder quadrature channel A pin. */
#define WHEEL_ENCODER_RIGHT_GPIO_A       GPIO_NUM_21
/** Right encoder quadrature channel B pin. */
#define WHEEL_ENCODER_RIGHT_GPIO_B       GPIO_NUM_14

/* ============================================================================
 *                          ENCODER PCNT LIMITS
 * ==========================================================================*/

/**
 * Upper pulse-count limit used by the PCNT unit.
 *
 * Maximum limits are critical for performance and driver validity:
 *       - Driver Initialization: Mandatory requirement (high_limit > 0 and low_limit < 0).
 *       - Internal ISR Overhead: When accum_count is enabled, reaching these thresholds 
 *         dispatches an internal driver ISR to update the 64-bit accumulator. Maximizing 
 *         the limits minimizes ISR trigger frequency, reducing CPU usage.
 *       - Wrap-Around Prevention: Prevents premature hardware counter saturation between 
 *         periodic polling cycles.
 */
#define WHEEL_ENCODER_PCNT_HIGH_LIMIT   7000
/** Lower pulse-count limit used by the PCNT unit (see #WHEEL_ENCODER_PCNT_HIGH_LIMIT). */
#define WHEEL_ENCODER_PCNT_LOW_LIMIT    -WHEEL_ENCODER_PCNT_HIGH_LIMIT

/* ============================================================================
 *                          ROBOT / WHEEL PHYSICAL PARAMETERS
 * ==========================================================================*/

/** Half of the robot's wheel axis length (track width), in cm. */
#define WHEEL_AXIS_LENGHT_2 10
/** Encoder pulses per revolution (PPR) of each wheel. */
#define WHEELS_ENCODER_PPR 900
/** Wheel radius, in meters (33 cm). */
#define WHELL_RADIUS 0.033
/** ADC values from current sensor that brake the motors and the release value to motor to go on */
#define WHEEL_STALL_LIMIT_VALUE  2500
#define WHEEL_MOTOR_RELEASE_LIMIT_VALUE 1500
#define WHEEL_MOTOR_STALL_TIME_VALUE 10
#define WHEEL_POWER_TRACKER_TASK_BASE_PERIOD 200
#define WHEEL_SPEED_CTRL_TASK_PERIOD 20 /*50 Hz*/


/* ============================================================================
 *                              DATA TYPES
 * ==========================================================================*/

/**
 * @brief Aggregates the handles used to control and monitor a single motor.
 */
typedef struct {
    bdc_motor_handle_t motor;          /**< Motor driver handle. */
    pcnt_unit_handle_t pcnt_encoder;   /**< Pulse counter unit handle for this motor's encoder. */
    int report_pulses;                 /**< Last reported encoder pulse count. */
} motor_control_context_t;

/**
 * @brief Rotation direction of a wheel.
 */
typedef enum {
    WHEEL_STOP = 0, /**< Wheel stopped/braked. */
    WHEEL_FORWARD,  /**< Wheel spinning forward. */
    WHEEL_REVERSE   /**< Wheel spinning in reverse. */
} wheel_dir_t;

/**
 * @brief Command describing the desired direction and PWM duty cycle for
 *        both wheels.
 */
typedef struct {
    wheel_dir_t dir_left;   /**< Desired direction for the left wheel. */
    wheel_dir_t dir_right;  /**< Desired direction for the right wheel. */
    uint32_t pwm_left;      /**< Desired PWM duty cycle for the left wheel. */
    uint32_t pwm_right;     /**< Desired PWM duty cycle for the right wheel. */
} wheel_cmd_t;

/* ============================================================================
 *                              PUBLIC API
 * ==========================================================================*/

/**
 * @brief Initialize the wheel subsystem (ADC, motors and encoders).
 *
 * Must be called once before any other function in this module is used.
 *
 * @return int 1 on success (errors abort execution via ESP_ERROR_CHECK).
 */
int wheel_Init( void );

/**
 * @brief Set the raw direction and PWM duty cycle for both wheels.
 *
 * The provided PWM values are clamped to the maximum allowed duty cycle,
 * stored as the current command, and immediately applied to the motors.
 * 
 * @note To minimize overhead and latency, this function is not thread-safe.
 *       Any concurrent access must be handled at the application level
 *       (e.g., using a gatekeeper task or a mutex).
 * 
 * @note To control only one wheel, the caller must explicitly pass the current
 *       state (direction and PWM) of the other wheel to preserve its operation.
 *
 * @param dir_left  Desired direction for the left wheel.
 * @param pwm_left  Desired PWM duty cycle for the left wheel.
 * @param dir_right Desired direction for the right wheel.
 * @param pwm_right Desired PWM duty cycle for the right wheel.
 */
void wheel_SetDutyCycle(
    wheel_dir_t dir_left, uint32_t pwm_left,
    wheel_dir_t dir_right, uint32_t pwm_right);

/**
 * @brief Set the power and direction for both wheels using signed power values.
 *
 * Positive values set the wheel direction to forward, while negative values set it to
 * backward. The magnitude of the value represents the PWM duty cycle, which is clamped
 * to the maximum allowed limit.
 *
 * @note To minimize overhead and latency, this function is not thread-safe.
 *       Any concurrent access must be handled at the application level
 *       (e.g., using a gatekeeper task or a mutex).
 *
 * @note To control only one wheel, the caller must explicitly pass the current
 *       power value of the other wheel to preserve its operation.
 *
 * @param power_left  Signed power for left wheel (-WHEEL_POWER_MAX to WHEEL_POWER_MAX).
 * @param power_right Signed power for right wheel (-WHEEL_POWER_MAX to WHEEL_POWER_MAX).
 */
void wheel_SetPower(int32_t power_left, int32_t power_right);

/**
 * @brief Read the current motor power (raw ADC current readings).
 *
 * @param[out] pL Pointer where the left motor's raw ADC reading is stored.
 * @param[out] pR Pointer where the right motor's raw ADC reading is stored.
 */
void wheel_GetPower( uint32_t *pL, uint32_t *pR );

/**
 * @brief Read the current encoder pulse counts for both wheels.
 *
 * @param[out] pL Pointer where the left encoder pulse count is stored.
 * @param[out] pR Pointer where the right encoder pulse count is stored.
 */
void wheel_GetEndoderPulses( int *pL, int *pR );

#endif /* MAIN_WHEEL_H_ */