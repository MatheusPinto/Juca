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
#include "pid_ctrl.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* ============================================================================
 *                          MCPWM / PWM CONFIGURATION
 * ==========================================================================*/

/** MCPWM timer resolution, in Hz (10 MHz -> 1 tick = 0.1 us). */
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000
/** PWM switching frequency applied to the motor drivers, in Hz. */
#define BDC_MCPWM_FREQ_HZ             25000
/** Maximum value that can be set for the PWM duty cycle, in timer ticks. */
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ)

/* ============================================================================
 *                          MOTOR DRIVER GPIO PINS
 * ==========================================================================*/

/** Left motor MCPWM output pin A. */
#define BDC_LEFT_MCPWM_GPIO_A          GPIO_NUM_12
/** Left motor MCPWM output pin B. */
#define BDC_LEFT_MCPWM_GPIO_B          GPIO_NUM_13
/** Right motor MCPWM output pin A. */
#define BDC_RIGHT_MCPWM_GPIO_A         GPIO_NUM_11
/** Right motor MCPWM output pin B. */
#define BDC_RIGHT_MCPWM_GPIO_B         GPIO_NUM_10

/* ============================================================================
 *                          ENCODER GPIO PINS
 * ==========================================================================*/

/** Left encoder quadrature channel A pin. */
#define BDC_ENCODER_LEFT_GPIO_A        GPIO_NUM_7
/** Left encoder quadrature channel B pin. */
#define BDC_ENCODER_LEFT_GPIO_B        GPIO_NUM_6
/** Right encoder quadrature channel A pin. */
#define BDC_ENCODER_RIGHT_GPIO_A       GPIO_NUM_21
/** Right encoder quadrature channel B pin. */
#define BDC_ENCODER_RIGHT_GPIO_B       GPIO_NUM_14

/* ============================================================================
 *                          ENCODER PCNT LIMITS
 * ==========================================================================*/

/**
 * Upper pulse-count limit used by the PCNT unit.
 *
 * These limit values can be used to auto-reset the count and/or to raise
 * an interrupt when the limit is reached. Here they are irrelevant, since
 * neither auto-reset nor the watch-point interrupt is actually used.
 */
#define BDC_ENCODER_PCNT_HIGH_LIMIT   7000
/** Lower pulse-count limit used by the PCNT unit (see #BDC_ENCODER_PCNT_HIGH_LIMIT). */
#define BDC_ENCODER_PCNT_LOW_LIMIT    -7000

/* ============================================================================
 *                          SPEED CONTROL (PID) PARAMETERS
 * ==========================================================================*/

/** Speed control loop period: the motor speed is (re)computed at this rate, in ms. */
#define BDC_PID_LOOP_PERIOD_MS        100
/** Target motor speed, expressed in encoder pulses counted per control period. */
#define BDC_PID_EXPECT_SPEED          400

/* ============================================================================
 *                          ROBOT / WHEEL PHYSICAL PARAMETERS
 * ==========================================================================*/

/** Half of the robot's wheel axis length (track width), in cm. */
#define WHEEL_AXIS_LENGHT_2 10
/** Encoder pulses per revolution (PPR) of each wheel. */
#define WHEELS_ENCODER_PPR 900
/** Wheel radius, in meters (33 cm). */
#define WHELL_RADIUS 0.033

/** Maximum allowed PWM duty cycle value (currently the MCPWM tick resolution). */
#define PWM_MAX BDC_MCPWM_DUTY_TICK_MAX

/* ============================================================================
 *                              DATA TYPES
 * ==========================================================================*/

/**
 * @brief Aggregates the handles used to control and monitor a single motor.
 */
typedef struct {
    bdc_motor_handle_t motor;          /**< Motor driver handle. */
    pcnt_unit_handle_t pcnt_encoder;   /**< Pulse counter unit handle for this motor's encoder. */
    pid_ctrl_block_handle_t pid_ctrl;  /**< PID control block handle used for closed-loop speed control. */
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
 * @param dir_left  Desired direction for the left wheel.
 * @param pwm_left  Desired PWM duty cycle for the left wheel.
 * @param dir_right Desired direction for the right wheel.
 * @param pwm_right Desired PWM duty cycle for the right wheel.
 */
void wheel_SetRawSpeed(
    wheel_dir_t dir_left, uint32_t pwm_left,
    wheel_dir_t dir_right, uint32_t pwm_right);

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