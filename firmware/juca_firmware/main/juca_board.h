/**
 * @file    juca_board.h
 * @brief   Pin map and physical constants of the Juca robot (ESP32-S3).
 *
 * @details Single source of truth for the pins used by the examples in main/, and for the
 *          physical facts of the robot (geometry, encoder resolution, motor power cap and the
 *          current-sensor factor). Examples must not define their own GPIOs or these constants:
 *          include this header instead. Tuning choices (speed limits, ramps, task priorities,
 *          overcurrent threshold) are application decisions and stay in the examples.
 *
 *          Currently covers motors, encoders, current sensors and the IMU I2C bus. Still
 *          outside this file (they migrate together with their components): IR line
 *          follower (ir_line.h) and HC-SR04 (threeeyes.h).
 *
 * @note    Components under components/ must NOT include this header (it would invert
 *          the dependency direction). They receive pins and constants through a configuration
 *          struct, as the wheel and diff_drive components do.
 *
 * @date    Oct 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include "hal/gpio_types.h"
#include "hal/adc_types.h"
#include "hal/i2c_types.h"

/* ============================================================================
 *                          MOTORS (H-bridge, 2 PWM pins per motor)
 * ==========================================================================*/

/** @brief Left motor MCPWM output pin A. */
#define JUCA_PIN_MOTOR_LEFT_PWM_A       GPIO_NUM_12
/** @brief Left motor MCPWM output pin B. */
#define JUCA_PIN_MOTOR_LEFT_PWM_B       GPIO_NUM_13
/** @brief Right motor MCPWM output pin A. */
#define JUCA_PIN_MOTOR_RIGHT_PWM_A      GPIO_NUM_11
/** @brief Right motor MCPWM output pin B. */
#define JUCA_PIN_MOTOR_RIGHT_PWM_B      GPIO_NUM_10

/* ============================================================================
 *                          QUADRATURE ENCODERS
 * ==========================================================================*/

/** @brief Left encoder quadrature channel A pin. */
#define JUCA_PIN_ENCODER_LEFT_A         GPIO_NUM_7
/** @brief Left encoder quadrature channel B pin. */
#define JUCA_PIN_ENCODER_LEFT_B         GPIO_NUM_6
/** @brief Right encoder quadrature channel A pin. */
#define JUCA_PIN_ENCODER_RIGHT_A        GPIO_NUM_21
/** @brief Right encoder quadrature channel B pin. */
#define JUCA_PIN_ENCODER_RIGHT_B        GPIO_NUM_14

/* ============================================================================
 *                          CURRENT SENSORS (ADC)
 * ==========================================================================*/

/** @brief ADC unit used by the current sensors (ADC2 conflicts with Wi-Fi). */
#define JUCA_ADC_UNIT_CURRENT           ADC_UNIT_1
/** @brief ADC channel of the left motor current sensor (ADC1_CH8 = GPIO9 on ESP32-S3). */
#define JUCA_ADC_CHANNEL_CURRENT_LEFT   ADC_CHANNEL_8
/** @brief ADC channel of the right motor current sensor (ADC1_CH1 = GPIO2 on ESP32-S3). */
#define JUCA_ADC_CHANNEL_CURRENT_RIGHT  ADC_CHANNEL_1

/* ============================================================================
 *                          I2C BUS (MPU6050 IMU)
 * ==========================================================================*/

/** @brief I2C controller used by the IMU. */
#define JUCA_I2C_PORT_IMU               I2C_NUM_0
/**
 * @brief I2C data line (SDA) pin.
 * @note  Not GPIO 21: it is the right encoder channel A (@ref JUCA_PIN_ENCODER_RIGHT_A).
 *        The ESP32-S3 has no GPIO 22 either (its pins are GPIO0..21 and GPIO26..48).
 */
#define JUCA_PIN_I2C_SDA                GPIO_NUM_40
/** @brief I2C clock line (SCL) pin. */
#define JUCA_PIN_I2C_SCL                GPIO_NUM_39

/* ============================================================================
 *                          ROBOT GEOMETRY
 * ==========================================================================*/

/** @brief Half of the track width (L/2), in meters: 0.15 m puts the wheel ground contacts 0.30 m apart. */
#define JUCA_HALF_TRACK_WIDTH_M         0.15f
/** @brief Effective rolling radius of the drive wheels, in meters. */
#define JUCA_WHEEL_RADIUS_M             0.03f

/* ============================================================================
 *                          ENCODER RESOLUTION
 * ==========================================================================*/

/** @brief Left encoder counts per WHEEL revolution, 4x quadrature (11 pulses per motor turn x 4 x ~1:20.46 reduction). */
#define JUCA_ENCODER_CPR_LEFT           900
/** @brief Right encoder counts per WHEEL revolution, 4x quadrature. */
#define JUCA_ENCODER_CPR_RIGHT          900

/* ============================================================================
 *                          MOTOR DRIVE AND CURRENT SENSING
 * ==========================================================================*/

/** @brief Duty cap of each motor, in PWM ticks (the full scale is WHEEL_PWM_DUTY_TICK_MAX, 400 with the default wheel Kconfig). */
#define JUCA_MOTOR_MAX_POWER_TICKS      350
/**
 * @brief Conversion from a raw current-sensor ADC reading to milliamps (mA per count).
 * @note  Measured on the LEFT channel only, at 100% duty (docs/wheel_open_loop_tests.md, section 8.2).
 *        The right channel was never calibrated (and has a solder fault), so its value is not reliable.
 */
#define JUCA_CURRENT_MA_PER_COUNT       0.271f