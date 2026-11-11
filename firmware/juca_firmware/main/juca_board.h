/**
 * @file    juca_board.h
 * @brief   Pin and hardware channel map for the Juca robot (ESP32-S3).
 *
 * @details Single source of truth for the pins used by the examples in main/.
 *          Examples must not define their own GPIOs: include this header instead.
 *
 *          Currently covers motors, encoders, current sensors and the IMU I2C bus. Still
 *          outside this file (they migrate together with their components): IR line
 *          follower (ir_line.h) and HC-SR04 (threeeyes.h).
 *
 * @note    Components under components/ must NOT include this header (it would invert
 *          the dependency direction). They receive pins through a configuration struct,
 *          as the wheel component does.
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
/** @brief Left encoder counts per revolution. */
#define JUCA_ENCODER_LEFT_CPR         900
/** @brief Right encoder counts per revolution. */
#define JUCA_ENCODER_RIGHT_CPR         900
/** @brief Radius of both drive wheels (r), in meters. */
#define JUCA_WHEEL_RADIUS_M    0.03f
/** @brief HALF of the track width (L/2) in meters: distance from the robot centerline to each wheel ground contact. */
#define JUCA_HALF_TRACK_WIDTH_M    0.15f



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
/** @brief I2C data line (SDA) pin. */
#define JUCA_PIN_I2C_SDA                GPIO_NUM_40
/** @brief I2C clock line (SCL) pin. */
#define JUCA_PIN_I2C_SCL                GPIO_NUM_39
