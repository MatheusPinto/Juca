/**
 * @file    juca_board.h
 * @brief   Pin map and fixed parameters of the Juca board (ESP32-S3).
 *
 * @details Single source of truth for the pins used by the examples in main/, and for the parameters
 *          of the board itself: the ADC unit and channels of the current sensors, the I2C bus of the
 *          IMU and the conversion factor of the current-sensing circuit (shunt and amplifier).
 *          These do not change with the motors or the chassis. Examples must not define their own
 *          GPIOs or these constants: include this header instead.
 *
 *          The parameters that depend on the motors, encoders, wheels and chassis fitted to the
 *          board (geometry, encoder resolution, motor power cap) are in @ref juca_platform.h.
 *
 *          Currently covers the motor driver, the encoder inputs, the current sensors and the IMU
 *          I2C bus. Still outside this file (they migrate together with their components): IR line
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
 *                          CURRENT SENSORS (shunt and amplifier, read by the ADC)
 * ==========================================================================*/

/** @brief ADC unit used by the current sensors (ADC2 conflicts with Wi-Fi). */
#define JUCA_ADC_UNIT_CURRENT           ADC_UNIT_1
/** @brief ADC channel of the left motor current sensor (ADC1_CH8 = GPIO9 on ESP32-S3). */
#define JUCA_ADC_CHANNEL_CURRENT_LEFT   ADC_CHANNEL_8
/** @brief ADC channel of the right motor current sensor (ADC1_CH1 = GPIO2 on ESP32-S3). */
#define JUCA_ADC_CHANNEL_CURRENT_RIGHT  ADC_CHANNEL_1
/**
 * @brief Conversion from a raw current-sensor ADC reading to milliamps (mA per count).
 * @note  Measured on the left channel at 100% duty cycle.
 */
#define JUCA_CURRENT_MA_PER_COUNT       0.271f

/* ============================================================================
 *                          I2C BUS (MPU6050 IMU)
 * ==========================================================================*/

/** @brief I2C controller used by the IMU. */
#define JUCA_I2C_PORT_IMU               I2C_NUM_0
/** @brief I2C data line (SDA) pin. */
#define JUCA_PIN_I2C_SDA                GPIO_NUM_40
/** @brief I2C clock line (SCL) pin. */
#define JUCA_PIN_I2C_SCL                GPIO_NUM_39