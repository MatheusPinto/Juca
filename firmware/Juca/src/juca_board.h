/**
 * @file    juca_board.h
 * @brief   Pin map and fixed parameters of the Juca board (ESP32-S3).
 *
 * @details Single source of truth for the pins of the robot. These do not change with the motors or
 *          the chassis; the parameters that do are in @ref juca_platform.h. Sketches should use these
 *          names instead of GPIO numbers.
 *
 *          Same content as juca_board.h of the Juca ESP-IDF firmware, with one difference: the
 *          current sensors are given as pins (for the Arduino analogRead()) instead of ADC channels.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include "hal/gpio_types.h"
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
/** @brief MCPWM group that generates the PWM of both motors. */
#define JUCA_MCPWM_GROUP_MOTORS         0

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

/** @brief Analog pin of the left motor current sensor (ADC1 channel 8). */
#define JUCA_PIN_CURRENT_LEFT           GPIO_NUM_9
/** @brief Analog pin of the right motor current sensor (ADC1 channel 1). */
#define JUCA_PIN_CURRENT_RIGHT          GPIO_NUM_2
/**
 * @brief Conversion from a raw current-sensor ADC reading to milliamps (mA per count).
 * @note  Measured on the left channel at 100% duty cycle.
 */
#define JUCA_CURRENT_MA_PER_COUNT       0.271f

/* ============================================================================
 *                          I2C BUS (MPU6050 IMU)
 * ==========================================================================*/

/** @brief I2C controller used by the IMU (I2C_NUM_0 is the Arduino `Wire` object). */
#define JUCA_I2C_PORT_IMU               I2C_NUM_0
/** @brief I2C data line (SDA) pin. */
#define JUCA_PIN_I2C_SDA                GPIO_NUM_40
/** @brief I2C clock line (SCL) pin. */
#define JUCA_PIN_I2C_SCL                GPIO_NUM_39

/* ============================================================================
 *                          HC-SR04 ULTRASONIC SENSORS (ThreeEyes)
 * ==========================================================================*/

/** @brief Trigger pin, shared by the three sensors. */
#define JUCA_PIN_HCSR04_TRIG            GPIO_NUM_42
/** @brief ECHO pin of the left sensor. */
#define JUCA_PIN_HCSR04_ECHO_LEFT       GPIO_NUM_5
/** @brief ECHO pin of the middle sensor. */
#define JUCA_PIN_HCSR04_ECHO_MIDDLE     GPIO_NUM_4
/** @brief ECHO pin of the right sensor. */
#define JUCA_PIN_HCSR04_ECHO_RIGHT      GPIO_NUM_1
/** @brief MCPWM group whose capture timer measures the echoes (the motors use the PWM timers of the same group). */
#define JUCA_MCPWM_GROUP_HCSR04         0

/* ============================================================================
 *                          INFRARED LINE SENSORS (5 digital sensors, left to right)
 * ==========================================================================*/

/** @brief Far left line sensor (seen from the robot, looking forward). */
#define JUCA_PIN_IR_LINE_FAR_LEFT       GPIO_NUM_15
/** @brief Center-left line sensor. */
#define JUCA_PIN_IR_LINE_LEFT           GPIO_NUM_16
/** @brief Middle line sensor. */
#define JUCA_PIN_IR_LINE_MIDDLE         GPIO_NUM_17
/** @brief Center-right line sensor. */
#define JUCA_PIN_IR_LINE_RIGHT          GPIO_NUM_18
/** @brief Far right line sensor. */
#define JUCA_PIN_IR_LINE_FAR_RIGHT      GPIO_NUM_8
/**
 * @brief Level (0 or 1) that a line sensor outputs when it is over the line.
 * @note  0 is for the tested track: a black line on a light floor. Use 1 if your sensors, or a
 *        light line on a dark floor, give the opposite.
 */
#define JUCA_IR_LINE_LEVEL_ON_LINE      0
