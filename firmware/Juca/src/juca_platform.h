/**
 * @file    juca_platform.h
 * @brief   Chassis and motor parameters of the platform Juca was developed and tested on.
 *
 * @details These values depend on the motors, encoders, wheels and chassis fitted to the board, so
 *          they are **not fixed**: they describe the tested platform. With other motors or a
 *          different chassis, change them here and run the calibration again. The pins and the
 *          parameters of the board itself are in @ref juca_board.h.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

/* ============================================================================
 *                          CHASSIS GEOMETRY
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
 *                          MOTOR DRIVE
 * ==========================================================================*/

/** @brief Duty cap of each motor, in PWM ticks (the full scale is WHEEL_PWM_DUTY_TICK_MAX = 400, so 400 is no cap). */
#define JUCA_MOTOR_MAX_POWER_TICKS      400
