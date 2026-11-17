/**
 * @file    juca_config.h
 * @brief   Build-time options of the wheel driver (Arduino replacement for the ESP-IDF menuconfig).
 *
 * @details In the ESP-IDF firmware these values come from `idf.py menuconfig` (Wheel Driver
 *          Configuration). Arduino has no menuconfig, so the same names are defined here with the
 *          same defaults. Each one can be overridden with a compiler flag (-DCONFIG_WHEEL_...=value).
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

/** @brief MCPWM timer resolution, in Hz (10 MHz = 0.1 us per tick). */
#ifndef CONFIG_WHEEL_TIMER_RESOLUTION_HZ
#define CONFIG_WHEEL_TIMER_RESOLUTION_HZ        10000000
#endif

/** @brief Motor PWM frequency, in Hz (25 kHz is above the audible range). With the resolution above, full duty is 400 ticks. */
#ifndef CONFIG_WHEEL_PWM_FREQ_HZ
#define CONFIG_WHEEL_PWM_FREQ_HZ                25000
#endif

/**
 * @brief Count at which the 16-bit hardware pulse counter restarts from zero, 1 to 32767.
 *
 * @details The driver adds this value to a software total each time the counter restarts, so the
 *          count returned by wheel_get_encoder_count() does not wrap. That only works because
 *          wheel.c adds a watch point exactly at +limit and at -limit.
 */
#ifndef CONFIG_WHEEL_ENCODER_PCNT_HIGH_LIMIT
#define CONFIG_WHEEL_ENCODER_PCNT_HIGH_LIMIT    7000
#endif

/** @brief Encoder pulses shorter than this are ignored as noise, in nanoseconds (at most 12787 on the ESP32-S3). */
#ifndef CONFIG_WHEEL_ENCODER_GLITCH_FILTER_NS
#define CONFIG_WHEEL_ENCODER_GLITCH_FILTER_NS   1000
#endif
