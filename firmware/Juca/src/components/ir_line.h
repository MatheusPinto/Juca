/**
 * @file    ir_line.h
 * @brief   Public interface of the ir_line component: a bar of 5 digital infrared line sensors.
 *
 * @details Each sensor is a reflective infrared module with a digital output: one level over the
 *          line and the other level over the floor. The component reads the five GPIOs and returns
 *          one bit per sensor, already corrected for the polarity, so the application never deals
 *          with pin numbers or with "is the line a 0 or a 1".
 *
 *          The component is a singleton (one bar) and knows nothing about the board: the pins and
 *          the polarity arrive through @ref ir_line_config_t. It only reads the sensors; deciding
 *          where to steer is the job of the application.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *                                 CONSTANTS
 * ==========================================================================*/

/** @brief Number of sensors in the bar. */
#define IR_LINE_SENSOR_COUNT    5

/**
 * @name Line patterns
 * @brief Values of the mask returned by @ref ir_line_read_mask for a line seen by one sensor or by two
 *        neighbours. Bit 0 is the far LEFT sensor and bit 4 the far RIGHT one; a bit at 1 means "this
 *        sensor is over the line". Written in binary the left sensor is therefore the rightmost digit.
 * @{
 */
#define IR_LINE_PATTERN_NONE             0x00  /**< 00000: no sensor sees the line. */
#define IR_LINE_PATTERN_FAR_LEFT         0x01  /**< 00001: far left sensor only. */
#define IR_LINE_PATTERN_LEFT_FAR_LEFT    0x03  /**< 00011: far left and center-left. */
#define IR_LINE_PATTERN_LEFT             0x02  /**< 00010: center-left sensor only. */
#define IR_LINE_PATTERN_LEFT_MIDDLE      0x06  /**< 00110: center-left and middle. */
#define IR_LINE_PATTERN_MIDDLE           0x04  /**< 00100: middle sensor only (line centered). */
#define IR_LINE_PATTERN_RIGHT_MIDDLE     0x0C  /**< 01100: middle and center-right. */
#define IR_LINE_PATTERN_RIGHT            0x08  /**< 01000: center-right sensor only. */
#define IR_LINE_PATTERN_RIGHT_FAR_RIGHT  0x18  /**< 11000: center-right and far right. */
#define IR_LINE_PATTERN_FAR_RIGHT        0x10  /**< 10000: far right sensor only. */
#define IR_LINE_PATTERN_ALL              0x1F  /**< 11111: every sensor sees the line (a crossing, or the robot is lifted). */
/** @} */

/* ============================================================================
 *                              DATA TYPES
 * ==========================================================================*/

/**
 * @brief Configuration of the sensor bar. Every field is mandatory.
 */
typedef struct {
    gpio_num_t sensor_gpios[IR_LINE_SENSOR_COUNT];  /**< Input pins, ordered from the far LEFT sensor (index 0) to the far RIGHT one (index 4), as seen from the robot looking forward. */
    uint32_t line_level;                            /**< GPIO level (0 or 1) that a sensor outputs when it is OVER THE LINE. It depends on the sensor module and on the track (dark line on a light floor, or the opposite). */
} ir_line_config_t;

/* ============================================================================
 *                              PUBLIC API
 * ==========================================================================*/

/**
 * @brief Configures the five sensor pins as inputs (no pull-up, no pull-down, no interrupt).
 *
 * @details Can be called again to change the configuration (for example the polarity).
 *
 * @param[in] config Pins and polarity. Must not be NULL.
 *
 * @return esp_err_t
 *         - ESP_OK: Sensors ready.
 *         - ESP_ERR_INVALID_ARG: `config` is NULL, a pin is not a valid GPIO, or `line_level` is not 0 or 1.
 *         - Other: error of the GPIO driver.
 */
esp_err_t ir_line_init(const ir_line_config_t *config);

/**
 * @brief Reads the five sensors and returns one bit per sensor.
 *
 * @details Bit i of the mask is 1 when sensor i (0 = far left ... 4 = far right) is over the line.
 *          Compare the result with the `IR_LINE_PATTERN_*` constants, or test the bits one by one.
 *          The read takes a few microseconds and does not block.
 *
 * @param[out] out_mask Receives the mask (0x00 to 0x1F). Must not be NULL.
 *
 * @return esp_err_t
 *         - ESP_OK: Mask valid.
 *         - ESP_ERR_INVALID_ARG: `out_mask` is NULL.
 *         - ESP_ERR_INVALID_STATE: @ref ir_line_init was not called (the mask is 0).
 */
esp_err_t ir_line_read_mask(uint8_t *out_mask);

#ifdef __cplusplus
}
#endif
