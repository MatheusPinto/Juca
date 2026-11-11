/**
 * @file    threeeyes.h
 * @brief   Public interface for the ultrasonic sensor module (ThreeEyes / HC-SR04).
 * @details Declares GPIO macros, data structures, and API functions to initialize,
 *          trigger (TRIG), and read the time-of-flight (ToF) of three ultrasonic
 *          sensors in parallel using ESP32's MCPWM Capture peripheral.
 *
 * @date    2025
 * @author  Matheus
 */

#ifndef THREE_EYES_LIB
#define THREE_EYES_LIB

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "hal/gpio_types.h"

/* ============================================================================
 *                            GPIO PIN MAPPING
 * ==========================================================================*/

/** @brief GPIO pin used to output the trigger pulse (TRIG) common to all sensors. */
#define HC_SR04_TRIG_GPIO         GPIO_NUM_42

/** @brief GPIO pin for receiving the ECHO signal from the left sensor. */
#define HC_SR04_ECHO_GPIO_LEFT    GPIO_NUM_5

/** @brief GPIO pin for receiving the ECHO signal from the middle sensor. */
#define HC_SR04_ECHO_GPIO_MIDDLE  GPIO_NUM_4

/** @brief GPIO pin for receiving the ECHO signal from the right sensor. */
#define HC_SR04_ECHO_GPIO_RIGHT   GPIO_NUM_1

/* ============================================================================
 *                    FREERTOS TASK NOTIFICATION BITS
 * ==========================================================================*/

/** @brief Task notification bit for the left sensor response (0x01). */
#define NOTIFY_BIT_LEFT_SENSOR    ( 1UL << 0UL )

/** @brief Task notification bit for the middle sensor response (0x02). */
#define NOTIFY_BIT_MIDDLE_SENSOR  ( 1UL << 1UL )

/** @brief Task notification bit for the right sensor response (0x04). */
#define NOTIFY_BIT_RIGHT_SENSOR   ( 1UL << 2UL )

/* ============================================================================
 *                              DATA TYPES
 * ==========================================================================*/

/**
 * @brief Structure holding the measurement result of an individual ultrasonic sensor.
 */
typedef struct {
    uint32_t tof_ticks;    /**< Time-of-Flight measured in capture timer ticks. */
    BaseType_t isUpdated;  /**< Indicates if the value was updated after the last trigger pulse (pdTRUE/pdFALSE). */
} ultrasonic_value_t;

/**
 * @brief Structure holding context and control information for an ultrasonic sensor.
 */
typedef struct {
    mcpwm_cap_channel_handle_t channel; /**< MCPWM capture channel handle associated with the ECHO pin. */
    gpio_num_t echo_gpio_num;            /**< GPIO pin number associated with this sensor's ECHO signal. */
    uint32_t start_time;                 /**< Timestamp (ticks) of the ECHO rising edge. */
    uint32_t notify_bit;                 /**< FreeRTOS task notification bit mask for this sensor. */
    ultrasonic_value_t value;            /**< Most recent measurement data of the sensor. */
} ultrasonic_sensor_t;

/* ============================================================================
 *                              PUBLIC API
 * ==========================================================================*/

/**
 * @brief Initializes the ultrasonic sensor subsystem (TRIG pin and MCPWM Capture peripheral).
 * @details Configures the capture timer, enables 3 input channels (ECHO), and 
 *          registers edge capture ISR callback functions.
 */
void ThreeEyes_Init( void );

/**
 * @brief Emits the TRIG pulse and waits for ECHO responses from active sensors via task notifications.
 *
 * @param[in] xTicksToWait Maximum wait time (timeout) in FreeRTOS ticks.
 */
void ThreeEyes_TrigAndWait( TickType_t xTicksToWait );

/**
 * @brief Reads the updated measurement values from all three sensors (left, middle, right).
 *
 * @param[out] left   Pointer to the structure where the left sensor data will be copied.
 * @param[out] middle Pointer to the structure where the middle sensor data will be copied.
 * @param[out] right  Pointer to the structure where the right sensor data will be copied.
 */
void ThreeEyes_Read( ultrasonic_value_t *left, ultrasonic_value_t *middle, ultrasonic_value_t *right);

/**
 * @brief Disables pulse capture on the left sensor channel.
 */
void ThreeEyes_DisableLeft(void);

/**
 * @brief Disables pulse capture on the middle sensor channel.
 */
void ThreeEyes_DisableMiddle(void);

/**
 * @brief Disables pulse capture on the right sensor channel.
 */
void ThreeEyes_DisableRight(void);

#endif // THREE_EYES_LIB