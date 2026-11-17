/**
 * @file    threeeyes.c
 * @brief   Implementation of the ThreeEyes component (three HC-SR04 sensors on MCPWM capture).
 *
 * @details How one measurement works:
 *          1. The task sends a 10 us pulse on the shared trigger pin.
 *          2. Each sensor emits its ultrasonic burst and raises its ECHO pin; the capture channel stores
 *             the timer value of that rising edge (in the interrupt, see @ref echo_edge_callback).
 *          3. When the sound comes back (or after about 38 ms if nothing is in range) the sensor lowers
 *             ECHO; the interrupt computes the pulse width and notifies the waiting task.
 *          4. The task wakes up on the first notification and records which sensors have answered.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#include "threeeyes.h"
#include "freertos/task.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_check.h"
#include "esp_log.h"
#include "juca_port.h"

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG __attribute__((unused)) = "threeeyes"; /* unused when the Arduino log level hides every message */

/** @brief Number of sensors handled by this component. */
#define THREEEYES_SENSOR_COUNT    3

/** @brief Width of the trigger pulse, in microseconds (the HC-SR04 needs at least 10 us). */
#define THREEEYES_TRIG_PULSE_US   10

/** @brief Index of each sensor in #s_sensors. Bit `1 << index` is its task notification bit. */
enum {
    THREEEYES_INDEX_LEFT = 0,
    THREEEYES_INDEX_MIDDLE = 1,
    THREEEYES_INDEX_RIGHT = 2,
};

/**
 * @brief Runtime context of one sensor.
 */
typedef struct {
    mcpwm_cap_channel_handle_t channel;  /**< Capture channel of the ECHO pin (NULL if not created). */
    bool is_enabled;                     /**< true while the capture channel is enabled. */
    volatile uint32_t start_ticks;       /**< Timer value at the rising edge of ECHO. Written by the interrupt. */
    volatile uint32_t tof_ticks;         /**< Width of the last echo pulse, in timer ticks. Written by the interrupt. */
    bool is_updated;                     /**< Result flag of the last trigger. Written by the task only. */
} threeeyes_sensor_t;

/** @brief The three sensors, in the order left, middle, right. */
static threeeyes_sensor_t s_sensors[THREEEYES_SENSOR_COUNT];

/** @brief Capture timer shared by the three channels (NULL before @ref threeeyes_init). */
static mcpwm_cap_timer_handle_t s_cap_timer = NULL;

/** @brief Capture timer resolution in Hz (ticks per second), read from the driver at init. */
static uint32_t s_timer_resolution_hz = 0;

/** @brief Trigger pin, copied from the configuration. */
static gpio_num_t s_trig_gpio = GPIO_NUM_NC;

/** @brief true after a successful @ref threeeyes_init. */
static bool s_is_initialized = false;

/**
 * @brief Task waiting for echoes, or NULL when nobody waits.
 *
 * @details The interrupt only notifies while this is set, so an echo that ends after the task stopped
 *          waiting does not leave a notification behind.
 */
static volatile TaskHandle_t s_waiting_task_handle = NULL;

/* ============================================================================
 *                      INTERRUPT CALLBACK FUNCTION (ISR)
 * ==========================================================================*/

/**
 * @brief Called by the MCPWM driver, in interrupt context, on every edge of an ECHO pin.
 *
 * @details Rising edge: store the start time. Falling edge: compute the pulse width and notify the
 *          waiting task with this sensor's bit. The unsigned subtraction stays correct when the 32-bit
 *          timer wraps around between the two edges.
 *
 * @param[in] cap_chan  Capture channel that generated the event (unused).
 * @param[in] edata     Edge type and captured timer value.
 * @param[in] user_data The sensor index (0 to 2), passed as a pointer-sized integer.
 * @return true if a higher-priority task was woken and a context switch is needed.
 */
static bool echo_edge_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    (void)cap_chan;
    const uint32_t sensor_index = (uint32_t)(uintptr_t)user_data;
    threeeyes_sensor_t *sensor = &s_sensors[sensor_index];

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS)
    {
        sensor->start_ticks = edata->cap_value;
        return false;
    }

    sensor->tof_ticks = edata->cap_value - sensor->start_ticks;

    BaseType_t is_higher_priority_task_woken = pdFALSE;
    TaskHandle_t waiting_task_handle = s_waiting_task_handle;
    if (waiting_task_handle != NULL)
    {
        xTaskNotifyFromISR(waiting_task_handle, 1UL << sensor_index, eSetBits, &is_higher_priority_task_woken);
    }
    return is_higher_priority_task_woken == pdTRUE;
}

/* ============================================================================
 *                     PRIVATE HELPER FUNCTIONS
 * ==========================================================================*/

/**
 * @brief Releases everything @ref threeeyes_init may have created (used as its rollback).
 *
 * @param[in] is_timer_enabled true if the capture timer was already enabled.
 */
static void release_resources(bool is_timer_enabled)
{
    for (int i = 0; i < THREEEYES_SENSOR_COUNT; i++)
    {
        if (s_sensors[i].channel != NULL)
        {
            if (s_sensors[i].is_enabled)
            {
                mcpwm_capture_channel_disable(s_sensors[i].channel);
            }
            mcpwm_del_capture_channel(s_sensors[i].channel);
        }
        s_sensors[i] = (threeeyes_sensor_t){0};
    }

    if (s_cap_timer != NULL)
    {
        if (is_timer_enabled)
        {
            mcpwm_capture_timer_disable(s_cap_timer);
        }
        mcpwm_del_capture_timer(s_cap_timer);
        s_cap_timer = NULL;
    }
}

/**
 * @brief Disables the capture channel of one sensor.
 *
 * @param[in] sensor_index Index in #s_sensors.
 * @return esp_err_t ESP_OK, ESP_ERR_INVALID_STATE, or an MCPWM driver error.
 */
static esp_err_t disable_sensor(int sensor_index)
{
    ESP_RETURN_ON_FALSE(s_is_initialized, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    ESP_RETURN_ON_FALSE(s_sensors[sensor_index].is_enabled, ESP_ERR_INVALID_STATE, TAG, "Sensor %d is already disabled", sensor_index);
    ESP_RETURN_ON_ERROR(mcpwm_capture_channel_disable(s_sensors[sensor_index].channel), TAG, "Failed to disable sensor %d", sensor_index);

    s_sensors[sensor_index].is_enabled = false;
    s_sensors[sensor_index].is_updated = false;
    return ESP_OK;
}

/* ============================================================================
 *                         PUBLIC API IMPLEMENTATION
 * ==========================================================================*/

esp_err_t threeeyes_init(const threeeyes_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Null configuration pointer provided");
    ESP_RETURN_ON_FALSE(!s_is_initialized, ESP_ERR_INVALID_STATE, TAG, "Already initialized");

    esp_err_t ret = ESP_OK; /* Used by the ESP_GOTO_ON_ERROR macro */
    bool is_timer_enabled = false;
    const gpio_num_t echo_gpios[THREEEYES_SENSOR_COUNT] = {
        [THREEEYES_INDEX_LEFT] = config->echo_left_gpio,
        [THREEEYES_INDEX_MIDDLE] = config->echo_middle_gpio,
        [THREEEYES_INDEX_RIGHT] = config->echo_right_gpio,
    };

    /* 1. Capture timer: a free-running 32-bit counter that the channels sample on each edge. */
    mcpwm_capture_timer_config_t timer_config = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = config->mcpwm_group_id,
    };
    ESP_GOTO_ON_ERROR(mcpwm_new_capture_timer(&timer_config, &s_cap_timer), err, TAG, "Failed to create the capture timer");
    ESP_GOTO_ON_ERROR(mcpwm_capture_timer_get_resolution(s_cap_timer, &s_timer_resolution_hz), err, TAG, "Failed to read the timer resolution");

    /* 2. One capture channel per ECHO pin, on both edges. The callback must be registered BEFORE the
     *    channel is enabled (requirement of the MCPWM driver). */
    for (int i = 0; i < THREEEYES_SENSOR_COUNT; i++)
    {
        mcpwm_capture_channel_config_t channel_config = {
            .gpio_num = echo_gpios[i],
            .prescale = 1,
            .flags.neg_edge = true,
            .flags.pos_edge = true,
            .flags.pull_up = false,
        };
        ESP_GOTO_ON_ERROR(mcpwm_new_capture_channel(s_cap_timer, &channel_config, &s_sensors[i].channel), err, TAG, "Failed to create the capture channel of sensor %d", i);

        mcpwm_capture_event_callbacks_t callbacks = {
            .on_cap = echo_edge_callback,
        };
        ESP_GOTO_ON_ERROR(mcpwm_capture_channel_register_event_callbacks(s_sensors[i].channel, &callbacks, (void *)(uintptr_t)i), err, TAG, "Failed to register the callback of sensor %d", i);
        ESP_GOTO_ON_ERROR(mcpwm_capture_channel_enable(s_sensors[i].channel), err, TAG, "Failed to enable the capture channel of sensor %d", i);
        s_sensors[i].is_enabled = true;
    }

    /* 3. Trigger pin: output, idle low. */
    gpio_config_t trig_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << config->trig_gpio,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&trig_config), err, TAG, "Failed to configure the trigger pin");
    ESP_GOTO_ON_ERROR(gpio_set_level(config->trig_gpio, 0), err, TAG, "Failed to set the trigger pin low");
    s_trig_gpio = config->trig_gpio;

    /* 4. Start counting. */
    ESP_GOTO_ON_ERROR(mcpwm_capture_timer_enable(s_cap_timer), err, TAG, "Failed to enable the capture timer");
    is_timer_enabled = true;
    ESP_GOTO_ON_ERROR(mcpwm_capture_timer_start(s_cap_timer), err, TAG, "Failed to start the capture timer");

    s_is_initialized = true;
    ESP_LOGI(TAG, "Initialized (capture timer at %lu Hz).", (unsigned long)s_timer_resolution_hz);
    return ESP_OK;

err:
    release_resources(is_timer_enabled);
    return ret;
}

esp_err_t threeeyes_trigger_and_wait(TickType_t timeout_ticks)
{
    ESP_RETURN_ON_FALSE(s_is_initialized, ESP_ERR_INVALID_STATE, TAG, "Not initialized");

    /* Start from a clean notification: clear BOTH the pending state and the value. Clearing only the
     * value is not enough: an echo of the previous trigger that ended after the previous wait returned
     * would leave the notification "pending", and the wait below would then return at once with no bits. */
    xTaskNotifyStateClear(NULL);
    ulTaskNotifyValueClear(NULL, UINT32_MAX);
    s_waiting_task_handle = xTaskGetCurrentTaskHandle();

    /* Trigger pulse. The busy-wait is intentional: 10 us is far below one RTOS tick. */
    gpio_set_level(s_trig_gpio, 1);
    esp_rom_delay_us(THREEEYES_TRIG_PULSE_US);
    gpio_set_level(s_trig_gpio, 0);

    /* Sleep until the interrupt reports the first echo (or the timeout expires). */
    uint32_t notified_bits = 0;
    BaseType_t is_notified = xTaskNotifyWait(0, UINT32_MAX, &notified_bits, timeout_ticks);
    s_waiting_task_handle = NULL;

    /* On a timeout notified_bits is 0, so every flag is cleared: no stale result survives. */
    for (int i = 0; i < THREEEYES_SENSOR_COUNT; i++)
    {
        s_sensors[i].is_updated = (notified_bits & (1UL << i)) != 0;
    }

    return (is_notified == pdTRUE && notified_bits != 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t threeeyes_read_values(threeeyes_value_t *left, threeeyes_value_t *middle, threeeyes_value_t *right)
{
    ESP_RETURN_ON_FALSE(left && middle && right, ESP_ERR_INVALID_ARG, TAG, "Output pointer is NULL");

    threeeyes_value_t *const outputs[THREEEYES_SENSOR_COUNT] = {
        [THREEEYES_INDEX_LEFT] = left,
        [THREEEYES_INDEX_MIDDLE] = middle,
        [THREEEYES_INDEX_RIGHT] = right,
    };
    for (int i = 0; i < THREEEYES_SENSOR_COUNT; i++)
    {
        outputs[i]->tof_ticks = s_sensors[i].tof_ticks;
        outputs[i]->is_updated = s_sensors[i].is_updated;
    }
    return ESP_OK;
}

float threeeyes_convert_ticks_to_cm(uint32_t tof_ticks)
{
    if (s_timer_resolution_hz == 0)
    {
        return 0.0f;
    }
    const float tof_us = (float)tof_ticks * (1000000.0f / (float)s_timer_resolution_hz);
    return tof_us / THREEEYES_US_PER_CM;
}

esp_err_t threeeyes_disable_left(void)
{
    return disable_sensor(THREEEYES_INDEX_LEFT);
}

esp_err_t threeeyes_disable_middle(void)
{
    return disable_sensor(THREEEYES_INDEX_MIDDLE);
}

esp_err_t threeeyes_disable_right(void)
{
    return disable_sensor(THREEEYES_INDEX_RIGHT);
}
