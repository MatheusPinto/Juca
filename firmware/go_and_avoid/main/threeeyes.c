/**
 * @file    threeeyes.c
 * @brief   Implementation of the multiple ultrasonic sensor module (ThreeEyes).
 * @details This module controls 3 HC-SR04 ultrasonic sensors in parallel:
 *          - Generates a 10 us trigger pulse on a shared GPIO pin;
 *          - Measures individual return pulse widths (ECHO) using MCPWM capture channels;
 *          - Uses FreeRTOS task notifications to signal measurement completion from ISR.
 *
 * @date    2025
 * @author  Matheus
 */

#include "threeeyes.h"
#include "freertos/semphr.h"
#include <inttypes.h>

/**
 * @brief Tag used for ESP32 logging outputs (ESP_LOG*).
 */
const static char *TAG = "TreeEyes";

/**
 * @brief Array containing context structures for the 3 ultrasonic sensors (Left, Middle, Right).
 */
volatile ultrasonic_sensor_t sensors[3];

/**
 * @brief Handle of the task that requested measurement and should receive notifications from ISR.
 */
static volatile TaskHandle_t actualTaskHandle;

/* ============================================================================
 *                      INTERRUPT CALLBACK FUNCTION (ISR)
 * ==========================================================================*/

/**
 * @brief MCPWM capture event callback triggered on edges of the ECHO pin.
 * @details On rising edge (POS edge), stores the initial timestamp. 
 *          On falling edge (NEG edge), calculates Time-of-Flight (tof_ticks) and 
 *          notifies the receiving task via `xTaskNotifyFromISR()`.
 *
 * @param[in] cap_chan  Handle of the capture channel that generated the event.
 * @param[in] edata     Pointer to event data structure (edge type and counter value).
 * @param[in] user_data User pointer corresponding to the sensor's @ref ultrasonic_sensor_t structure.
 * @return true If a higher priority task wake-up (context switch) is required.
 * @return false Otherwise.
 */
static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    ultrasonic_sensor_t *sensor = (ultrasonic_sensor_t *)user_data;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS)
    {
        /* Rising edge: Start of ECHO pulse */
        sensor->start_time = edata->cap_value;
    } 
    else if (edata->cap_edge == MCPWM_CAP_EDGE_NEG)
    {
        /* Falling edge: End of ECHO pulse */
        uint32_t end_time = edata->cap_value;
        sensor->value.tof_ticks = end_time - sensor->start_time;

        /* Notify task waiting for reading */
        BaseType_t high_task_wakeup = pdFALSE;
        if (actualTaskHandle != NULL)
        {
            xTaskNotifyFromISR(actualTaskHandle, sensor->notify_bit, eSetBits, &high_task_wakeup);
            return high_task_wakeup == pdTRUE;
        }
    }

    return false;
}

/* ============================================================================
 *                              PUBLIC API
 * ==========================================================================*/

/**
 * @brief Triggers the TRIG pulse and blocks the calling task until ECHO reception or timeout.
 * @details Performs the following sequence:
 *          1. Saves current task handle into @ref actualTaskHandle;
 *          2. Clears pending notifications on the task;
 *          3. Outputs a high pulse on the TRIG pin for 10 us;
 *          4. Waits for notification bits from sensors via `xTaskNotifyWait()`;
 *          5. Updates the `isUpdated` flag for each sensor.
 *
 * @param[in] xTicksToWait Maximum wait time in ticks.
 */
void ThreeEyes_TrigAndWait( TickType_t xTicksToWait )
{
    actualTaskHandle = xTaskGetCurrentTaskHandle();
    
    /* 1. Clear any old notification before starting */
    ulTaskNotifyValueClear(NULL, 0xFFFFFFFF);

    /* 2. Trigger 10-microsecond TRIG pulse */
    gpio_set_level(HC_SR04_TRIG_GPIO, 1);
    esp_rom_delay_us(10); 
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);

    uint32_t notifiedBits = 0;
    
    /* 3. Wait for notifications coming from ISR */
    if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedBits, xTicksToWait) == pdTRUE) 
    {
        if (notifiedBits == 0) {
            printf("WARNING: Woke up but notification bits were ZERO!\n");
        }
        /* 4. Map which sensors responded within time */
        sensors[0].value.isUpdated = (notifiedBits & NOTIFY_BIT_LEFT_SENSOR)   ? pdTRUE : pdFALSE;
        sensors[1].value.isUpdated = (notifiedBits & NOTIFY_BIT_MIDDLE_SENSOR) ? pdTRUE : pdFALSE;
        sensors[2].value.isUpdated = (notifiedBits & NOTIFY_BIT_RIGHT_SENSOR)  ? pdTRUE : pdFALSE;
    }
    else {
        printf("ERROR: Timed out even with portMAX_DELAY (This should not happen!)\n");
    }
    actualTaskHandle = NULL;
}

/**
 * @brief Retrieves the measured Time-of-Flight values and update flags for all sensors.
 *
 * @param[out] left   Pointer for copying left sensor reading.
 * @param[out] middle Pointer for copying middle sensor reading.
 * @param[out] right  Pointer for copying right sensor reading.
 */
void ThreeEyes_Read( ultrasonic_value_t *left, ultrasonic_value_t *middle, ultrasonic_value_t *right)
{
    left->tof_ticks = sensors[0].value.tof_ticks;
    left->isUpdated = sensors[0].value.isUpdated;

    middle->tof_ticks = sensors[1].value.tof_ticks;
    middle->isUpdated = sensors[1].value.isUpdated;

    right->tof_ticks = sensors[2].value.tof_ticks;
    right->isUpdated = sensors[2].value.isUpdated;
}

/**
 * @brief Initializes trigger GPIO pin and MCPWM capture peripheral for 3 sensors.
 */
void ThreeEyes_Init( void )
{
    ESP_LOGI(TAG, "Configuring Capture Timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_timer_config = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_config, &cap_timer));

    ESP_LOGI(TAG, "Configuring TRIG pin");
    gpio_config_t trig_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&trig_config));
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    ESP_LOGI(TAG, "Starting Capture Timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    gpio_num_t echo_gpios[3] = {HC_SR04_ECHO_GPIO_LEFT, HC_SR04_ECHO_GPIO_MIDDLE, HC_SR04_ECHO_GPIO_RIGHT};

    sensors[0].notify_bit = NOTIFY_BIT_LEFT_SENSOR;
    sensors[1].notify_bit = NOTIFY_BIT_MIDDLE_SENSOR;
    sensors[2].notify_bit = NOTIFY_BIT_RIGHT_SENSOR;

    for (int i = 0; i < 3; i++) 
    {
        ESP_LOGI(TAG, "Configuring capture channel for Sensor %d", i + 1);

        mcpwm_capture_channel_config_t channel_config = {
            .gpio_num = echo_gpios[i],
            .prescale = 1,
            .flags.neg_edge = true,
            .flags.pos_edge = true,
            .flags.pull_up = false,
        };

        ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &channel_config, &sensors[i].channel));

        mcpwm_capture_event_callbacks_t callbacks = {
            .on_cap = hc_sr04_echo_callback,
        };

        ESP_LOGI(TAG, "Registering callback for Sensor %d", i + 1);
        ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(sensors[i].channel, &callbacks, &sensors[i]));

        ESP_LOGI(TAG, "Enabling capture channel for Sensor %d", i + 1);
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(sensors[i].channel));
        sensors[i].echo_gpio_num = echo_gpios[i];
        sensors[i].value.isUpdated = pdFALSE;
    }
}

/**
 * @brief Disables capture channel for the left sensor.
 */
void ThreeEyes_DisableLeft(void)
{
    ESP_LOGI(TAG, "Disabling capture channel for Left Sensor");
    ESP_ERROR_CHECK(mcpwm_capture_channel_disable(sensors[0].channel));
}

/**
 * @brief Disables capture channel for the middle sensor.
 */
void ThreeEyes_DisableMiddle(void)
{
    ESP_LOGI(TAG, "Disabling capture channel for Middle Sensor");
    ESP_ERROR_CHECK(mcpwm_capture_channel_disable(sensors[1].channel));
}

/**
 * @brief Disables capture channel for the right sensor.
 */
void ThreeEyes_DisableRight(void)
{
    ESP_LOGI(TAG, "Disabling capture channel for Right Sensor");
    ESP_ERROR_CHECK(mcpwm_capture_channel_disable(sensors[2].channel));
}