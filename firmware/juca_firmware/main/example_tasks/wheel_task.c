/**
 * @file    wheel_task.c
 * @brief   FreeRTOS task implementation demonstrating dual and single wheel control routines.
 *
 * @date    Jan 2025
 * @author  Matheus
 */

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "wheel.h"
#include "wheel_task.h"
#include "juca_board.h"

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG = "wheels";

/* Pin and ADC channel map: see juca_board.h (single source of truth). */

/* ============================================================================
 *                          TASK IMPLEMENTATIONS
 * ==========================================================================*/

#ifdef WHEEL_EXAMPLE_TWO_WHEELS
portTASK_FUNCTION(wheel_ctrl_task, arg)
{
    /* ----------------------------------------------------------------
     * 1. Initialize Shared ADC Oneshot Unit
     * ---------------------------------------------------------------- */
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = JUCA_ADC_UNIT_CURRENT,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));

    /* Configure the ADC Channel */
    adc_oneshot_chan_cfg_t adc_channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JUCA_ADC_CHANNEL_CURRENT_LEFT, &adc_channel_config));

    /* ----------------------------------------------------------------
     * 2. Configure Wheel Instances
     * ---------------------------------------------------------------- */
    wheel_config_t left_config = {
        .pwm_a_gpio = JUCA_PIN_MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = JUCA_PIN_MOTOR_LEFT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_ticks = 350,          /* Custom power scale: [-400 to +400] */
        .encoder_a_gpio = JUCA_PIN_ENCODER_LEFT_A,
        .encoder_b_gpio = JUCA_PIN_ENCODER_LEFT_B,
        .adc_handle = adc_handle,
        .adc_channel = JUCA_ADC_CHANNEL_CURRENT_LEFT,
    };

    wheel_config_t right_config = {
        .pwm_a_gpio = JUCA_PIN_MOTOR_RIGHT_PWM_A,
        .pwm_b_gpio = JUCA_PIN_MOTOR_RIGHT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_ticks = 350,          /* Custom power scale: [-400 to +400] */
        .encoder_a_gpio = JUCA_PIN_ENCODER_RIGHT_A,
        .encoder_b_gpio = JUCA_PIN_ENCODER_RIGHT_B,
        .adc_handle = adc_handle,
        .adc_channel = JUCA_ADC_CHANNEL_CURRENT_RIGHT,
    };

    wheel_handle_t left_wheel = NULL;
    wheel_handle_t right_wheel = NULL;

    ESP_ERROR_CHECK(wheel_create(&left_config, &left_wheel));
    ESP_ERROR_CHECK(wheel_create(&right_config, &right_wheel));

    ESP_LOGI(TAG, "Wheel instances created successfully.");

    /* ----------------------------------------------------------------
     * 3. Example Execution Loop
     * ---------------------------------------------------------------- */

    /* Drive Forward at half power (+200 out of +/-400) */
    ESP_LOGI(TAG, "Moving forward...");
    wheel_set_power(left_wheel, -200);
    wheel_set_power(right_wheel, -200);

    int dir = 0;
    int count = 0;
    for ( ; ; ) 
    {
        if (count == 6)
        {
            count = 0;
            if (dir == 0)
            {
                /* Motor operates normally via PWM */
                wheel_set_power(left_wheel, 200);
                wheel_set_power(right_wheel, 200);
                ESP_LOGI(TAG, "Go forward");
                dir = 1;
            }
            else
            {
                wheel_set_power(left_wheel, -200);
                wheel_set_power(right_wheel, -200);
                ESP_LOGI(TAG, "Go backward");
                dir = 0;
            }
        }
        count++;

        int left_encoder_count = 0, right_encoder_count = 0;
        int left_current_raw = 0, right_current_raw = 0;

        /* Read telemetry from both wheels */
        wheel_get_encoder_count(left_wheel, &left_encoder_count);
        wheel_get_encoder_count(right_wheel, &right_encoder_count);
        wheel_get_current_raw(left_wheel, &left_current_raw);
        wheel_get_current_raw(right_wheel, &right_current_raw);

        ESP_LOGI(TAG, "[FWD] Left: Enc=%d, ADC=%d | Right: Enc=%d, ADC=%d",
                 left_encoder_count, left_current_raw, right_encoder_count, right_current_raw);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

#ifdef WHEEL_EXAMPLE_SINGLE_BASIC
portTASK_FUNCTION(wheel_ctrl_task, arg)
{
    wheel_config_t minimal_wheel_config = {
        .pwm_a_gpio = JUCA_PIN_MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = JUCA_PIN_MOTOR_LEFT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_ticks = 300,

        /* Encoders disabled */
        .encoder_a_gpio = GPIO_NUM_NC,
        .encoder_b_gpio = GPIO_NUM_NC,

        /* ADC disabled (remaining parameters can be omitted or set to 0) */
        .adc_handle = NULL,
    };

    wheel_handle_t wheel_basic = NULL;
    ESP_ERROR_CHECK(wheel_create(&minimal_wheel_config, &wheel_basic));

    int dir = 0;
    int count = 0;

    wheel_set_power(wheel_basic, -WHEEL_PWM_DUTY_TICK_MAX);
    while (1)
    {
        if (count == 6)
        {
            count = 0;
            if (dir == 0)
            {
                /* Motor operates normally via PWM */
                wheel_set_power(wheel_basic, WHEEL_PWM_DUTY_TICK_MAX);
                ESP_LOGI(TAG, "Go forward");
                dir = 1;
            }
            else
            {
                wheel_set_power(wheel_basic, -WHEEL_PWM_DUTY_TICK_MAX);
                ESP_LOGI(TAG, "Go backward");
                dir = 0;
            }
        }
        count++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

#ifdef WHEEL_EXAMPLE_ENCODER_TEST
portTASK_FUNCTION(wheel_ctrl_task, arg)
{

    /* ----------------------------------------------------------------
     * 1. Configure Wheel Instances
     * ---------------------------------------------------------------- */
    wheel_config_t left_config = {
        .pwm_a_gpio = JUCA_PIN_MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = JUCA_PIN_MOTOR_LEFT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_ticks = 350,          /* Custom power scale: [-400 to +400] */
        .encoder_a_gpio = JUCA_PIN_ENCODER_LEFT_A,
        .encoder_b_gpio = JUCA_PIN_ENCODER_LEFT_B,
    };

    

    wheel_handle_t left_wheel = NULL;

    ESP_ERROR_CHECK(wheel_create(&left_config, &left_wheel));

    ESP_LOGI(TAG, "Wheel instances created successfully.");

    /* ----------------------------------------------------------------
     * 2. Example Execution Loop
     * ---------------------------------------------------------------- */

    for ( ; ; ) 
    {
        int left_encoder_count = 0;

        /* Read telemetry from both wheels */
        wheel_get_encoder_count(left_wheel, &left_encoder_count);
        
        ESP_LOGI(TAG, "[FWD] Encoder count=%d", left_encoder_count);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif
