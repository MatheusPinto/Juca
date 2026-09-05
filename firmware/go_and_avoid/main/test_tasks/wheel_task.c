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

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG = "wheels";

/* ============================================================================
 *                          MOTOR DRIVER GPIO PINS
 * ==========================================================================*/

/** Left motor MCPWM output pin A. */
#define MOTOR_LEFT_PWM_A          GPIO_NUM_12
/** Left motor MCPWM output pin B. */
#define MOTOR_LEFT_PWM_B          GPIO_NUM_13
/** Right motor MCPWM output pin A. */
#define MOTOR_RIGHT_PWM_A         GPIO_NUM_11
/** Right motor MCPWM output pin B. */
#define MOTOR_RIGHT_PWM_B         GPIO_NUM_10

/** Maximum allowed power limit derived from wheel driver tick definitions. */
#define WHEEL_POWER_MAX           WHEEL_PWM_DUTY_TICK_MAX

/* ============================================================================
 *                          ENCODER GPIO PINS
 * ==========================================================================*/

/** Left encoder quadrature channel A pin. */
#define ENCODER_LEFT_A            GPIO_NUM_7
/** Left encoder quadrature channel B pin. */
#define ENCODER_LEFT_B            GPIO_NUM_6
/** Right encoder quadrature channel A pin. */
#define ENCODER_RIGHT_A           GPIO_NUM_21
/** Right encoder quadrature channel B pin. */
#define ENCODER_RIGHT_B           GPIO_NUM_14

/* ============================================================================
 *                          ADC CURRENT SENSE CHANNELS
 * ==========================================================================*/

/** ADC channel mapped to left motor current sensor. */
#define ADC_LEFT_CHANNEL          ADC_CHANNEL_8
/** ADC channel mapped to right motor current sensor. */
#define ADC_RIGHT_CHANNEL         ADC_CHANNEL_1

/* ============================================================================
 *                          TASK IMPLEMENTATIONS
 * ==========================================================================*/

#ifdef TWO_WHEELS_CTRL_TASK
portTASK_FUNCTION(WheelCtrl, arg)
{
    /* ----------------------------------------------------------------
     * 1. Initialize Shared ADC Oneshot Unit
     * ---------------------------------------------------------------- */
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));

    /* Configure the ADC Channel */
    adc_oneshot_chan_cfg_t adc_chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_LEFT_CHANNEL, &adc_chan_config));

    /* ----------------------------------------------------------------
     * 2. Configure Wheel Instances
     * ---------------------------------------------------------------- */
    wheelConfig_t left_config = {
        .pwm_a_gpio = MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = MOTOR_LEFT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_limit = 350,          /* Custom power scale: [-400 to +400] */
        .encoder_a_gpio = ENCODER_LEFT_A,
        .encoder_b_gpio = ENCODER_LEFT_B,
        .adc_handle = adc_handle,
        .adc_channel = ADC_LEFT_CHANNEL,
    };

    wheelConfig_t right_config = {
        .pwm_a_gpio = MOTOR_RIGHT_PWM_A,
        .pwm_b_gpio = MOTOR_RIGHT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_limit = 350,          /* Custom power scale: [-400 to +400] */
        .encoder_a_gpio = ENCODER_RIGHT_A,
        .encoder_b_gpio = ENCODER_RIGHT_B,
        .adc_handle = adc_handle,
        .adc_channel = ADC_RIGHT_CHANNEL,
    };

    wheelHandle_t wheel_left = NULL;
    wheelHandle_t wheel_right = NULL;

    ESP_ERROR_CHECK(Wheel_Create(&left_config, &wheel_left));
    ESP_ERROR_CHECK(Wheel_Create(&right_config, &wheel_right));

    ESP_LOGI(TAG, "Wheel instances created successfully.");

    /* ----------------------------------------------------------------
     * 3. Example Execution Loop
     * ---------------------------------------------------------------- */

    /* Drive Forward at half power (+200 out of +/-400) */
    ESP_LOGI(TAG, "Moving forward...");
    Wheel_SetPower(wheel_left, -200);
    Wheel_SetPower(wheel_right, -200);

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
                Wheel_SetPower(wheel_left, 200);
                Wheel_SetPower(wheel_right, 200);
                ESP_LOGI(TAG, "Go forward");
                dir = 1;
            }
            else
            {
                Wheel_SetPower(wheel_left, -200);
                Wheel_SetPower(wheel_right, -200);
                ESP_LOGI(TAG, "Go backward");
                dir = 0;
            }
        }
        count++;

        int left_enc = 0, right_enc = 0;
        int left_adc = 0, right_adc = 0;

        /* Read telemetry from both wheels */
        Wheel_GetEncoderCount(wheel_left, &left_enc);
        Wheel_GetEncoderCount(wheel_right, &right_enc);
        Wheel_GetCurrentRaw(wheel_left, &left_adc);
        Wheel_GetCurrentRaw(wheel_right, &right_adc);

        ESP_LOGI(TAG, "[FWD] Left: Enc=%d, ADC=%d | Right: Enc=%d, ADC=%d",
                 left_enc, left_adc, right_enc, right_adc);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

#ifdef SINGLE_BASIC_WHEEL_CTRL_TASK
portTASK_FUNCTION(WheelCtrl, arg)
{
    wheelConfig_t minimal_wheel_cfg = {
        .pwm_a_gpio = MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = MOTOR_LEFT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_limit = 300,

        /* Encoders disabled */
        .encoder_a_gpio = GPIO_NUM_NC,
        .encoder_b_gpio = GPIO_NUM_NC,

        /* ADC disabled (remaining parameters can be omitted or set to 0) */
        .adc_handle = NULL,
    };

    wheelHandle_t wheel_basic = NULL;
    ESP_ERROR_CHECK(Wheel_Create(&minimal_wheel_cfg, &wheel_basic));

    int dir = 0;
    int count = 0;

    Wheel_SetPower(wheel_basic, -WHEEL_PWM_DUTY_TICK_MAX);
    while (1)
    {
        if (count == 6)
        {
            count = 0;
            if (dir == 0)
            {
                /* Motor operates normally via PWM */
                Wheel_SetPower(wheel_basic, WHEEL_PWM_DUTY_TICK_MAX);
                ESP_LOGI(TAG, "Go forward");
                dir = 1;
            }
            else
            {
                Wheel_SetPower(wheel_basic, -WHEEL_PWM_DUTY_TICK_MAX);
                ESP_LOGI(TAG, "Go backward");
                dir = 0;
            }
        }
        count++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

#ifdef ENCODER_TEST_TASK
portTASK_FUNCTION(WheelCtrl, arg)
{

    /* ----------------------------------------------------------------
     * 1. Configure Wheel Instances
     * ---------------------------------------------------------------- */
    wheelConfig_t left_config = {
        .pwm_a_gpio = MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = MOTOR_LEFT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_limit = 350,          /* Custom power scale: [-400 to +400] */
        .encoder_a_gpio = ENCODER_LEFT_A,
        .encoder_b_gpio = ENCODER_LEFT_B,
    };

    

    wheelHandle_t wheel_left = NULL;

    ESP_ERROR_CHECK(Wheel_Create(&left_config, &wheel_left));

    ESP_LOGI(TAG, "Wheel instances created successfully.");

    /* ----------------------------------------------------------------
     * 2. Example Execution Loop
     * ---------------------------------------------------------------- */

    for ( ; ; ) 
    {
        int left_enc = 0;

        /* Read telemetry from both wheels */
        Wheel_GetEncoderCount(wheel_left, &left_enc);
        
        ESP_LOGI(TAG, "[FWD] Encoder count=%d", left_enc);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif