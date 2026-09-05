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
#include "diff_drive.h"
#include "diff_drive_task.h"

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG = "diff_drive_task";

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

portTASK_FUNCTION(DiffDriveCtrl, arg)
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

    /* 2. Populate differential drive configuration structure */
    diffDriveConfig_t drive_config = {
        .left_wheel = wheel_left,
        .right_wheel = wheel_right,
        
        /* Physical vehicle parameters */
        .half_track_width = 0.15f,               // Distance between wheels and the center of the robot: 15 cm
        .wheel_radius = 0.03f,              // Wheel radius: 3 cm
        .encoder_cpr = 900,                // Quadrature encoder CPR (4x mode)
        
        /* Operating speed limits */
        .max_linear_velocity = 0.8f,        // Max forward speed: 0.8 m/s
        .max_angular_velocity = 2.0f,       // Max turning rate: 2.0 rad/s
        .max_wheel_rad_s = 20.0f,           // Estimated max wheel speed in rad/s (It can be ignored if you apply wheels calibration later)
        
        /* Current safety task parameters */
        .max_adc_raw_threshold = 2500,      // Emergency ADC current threshold
        .monitor_period_ms = 50,            // Safety check period (50 ms)
        .safety_task_priority = 5,          // Higher priority for safety background thread
        .safety_task_stack_size = 3072      // Stack size in bytes
    };

    /* 3. Initialize differential drive controller */
    ESP_ERROR_CHECK(DiffDrive_Init(&drive_config));

    /* 4. Perform dynamic maximum speed calibration */
    ESP_LOGI(TAG, "Starting calibration. Ensure robot path is clear!");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Delay to allow safe placement before movement

    float measured_max_rad_s = 0.0f;
    esp_err_t cal_err = DiffDrive_CalibrateMaxSpeed(4000, &measured_max_rad_s);
    if (cal_err == ESP_OK)
    {
        ESP_LOGI(TAG, "Auto-calibration successful! Max wheel speed: %.2f rad/s", measured_max_rad_s);
    }
    else
    {
        ESP_LOGW(TAG, "Calibration skipped or failed (%s). Using fallback configuration.", esp_err_to_name(cal_err));
    }


    diffDriveTwist_t cmd_vel = {
        .linear_x = 0.3f,   // Drive forward at 0.3 m/s
        .angular_z = 0.2f   // Gentle left yaw rotation at 0.2 rad/s
    };

    diffDriveTwist_t estimated_vel = {0};

    ESP_LOGI(TAG, "Periodic control task running at 20 Hz...");

    const TickType_t task_period = pdMS_TO_TICKS(50); // 20 Hz loop frequency
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t previous_tick = last_wake_time;

    while (1)
    {
        /* 1. Check safety status */
        if (DiffDrive_IsFaultActive())
        {
            ESP_LOGW(TAG, "Safety lock active! Attempting fault recovery...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            /* Attempt to clear fault state */
            DiffDrive_ClearFault();
            continue;
        }

        /* 2. Calculate actual delta time (dt) in seconds */
        TickType_t current_tick = xTaskGetTickCount();
        float dt_seconds = (float)(current_tick - previous_tick) * portTICK_PERIOD_MS / 1000.0f;
        previous_tick = current_tick;

        /* 3. Retrieve estimated platform odometry via Forward Kinematics */
        if (dt_seconds > 0.0f)
        {
            esp_err_t err = DiffDrive_GetTwist(dt_seconds, &estimated_vel);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "[Odometry] Linear X: %.3f m/s | Angular Z: %.3f rad/s",
                         estimated_vel.linear_x, estimated_vel.angular_z);
            }
        }

        /* 4. Dispatch target velocity command via Inverse Kinematics */
        esp_err_t err = DiffDrive_SetTwist(&cmd_vel);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to apply target twist command: %s", esp_err_to_name(err));
        }

        /* Wait until the start of the next 50 ms cycle */
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}
