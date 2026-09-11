/**
 * @file      diff_drive_task.c
 * @brief     FreeRTOS application entry point for differential drive mobile robot control.
 * @details   This module implements the main control task (`DiffDriveCtrl`) and an asynchronous
 *            interactive console task (`ConsoleInputTask`). It initializes the hardware peripherals 
 *            (MCPWM, Quadrature Encoders, ADC Oneshot), sets up the differential drive kinematics 
 *            and PID controllers, executes automatic max-speed calibration, and handles 
 *            non-blocking velocity commands alongside automatic safety fault monitoring.
 *
 * @date      Jan 2025 / Refactored
 * @author    Matheus
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#else
#include "driver/uart_vfs.h"
#endif
#include "diff_drive.h"
#include "diff_drive_task.h"

/** 
 * @brief Logging tag used across ESP-IDF system log macros. 
 */
static const char *TAG = "diff_drive_task";

/* ============================================================================
 *                          MOTOR DRIVER GPIO PINS
 * ==========================================================================*/

/** @brief Left motor MCPWM output pin A. */
#define MOTOR_LEFT_PWM_A            GPIO_NUM_12

/** @brief Left motor MCPWM output pin B. */
#define MOTOR_LEFT_PWM_B            GPIO_NUM_13

/** @brief Right motor MCPWM output pin A. */
#define MOTOR_RIGHT_PWM_A           GPIO_NUM_11

/** @brief Right motor MCPWM output pin B. */
#define MOTOR_RIGHT_PWM_B           GPIO_NUM_10

/** @brief Maximum allowed power limit derived from wheel driver tick definitions. */
#define WHEEL_POWER_MAX             WHEEL_PWM_DUTY_TICK_MAX

/* ============================================================================
 *                          ENCODER GPIO PINS
 * ==========================================================================*/

/** @brief Left encoder quadrature channel A pin. */
#define ENCODER_LEFT_A              GPIO_NUM_7

/** @brief Left encoder quadrature channel B pin. */
#define ENCODER_LEFT_B              GPIO_NUM_6

/** @brief Right encoder quadrature channel A pin. */
#define ENCODER_RIGHT_A             GPIO_NUM_21

/** @brief Right encoder quadrature channel B pin. */
#define ENCODER_RIGHT_B             GPIO_NUM_14

/* ============================================================================
 *                      ADC CURRENT SENSE CHANNELS
 * ==========================================================================*/

/** @brief ADC channel mapped to the left motor current sensor. */
#define ADC_LEFT_CHANNEL            ADC_CHANNEL_8

/** @brief ADC channel mapped to the right motor current sensor. */
#define ADC_RIGHT_CHANNEL           ADC_CHANNEL_1

/* ============================================================================
 *                    COMMUNICATION AND CONSOLE GLOBALS
 * ==========================================================================*/

/** 
 * @brief Global queue handle for passing velocity commands (`diffDriveTwist_t`) 
 *        from the console reader task to the main control task.
 */
static QueueHandle_t s_cmd_queue = NULL;

/**
 * @brief Configures standard I/O streams for non-buffered, truly blocking read operations.
 *
 * @details By default, the ESP-IDF standard console VFS operates in non-blocking mode, 
 *          causing functions like `fgets()` or `scanf()` to return immediately with an empty string 
 *          when no input is available. This helper initializes the underlying hardware driver 
 *          (USB-Serial JTAG or UART) and installs it into the VFS layer with blocking line-ending 
 *          configurations.
 *
 * @note Automatically detects target configurations via `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG` 
 *       at compile time.
 */
static void init_usb_cdc_stdio(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    usb_serial_jtag_vfs_use_driver();
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
#else
    if (!uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM))
    {
        const uart_config_t uart_config = {
            .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));
    }

    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
#endif
}

/**
 * @brief FreeRTOS task dedicated to reading user input from the console asynchronously.
 *
 * @details Suspends execution until notified by the control task. Upon notification, 
 *          it prompts the operator for target linear and angular velocities (`linear_x` and `angular_z`), 
 *          blocks until input is submitted via stdin, parses the input, and pushes the parsed command 
 *          into `s_cmd_queue`.
 *
 * @param[in] arg Pointer to task parameter list (unused).
 */
static void ConsoleInputTask(void *arg)
{
    (void)arg;
    char line[64];
    diffDriveTwist_t new_cmd = {0};

    /* Initialize USB Serial/JTAG VFS drivers */
    init_usb_cdc_stdio();

    printf("\n--- ESP32-S3 Console Ready ---\n");
    
    uint32_t notifiedBits;
    
    while (1)
    {
        /* Wait for notification from main controller before asking for input */
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedBits, portMAX_DELAY); 

        printf("Enter target velocity (linear_x angular_z): \n");
        fflush(stdout); /* Flush output buffer immediately */

        /* Block until the user submits input with ENTER */
        if (fgets(line, sizeof(line), stdin) != NULL)
        {
            /* Strip trailing carriage return and newline characters */
            line[strcspn(line, "\r\n")] = 0;

            if (sscanf(line, "%f %f", &(new_cmd.linear_x), &(new_cmd.angular_z)) == 2)
            {
                printf("Accepted target -> linear_x = %.2f, angular_z = %.2f\n",
                       new_cmd.linear_x, new_cmd.angular_z);
            }
            else
            {
                new_cmd.linear_x = 0;
                new_cmd.angular_z = 0;
                printf("Invalid input! Received: '%s'\n", line);
            }
        }

        if (s_cmd_queue != NULL)
        {
            xQueueSend(s_cmd_queue, &new_cmd, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ============================================================================
 *                          TASK IMPLEMENTATIONS
 * ==========================================================================*/

/**
 * @brief Main differential drive management task.
 *
 * @details Executes setup routines, initializes hardware interfaces, calibrates motor thresholds,
 *          and enters a 20 Hz periodic loop. In each cycle, it:
 *          1. Monitors and resolves safety fault states.
 *          2. Consumes non-blocking velocity command updates from the console queue.
 *          3. Computes forward kinematics odometry and outputs real-time telemetry.
 *
 * @param[in] arg FreeRTOS task argument (unused).
 */
portTASK_FUNCTION(DiffDriveCtrl, arg)
{
    s_cmd_queue = xQueueCreate(5, sizeof(diffDriveTwist_t));
    if (s_cmd_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create command queue!");
        vTaskDelete(NULL);
        return;
    }

    TaskHandle_t console_in_task_handle = NULL;
    /* Create console input task with priority 1 and adequate stack depth */
    xTaskCreate(ConsoleInputTask, "console_input_task", 3072, NULL, 1, &console_in_task_handle);

    /* ----------------------------------------------------------------
     * 1. Initialize Shared ADC Oneshot Unit
     * ---------------------------------------------------------------- */
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));

    adc_oneshot_chan_cfg_t adc_chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_LEFT_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_RIGHT_CHANNEL, &adc_chan_config));

    /* ----------------------------------------------------------------
     * 2. Configure Wheel Instances
     * ---------------------------------------------------------------- */
    wheelConfig_t left_config = {
        .pwm_a_gpio = MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = MOTOR_LEFT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_limit = 350,
        .encoder_a_gpio = ENCODER_LEFT_A,
        .encoder_b_gpio = ENCODER_LEFT_B,
        .adc_handle = adc_handle,
        .adc_channel = ADC_LEFT_CHANNEL,
    };

    wheelConfig_t right_config = {
        .pwm_a_gpio = MOTOR_RIGHT_PWM_A,
        .pwm_b_gpio = MOTOR_RIGHT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_limit = 350,
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
     * 3. Populate Differential Drive Configuration Structure
     * ---------------------------------------------------------------- */
    diffDriveConfig_t drive_config = {
        .left_wheel = wheel_left,
        .right_wheel = wheel_right,
        
        .half_track_width = 0.15f,
        .wheel_radius = 0.03f,
        .encoder_cpr = 900,
        
        .max_linear_velocity = 0.8f,
        .max_angular_velocity = 2.0f,
        .max_wheel_rad_s = 20.0f,
        
        .max_adc_raw_threshold = 3000,
        .monitor_period_ms = 50,
        .safety_task_priority = 5,
        .safety_task_stack_size = configMINIMAL_STACK_SIZE * 4,

        .max_accel_linear = 0.5f,
        .max_accel_angular = 1.5f,
        .control_period_ms = 20,
        .control_task_priority = 4,
        .control_task_stack_size = 3072
    };

    /* Initialize differential drive controller */
    ESP_ERROR_CHECK(DiffDrive_Init(&drive_config));

    /* ----------------------------------------------------------------
     * 4. Perform Dynamic Maximum Speed Calibration
     * ---------------------------------------------------------------- */
    ESP_LOGI(TAG, "Starting calibration. Ensure robot path is clear!");
    vTaskDelay(pdMS_TO_TICKS(2000));

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

    diffDriveTwist_t cmd_vel = {0};
  
    /* Request first target command from user console */
    xTaskNotify(console_in_task_handle, 0, eNoAction);
    
    /* Block execution until initial command is received */
    xQueueReceive(s_cmd_queue, &cmd_vel, portMAX_DELAY);

    ESP_ERROR_CHECK(DiffDrive_SetTwist(&cmd_vel));

    /* Restore normal system logging levels */
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("diff_drive", ESP_LOG_INFO);
    esp_log_level_set("wheel", ESP_LOG_INFO);

    ESP_LOGI(TAG, "First valid command applied (Linear=%.3f, Angular=%.3f). Restoring log verbosity.",
             cmd_vel.linear_x, cmd_vel.angular_z);

    diffDriveTwist_t estimated_vel = {0};

    ESP_LOGI(TAG, "Application loop (odometry + telemetry) running at 20 Hz...");

    const TickType_t task_period = pdMS_TO_TICKS(50); // 20 Hz
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t previous_tick = last_wake_time;

    const TickType_t fault_retry_cooldown = pdMS_TO_TICKS(3000);
    TickType_t last_fault_retry_tick = 0;

    /* ----------------------------------------------------------------
     * 5. Main Execution Loop
     * ---------------------------------------------------------------- */
    for ( ; ; )
    {
        /* 1. Check safety fault status */
        if (DiffDrive_IsFaultActive())
        {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_fault_retry_tick) >= fault_retry_cooldown)
            {
                xTaskNotify(console_in_task_handle, 0, eNoAction);
                if (xQueueReceive(s_cmd_queue, &cmd_vel, 0) == pdTRUE)
                {
                    DiffDrive_ClearFault();
                    
                    esp_err_t resume_err = DiffDrive_SetTwist(&cmd_vel);
                    if (resume_err != ESP_OK)
                    {
                        ESP_LOGW(TAG, "Resume command rejected (%s) -- retrying after cooldown.",
                                 esp_err_to_name(resume_err));
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Safety lock active! Cleared fault and re-issued target twist.");
                    }

                    last_fault_retry_tick = now;
                }
            }

            previous_tick = xTaskGetTickCount();
            vTaskDelayUntil(&last_wake_time, task_period);
            continue;
        }

        /* 2. Calculate actual delta time (dt) in seconds */
        TickType_t current_tick = xTaskGetTickCount();
        float dt_seconds = (float)(current_tick - previous_tick) * portTICK_PERIOD_MS / 1000.0f;
        previous_tick = current_tick;

        /* 3. Non-blocking check for updated velocity commands from console */
        diffDriveTwist_t new_cmd_from_console;
        if (xQueueReceive(s_cmd_queue, &new_cmd_from_console, 0) == pdTRUE)
        {
            cmd_vel = new_cmd_from_console;
            esp_err_t set_err = DiffDrive_SetTwist(&cmd_vel);
            if (set_err != ESP_OK)
            {
                ESP_LOGW(TAG, "New command rejected: %s", esp_err_to_name(set_err));
            }
        }

        /* 4. Retrieve platform odometry estimation via Forward Kinematics */
        if (dt_seconds > 0.0f)
        {
            esp_err_t err = DiffDrive_GetTwist(dt_seconds, &estimated_vel);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "[Odometry] Linear X: %.3f m/s | Angular Z: %.3f rad/s",
                         estimated_vel.linear_x, estimated_vel.angular_z);
            }
        }

        /* Maintain 20 Hz periodic timing */
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}