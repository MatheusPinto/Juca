/**
 * @file      diff_drive_task.c
 * @brief     FreeRTOS application entry point for differential drive mobile robot control.
 * @details   This module implements the main control task (`diff_drive_ctrl_task`) and an asynchronous
 *            interactive console task (`console_input_task`). It initializes the hardware peripherals 
 *            (MCPWM, Quadrature Encoders, ADC Oneshot), sets up the differential drive controller 
 *            (kinematics, acceleration ramp and overcurrent protection), executes automatic max-speed 
 *            calibration, and handles non-blocking velocity commands alongside automatic safety fault monitoring.
 *
 * @date      Jan 2025 / Refactored
 * @author    Matheus
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#else
#include "driver/uart_vfs.h"
#endif
#include "diff_drive.h"
#include "diff_drive_task.h"
#include "juca_board.h"

/** 
 * @brief Logging tag used across ESP-IDF system log macros. 
 */
static const char *TAG = "diff_drive_task";

/* Pin and ADC channel map: see juca_board.h (single source of truth). */

/* ============================================================================
 *                    COMMUNICATION AND CONSOLE GLOBALS
 * ==========================================================================*/

/** 
 * @brief Global queue handle for passing velocity commands (`diff_drive_twist_t`) 
 *        from the console reader task to the main control task.
 */
static QueueHandle_t s_command_queue = NULL;

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
    usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

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
 * @brief Parses a console line of the form "<linear_x> <angular_z>".
 *
 * @details `sscanf("%f")` also accepts "nan" and "inf", which are not valid velocities, so non-finite
 *          values are rejected here.
 *
 * @param[in]  line      NUL-terminated input line.
 * @param[out] out_twist Receives the parsed command; left untouched when parsing fails.
 * @return true  Both values were parsed and are finite.
 * @return false The line is malformed or contains NaN/Inf.
 */
static bool parse_twist_line(const char *line, diff_drive_twist_t *out_twist)
{
    float linear_x = 0.0f;
    float angular_z = 0.0f;

    if (sscanf(line, "%f %f", &linear_x, &angular_z) != 2 || !isfinite(linear_x) || !isfinite(angular_z))
    {
        return false;
    }

    out_twist->linear_x = linear_x;
    out_twist->angular_z = angular_z;
    return true;
}

/**
 * @brief FreeRTOS task dedicated to reading user input from the console asynchronously.
 *
 * @details Suspends execution until notified by the control task. Upon notification, 
 *          it prompts the operator for target linear and angular velocities (`linear_x` and `angular_z`), 
 *          blocks until input is submitted via stdin, parses the input, and pushes the parsed command 
 *          into `s_command_queue`. The control task notifies it again after consuming each command, so 
 *          any number of commands can be typed. A malformed or non-finite line is sent as a stop 
 *          command (0 0); an end-of-file/read error sends nothing and keeps waiting for a line.
 *
 * @param[in] arg Pointer to task parameter list (unused).
 */
static void console_input_task(void *arg)
{
    (void)arg;
    char line[64];
    diff_drive_twist_t new_command = {0};

    /* Initialize USB Serial/JTAG VFS drivers */
    init_usb_cdc_stdio();

    printf("\n--- ESP32-S3 Console Ready ---\n");
    
    uint32_t notified_bits;
    
    while (1)
    {
        /* Wait for notification from main controller before asking for input */
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &notified_bits, portMAX_DELAY); 

        printf("Enter target velocity (linear_x angular_z): \n");
        fflush(stdout); /* Flush output buffer immediately */

        /* Block until the user submits input with ENTER. End-of-file or an I/O error yields no line:
         * there is no command to send, and resending the previous one would make the control task apply
         * it again (even to clear a fault) -- so keep waiting instead. */
        while (fgets(line, sizeof(line), stdin) == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        /* Strip trailing carriage return and newline characters */
        line[strcspn(line, "\r\n")] = 0;

        if (parse_twist_line(line, &new_command))
        {
            printf("Accepted target -> linear_x = %.2f, angular_z = %.2f\n",
                   new_command.linear_x, new_command.angular_z);
        }
        else
        {
            new_command.linear_x = 0;
            new_command.angular_z = 0;
            printf("Invalid input! Received: '%s'\n", line);
        }

        if (s_command_queue != NULL)
        {
            xQueueSend(s_command_queue, &new_command, portMAX_DELAY);
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
portTASK_FUNCTION(diff_drive_ctrl_task, arg)
{
    s_command_queue = xQueueCreate(5, sizeof(diff_drive_twist_t));
    if (s_command_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create command queue!");
        vTaskDelete(NULL);
        return;
    }

    TaskHandle_t console_input_task_handle = NULL;
    /* Create console input task with priority 1 and adequate stack depth */
    xTaskCreate(console_input_task, "console_input", 3072, NULL, 1, &console_input_task_handle);

    /* ----------------------------------------------------------------
     * 1. Initialize Shared ADC Oneshot Unit
     * ---------------------------------------------------------------- */
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = JUCA_ADC_UNIT_CURRENT,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));

    adc_oneshot_chan_cfg_t adc_channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JUCA_ADC_CHANNEL_CURRENT_LEFT, &adc_channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JUCA_ADC_CHANNEL_CURRENT_RIGHT, &adc_channel_config));

    /* ----------------------------------------------------------------
     * 2. Configure Wheel Instances
     * ---------------------------------------------------------------- */
    wheel_config_t left_config = {
        .pwm_a_gpio = JUCA_PIN_MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = JUCA_PIN_MOTOR_LEFT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_ticks = 400,
        .encoder_a_gpio = JUCA_PIN_ENCODER_LEFT_A,
        .encoder_b_gpio = JUCA_PIN_ENCODER_LEFT_B,
        .adc_handle = adc_handle,
        .adc_channel = JUCA_ADC_CHANNEL_CURRENT_LEFT,
    };

    wheel_config_t right_config = {
        .pwm_a_gpio = JUCA_PIN_MOTOR_RIGHT_PWM_A,
        .pwm_b_gpio = JUCA_PIN_MOTOR_RIGHT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_ticks = 400,
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
     * 3. Populate Differential Drive Configuration Structure
     * ---------------------------------------------------------------- */
    diff_drive_config_t drive_config = {
        .left_wheel = left_wheel,
        .right_wheel = right_wheel,
        
        .left_encoder_cpr = JUCA_ENCODER_LEFT_CPR,
        .right_encoder_cpr = JUCA_ENCODER_RIGHT_CPR,

        .half_track_width_m = JUCA_HALF_TRACK_WIDTH_M,
        .wheel_radius_m = JUCA_WHEEL_RADIUS_M,
        
        /* Putting this values to zero, disable them */
        .max_linear_velocity_m_s = 0,
        .max_angular_velocity_rad_s = 0,
        
        /* It is a stimated initial value. It must be > 0. However, it is ignored if calibration is run. */
        .max_wheel_rad_s = 27,
        
        .max_adc_raw_threshold = 3000,
        .monitor_period_ms = 50,
        .safety_task_priority = 5,
        .safety_task_stack_size = configMINIMAL_STACK_SIZE * 4,

        .max_accel_linear_m_s2 = 0.5f,
        .max_accel_angular_rad_s2 = 1.5f,
        .control_period_ms = 20,
        .control_task_priority = 4,
        .control_task_stack_size = 3072
    };

    /* Initialize differential drive controller */
    ESP_ERROR_CHECK(diff_drive_init(&drive_config));

    /* ----------------------------------------------------------------
     * 4. Perform Dynamic Maximum Speed Calibration
     * ---------------------------------------------------------------- */
    ESP_LOGI(TAG, "Starting calibration. Ensure robot path is clear!");
    vTaskDelay(pdMS_TO_TICKS(2000));

    float measured_max_rad_s = 0.0f;
    esp_err_t calibration_status = diff_drive_calibrate_max_speed(6000, &measured_max_rad_s);
    if (calibration_status == ESP_OK)
    {
        ESP_LOGI(TAG, "Auto-calibration successful! Max wheel speed: %.2f rad/s", measured_max_rad_s);
    }
    else
    {
        ESP_LOGW(TAG, "Calibration skipped or failed (%s). Using fallback configuration.", esp_err_to_name(calibration_status));
    }

    diff_drive_twist_t command_twist = {0};
  
    /* Request first target command from user console */
    xTaskNotify(console_input_task_handle, 0, eNoAction);
    
    /* Block execution until initial command is received */
    xQueueReceive(s_command_queue, &command_twist, portMAX_DELAY);

    ESP_ERROR_CHECK(diff_drive_set_twist(&command_twist));

     ESP_LOGI(TAG, "First valid command applied (Linear=%.3f, Angular=%.3f). Restoring log verbosity.",
             command_twist.linear_x, command_twist.angular_z);

    /* The console task only prompts when notified: ask for the next command right away */
    xTaskNotify(console_input_task_handle, 0, eNoAction);

    diff_drive_twist_t estimated_twist = {0};

    ESP_LOGI(TAG, "Application loop (odometry + telemetry) running at 20 Hz...");

    const TickType_t task_period_ticks = pdMS_TO_TICKS(50); // 20 Hz
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t previous_tick = last_wake_tick;

    const TickType_t fault_retry_cooldown_ticks = pdMS_TO_TICKS(3000);
    TickType_t last_fault_retry_tick = 0;

    int left_current_raw = 0, right_current_raw = 0;

    /* ----------------------------------------------------------------
     * 5. Main Execution Loop
     * ---------------------------------------------------------------- */
    for ( ; ; )
    {
        /* 1. Check safety fault status */
        if (diff_drive_is_fault_active())
        {
            TickType_t now_tick = xTaskGetTickCount();
            if ((now_tick - last_fault_retry_tick) >= fault_retry_cooldown_ticks)
            {
                xTaskNotify(console_input_task_handle, 0, eNoAction);
                if (xQueueReceive(s_command_queue, &command_twist, 0) == pdTRUE)
                {
                    diff_drive_clear_fault();
                    
                    esp_err_t resume_status = diff_drive_set_twist(&command_twist);
                    if (resume_status != ESP_OK)
                    {
                        ESP_LOGW(TAG, "Resume command rejected (%s) -- retrying after cooldown.",
                                 esp_err_to_name(resume_status));
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Safety lock active! Cleared fault and re-issued target twist.");
                    }

                    last_fault_retry_tick = now_tick;

                    /* Ask for the next command */
                    xTaskNotify(console_input_task_handle, 0, eNoAction);
                }
            }

            previous_tick = xTaskGetTickCount();
            vTaskDelayUntil(&last_wake_tick, task_period_ticks);
            continue;
        }

        /* 2. Calculate actual delta time (dt) in seconds */
        TickType_t current_tick = xTaskGetTickCount();
        float delta_time_s = (float)(current_tick - previous_tick) * portTICK_PERIOD_MS / 1000.0f;
        previous_tick = current_tick;

        /* 3. Non-blocking check for updated velocity commands from console */
        diff_drive_twist_t new_command_from_console;
        if (xQueueReceive(s_command_queue, &new_command_from_console, 0) == pdTRUE)
        {
            command_twist = new_command_from_console;
            esp_err_t set_twist_status = diff_drive_set_twist(&command_twist);
            if (set_twist_status != ESP_OK)
            {
                ESP_LOGW(TAG, "New command rejected: %s", esp_err_to_name(set_twist_status));
            }

            /* Ask for the next command */
            xTaskNotify(console_input_task_handle, 0, eNoAction);
        }

        /* 4. Retrieve platform odometry estimation via Forward Kinematics */
        if (delta_time_s > 0.0f)
        {            
            esp_err_t odometry_status = diff_drive_get_twist(delta_time_s, &estimated_twist);
            if (odometry_status == ESP_OK)
            {
               /*ESP_LOGI(TAG, "[Odometry] Linear X: %.3f m/s | Angular Z: %.3f rad/s",
                         estimated_twist.linear_x, estimated_twist.angular_z);*/
            }

            wheel_get_current_raw(left_wheel, &left_current_raw);
            wheel_get_current_raw(right_wheel, &right_current_raw);

            ESP_LOGI(TAG, "[FWD] Left ADC=%d | Right ADC=%d",
                 left_current_raw, right_current_raw);
        }  

        /* Maintain 20 Hz periodic timing */
        vTaskDelayUntil(&last_wake_tick, task_period_ticks);
    }
}
