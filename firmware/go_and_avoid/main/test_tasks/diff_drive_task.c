/**
 * @file    diff_drive_task.c
 * @brief   FreeRTOS task implementation demonstrating usage of the diff_drive library, including
 *          the periodic motion control task (`diff_control_tsk`), background safety task
 *          (`diff_safety_tsk`), and interactive console task for user velocity commands.
 *
 * @date    Jan 2025 / Refactored
 * @author  Matheus
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h" // <-- ADICIONADO: Para gerenciar filas do FreeRTOS
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
#define MOTOR_LEFT_PWM_A           GPIO_NUM_12
/** Left motor MCPWM output pin B. */
#define MOTOR_LEFT_PWM_B           GPIO_NUM_13
/** Right motor MCPWM output pin A. */
#define MOTOR_RIGHT_PWM_A          GPIO_NUM_11
/** Right motor MCPWM output pin B. */
#define MOTOR_RIGHT_PWM_B          GPIO_NUM_10

/** Maximum allowed power limit derived from wheel driver tick definitions. */
#define WHEEL_POWER_MAX            WHEEL_PWM_DUTY_TICK_MAX

/* ============================================================================
 *                          ENCODER GPIO PINS
 * ==========================================================================*/

/** Left encoder quadrature channel A pin. */
#define ENCODER_LEFT_A             GPIO_NUM_7
/** Left encoder quadrature channel B pin. */
#define ENCODER_LEFT_B             GPIO_NUM_6
/** Right encoder quadrature channel A pin. */
#define ENCODER_RIGHT_A            GPIO_NUM_21
/** Right encoder quadrature channel B pin. */
#define ENCODER_RIGHT_B            GPIO_NUM_14

/* ============================================================================
 *                          ADC CURRENT SENSE CHANNELS
 * ==========================================================================*/

/** ADC channel mapped to left motor current sensor. */
#define ADC_LEFT_CHANNEL           ADC_CHANNEL_8
/** ADC channel mapped to right motor current sensor. */
#define ADC_RIGHT_CHANNEL          ADC_CHANNEL_1

/* ============================================================================
 *                     COMUNICAÇÃO E CONSOLE (NOVO)
 * ==========================================================================*/

/** Fila global para passar novos comandos de velocidade vindos do terminal */
static QueueHandle_t s_cmd_queue = NULL;

/**
 * @brief Task dedicada a aguardar entradas no terminal sem travar a malha de controle.
 */
static void ConsoleInputTask(void *pvParameters)
{
    char buffer[128];
    diffDriveTwist_t new_cmd;

    // Desativa buffer para garantir leitura/escrita imediata no stdout/stdin
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    for (;;)
    {
        printf("\n[Console] Digite a nova velocidade (linear_x angular_z): ");
        fflush(stdout);

        // Bloqueia AQUI aguardando entrada do usuário. Como é uma task separada, 
        // a DiffDriveCtrl continua rodando livremente na CPU.
        if (fgets(buffer, sizeof(buffer), stdin) != NULL)
        {
            // Remove quebras de linha
            buffer[strcspn(buffer, "\r\n")] = 0;

            if (strlen(buffer) > 0)
            {
                // Tenta extrair os 2 floats
                if (sscanf(buffer, "%f %f", &new_cmd.linear_x, &new_cmd.angular_z) == 2)
                {
                    ESP_LOGI(TAG, "Novo comando recebido do terminal: Linear=%.3f, Angular=%.3f",
                             new_cmd.linear_x, new_cmd.angular_z);

                    // Envia para a fila da task principal
                    if (s_cmd_queue != NULL)
                    {
                        xQueueSend(s_cmd_queue, &new_cmd, portMAX_DELAY);
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Formato inválido! Digite dois números separados por espaço (ex: 0.3 0.1).");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================================
 *                          TASK IMPLEMENTATIONS
 * ==========================================================================*/

portTASK_FUNCTION(DiffDriveCtrl, arg)
{
    /* ----------------------------------------------------------------
     * 0. Criar Fila e Task de Console
     * ---------------------------------------------------------------- */
    s_cmd_queue = xQueueCreate(5, sizeof(diffDriveTwist_t));
    if (s_cmd_queue == NULL)
    {
        ESP_LOGE(TAG, "Falha ao criar fila de comandos!");
        vTaskDelete(NULL);
        return;
    }

    // Cria a task do console com prioridade mais baixa (1) e stack suficiente para stdio
    xTaskCreate(ConsoleInputTask, "console_input_task", 3072, NULL, 1, NULL);

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

    /* 2. Populate differential drive configuration structure */
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
        .safety_task_stack_size = 3072,

        .max_accel_linear = 0.5f,
        .max_accel_angular = 1.5f,
        .control_period_ms = 20,
        .control_task_priority = 4,
        .control_task_stack_size = 3072
    };

    /* 3. Initialize differential drive controller */
    ESP_ERROR_CHECK(DiffDrive_Init(&drive_config));

    /* 4. Perform dynamic maximum speed calibration */
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

    diffDriveTwist_t cmd_vel = {
        .linear_x = 0.3f,
        .angular_z = 0.2f
    };

    ESP_ERROR_CHECK(DiffDrive_SetTwist(&cmd_vel));

    diffDriveTwist_t estimated_vel = {0};

    ESP_LOGI(TAG, "Application loop (odometry + telemetry) running at 20 Hz...");

    const TickType_t task_period = pdMS_TO_TICKS(50); // 20 Hz
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t previous_tick = last_wake_time;

    const TickType_t fault_retry_cooldown = pdMS_TO_TICKS(3000);
    TickType_t last_fault_retry_tick = 0;
    bool waitForVel = false;

    for ( ; ; )
    {
        /* 1. Check safety status */
        if (DiffDrive_IsFaultActive())
        {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_fault_retry_tick) >= fault_retry_cooldown) // Se passou o tempo dos motores se restabelecerem
            {
                /* 0. E se chegou um novo comando vindo da ConsoleInputTask (Não bloqueante) */
                diffDriveTwist_t new_cmd_from_console;
                if (xQueueReceive(s_cmd_queue, &new_cmd_from_console, 0) == pdTRUE)
                {
                    cmd_vel = new_cmd_from_console; // Atualiza o comando local de referência
                    
                    DiffDrive_ClearFault();
                    
                    esp_err_t resume_err = DiffDrive_SetTwist(&cmd_vel);
                    // Re-aplica o último comando válido que foi salvo
                    if (resume_err != ESP_OK)
                    {
                        ESP_LOGW(TAG, "Resume command rejected (%s) -- will retry after cooldown.",
                                 esp_err_to_name(resume_err));
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Safety lock active! Clearing fault and re-issuing target twist...");
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

        /* Wait until the start of the next 50 ms cycle */
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}