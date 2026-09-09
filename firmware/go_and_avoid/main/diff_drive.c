/**
 * @file    diff_drive.c
 * @brief   Implementation file for the differential drive control library.
 *
 * @details Implements all API functions declared in diff_drive.h using a thread-safe Singleton pattern.
 *          Handles mathematical transformations for forward/inverse kinematics, auto-calibration routines,
 *          and FreeRTOS background task management for software safety checks.
 *
 * @author  Matheus
 * @date    Jan 2025
 */

#include "diff_drive.h"
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"

/** @brief Global logging tag used across ESP-IDF system logging macros in this module. */
static const char *TAG = "diff_drive";

/**
 * @brief Internal static context encapsulating all runtime variables for the Singleton instance.
 *
 * Allocated strictly within the `.bss` static memory segment during compilation to ensure 
 * deterministic memory footprint and eliminate dynamic allocation (heap fragmentation).
 */
typedef struct {
    bool is_initialized;               /**< Flag indicating if the module has undergone successful initialization. */
    wheelHandle_t left_wheel;          /**< Left motor driver abstraction handle. */
    wheelHandle_t right_wheel;         /**< Right motor driver abstraction handle. */

    float half_track_width;                 /**< Lateral distance between drive wheel centers (L) in meters. */
    float wheel_radius;                /**< Radius of drive wheels (r) in meters. */
    uint32_t encoder_cpr;              /**< Encoder pulses per single full revolution in 4x mode. */

    float max_linear_velocity;         /**< Configured software safety upper bound for linear speed (m/s). */
    float max_angular_velocity;        /**< Configured software safety upper bound for angular speed (rad/s). */
    float max_wheel_rad_s;             /**< Absolute maximum wheel angular velocity (rad/s) at 100% duty cycle. */

    /* --- Calibration Trim Multipliers --- */
    float left_trim_factor;            /**< Multiplicative scaling factor for left motor power compensation [0.0 - 1.0]. */
    float right_trim_factor;           /**< Multiplicative scaling factor for right motor power compensation [0.0 - 1.0]. */

    /* --- Safety Task State & Context --- */
    int max_adc_raw_threshold;         /**< Raw ADC threshold limit for current protection. */
    uint32_t monitor_period_ms;        /**< Execution cycle period for safety monitor task in ms. */
    bool is_fault_active;              /**< Emergency latch flag. When true, blocks motion command execution. */
    TaskHandle_t monitor_task_handle;  /**< Task handle returned by xTaskCreate for safety task management. */
    bool monitor_task_running;         /**< Flag controlling the lifetime loop of the safety monitoring task. */

    int last_left_enc_count;           /**< Encoder count for left wheel cached from previous telemetry sample. */
    int last_right_enc_count;          /**< Encoder count for right wheel cached from previous telemetry sample. */

    /* --- Periodic Motion Control Task State & Context --- */
    float max_accel_linear;            /**< Configured max linear acceleration (m/s^2) for the ramp limiter. <= 0.0f disables ramping. */
    float max_accel_angular;           /**< Configured max angular acceleration (rad/s^2) for the ramp limiter. <= 0.0f disables ramping. */
    uint32_t control_period_ms;        /**< Execution cycle period for the periodic control task in ms. */
    diffDriveTwist_t target_twist;     /**< Desired twist most recently registered via @ref DiffDrive_SetTwist. Guarded by #twist_spin. */
    diffDriveTwist_t applied_twist;    /**< Ramped twist currently being tracked/output by the control task. Exclusively owned by `diff_control_tsk` -- no locking needed. */
    TaskHandle_t control_task_handle;  /**< Task handle returned by xTaskCreate for the periodic control task. */
    bool control_task_running;         /**< Flag controlling the lifetime loop of the periodic control task. */
} diffDriveContext_t;

/**
 * @brief Static Singleton context instance initialized with default zero memory.
 */
static diffDriveContext_t s_ctx = {0};


/** 
 * @brief Spinlock protecting concurrent access to #is_fault_active.
 */
static portMUX_TYPE safety_flag_spin = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief Spinlock protecting concurrent access to #target_twist.
 *
 * @details `target_twist` is written by any task calling @ref DiffDrive_SetTwist and read once per 
 *          cycle by the periodic control task. `applied_twist` is exclusively owned by the control 
 *          task and therefore does NOT need this lock.
 */
static portMUX_TYPE twist_spin = portMUX_INITIALIZER_UNLOCKED;

/* ============================================================================
 *                     PRIVATE HELPER FUNCTIONS
 * ==========================================================================*/

/**
 * @brief Helper utility to clamp a floating-point value within a strictly closed interval [min, max].
 *
 * @param[in] val Value to clamp.
 * @param[in] min_val Lower boundary limit.
 * @param[in] max_val Upper boundary limit.
 * @return float Clamped result bounded within [min_val, max_val].
 */
static inline float DiffDrive_Clampf(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

/**
 * @brief Applies inverse differential kinematics and dispatches PWM power to both wheel drivers.
 *
 * @details Internal counterpart of the logic formerly inlined in the public `DiffDrive_SetTwist`. 
 *          Called exclusively by `diff_control_tsk` once per control cycle, with the already-ramped 
 *          twist. Performs NO clamping, NO initialization checks, and NO fault checks -- those are 
 *          the caller's responsibility (see @ref DiffDrive_SetTwist and `DiffDrive_ControlTask`).
 *
 * @param[in] twist Pointer to the twist to apply this cycle (already clamped/ramped).
 * @return esp_err_t ESP_OK on success, or the first underlying @ref Wheel_SetPower error.
 */
static esp_err_t DiffDrive_ApplyTwistInternal(const diffDriveTwist_t *twist)
{
    /* Inverse Kinematics Step 1: Calculate linear tangential velocities for left and right wheels (m/s) */
    float v_left = twist->linear_x - (twist->angular_z * s_ctx.half_track_width);
    float v_right = twist->linear_x + (twist->angular_z * s_ctx.half_track_width);

    /* Inverse Kinematics Step 2: Convert linear tangential speeds (m/s) into wheel rotational speeds (rad/s) */
    float omega_left = v_left / s_ctx.wheel_radius;
    float omega_right = v_right / s_ctx.wheel_radius;

    /* Inverse Kinematics Step 3: Compute conversion scalar ratio (PWM Duty Ticks / Maximum rad/s) */
    float power_scale = (float)WHEEL_PWM_DUTY_TICK_MAX / s_ctx.max_wheel_rad_s;

    /* Inverse Kinematics Step 4: Scale to PWM values and apply dynamic balance trim multipliers */
    int32_t left_power = (int32_t)lroundf(omega_left * power_scale * s_ctx.left_trim_factor);
    int32_t right_power = (int32_t)lroundf(omega_right * power_scale * s_ctx.right_trim_factor);

    /* Inverse Kinematics Step 5: Dispatch calculated power levels to low-level motor drivers */
    ESP_RETURN_ON_ERROR(Wheel_SetPower(s_ctx.left_wheel, left_power), TAG, "Failed to apply power to left wheel driver");
    ESP_RETURN_ON_ERROR(Wheel_SetPower(s_ctx.right_wheel, right_power), TAG, "Failed to apply power to right wheel driver");

    return ESP_OK;
}

/**
 * @brief Applies a linear acceleration rate limiter (ramp) independently on each twist axis.
 *
 * @details Moves `current` towards `target` by at most `max_accel * dt` per axis, per call. 
 *          Passing `max_accel <= 0.0f` for an axis disables limiting on that axis (jumps straight 
 *          to target), which is convenient for testing or for platforms where the caller already 
 *          shapes the trajectory upstream.
 *
 * @param[in]  current            Twist currently being tracked (previous cycle's ramped output).
 * @param[in]  target              Desired twist to ramp towards.
 * @param[in]  dt                  Elapsed time since the previous cycle, in seconds.
 * @param[in]  max_accel_linear    Maximum linear acceleration magnitude (m/s^2). <= 0.0f disables.
 * @param[in]  max_accel_angular   Maximum angular acceleration magnitude (rad/s^2). <= 0.0f disables.
 * @param[out] out                 Resulting ramped twist for this cycle.
 */
static void DiffDrive_ApplyRamp(const diffDriveTwist_t *current, const diffDriveTwist_t *target, float dt,
                                 float max_accel_linear, float max_accel_angular, diffDriveTwist_t *out)
{
    if (max_accel_linear <= 0.0f)
    {
        out->linear_x = target->linear_x;
    }
    else
    {
        float max_dv = max_accel_linear * dt;
        float dv = DiffDrive_Clampf(target->linear_x - current->linear_x, -max_dv, max_dv);
        out->linear_x = current->linear_x + dv;
    }

    if (max_accel_angular <= 0.0f)
    {
        out->angular_z = target->angular_z;
    }
    else
    {
        float max_dw = max_accel_angular * dt;
        float dw = DiffDrive_Clampf(target->angular_z - current->angular_z, -max_dw, max_dw);
        out->angular_z = current->angular_z + dw;
    }
}

/**
 * @brief Background FreeRTOS task that periodically ramps towards the last commanded twist and 
 *        dispatches the resulting motor power.
 *
 * @details Runs at a fixed period (`control_period_ms`) and LOWER priority than `diff_safety_tsk`, 
 *          so an overcurrent/stall detection can always preempt and brake immediately, even if this 
 *          task is mid-cycle. Each cycle:
 *          1. Atomically samples the current target twist (set by @ref DiffDrive_SetTwist).
 *          2. If an emergency fault is latched, resets the internal ramp state to zero -- so that 
 *             once the fault is cleared, motion resumes gradually from a stop instead of jumping 
 *             straight to whatever speed was last commanded.
 *          3. Otherwise, advances the ramp towards the target and applies it via 
 *             @ref DiffDrive_ApplyTwistInternal.
 *
 * @param[in] pvParameters Unused parameter pointer required by FreeRTOS task function signatures.
 */
static void DiffDrive_ControlTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(s_ctx.control_period_ms);
    const float dt = (float)s_ctx.control_period_ms / 1000.0f;

    ESP_LOGI(TAG, "Periodic control task started successfully (period=%lu ms).", s_ctx.control_period_ms);

    while (s_ctx.control_task_running)
    {
        diffDriveTwist_t target;
        portENTER_CRITICAL(&twist_spin);
        target = s_ctx.target_twist;
        portEXIT_CRITICAL(&twist_spin);

        portENTER_CRITICAL(&safety_flag_spin);
        bool fault = s_ctx.is_fault_active;
        portEXIT_CRITICAL(&safety_flag_spin);

        if (fault)
        {
            /* Freeze the ramp at zero while faulted so recovery after DiffDrive_ClearFault()
             * accelerates smoothly from a stop rather than snapping to the stale target. */
            s_ctx.applied_twist.linear_x = 0.0f;
            s_ctx.applied_twist.angular_z = 0.0f;
        }
        else
        {
            diffDriveTwist_t ramped;
            DiffDrive_ApplyRamp(&s_ctx.applied_twist, &target, dt,
                                 s_ctx.max_accel_linear, s_ctx.max_accel_angular, &ramped);
            s_ctx.applied_twist = ramped;
            DiffDrive_ApplyTwistInternal(&s_ctx.applied_twist);
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }

    s_ctx.control_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Background FreeRTOS task routine that monitors motor driver current draw.
 *
 * @details Periodically polls raw ADC current sensor readings from both wheel handles. If either 
 *          reading exceeds `max_adc_raw_threshold` for `STALL_COUNTER_LIMIT` consecutive debounced 
 *          cycles, the routine latches `is_fault_active = true`, forces an emergency brake on both 
 *          wheels, discards the currently registered target twist (so the platform does not resume 
 *          its previous motion the instant the fault is cleared), and logs an error.
 *
 * @note This is a HARD LATCH, not a self-clearing condition: once tripped, `is_fault_active` stays 
 *       true -- and the control task keeps the wheels at zero -- until the application explicitly 
 *       calls @ref DiffDrive_ClearFault(). A version of this task used to auto-release the latch as 
 *       soon as current dropped back down (e.g. right after the emergency brake stopped the motor), 
 *       but that caused an oscillation: the platform would un-latch, immediately try to resume its 
 *       last command, stall again against whatever was blocking it, and re-trip -- repeatedly 
 *       jerking on and off. Requiring an explicit @ref DiffDrive_ClearFault() call (ideally after 
 *       confirming the obstruction is gone) breaks that loop.
 *
 * @param[in] pvParameters Unused parameter pointer required by FreeRTOS task function signatures.
 */
static void DiffDrive_SafetyTaskRoutine(void *pvParameters)
{
    (void)pvParameters;
    int left_adc = 0;
    int right_adc = 0;
    int stall_counter = 0;
    bool active_stall = false; /* Local mirror of the latch, used for debounce/edge detection only. */

    /* Detection parameters */
    const int STALL_COUNTER_LIMIT = 10; /* number of cycles */
    
    /* Adaptive polling period */
    int ms_period = s_ctx.monitor_period_ms;

    ESP_LOGI(TAG, "Continuous current monitoring task started successfully.");

    TickType_t last_wake = xTaskGetTickCount();

    while (s_ctx.monitor_task_running) 
    {
        /* If the latch is currently active but the application has cleared it externally via
         * DiffDrive_ClearFault(), re-arm this task's local debounce state so a fresh stall can be
         * detected cleanly (otherwise stall_counter could still be sitting at the limit). */
        if (active_stall)
        {
            portENTER_CRITICAL(&safety_flag_spin);
            bool still_faulted = s_ctx.is_fault_active;
            portEXIT_CRITICAL(&safety_flag_spin);

            if (!still_faulted)
            {
                active_stall = false;
                stall_counter = 0;
                ESP_LOGI(TAG, "Fault cleared by application -- safety task re-armed.");
            }
        }

        /* Poll current sensor readings from lower-level motor driver handles */
        esp_err_t err_l = Wheel_GetCurrentRaw(s_ctx.left_wheel, &left_adc);
        esp_err_t err_r = Wheel_GetCurrentRaw(s_ctx.right_wheel, &right_adc);

        if (err_l == ESP_OK || err_r == ESP_OK) 
        {
            /* Check if raw current reading exceeds configured safety limit */
           bool high_current = (left_adc > s_ctx.max_adc_raw_threshold || 
                                right_adc > s_ctx.max_adc_raw_threshold);

           /* ============================
            * Dynamic period adjustment
            * ============================ */
           if (high_current)
           {
               if (ms_period > 10)
               {
                   ms_period /= 2;
                   if (ms_period < 10)
                       ms_period = 10;

                   /* Avoid drift when changing the period too fast */
                   last_wake = xTaskGetTickCount();
               }
           }
           else
           {
               if (ms_period < s_ctx.monitor_period_ms)
               {
                   ms_period += 10;
                   if (ms_period > s_ctx.monitor_period_ms)
                        ms_period = s_ctx.monitor_period_ms;
               }
           }

           /* ============================
            * Stall counter (debounce)
            * ============================ */
           if (!active_stall)
           {
               /* Only accumulate/decay the debounce counter while not yet latched -- once latched,
                * the counter is irrelevant until the application clears the fault and we re-arm above. */
               if (high_current)
               {
                   if (stall_counter < STALL_COUNTER_LIMIT)
                       stall_counter++;
               }
               else
               {
                   if (stall_counter > 0)
                       stall_counter--;
               }
           }

           /* ============================
            * Stall detection -> latch fault (no automatic release)
            * ============================ */
           if (!active_stall && stall_counter >= STALL_COUNTER_LIMIT)
           {
               active_stall = true;

               portENTER_CRITICAL(&safety_flag_spin);
               s_ctx.is_fault_active = true;
               portEXIT_CRITICAL(&safety_flag_spin);

               /* Trigger immediate hardware-level active brake stop */
               Wheel_Brake(s_ctx.left_wheel);
               Wheel_Brake(s_ctx.right_wheel);

               /* Discard the previously registered target so that, once the application calls
                * DiffDrive_ClearFault(), the control task ramps from zero instead of instantly
                * driving straight back into whatever caused the stall. A fresh DiffDrive_SetTwist()
                * call is required to move again after recovery. */
               portENTER_CRITICAL(&twist_spin);
               s_ctx.target_twist.linear_x = 0.0f;
               s_ctx.target_twist.angular_z = 0.0f;
               portEXIT_CRITICAL(&twist_spin);

               ESP_LOGE(TAG, "OVERCURRENT DETECTED! Left ADC: %d, Right ADC: %d (Threshold: %d). Emergency brake activated! "
                             "Fault latched -- call DiffDrive_ClearFault() and re-issue DiffDrive_SetTwist() to resume.",
                        left_adc, right_adc, s_ctx.max_adc_raw_threshold);
           }

           /* ============================
            * Deterministic delay
            * ============================ */
           vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ms_period));
        }
    }

    /* Clean exit procedure for FreeRTOS task deletion */
    s_ctx.monitor_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ============================================================================
 *                         PUBLIC API IMPLEMENTATION
 * ==========================================================================*/

/**
 * @brief Initializes the differential drive control module, configures internal state, and launches the safety task.
 *
 * @details Copies all geometric and operational parameters into static memory context, establishes 
 *          the baseline reading for optical encoders, and launches a background FreeRTOS task 
 *          (`diff_safety_tsk`) to continuously monitor motor current consumption if thresholds are enabled.
 *
 * @param[in] config Pointer to an initialized @ref diffDriveConfig_t structure containing system parameters.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Module initialized successfully and safety task is running.
 *         - ESP_ERR_INVALID_STATE: Module is already initialized.
 *         - ESP_ERR_INVALID_ARG: Null pointer provided, or invalid geometric parameters ($L \le 0$, $r \le 0$).
 *         - ESP_FAIL: Failed to allocate or spawn the FreeRTOS safety task.
 *
 * @note Must be called once before invoking any motion control or odometry functions.
 */
esp_err_t DiffDrive_Init(const diffDriveConfig_t *config)
{
    /* Validate input arguments and initialization precondition state */
    ESP_RETURN_ON_FALSE(!s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is already initialized");
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Null configuration pointer provided");
    ESP_RETURN_ON_FALSE(config->left_wheel && config->right_wheel, ESP_ERR_INVALID_ARG, TAG, "Wheel driver handles must not be NULL");
    ESP_RETURN_ON_FALSE(config->half_track_width > 0.0f && config->wheel_radius > 0.0f, ESP_ERR_INVALID_ARG, TAG, "Physical robot dimensions must be > 0");
    ESP_RETURN_ON_FALSE(config->max_wheel_rad_s > 0.0f, ESP_ERR_INVALID_ARG, TAG, "Maximum wheel angular velocity must be > 0");

    /* Store geometric and performance parameters into global Singleton context */
    s_ctx.left_wheel = config->left_wheel;
    s_ctx.right_wheel = config->right_wheel;
    s_ctx.half_track_width = config->half_track_width;
    s_ctx.wheel_radius = config->wheel_radius;
    s_ctx.encoder_cpr = config->encoder_cpr;
    s_ctx.max_linear_velocity = config->max_linear_velocity;
    s_ctx.max_angular_velocity = config->max_angular_velocity;
    s_ctx.max_wheel_rad_s = config->max_wheel_rad_s;

    /* Initialize default power balance trim multipliers (100% gain each) */
    s_ctx.left_trim_factor = 1.0f;
    s_ctx.right_trim_factor = 1.0f;

    s_ctx.max_adc_raw_threshold = config->max_adc_raw_threshold;
    s_ctx.monitor_period_ms = (config->monitor_period_ms > 0) ? config->monitor_period_ms : 50;
    s_ctx.is_fault_active = false;

    /* Store periodic control task ramp parameters */
    s_ctx.max_accel_linear = config->max_accel_linear;
    s_ctx.max_accel_angular = config->max_accel_angular;
    s_ctx.control_period_ms = (config->control_period_ms > 0) ? config->control_period_ms : 20;
    s_ctx.target_twist = (diffDriveTwist_t){0};
    s_ctx.applied_twist = (diffDriveTwist_t){0};

    /* Seed initial encoder count values to prevent step jump in first GetTwist calculation */
    Wheel_GetEncoderCount(s_ctx.left_wheel, &s_ctx.last_left_enc_count);
    Wheel_GetEncoderCount(s_ctx.right_wheel, &s_ctx.last_right_enc_count);

    /* Resolve task parameters up front so priority ordering can be validated before anything is spawned */
    UBaseType_t safety_task_prio = (config->safety_task_priority > 0) ? config->safety_task_priority : 5;
    uint32_t safety_stack_sz = (config->safety_task_stack_size > 0) ? config->safety_task_stack_size : 3072;

    UBaseType_t control_task_prio = (config->control_task_priority > 0) ? config->control_task_priority : 4;
    uint32_t control_stack_sz = (config->control_task_stack_size > 0) ? config->control_task_stack_size : 3072;

    /* Safety invariant: the control task must never be able to starve or outrun the safety task.
     * Enforced only when the safety task is actually going to run. */
    if (s_ctx.max_adc_raw_threshold > 0)
    {
        ESP_RETURN_ON_FALSE(control_task_prio < safety_task_prio, ESP_ERR_INVALID_ARG, TAG,
                             "control_task_priority (%u) must be strictly lower than safety_task_priority (%u)",
                             (unsigned)control_task_prio, (unsigned)safety_task_prio);
    }

    /* Launch periodic motion control task (always active, regardless of safety task) */
    s_ctx.control_task_running = true;
    BaseType_t control_res = xTaskCreate(DiffDrive_ControlTask,
                                          "diff_control_tsk",
                                          control_stack_sz,
                                          NULL,
                                          control_task_prio,
                                          &s_ctx.control_task_handle);

    if (control_res != pdPASS)
    {
        s_ctx.control_task_running = false;
        ESP_LOGE(TAG, "Failed to spawn FreeRTOS periodic control task");
        return ESP_FAIL;
    }

    /* Launch current safety task if protection threshold is set */
    if (s_ctx.max_adc_raw_threshold > 0) 
    {
        s_ctx.monitor_task_running = true;

        BaseType_t res = xTaskCreate(DiffDrive_SafetyTaskRoutine,
                                     "diff_safety_tsk",
                                     safety_stack_sz,
                                     NULL,
                                     safety_task_prio,
                                     &s_ctx.monitor_task_handle);

        if (res != pdPASS) 
        {
            s_ctx.monitor_task_running = false;
            /* Roll back the already-spawned control task so Init() fails cleanly and atomically */
            s_ctx.control_task_running = false;
            ESP_LOGE(TAG, "Failed to spawn FreeRTOS current safety task");
            return ESP_FAIL;
        }
    }

    s_ctx.is_initialized = true;
    ESP_LOGI(TAG, "Differential drive module initialized successfully.");
    return ESP_OK;
}

/**
 * @brief Queries the initialization status of the differential drive controller module.
 *
 * @return true The module is initialized and ready for execution.
 * @return false The module is uninitialized or shut down.
 */
bool DiffDrive_IsInitialized(void)
{
    return s_ctx.is_initialized;
}

/**
 * @brief Safely shuts down the differential controller module, stops background tasks, and brakes motors.
 *
 * @details Flags the background safety task for clean termination, sends active braking signals 
 *          to both motor channels, and resets all internal static variables to uninitialized defaults.
 *
 * @return esp_err_t 
 *         - ESP_OK: Module cleanly deinitialized.
 *         - ESP_ERR_INVALID_STATE: Module was not previously initialized.
 */
esp_err_t DiffDrive_Deinit(void)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");

    /* Signal both background tasks to terminate their loops */
    s_ctx.control_task_running = false;
    s_ctx.monitor_task_running = false;

    /* Wait deterministically for each task to actually self-delete before proceeding.
     * Without this, either task could still run one more cycle (and, for the control
     * task, call Wheel_SetPower) after we brake below, silently re-enabling motion. */
    const TickType_t poll_step = pdMS_TO_TICKS(2);
    TickType_t waited = 0;
    const TickType_t timeout = pdMS_TO_TICKS(s_ctx.control_period_ms + s_ctx.monitor_period_ms + 100);

    while ((s_ctx.control_task_handle != NULL || s_ctx.monitor_task_handle != NULL) && waited < timeout)
    {
        vTaskDelay(poll_step);
        waited += poll_step;
    }

    if (s_ctx.control_task_handle != NULL || s_ctx.monitor_task_handle != NULL)
    {
        ESP_LOGW(TAG, "Timed out waiting for background tasks to exit; forcing deletion.");
        if (s_ctx.control_task_handle != NULL)
        {
            vTaskDelete(s_ctx.control_task_handle);
            s_ctx.control_task_handle = NULL;
        }
        if (s_ctx.monitor_task_handle != NULL)
        {
            vTaskDelete(s_ctx.monitor_task_handle);
            s_ctx.monitor_task_handle = NULL;
        }
    }

    /* Apply immediate active brake on both drive channels, now that no background task can override it */
    Wheel_Brake(s_ctx.left_wheel);
    Wheel_Brake(s_ctx.right_wheel);

    /* Reset global Singleton state context */
    s_ctx.is_initialized = false;
    ESP_LOGI(TAG, "Differential drive module deinitialized cleanly.");
    return ESP_OK;
}

/**
 * @brief Performs an empirical auto-calibration routine to match maximum motor speeds and correct straight-line drift.
 *
 * @details Drives both wheels at 100% duty cycle for a controlled duration (`test_duration_ms`), 
 *          measures actual traveled angle via encoder feedback, and calculates trim scaling factors:
 *          - If $\omega_{\text{left}} > \omega_{\text{right}}$, $\text{left\_trim\_factor} = \frac{\omega_{\text{right}}}{\omega_{\text{left}}}$ and $\text{right\_trim\_factor} = 1.0$.
 *          - If $\omega_{\text{right}} > \omega_{\text{left}}$, $\text{right\_trim\_factor} = \frac{\omega_{\text{left}}}{\omega_{\text{right}}}$ and $\text{left\_trim\_factor} = 1.0$.
 *
 * @warning This function triggers physical robot motion! Ensure the platform is located on open ground 
 *          with at least 2 meters of clear unobstructed path before invocation.
 *
 * @param[in]  test_duration_ms Duration of test run pulse in milliseconds (minimum required: 500 ms).
 * @param[out] out_max_wheel_rad_s Optional pointer to store the measured maximum velocity of the slower wheel in rad/s. Pass NULL if unneeded.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Calibration completed successfully and trim parameters saved.
 *         - ESP_ERR_INVALID_STATE: Module not initialized, or encoders disabled (`encoder_cpr` == 0).
 *         - ESP_ERR_INVALID_ARG: Provided test duration is less than 500 ms.
 *         - ESP_ERR_INVALID_RESPONSE: Zero encoder pulses detected (motors unplugged, jammed, or hardware failure).
 */
esp_err_t DiffDrive_CalibrateMaxSpeed(uint32_t test_duration_ms, float *out_max_wheel_rad_s)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");
    ESP_RETURN_ON_FALSE(s_ctx.encoder_cpr > 0, ESP_ERR_INVALID_STATE, TAG, "Encoder CPR must be > 0 for dynamic calibration");
    ESP_RETURN_ON_FALSE(test_duration_ms >= 500, ESP_ERR_INVALID_ARG, TAG, "Calibration test duration must be at least 500 ms");

    /* Suspend BOTH background tasks: the safety task must not interfere with the forced
     * 100% duty test, and the control task must not overwrite it every control_period_ms
     * with whatever twist was last commanded (it would otherwise fight this routine). */
    if (s_ctx.monitor_task_handle != NULL)
    {
        vTaskSuspend(s_ctx.monitor_task_handle);
    }
    if (s_ctx.control_task_handle != NULL)
    {
        vTaskSuspend(s_ctx.control_task_handle);
    }

    ESP_LOGI(TAG, "Starting maximum speed auto-calibration pulse (%lu ms)...", test_duration_ms);

    /* Step 1: Read starting encoder baseline counts */
    int initial_left = 0, initial_right = 0;
    Wheel_GetEncoderCount(s_ctx.left_wheel, &initial_left);
    Wheel_GetEncoderCount(s_ctx.right_wheel, &initial_right);

    /* Step 2: Command 100% full duty cycle to both motor drivers */
    Wheel_SetPower(s_ctx.left_wheel, WHEEL_PWM_DUTY_TICK_MAX);
    Wheel_SetPower(s_ctx.right_wheel, WHEEL_PWM_DUTY_TICK_MAX);

    /* Allow the physical robot to move for the designated test window */
    vTaskDelay(pdMS_TO_TICKS(test_duration_ms));

    /* Step 3: Cut power and apply active braking */
    Wheel_Brake(s_ctx.left_wheel);
    Wheel_Brake(s_ctx.right_wheel);

    /* Step 4: Read final encoder count values */
    int final_left = 0, final_right = 0;
    Wheel_GetEncoderCount(s_ctx.left_wheel, &final_left);
    Wheel_GetEncoderCount(s_ctx.right_wheel, &final_right);

    int delta_left = abs(final_left - initial_left);
    int delta_right = abs(final_right - initial_right);

    /* Step 5: Verify hardware responsiveness (non-zero displacement) */
    if (delta_left == 0 || delta_right == 0) 
    {
        ESP_LOGE(TAG, "Calibration failed: Zero encoder ticks detected. Left delta: %d, Right delta: %d", delta_left, delta_right);
        /* Wheels are physically braked above -- reset the ramp state to match before resuming,
         * otherwise the control task would instantly re-apply whatever twist was ramped-to
         * before calibration started. */
        s_ctx.applied_twist = (diffDriveTwist_t){0};
        if (s_ctx.monitor_task_handle != NULL)
        {
            vTaskResume(s_ctx.monitor_task_handle);
        }
        if (s_ctx.control_task_handle != NULL)
        {
            vTaskResume(s_ctx.control_task_handle);
        }
        return ESP_ERR_INVALID_RESPONSE;
    }

    float duration_s = (float)test_duration_ms / 1000.0f;

    /* Step 6: Convert pulse deltas into actual physical angular speeds (rad/s) */
    float rad_per_tick = (2.0f * (float)M_PI) / (float)s_ctx.encoder_cpr;
    float omega_left = ((float)delta_left * rad_per_tick) / duration_s;
    float omega_right = ((float)delta_right * rad_per_tick) / duration_s;

    ESP_LOGI(TAG, "Measured uncompensated wheel speeds -> Left: %.2f rad/s, Right: %.2f rad/s", omega_left, omega_right);

    /* Step 7: Scale down the faster wheel to match the maximum attainable speed of the slower wheel */
    if (omega_left > omega_right) 
    {
        s_ctx.left_trim_factor = omega_right / omega_left;
        s_ctx.right_trim_factor = 1.0f;
        s_ctx.max_wheel_rad_s = omega_right;
    } 
    else 
    {
        s_ctx.left_trim_factor = 1.0f;
        s_ctx.right_trim_factor = omega_left / omega_right;
        s_ctx.max_wheel_rad_s = omega_left;
    }

    ESP_LOGI(TAG, "Calibration completed. Trim Factors -> Left: %.3f, Right: %.3f | Calibrated Speed Cap: %.2f rad/s",
             s_ctx.left_trim_factor, s_ctx.right_trim_factor, s_ctx.max_wheel_rad_s);

    if (out_max_wheel_rad_s != NULL) 
    {
        *out_max_wheel_rad_s = s_ctx.max_wheel_rad_s;
    }

    /* Wheels are physically braked above -- reset the ramp state before resuming the control
     * task so it ramps up fresh instead of snapping back to the pre-calibration twist. */
    s_ctx.applied_twist = (diffDriveTwist_t){0};

    if (s_ctx.monitor_task_handle != NULL)
    {
        vTaskResume(s_ctx.monitor_task_handle);
    }
    if (s_ctx.control_task_handle != NULL)
    {
        vTaskResume(s_ctx.control_task_handle);
    }

    return ESP_OK;
}

/**
 * @brief Computes and applies motor duty cycles based on inverse differential kinematics.
 *
 * @details Translates a target platform velocity vector ($v_x, \omega_z$) into PWM output signals using 
 *          the inverse kinematics equations:
 *          1. **Velocity Clamping**: Restricts $v_x$ and $\omega_z$ to limits defined in configuration.
 *          2. **Linear Wheel Speeds**:
 *             $$v_{\text{left}} = v_x - \frac{\omega_z \cdot L}{2}$$
 *             $$v_{\text{right}} = v_x + \frac{\omega_z \cdot L}{2}$$
 *          3. **Angular Speeds**:
 *             $$\omega_{\text{left}} = \frac{v_{\text{left}}}{r}, \quad \omega_{\text{right}} = \frac{v_{\text{right}}}{r}$$
 *          4. **Trimmed PWM Scaling**:
 *             $$\text{Power}_{\text{left}} = \text{round}\left( \omega_{\text{left}} \cdot \text{Scale} \cdot \text{left\_trim\_factor} \right)$$
 *             $$\text{Power}_{\text{right}} = \text{round}\left( \omega_{\text{right}} \cdot \text{Scale} \cdot \text{right\_trim\_factor} \right)$$
 *
 * @param[in] twist Pointer to a @ref diffDriveTwist_t structure containing target linear and angular velocities.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Velocities successfully calculated and dispatched to motor drivers.
 *         - ESP_ERR_INVALID_STATE: Module not initialized or overcurrent fault state is active.
 *         - ESP_ERR_INVALID_ARG: Null pointer passed for `twist`.
 */
esp_err_t DiffDrive_SetTwist(const diffDriveTwist_t *twist)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");
    ESP_RETURN_ON_FALSE(twist, ESP_ERR_INVALID_ARG, TAG, "Target twist pointer is NULL");

    portENTER_CRITICAL(&safety_flag_spin);
    bool is_fault_active = s_ctx.is_fault_active;
    portEXIT_CRITICAL(&safety_flag_spin);

    /* Safety Check: Prevent new motion command execution while safety lock is triggered */
    if (is_fault_active) 
    {
        ESP_LOGW(TAG, "Motion command rejected: Overcurrent system fault is currently active!");
        return ESP_ERR_INVALID_STATE;
    }

    float linear_x = twist->linear_x;
    float angular_z = twist->angular_z;

    /* Enforce software velocity constraints */
    if (s_ctx.max_linear_velocity > 0.0f) 
    {
        linear_x = DiffDrive_Clampf(linear_x, -s_ctx.max_linear_velocity, s_ctx.max_linear_velocity);
    }
    if (s_ctx.max_angular_velocity > 0.0f) 
    {
        angular_z = DiffDrive_Clampf(angular_z, -s_ctx.max_angular_velocity, s_ctx.max_angular_velocity);
    }

    /* Atomically register the new target. The periodic control task (diff_control_tsk, lower
     * priority than the safety task) is the sole owner of ramping towards this target and
     * dispatching the resulting PWM to the wheels -- this function never touches hardware. */
    portENTER_CRITICAL(&twist_spin);
    s_ctx.target_twist.linear_x = linear_x;
    s_ctx.target_twist.angular_z = angular_z;
    portEXIT_CRITICAL(&twist_spin);

    return ESP_OK;
}

/**
 * @brief Calculates current platform translational and rotational velocities via forward kinematics (Odometry).
 *
 * @details Samples optical incremental encoders over a discrete time delta ($\Delta t$), computing wheel 
 *          displacement and chassis movement using forward kinematics:
 *          1. **Pulse Delta**: $\Delta \text{ticks} = \text{ticks}_{\text{current}} - \text{ticks}_{\text{previous}}$
 *          2. **Wheel Distances**:
 *             $$d_{\text{wheel}} = \Delta \text{ticks} \cdot \frac{2 \pi r}{\text{CPR}}$$
 *          3. **Wheel Velocities**:
 *             $$v_{\text{wheel}} = \frac{d_{\text{wheel}}}{\Delta t}$$
 *          4. **Chassis Twist Telemetry**:
 *             $$v_x = \frac{v_{\text{right}} + v_{\text{left}}}{2}$$
 *             $$\omega_z = \frac{v_{\text{right}} - v_{\text{left}}}{L}$$
 *
 * @param[in]  dt_seconds Elapsed sampling interval since last call in seconds ($\Delta t > 0.0$).
 * @param[out] out_twist Pointer to a @ref diffDriveTwist_t structure where computed telemetry will be stored.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Telemetry successfully computed.
 *         - ESP_ERR_INVALID_STATE: Module uninitialized or encoder CPR set to zero.
 *         - ESP_ERR_INVALID_ARG: Null output pointer or non-positive time interval ($\Delta t \le 0.0$).
 */
esp_err_t DiffDrive_GetTwist(float dt_seconds, diffDriveTwist_t *out_twist)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");
    ESP_RETURN_ON_FALSE(out_twist, ESP_ERR_INVALID_ARG, TAG, "Output telemetry pointer is NULL");
    ESP_RETURN_ON_FALSE(dt_seconds > 0.0f, ESP_ERR_INVALID_ARG, TAG, "Delta time must be > 0.0 seconds");
    ESP_RETURN_ON_FALSE(s_ctx.encoder_cpr > 0, ESP_ERR_INVALID_STATE, TAG, "Encoder resolution CPR must be configured");

    int current_left_count = 0;
    int current_right_count = 0;

    /* Forward Kinematics Step 1: Query hardware encoder pulse accumulators */
    ESP_RETURN_ON_ERROR(Wheel_GetEncoderCount(s_ctx.left_wheel, &current_left_count), TAG, "Failed to sample left encoder");
    ESP_RETURN_ON_ERROR(Wheel_GetEncoderCount(s_ctx.right_wheel, &current_right_count), TAG, "Failed to sample right encoder");

    /* Forward Kinematics Step 2: Determine pulse count difference (delta) since previous call */
    int delta_left_ticks = current_left_count - s_ctx.last_left_enc_count;
    int delta_right_ticks = current_right_count - s_ctx.last_right_enc_count;

    /* Update cached counter state */
    s_ctx.last_left_enc_count = current_left_count;
    s_ctx.last_right_enc_count = current_right_count;

    /* Forward Kinematics Step 3: Compute meters per encoder pulse tick */
    float meters_per_tick = (2.0f * (float)M_PI * s_ctx.wheel_radius) / (float)s_ctx.encoder_cpr;
    float dist_left = (float)delta_left_ticks * meters_per_tick;
    float dist_right = (float)delta_right_ticks * meters_per_tick;

    /* Forward Kinematics Step 4: Calculate linear speeds per wheel in m/s */
    float v_left = dist_left / dt_seconds;
    float v_right = dist_right / dt_seconds;

    /* Forward Kinematics Step 5: Solve differential forward kinematics equations for chassis twist */
    out_twist->linear_x = (v_right + v_left) / 2.0f;
    out_twist->angular_z = (v_right - v_left) / s_ctx.half_track_width;

    return ESP_OK;
}

/**
 * @brief Immediately engages active electronic braking on both motor drivers.
 *
 * @details Overrides any ongoing PWM output and commands the underlying driver channels into 
 *          a high-side or low-side short circuit state (active brake / short brake).
 *
 * @return esp_err_t 
 *         - ESP_OK: Active braking engaged on left and right wheel drivers.
 *         - ESP_ERR_INVALID_STATE: Module not initialized.
 */
esp_err_t DiffDrive_Brake(void)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");

    /* Send brake command to both underlying wheel instances */
    ESP_RETURN_ON_ERROR(Wheel_Brake(s_ctx.left_wheel), TAG, "Failed to apply brake to left wheel driver");
    ESP_RETURN_ON_ERROR(Wheel_Brake(s_ctx.right_wheel), TAG, "Failed to apply brake to right wheel driver");

    return ESP_OK;
}

/**
 * @brief Checks if an emergency fault (overcurrent/stall) is latching the system shut down.
 *
 * @return true An emergency lockout is currently active. Motion commands will be rejected.
 * @return false Normal system state. Control commands are permitted.
 */
bool DiffDrive_IsFaultActive(void)
{
    if (!s_ctx.is_initialized) return false;
    return s_ctx.is_fault_active;
}

/**
 * @brief Clears an active emergency lockout state, restoring normal motion command execution.
 *
 * @return esp_err_t 
 *         - ESP_OK: Fault flag cleared.
 *         - ESP_ERR_INVALID_STATE: Module not initialized.
 */
esp_err_t DiffDrive_ClearFault(void)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");

    portENTER_CRITICAL(&safety_flag_spin);
    s_ctx.is_fault_active = false;
    portEXIT_CRITICAL(&safety_flag_spin);
  
    ESP_LOGI(TAG, "Emergency overcurrent fault latch cleared successfully.");
    return ESP_OK;
}