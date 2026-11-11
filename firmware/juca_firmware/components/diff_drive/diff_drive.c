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
    wheel_handle_t left_wheel;         /**< Left motor driver abstraction handle. */
    wheel_handle_t right_wheel;        /**< Right motor driver abstraction handle. */

    float half_track_width_m;               /**< HALF of the track width (L/2) in meters: distance from the robot centerline to each wheel ground contact. */
    float wheel_radius_m;              /**< Radius of drive wheels (r) in meters. */
    uint32_t left_encoder_cpr;         /**< Left encoder pulses per single full revolution in 4x mode. */
    uint32_t right_encoder_cpr;        /**< Right encoder pulses per single full revolution in 4x mode. */

    float max_linear_velocity_m_s;     /**< Configured software safety upper bound for linear speed (m/s). */
    float max_angular_velocity_rad_s;  /**< Configured software safety upper bound for angular speed (rad/s). */
    float max_wheel_rad_s;             /**< Absolute maximum wheel angular velocity (rad/s) at 100% duty cycle. */
    uint32_t common_max_power_ticks;   /**< Lowest `max_power_ticks` of the two wheels: the duty both wheels can always reach (read once at init). */

    /* --- Calibration Trim Multipliers --- */
    float left_trim_factor;            /**< Multiplicative scaling factor for left motor power compensation [0.0 - 1.0]. */
    float right_trim_factor;           /**< Multiplicative scaling factor for right motor power compensation [0.0 - 1.0]. */

    /* --- Safety Task State & Context --- */
    int max_adc_raw_threshold;         /**< Raw ADC threshold limit for current protection. */
    float current_ma_per_count;        /**< Conversion factor from raw ADC counts to mA (0 = not configured). */
    uint32_t monitor_period_ms;        /**< Execution cycle period for safety monitor task in ms. */
    bool is_fault_active;              /**< Emergency latch flag. When true, blocks motion command execution. */
    bool is_brake_requested;           /**< Set by @ref diff_drive_brake, consumed by the control task: zero the ramp and re-assert the brake. Guarded by #s_twist_spinlock. */
    TaskHandle_t monitor_task_handle;  /**< Task handle returned by xTaskCreate for safety task management. */
    bool monitor_task_running;         /**< Flag controlling the lifetime loop of the safety monitoring task. */

    int left_last_encoder_count;       /**< Encoder count for left wheel cached from previous telemetry sample. */
    int right_last_encoder_count;      /**< Encoder count for right wheel cached from previous telemetry sample. */

    /* --- Periodic Motion Control Task State & Context --- */
    float max_accel_linear_m_s2;       /**< Configured max linear acceleration (m/s^2) for the ramp limiter. <= 0.0f disables ramping. */
    float max_accel_angular_rad_s2;    /**< Configured max angular acceleration (rad/s^2) for the ramp limiter. <= 0.0f disables ramping. */
    uint32_t control_period_ms;        /**< Execution cycle period for the periodic control task in ms. */
    diff_drive_twist_t target_twist;   /**< Desired twist most recently registered via @ref diff_drive_set_twist. Guarded by #s_twist_spinlock. */
    diff_drive_twist_t applied_twist;  /**< Ramped twist currently being tracked/output by the control task. Exclusively owned by `diff_control` -- no locking needed. */
    TaskHandle_t control_task_handle;  /**< Task handle returned by xTaskCreate for the periodic control task. */
    bool control_task_running;         /**< Flag controlling the lifetime loop of the periodic control task. */
} diff_drive_context_t;

/**
 * @brief Static Singleton context instance initialized with default zero memory.
 */
static diff_drive_context_t s_ctx = {0};


/** 
 * @brief Spinlock protecting concurrent access to #is_fault_active.
 */
static portMUX_TYPE s_safety_flag_spinlock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief Spinlock protecting concurrent access to #target_twist and #is_brake_requested.
 *
 * @details `target_twist` is written by any task calling @ref diff_drive_set_twist and read once per 
 *          cycle by the periodic control task. `applied_twist` is exclusively owned by the control 
 *          task and therefore does NOT need this lock.
 */
static portMUX_TYPE s_twist_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ============================================================================
 *                     PRIVATE HELPER FUNCTIONS
 * ==========================================================================*/

/**
 * @brief Helper utility to clamp a floating-point value within a strictly closed interval [min, max].
 *
 * @param[in] value Value to clamp.
 * @param[in] min_value Lower boundary limit.
 * @param[in] max_value Upper boundary limit.
 * @return float Clamped result bounded within [min_value, max_value].
 */
static inline float clampf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

/**
 * @brief Converts a period in milliseconds into RTOS ticks, never returning 0.
 *
 * @details `pdMS_TO_TICKS()` rounds down, so a period shorter than one tick (10 ms at 100 Hz) becomes 0 and
 *          `vTaskDelayUntil()` with a 0-tick period returns immediately: the calling task would spin without
 *          ever blocking, starving every task of lower priority.
 *
 * @param[in] period_ms Period in milliseconds.
 * @return TickType_t Period in RTOS ticks, at least 1.
 */
static inline TickType_t period_ms_to_ticks(uint32_t period_ms)
{
    TickType_t period_ticks = pdMS_TO_TICKS(period_ms);
    return (period_ticks > 0) ? period_ticks : 1;
}

/**
 * @brief `vTaskDelayUntil()` that resynchronizes instead of catching up on missed periods.
 *
 * @details When a period has already elapsed, `vTaskDelayUntil()` returns immediately and advances the wake
 *          time by one period only, so after a long pause (e.g. both tasks suspended during calibration) the
 *          loop would run dozens of cycles back to back with no time in between -- for the control task that
 *          advances the acceleration ramp by dozens of steps instantly. If the schedule is more than one period
 *          behind, the reference is moved to "now" first.
 *
 * @param[in,out] last_wake_tick Reference wake time, as for `vTaskDelayUntil()`.
 * @param[in]     period_ticks   Period in RTOS ticks.
 */
static void delay_until_period(TickType_t *last_wake_tick, TickType_t period_ticks)
{
    TickType_t now_tick = xTaskGetTickCount();
    if ((TickType_t)(now_tick - *last_wake_tick) > period_ticks)
    {
        *last_wake_tick = now_tick;
    }
    vTaskDelayUntil(last_wake_tick, period_ticks);
}

/**
 * @brief Applies inverse differential kinematics and dispatches PWM power to both wheel drivers.
 *
 * @details Internal counterpart of the logic formerly inlined in the public `diff_drive_set_twist`. 
 *          Called exclusively by `diff_control` once per control cycle, with the already-ramped 
 *          twist. Performs NO twist clamping, NO initialization checks, and NO fault checks -- those are
 *          the caller's responsibility (see @ref diff_drive_set_twist and `control_task`). The only limit it
 *          applies itself is the common wheel duty limit, as a last safety net (the twist has already been
 *          brought inside it by @ref scale_twist_to_wheel_limit).
 *
 * @param[in] twist Pointer to the twist to apply this cycle (already clamped/ramped).
 * @return esp_err_t ESP_OK on success, or the first underlying @ref wheel_set_power error.
 */
static esp_err_t apply_twist(const diff_drive_twist_t *twist)
{
    /* Inverse Kinematics Step 1: Calculate linear tangential velocities for left and right wheels (m/s) */
    float angular_component_m_s = twist->angular_z * s_ctx.half_track_width_m;
    float left_velocity_m_s = twist->linear_x - angular_component_m_s;
    float right_velocity_m_s = twist->linear_x + angular_component_m_s;

    /* Inverse Kinematics Step 2: Convert linear tangential speeds (m/s) into wheel rotational speeds (rad/s) */
    float left_omega_rad_s = left_velocity_m_s / s_ctx.wheel_radius_m;
    float right_omega_rad_s = right_velocity_m_s / s_ctx.wheel_radius_m;

    /* Inverse Kinematics Step 3: Compute conversion scalar ratio (PWM Duty Ticks / Maximum rad/s) */
    float power_ticks_per_rad_s = (float)WHEEL_PWM_DUTY_TICK_MAX / s_ctx.max_wheel_rad_s;

    /* Inverse Kinematics Step 4: Scale to PWM values, then apply dynamic balance trim multipliers. The nominal power is
     * clamped to the common wheel limit BEFORE the trim: if the clamp came after it (inside wheel_set_power), a saturated
     * command would put both wheels at the same duty and the trim would have no effect. */
    const float limit_ticks = (float)s_ctx.common_max_power_ticks;
    float left_nominal_ticks = fminf(fmaxf(left_omega_rad_s * power_ticks_per_rad_s, -limit_ticks), limit_ticks);
    float right_nominal_ticks = fminf(fmaxf(right_omega_rad_s * power_ticks_per_rad_s, -limit_ticks), limit_ticks);
    int32_t left_power_ticks = (int32_t)lroundf(left_nominal_ticks * s_ctx.left_trim_factor);
    int32_t right_power_ticks = (int32_t)lroundf(right_nominal_ticks * s_ctx.right_trim_factor);

    /* Inverse Kinematics Step 5: Dispatch calculated power levels to low-level motor drivers */
    ESP_RETURN_ON_ERROR(wheel_set_power(s_ctx.left_wheel, left_power_ticks), TAG, "Failed to apply power to left wheel driver");
    ESP_RETURN_ON_ERROR(wheel_set_power(s_ctx.right_wheel, right_power_ticks), TAG, "Failed to apply power to right wheel driver");

    return ESP_OK;
}

/**
 * @brief Applies a linear acceleration rate limiter (ramp) independently on each twist axis.
 *
 * @details Moves `current_twist` towards `target_twist` by at most `max_accel * delta_time_s` per axis, per call. 
 *          Passing `max_accel <= 0.0f` for an axis disables limiting on that axis (jumps straight 
 *          to target), which is convenient for testing or for platforms where the caller already 
 *          shapes the trajectory upstream.
 *
 * @param[in]  current_twist             Twist currently being tracked (previous cycle's ramped output).
 * @param[in]  target_twist              Desired twist to ramp towards.
 * @param[in]  delta_time_s              Elapsed time since the previous cycle, in seconds.
 * @param[in]  max_accel_linear_m_s2     Maximum linear acceleration magnitude (m/s^2). <= 0.0f disables.
 * @param[in]  max_accel_angular_rad_s2  Maximum angular acceleration magnitude (rad/s^2). <= 0.0f disables.
 * @param[out] out_twist                 Resulting ramped twist for this cycle.
 */
static void apply_ramp(const diff_drive_twist_t *current_twist, const diff_drive_twist_t *target_twist, float delta_time_s,
                        float max_accel_linear_m_s2, float max_accel_angular_rad_s2, diff_drive_twist_t *out_twist)
{
    if (max_accel_linear_m_s2 <= 0.0f)
    {
        out_twist->linear_x = target_twist->linear_x;
    }
    else
    {
        float max_delta_linear_m_s = max_accel_linear_m_s2 * delta_time_s;
        float delta_linear_m_s = clampf(target_twist->linear_x - current_twist->linear_x, -max_delta_linear_m_s, max_delta_linear_m_s);
        out_twist->linear_x = current_twist->linear_x + delta_linear_m_s;
    }

    if (max_accel_angular_rad_s2 <= 0.0f)
    {
        out_twist->angular_z = target_twist->angular_z;
    }
    else
    {
        float max_delta_angular_rad_s = max_accel_angular_rad_s2 * delta_time_s;
        float delta_angular_rad_s = clampf(target_twist->angular_z - current_twist->angular_z, -max_delta_angular_rad_s, max_delta_angular_rad_s);
        out_twist->angular_z = current_twist->angular_z + delta_angular_rad_s;
    }
}

/**
 * @brief Scales a twist down, if needed, so that neither wheel is asked for more speed than it can deliver.
 *
 * @details The fastest wheel of a twist (v, w) turns at (|v| + |w| * half_track_width_m) / wheel_radius_m.
 *          The speed a wheel can really deliver is `max_wheel_rad_s` (100% duty) times the fraction of the duty
 *          that is allowed, `common_max_power_ticks / WHEEL_PWM_DUTY_TICK_MAX`; `max_wheel_rad_s` is the
 *          calibrated value once @ref diff_drive_calibrate_max_speed has run. When the twist asks for more, BOTH
 *          components are multiplied by the same factor: the robot goes slower along the same path (same
 *          curvature) instead of one wheel saturating and the path bending. A twist that fits is copied
 *          unchanged, so a command inside the limits is never altered.
 *
 *          Applied to the target before the ramp, so the ramp never climbs towards a speed the wheels cannot
 *          reach (it would keep "winding up" past the saturation point and the robot would keep running at full
 *          power long after the target was lowered), and to the ramp output too, because the two axes ramp
 *          independently and a transient between two valid twists can still exceed the limit.
 *
 * @param[in]  twist     Twist to check.
 * @param[out] out_twist Resulting twist (may be the same object as `twist`).
 */
static void scale_twist_to_wheel_limit(const diff_drive_twist_t *twist, diff_drive_twist_t *out_twist)
{
    const float usable_wheel_rad_s = s_ctx.max_wheel_rad_s * ((float)s_ctx.common_max_power_ticks / (float)WHEEL_PWM_DUTY_TICK_MAX);
    const float usable_wheel_m_s = usable_wheel_rad_s * s_ctx.wheel_radius_m;
    const float demand_m_s = fabsf(twist->linear_x) + fabsf(twist->angular_z) * s_ctx.half_track_width_m;

    /* demand_m_s > usable_wheel_m_s >= 0 here, so the division is safe */
    const float scale = (demand_m_s > usable_wheel_m_s) ? (usable_wheel_m_s / demand_m_s) : 1.0f;

    out_twist->linear_x = twist->linear_x * scale;
    out_twist->angular_z = twist->angular_z * scale;
}

/**
 * @brief Background FreeRTOS task that periodically ramps towards the last commanded twist and
 *        dispatches the resulting motor power.
 *
 * @details Runs at a fixed period (`control_period_ms`, rounded down to whole RTOS ticks, at least 1 tick)
 *          and LOWER priority than `diff_safety`, so an overcurrent/stall detection can always preempt and
 *          brake immediately, even if this task is mid-cycle. Each cycle:
 *          1. Atomically samples the current target twist (set by @ref diff_drive_set_twist) and consumes
 *             any pending brake request (see @ref diff_drive_brake).
 *          2. If an emergency fault is latched, or a brake was requested, resets the internal ramp state to
 *             zero and commands an active brake on both wheels. Re-asserting the brake every cycle (instead
 *             of only skipping the output) guarantees that a wheel left energized by a cycle that the safety
 *             task preempted is stopped within one period. Zeroing the ramp makes motion resume gradually
 *             from a stop once the fault is cleared and a new target is set.
 *          3. Otherwise, advances the ramp towards the target and applies it via
 *             @ref apply_twist. Both the target the ramp climbs towards and the twist that is applied are first
 *             brought inside what the wheels can deliver (see @ref scale_twist_to_wheel_limit), so the ramp
 *             state never goes beyond the saturation point of the wheels.
 *
 * @param[in] arg Unused parameter pointer required by FreeRTOS task function signatures.
 */
static void control_task(void *arg)
{
    (void)arg;

    TickType_t last_wake_tick = xTaskGetTickCount();
    const TickType_t period_ticks = period_ms_to_ticks(s_ctx.control_period_ms);
    /* The ramp step must use the period the RTOS really provides (whole ticks), not the requested one:
     * with a 10 ms tick a requested 25 ms period runs every 20 ms. */
    const float delta_time_s = (float)(period_ticks * portTICK_PERIOD_MS) / 1000.0f;

    ESP_LOGI(TAG, "Periodic control task started successfully (period=%lu ms).", (unsigned long)(period_ticks * portTICK_PERIOD_MS));

    while (s_ctx.control_task_running)
    {
        diff_drive_twist_t target_twist;
        bool is_brake_requested;
        portENTER_CRITICAL(&s_twist_spinlock);
        target_twist = s_ctx.target_twist;
        is_brake_requested = s_ctx.is_brake_requested;
        s_ctx.is_brake_requested = false;
        portEXIT_CRITICAL(&s_twist_spinlock);

        portENTER_CRITICAL(&s_safety_flag_spinlock);
        bool fault = s_ctx.is_fault_active;
        portEXIT_CRITICAL(&s_safety_flag_spinlock);

        if (fault || is_brake_requested)
        {
            /* Freeze the ramp at zero so recovery accelerates smoothly from a stop rather than snapping
             * to a stale target, and keep the wheels braked. */
            s_ctx.applied_twist.linear_x = 0.0f;
            s_ctx.applied_twist.angular_z = 0.0f;
            wheel_brake(s_ctx.left_wheel);
            wheel_brake(s_ctx.right_wheel);
        }
        else
        {
            /* Ramp towards what the wheels can reach, not towards the raw target (see scale_twist_to_wheel_limit) */
            diff_drive_twist_t reachable_target;
            scale_twist_to_wheel_limit(&target_twist, &reachable_target);

            diff_drive_twist_t ramped_twist;
            apply_ramp(&s_ctx.applied_twist, &reachable_target, delta_time_s,
                        s_ctx.max_accel_linear_m_s2, s_ctx.max_accel_angular_rad_s2, &ramped_twist);
            s_ctx.applied_twist = ramped_twist;

            /* The two axes ramp independently: the point between two reachable twists may not be reachable */
            diff_drive_twist_t output_twist;
            scale_twist_to_wheel_limit(&ramped_twist, &output_twist);
            apply_twist(&output_twist);
        }

        delay_until_period(&last_wake_tick, period_ticks);
    }

    s_ctx.control_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Background FreeRTOS task routine that monitors motor driver current draw.
 *
 * @details Periodically polls raw ADC current sensor readings from both wheel handles. If either 
 *          reading exceeds `max_adc_raw_threshold` for `stall_counter_limit` consecutive debounced 
 *          cycles, the routine latches `is_fault_active = true`, forces an emergency brake on both 
 *          wheels, discards the currently registered target twist (so the platform does not resume 
 *          its previous motion the instant the fault is cleared), and logs an error.
 *
 * @note This is a HARD LATCH, not a self-clearing condition: once tripped, `is_fault_active` stays 
 *       true -- and the control task keeps the wheels at zero -- until the application explicitly 
 *       calls @ref diff_drive_clear_fault(). A version of this task used to auto-release the latch as 
 *       soon as current dropped back down (e.g. right after the emergency brake stopped the motor), 
 *       but that caused an oscillation: the platform would un-latch, immediately try to resume its 
 *       last command, stall again against whatever was blocking it, and re-trip -- repeatedly 
 *       jerking on and off. Requiring an explicit @ref diff_drive_clear_fault() call (ideally after 
 *       confirming the obstruction is gone) breaks that loop.
 *
 * @param[in] arg Unused parameter pointer required by FreeRTOS task function signatures.
 */
static void safety_task(void *arg)
{
    (void)arg;
    int left_current_raw = 0;
    int right_current_raw = 0;
    int stall_counter = 0;
    bool active_stall = false; /* Local mirror of the latch, used for debounce/edge detection only. */
    bool is_read_failure_reported = false; /* Limits the "sensors unavailable" warning to one log per outage. */

    /* Detection parameters */
    const int stall_counter_limit = 10; /* number of cycles */
    
    /* Adaptive polling period */
    uint32_t period_ms = s_ctx.monitor_period_ms;

    ESP_LOGI(TAG, "Continuous current monitoring task started successfully.");

    TickType_t last_wake_tick = xTaskGetTickCount();

    while (s_ctx.monitor_task_running) 
    {
        /* If the latch is currently active but the application has cleared it externally via
         * diff_drive_clear_fault(), re-arm this task's local debounce state so a fresh stall can be
         * detected cleanly (otherwise stall_counter could still be sitting at the limit). */
        if (active_stall)
        {
            portENTER_CRITICAL(&s_safety_flag_spinlock);
            bool still_faulted = s_ctx.is_fault_active;
            portEXIT_CRITICAL(&s_safety_flag_spinlock);

            if (!still_faulted)
            {
                active_stall = false;
                stall_counter = 0;
                ESP_LOGI(TAG, "Fault cleared by application -- safety task re-armed.");
            }
        }

        /* Poll current sensor readings from lower-level motor driver handles */
        esp_err_t left_read_status = wheel_get_current_raw(s_ctx.left_wheel, &left_current_raw);
        esp_err_t right_read_status = wheel_get_current_raw(s_ctx.right_wheel, &right_current_raw);

        if (left_read_status == ESP_OK || right_read_status == ESP_OK)
        {
            is_read_failure_reported = false;

            /* Check if raw current reading exceeds configured safety limit */
           bool high_current = (left_current_raw > s_ctx.max_adc_raw_threshold || 
                                right_current_raw > s_ctx.max_adc_raw_threshold);

           /* ============================
            * Dynamic period adjustment
            * ============================ */
           if (high_current)
           {
               if (period_ms > 10)
               {
                   period_ms /= 2;
                   if (period_ms < 10)
                       period_ms = 10;

                   /* Avoid drift when changing the period too fast */
                   last_wake_tick = xTaskGetTickCount();
               }
           }
           else
           {
               if (period_ms < s_ctx.monitor_period_ms)
               {
                   period_ms += 10;
                   if (period_ms > s_ctx.monitor_period_ms)
                        period_ms = s_ctx.monitor_period_ms;
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
                   if (stall_counter < stall_counter_limit)
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
           if (!active_stall && stall_counter >= stall_counter_limit)
           {
               active_stall = true;

               portENTER_CRITICAL(&s_safety_flag_spinlock);
               s_ctx.is_fault_active = true;
               portEXIT_CRITICAL(&s_safety_flag_spinlock);

               /* Trigger immediate hardware-level active brake stop */
               wheel_brake(s_ctx.left_wheel);
               wheel_brake(s_ctx.right_wheel);

               /* Discard the previously registered target so that, once the application calls
                * diff_drive_clear_fault(), the control task ramps from zero instead of instantly
                * driving straight back into whatever caused the stall. A fresh diff_drive_set_twist()
                * call is required to move again after recovery. */
               portENTER_CRITICAL(&s_twist_spinlock);
               s_ctx.target_twist.linear_x = 0.0f;
               s_ctx.target_twist.angular_z = 0.0f;
               portEXIT_CRITICAL(&s_twist_spinlock);

               ESP_LOGE(TAG, "OVERCURRENT DETECTED! Left ADC: %d, Right ADC: %d (Threshold: %d). Emergency brake activated! "
                             "Fault latched -- call diff_drive_clear_fault() and re-issue diff_drive_set_twist() to resume.",
                        left_current_raw, right_current_raw, s_ctx.max_adc_raw_threshold);
           }

        }
        else if (!is_read_failure_reported)
        {
            /* Neither wheel could be sampled (ADC error, or no current sensor wired on either wheel).
             * The cycle is skipped -- the debounce state is left untouched -- but the delay below is
             * still reached, so this task never spins at its (high) priority. Reported once per outage
             * to avoid flooding the log every cycle. */
            ESP_LOGW(TAG, "Current sensing unavailable on both wheels (left: %s, right: %s) -- overcurrent protection is NOT monitoring.",
                     esp_err_to_name(left_read_status), esp_err_to_name(right_read_status));
            is_read_failure_reported = true;
        }

        /* ============================
         * Deterministic delay -- reached on EVERY iteration, whatever the sensor readings were
         * ============================ */
        delay_until_period(&last_wake_tick, period_ms_to_ticks(period_ms));
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
 *          (`diff_safety`) to continuously monitor motor current consumption if thresholds are enabled.
 *
 * @param[in] config Pointer to an initialized @ref diff_drive_config_t structure containing system parameters.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Module initialized successfully and safety task is running.
 *         - ESP_ERR_INVALID_STATE: Module is already initialized.
 *         - ESP_ERR_INVALID_ARG: Null pointer provided (`config` or a wheel handle), invalid geometric parameters (`half_track_width_m` <= 0 or `wheel_radius_m` <= 0), or `max_wheel_rad_s` <= 0.
 *         - ESP_FAIL: Failed to allocate or spawn the FreeRTOS safety task.
 *
 * @note Must be called once before invoking any motion control or odometry functions.
 */
esp_err_t diff_drive_init(const diff_drive_config_t *config)
{
    /* Validate input arguments and initialization precondition state */
    ESP_RETURN_ON_FALSE(!s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is already initialized");
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Null configuration pointer provided");
    ESP_RETURN_ON_FALSE(config->left_wheel && config->right_wheel, ESP_ERR_INVALID_ARG, TAG, "Wheel driver handles must not be NULL");
    ESP_RETURN_ON_FALSE(config->half_track_width_m > 0.0f && config->wheel_radius_m > 0.0f, ESP_ERR_INVALID_ARG, TAG, "Physical robot dimensions must be > 0");
    ESP_RETURN_ON_FALSE(config->max_wheel_rad_s > 0.0f, ESP_ERR_INVALID_ARG, TAG, "Maximum wheel angular velocity must be > 0");
    ESP_RETURN_ON_FALSE(isfinite(config->current_ma_per_count) && config->current_ma_per_count >= 0.0f, ESP_ERR_INVALID_ARG, TAG,
                        "Current conversion factor must be finite and >= 0");

    uint32_t left_max_power_ticks = 0;
    uint32_t right_max_power_ticks = 0;
    ESP_RETURN_ON_ERROR(wheel_get_max_power_ticks(config->left_wheel, &left_max_power_ticks), TAG, "Failed to read left wheel power limit");
    ESP_RETURN_ON_ERROR(wheel_get_max_power_ticks(config->right_wheel, &right_max_power_ticks), TAG, "Failed to read right wheel power limit");

    /* Store geometric and performance parameters into global Singleton context */
    s_ctx.left_wheel = config->left_wheel;
    s_ctx.right_wheel = config->right_wheel;
    s_ctx.half_track_width_m = config->half_track_width_m;
    s_ctx.wheel_radius_m = config->wheel_radius_m;
    s_ctx.left_encoder_cpr = config->left_encoder_cpr;
    s_ctx.right_encoder_cpr = config->right_encoder_cpr;
    s_ctx.max_linear_velocity_m_s = config->max_linear_velocity_m_s;
    s_ctx.max_angular_velocity_rad_s = config->max_angular_velocity_rad_s;
    s_ctx.max_wheel_rad_s = config->max_wheel_rad_s;
    s_ctx.common_max_power_ticks = (left_max_power_ticks < right_max_power_ticks) ? left_max_power_ticks : right_max_power_ticks;

    /* Initialize default power balance trim multipliers (100% gain each) */
    s_ctx.left_trim_factor = 1.0f;
    s_ctx.right_trim_factor = 1.0f;

    s_ctx.max_adc_raw_threshold = config->max_adc_raw_threshold;
    s_ctx.current_ma_per_count = config->current_ma_per_count;
    s_ctx.monitor_period_ms = (config->monitor_period_ms > 0) ? config->monitor_period_ms : 50;
    s_ctx.is_fault_active = false;
    s_ctx.is_brake_requested = false;

    /* Store periodic control task ramp parameters */
    s_ctx.max_accel_linear_m_s2 = config->max_accel_linear_m_s2;
    s_ctx.max_accel_angular_rad_s2 = config->max_accel_angular_rad_s2;
    s_ctx.control_period_ms = (config->control_period_ms > 0) ? config->control_period_ms : 20;
    s_ctx.target_twist = (diff_drive_twist_t){0};
    s_ctx.applied_twist = (diff_drive_twist_t){0};

    /* Seed initial encoder count values to prevent step jump in first GetTwist calculation */
    wheel_get_encoder_count(s_ctx.left_wheel, &s_ctx.left_last_encoder_count);
    wheel_get_encoder_count(s_ctx.right_wheel, &s_ctx.right_last_encoder_count);

    /* Resolve task parameters up front so priority ordering can be validated before anything is spawned */
    UBaseType_t resolved_safety_priority = (config->safety_task_priority > 0) ? config->safety_task_priority : 5;
    uint32_t resolved_safety_stack_size = (config->safety_task_stack_size > 0) ? config->safety_task_stack_size : 3072;

    UBaseType_t resolved_control_priority = (config->control_task_priority > 0) ? config->control_task_priority : 4;
    uint32_t resolved_control_stack_size = (config->control_task_stack_size > 0) ? config->control_task_stack_size : 3072;

    /* Safety invariant: the control task must never be able to starve or outrun the safety task.
     * Enforced only when the safety task is actually going to run. */
    if (s_ctx.max_adc_raw_threshold > 0)
    {
        ESP_RETURN_ON_FALSE(resolved_control_priority < resolved_safety_priority, ESP_ERR_INVALID_ARG, TAG,
                             "control_task_priority (%u) must be strictly lower than safety_task_priority (%u)",
                             (unsigned)resolved_control_priority, (unsigned)resolved_safety_priority);
    }

    /* Launch periodic motion control task (always active, regardless of safety task) */
    s_ctx.control_task_running = true;
    BaseType_t control_create_result = xTaskCreate(control_task,
                                                    "diff_control",
                                                    resolved_control_stack_size,
                                                    NULL,
                                                    resolved_control_priority,
                                                    &s_ctx.control_task_handle);

    if (control_create_result != pdPASS)
    {
        s_ctx.control_task_running = false;
        ESP_LOGE(TAG, "Failed to spawn FreeRTOS periodic control task");
        return ESP_FAIL;
    }

    /* Launch current safety task if protection threshold is set */
    if (s_ctx.max_adc_raw_threshold > 0) 
    {
        s_ctx.monitor_task_running = true;

        BaseType_t safety_create_result = xTaskCreate(safety_task,
                                                      "diff_safety",
                                                      resolved_safety_stack_size,
                                                      NULL,
                                                      resolved_safety_priority,
                                                      &s_ctx.monitor_task_handle);

        if (safety_create_result != pdPASS) 
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
bool diff_drive_is_initialized(void)
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
esp_err_t diff_drive_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");

    /* Signal both background tasks to terminate their loops */
    s_ctx.control_task_running = false;
    s_ctx.monitor_task_running = false;

    /* Wait deterministically for each task to actually self-delete before proceeding.
     * Without this, either task could still run one more cycle (and, for the control
     * task, call wheel_set_power) after we brake below, silently re-enabling motion. */
    /* At least one RTOS tick: pdMS_TO_TICKS(2) is 0 at 100 Hz, so waited_ticks would never advance (the
     * timeout below could never expire) and vTaskDelay(0) would only yield -- a busy loop that starves the
     * very tasks being waited for when the caller has a higher priority than they do. */
    const TickType_t poll_step_ticks = period_ms_to_ticks(2);
    TickType_t waited_ticks = 0;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(s_ctx.control_period_ms + s_ctx.monitor_period_ms + 100);

    while ((s_ctx.control_task_handle != NULL || s_ctx.monitor_task_handle != NULL) && waited_ticks < timeout_ticks)
    {
        vTaskDelay(poll_step_ticks);
        waited_ticks += poll_step_ticks;
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
    wheel_brake(s_ctx.left_wheel);
    wheel_brake(s_ctx.right_wheel);

    /* Reset global Singleton state context */
    s_ctx.is_initialized = false;
    ESP_LOGI(TAG, "Differential drive module deinitialized cleanly.");
    return ESP_OK;
}

/**
 * @brief Waits (bounded) until both encoders stop changing, i.e. until the wheels are at rest after braking.
 *
 * @param[in] max_wait_ms Upper bound for the wait, in milliseconds.
 */
static void wait_wheels_at_rest(uint32_t max_wait_ms)
{
    const TickType_t poll_ticks = period_ms_to_ticks(20);
    const TickType_t max_wait_ticks = pdMS_TO_TICKS(max_wait_ms);
    TickType_t waited_ticks = 0;

    int left_count = 0;
    int right_count = 0;
    wheel_get_encoder_count(s_ctx.left_wheel, &left_count);
    wheel_get_encoder_count(s_ctx.right_wheel, &right_count);

    while (waited_ticks < max_wait_ticks)
    {
        vTaskDelay(poll_ticks);
        waited_ticks += poll_ticks;

        int left_now = left_count;
        int right_now = right_count;
        wheel_get_encoder_count(s_ctx.left_wheel, &left_now);
        wheel_get_encoder_count(s_ctx.right_wheel, &right_now);

        bool is_at_rest = (left_now == left_count) && (right_now == right_count);
        left_count = left_now;
        right_count = right_now;
        if (is_at_rest)
        {
            break;
        }
    }
}

/**
 * @brief Common exit path of @ref diff_drive_calibrate_max_speed: restores the background tasks.
 *
 * @details Called with both wheels already braked and both background tasks suspended.
 */
static void resume_after_calibration(void)
{
    /* Let the braking distance pass first, so it does not show up in the first odometry sample. */
    wait_wheels_at_rest(500);

    /* Wheels are physically braked -- reset the ramp state before resuming the control task so it ramps
     * up fresh instead of snapping back to the pre-calibration twist. */
    s_ctx.applied_twist = (diff_drive_twist_t){0};

    /* The wheels moved while odometry was not being sampled: re-seed the encoder baseline, otherwise the
     * next diff_drive_get_twist() would report the whole calibration run as one huge velocity sample. */
    wheel_get_encoder_count(s_ctx.left_wheel, &s_ctx.left_last_encoder_count);
    wheel_get_encoder_count(s_ctx.right_wheel, &s_ctx.right_last_encoder_count);

    if (s_ctx.monitor_task_handle != NULL)
    {
        vTaskResume(s_ctx.monitor_task_handle);
    }
    if (s_ctx.control_task_handle != NULL)
    {
        vTaskResume(s_ctx.control_task_handle);
    }
}

/**
 * @brief Performs an empirical auto-calibration routine to match maximum motor speeds and correct straight-line drift.
 *
 * @details Drives both wheels with the same constant duty for `test_duration_ms` -- 100% when no wheel limits
 *          its power, otherwise the lowest `max_power_ticks` of the two wheels (see @ref wheel_get_max_power_ticks),
 *          because a command above a wheel's limit would be silently clamped. The first third of the pulse lets
 *          the wheels accelerate and is discarded; the angle actually traveled during the remaining two thirds is
 *          measured via encoder feedback, and trim scaling factors are calculated:
 *          - If omega_left > omega_right: left_trim_factor = omega_right / omega_left and right_trim_factor = 1.0.
 *          - If omega_right > omega_left: right_trim_factor = omega_left / omega_right and left_trim_factor = 1.0.
 *
 *          The slower wheel's speed is then extrapolated linearly to 100% duty and stored as the new
 *          `max_wheel_rad_s`, so the duty-to-speed scale used by the control task stays correct even when the
 *          wheels are power-limited. The trim factors and `max_wheel_rad_s` live in RAM only: they are lost on
 *          reset, so call this function again after every boot (or persist them yourself, e.g. in NVS).
 *          At the end the odometry baseline is re-seeded, so the next @ref diff_drive_get_twist does not report
 *          the calibration run as a velocity spike.
 *
 * @warning This function triggers physical robot motion! Ensure the platform is located on open ground
 *          with at least 2 meters of clear unobstructed path before invocation.
 * @warning The safety task is suspended while the pulse runs: there is NO overcurrent protection until this
 *          function returns (hence the upper bound on the duration).
 *
 * @param[in]  test_duration_ms Duration of test run pulse in milliseconds, within
 *                              [@ref DIFF_DRIVE_CALIBRATION_MIN_DURATION_MS, @ref DIFF_DRIVE_CALIBRATION_MAX_DURATION_MS].
 * @param[out] out_max_wheel_rad_s Optional pointer to store the new maximum wheel velocity in rad/s at 100% duty (slower wheel, extrapolated from the test duty). Pass NULL if unneeded.
 *
 * @return esp_err_t
 *         - ESP_OK: Calibration completed successfully and the new trim parameters are in effect (RAM only).
 *         - ESP_ERR_INVALID_STATE: Module not initialized, encoder disabled (`left_encoder_cpr` or `right_encoder_cpr` == 0), overcurrent fault latched, or a wheel limited to 0 power.
 *         - ESP_ERR_INVALID_ARG: Provided test duration is outside the accepted range.
 *         - ESP_ERR_INVALID_RESPONSE: Zero encoder pulses detected (motors unplugged, jammed, or hardware failure).
 */
esp_err_t diff_drive_calibrate_max_speed(uint32_t test_duration_ms, float *out_max_wheel_rad_s)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");
    ESP_RETURN_ON_FALSE(s_ctx.left_encoder_cpr > 0, ESP_ERR_INVALID_STATE, TAG, "Left encoder CPR must be > 0 for dynamic calibration");
    ESP_RETURN_ON_FALSE(s_ctx.right_encoder_cpr > 0, ESP_ERR_INVALID_STATE, TAG, "Right encoder CPR must be > 0 for dynamic calibration");
    ESP_RETURN_ON_FALSE(test_duration_ms >= DIFF_DRIVE_CALIBRATION_MIN_DURATION_MS && test_duration_ms <= DIFF_DRIVE_CALIBRATION_MAX_DURATION_MS,
                        ESP_ERR_INVALID_ARG, TAG, "Calibration test duration must be within [%u, %u] ms",
                        (unsigned)DIFF_DRIVE_CALIBRATION_MIN_DURATION_MS, (unsigned)DIFF_DRIVE_CALIBRATION_MAX_DURATION_MS);
    ESP_RETURN_ON_FALSE(!diff_drive_is_fault_active(), ESP_ERR_INVALID_STATE, TAG,
                        "Calibration refused: overcurrent fault is latched (call diff_drive_clear_fault() first)");

    /* Both wheels are driven with the SAME duty, so their measured speeds are directly comparable. That duty
     * must not exceed the limit of either wheel: wheel_set_power() silently clamps to max_power_ticks, and a
     * clamped pulse would make the duty-to-speed scale used by apply_twist() wrong (and would fold a
     * difference between the two wheels' limits into the trim factors). */
    uint32_t left_max_power_ticks = 0;
    uint32_t right_max_power_ticks = 0;
    ESP_RETURN_ON_ERROR(wheel_get_max_power_ticks(s_ctx.left_wheel, &left_max_power_ticks), TAG, "Failed to read left wheel power limit");
    ESP_RETURN_ON_ERROR(wheel_get_max_power_ticks(s_ctx.right_wheel, &right_max_power_ticks), TAG, "Failed to read right wheel power limit");

    const uint32_t test_power_ticks = (left_max_power_ticks < right_max_power_ticks) ? left_max_power_ticks : right_max_power_ticks;
    ESP_RETURN_ON_FALSE(test_power_ticks > 0, ESP_ERR_INVALID_STATE, TAG, "Calibration impossible: a wheel has max_power_ticks == 0");

    /* Suspend BOTH background tasks: the safety task must not interfere with the forced
     * constant-duty test, and the control task must not overwrite it every control_period_ms
     * with whatever twist was last commanded (it would otherwise fight this routine). */
    if (s_ctx.monitor_task_handle != NULL)
    {
        vTaskSuspend(s_ctx.monitor_task_handle);
    }
    if (s_ctx.control_task_handle != NULL)
    {
        vTaskSuspend(s_ctx.control_task_handle);
    }

    /* The first third of the pulse is acceleration (not steady speed) and is excluded from the measurement */
    const uint32_t settle_ms = test_duration_ms / 3U;
    const uint32_t measure_ms = test_duration_ms - settle_ms;

    ESP_LOGI(TAG, "Starting maximum speed auto-calibration pulse (%lu ms at %lu of %lu duty ticks)...",
             (unsigned long)test_duration_ms, (unsigned long)test_power_ticks, (unsigned long)WHEEL_PWM_DUTY_TICK_MAX);

    /* Step 1: Command the same constant duty to both motor drivers */
    esp_err_t drive_status = wheel_set_power(s_ctx.left_wheel, (int32_t)test_power_ticks);
    if (drive_status == ESP_OK)
    {
        drive_status = wheel_set_power(s_ctx.right_wheel, (int32_t)test_power_ticks);
    }

    int left_initial_count = 0, right_initial_count = 0;
    int left_final_count = 0, right_final_count = 0;
    TickType_t measure_start_tick = 0;
    TickType_t measure_end_tick = 0;

    if (drive_status == ESP_OK)
    {
        /* Step 2: Let the physical robot accelerate to its steady speed (not measured) */
        vTaskDelay(pdMS_TO_TICKS(settle_ms));

        /* Step 3: Measure over the steady-speed window */
        wheel_get_encoder_count(s_ctx.left_wheel, &left_initial_count);
        wheel_get_encoder_count(s_ctx.right_wheel, &right_initial_count);
        measure_start_tick = xTaskGetTickCount();

        vTaskDelay(pdMS_TO_TICKS(measure_ms));

        wheel_get_encoder_count(s_ctx.left_wheel, &left_final_count);
        wheel_get_encoder_count(s_ctx.right_wheel, &right_final_count);
        measure_end_tick = xTaskGetTickCount();
    }

    /* Step 4: Cut power and apply active braking (also when the pulse could not even be started) */
    wheel_brake(s_ctx.left_wheel);
    wheel_brake(s_ctx.right_wheel);

    if (drive_status != ESP_OK)
    {
        ESP_LOGE(TAG, "Calibration aborted: failed to drive the wheels (%s)", esp_err_to_name(drive_status));
        resume_after_calibration();
        return drive_status;
    }

    int left_delta_count = abs(left_final_count - left_initial_count);
    int right_delta_count = abs(right_final_count - right_initial_count);

    /* Step 5: Verify hardware responsiveness (non-zero displacement) */
    if (left_delta_count == 0 || right_delta_count == 0)
    {
        ESP_LOGE(TAG, "Calibration failed: Zero encoder ticks detected. Left delta: %d, Right delta: %d", left_delta_count, right_delta_count);
        resume_after_calibration();
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Measured window, from the RTOS tick counter (vTaskDelay only guarantees "at least") */
    float duration_s = (float)((measure_end_tick - measure_start_tick) * portTICK_PERIOD_MS) / 1000.0f;

    /* Step 6: Convert pulse deltas into actual physical angular speeds (rad/s) at the test duty */
    float left_rad_per_count = (2.0f * (float)M_PI) / (float)s_ctx.left_encoder_cpr;
    float right_rad_per_count = (2.0f * (float)M_PI) / (float)s_ctx.right_encoder_cpr;
    float left_omega_rad_s = ((float)left_delta_count * left_rad_per_count) / duration_s;
    float right_omega_rad_s = ((float)right_delta_count * right_rad_per_count) / duration_s;

    ESP_LOGI(TAG, "Measured uncompensated wheel speeds at test duty -> Left: %.2f rad/s, Right: %.2f rad/s", left_omega_rad_s, right_omega_rad_s);

    /* Step 7: Scale down the faster wheel to match the slower one, and extrapolate the slower wheel's speed
     * to 100% duty -- that is what max_wheel_rad_s means to apply_twist(), whose duty-to-speed ratio is
     * WHEEL_PWM_DUTY_TICK_MAX / max_wheel_rad_s. */
    float slower_omega_rad_s;
    if (left_omega_rad_s > right_omega_rad_s)
    {
        s_ctx.left_trim_factor = right_omega_rad_s / left_omega_rad_s;
        s_ctx.right_trim_factor = 1.0f;
        slower_omega_rad_s = right_omega_rad_s;
    }
    else
    {
        s_ctx.left_trim_factor = 1.0f;
        s_ctx.right_trim_factor = left_omega_rad_s / right_omega_rad_s;
        slower_omega_rad_s = left_omega_rad_s;
    }
    s_ctx.max_wheel_rad_s = slower_omega_rad_s * ((float)WHEEL_PWM_DUTY_TICK_MAX / (float)test_power_ticks);

    ESP_LOGI(TAG, "Calibration completed. Trim Factors -> Left: %.3f, Right: %.3f | Calibrated Speed at 100%% duty: %.2f rad/s",
             s_ctx.left_trim_factor, s_ctx.right_trim_factor, s_ctx.max_wheel_rad_s);

    if (out_max_wheel_rad_s != NULL)
    {
        *out_max_wheel_rad_s = s_ctx.max_wheel_rad_s;
    }

    resume_after_calibration();
    return ESP_OK;
}

/**
 * @brief Computes and applies motor duty cycles based on inverse differential kinematics.
 *
 * @details Translates a target platform velocity vector (v_x, omega_z) into PWM output signals using 
 *          the inverse kinematics equations:
 *          1. **Velocity Clamping**: Restricts v_x and omega_z to limits defined in configuration.
 *             The control task then scales the pair down, keeping its ratio, if the faster wheel would need more
 *             than the wheels can deliver (`max_wheel_rad_s` times the allowed duty fraction).
 *          2. **Linear Wheel Speeds**:
 *             v_left  = v_x - omega_z * L / 2
 *             v_right = v_x + omega_z * L / 2
 *             where L = 2 * half_track_width_m is the full track width.
 *          3. **Angular Speeds**:
 *             omega_left = v_left / r,  omega_right = v_right / r   (r = wheel_radius_m)
 *          4. **Trimmed PWM Scaling**:
 *             power_left  = round(omega_left  * power_ticks_per_rad_s * left_trim_factor)
 *             power_right = round(omega_right * power_ticks_per_rad_s * right_trim_factor)
 *             where power_ticks_per_rad_s = WHEEL_PWM_DUTY_TICK_MAX / max_wheel_rad_s.
 *
 * @param[in] twist Pointer to a @ref diff_drive_twist_t structure containing target linear and angular velocities.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Velocities successfully calculated and dispatched to motor drivers.
 *         - ESP_ERR_INVALID_STATE: Module not initialized or overcurrent fault state is active.
 *         - ESP_ERR_INVALID_ARG: Null pointer passed for `twist`, or a non-finite (NaN/Inf) component.
 */
esp_err_t diff_drive_set_twist(const diff_drive_twist_t *twist)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");
    ESP_RETURN_ON_FALSE(twist, ESP_ERR_INVALID_ARG, TAG, "Target twist pointer is NULL");
    /* NaN would slip through clampf() (every comparison is false) and reach lroundf(), which is undefined for it */
    ESP_RETURN_ON_FALSE(isfinite(twist->linear_x) && isfinite(twist->angular_z), ESP_ERR_INVALID_ARG, TAG,
                        "Target twist must be finite (NaN or Inf received)");

    portENTER_CRITICAL(&s_safety_flag_spinlock);
    bool is_fault_active = s_ctx.is_fault_active;
    portEXIT_CRITICAL(&s_safety_flag_spinlock);

    /* Safety Check: Prevent new motion command execution while safety lock is triggered */
    if (is_fault_active) 
    {
        ESP_LOGW(TAG, "Motion command rejected: Overcurrent system fault is currently active!");
        return ESP_ERR_INVALID_STATE;
    }

    float linear_x = twist->linear_x;
    float angular_z = twist->angular_z;

    /* Enforce software velocity constraints */
    if (s_ctx.max_linear_velocity_m_s > 0.0f) 
    {
        linear_x = clampf(linear_x, -s_ctx.max_linear_velocity_m_s, s_ctx.max_linear_velocity_m_s);
    }
    if (s_ctx.max_angular_velocity_rad_s > 0.0f) 
    {
        angular_z = clampf(angular_z, -s_ctx.max_angular_velocity_rad_s, s_ctx.max_angular_velocity_rad_s);
    }

    /* Atomically register the new target. The periodic control task (diff_control, lower
     * priority than the safety task) is the sole owner of ramping towards this target and
     * dispatching the resulting PWM to the wheels -- this function never touches hardware. */
    portENTER_CRITICAL(&s_twist_spinlock);
    s_ctx.target_twist.linear_x = linear_x;
    s_ctx.target_twist.angular_z = angular_z;
    portEXIT_CRITICAL(&s_twist_spinlock);

    return ESP_OK;
}

/**
 * @brief Calculates current platform translational and rotational velocities via forward kinematics (Odometry).
 *
 * @details Samples optical incremental encoders over a discrete time delta (dt), computing wheel 
 *          displacement and chassis movement using forward kinematics:
 *          1. **Pulse Delta**: delta_ticks = ticks_current - ticks_previous
 *          2. **Wheel Distances**:
 *             d_wheel = delta_ticks * 2 * pi * r / CPR_wheel
 *             using the CPR of each wheel's own encoder (the two encoders may differ).
 *          3. **Wheel Velocities**:
 *             v_wheel = d_wheel / dt
 *          4. **Chassis Twist Telemetry**:
 *             v_x = (v_right + v_left) / 2
 *             omega_z = (v_right - v_left) / L
 *             with L = 2 * half_track_width_m (the configuration field stores L/2).
 *
 * @note Stateful: the previous encoder sample is kept between calls (it is re-seeded by 
 *       @ref diff_drive_init and @ref diff_drive_calibrate_max_speed), so call it from a single task.
 *
 * @param[in]  delta_time_s Elapsed sampling interval since last call in seconds (dt > 0.0).
 * @param[out] out_twist Pointer to a @ref diff_drive_twist_t structure where computed telemetry will be stored.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Telemetry successfully computed.
 *         - ESP_ERR_INVALID_STATE: Module uninitialized, or the CPR of either encoder is zero.
 *         - ESP_ERR_INVALID_ARG: Null output pointer, or a time interval that is non-positive (dt <= 0.0) or non-finite.
 */
esp_err_t diff_drive_get_twist(float delta_time_s, diff_drive_twist_t *out_twist)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");
    ESP_RETURN_ON_FALSE(out_twist, ESP_ERR_INVALID_ARG, TAG, "Output telemetry pointer is NULL");
    ESP_RETURN_ON_FALSE(delta_time_s > 0.0f && isfinite(delta_time_s), ESP_ERR_INVALID_ARG, TAG, "Delta time must be finite and > 0.0 seconds");
    ESP_RETURN_ON_FALSE(s_ctx.left_encoder_cpr > 0 && s_ctx.right_encoder_cpr > 0,
                        ESP_ERR_INVALID_STATE, TAG, "Both encoder CPRs must be configured");

    int left_encoder_count = 0;
    int right_encoder_count = 0;

    /* Forward Kinematics Step 1: Query hardware encoder pulse accumulators */
    ESP_RETURN_ON_ERROR(wheel_get_encoder_count(s_ctx.left_wheel, &left_encoder_count), TAG, "Failed to sample left encoder");
    ESP_RETURN_ON_ERROR(wheel_get_encoder_count(s_ctx.right_wheel, &right_encoder_count), TAG, "Failed to sample right encoder");

    /* Forward Kinematics Step 2: Determine pulse count difference (delta) since previous call */
    int left_delta_count = left_encoder_count - s_ctx.left_last_encoder_count;
    int right_delta_count = right_encoder_count - s_ctx.right_last_encoder_count;

    /* Update cached counter state */
    s_ctx.left_last_encoder_count = left_encoder_count;
    s_ctx.right_last_encoder_count = right_encoder_count;

    /* Forward Kinematics Step 3: Compute meters per encoder pulse, per wheel (the two encoders may differ) */
    const float wheel_circumference_m = 2.0f * (float)M_PI * s_ctx.wheel_radius_m;

    float left_meters_per_count  = wheel_circumference_m / (float)s_ctx.left_encoder_cpr;
    float right_meters_per_count = wheel_circumference_m / (float)s_ctx.right_encoder_cpr;
    float left_distance_m  = (float)left_delta_count  * left_meters_per_count;
    float right_distance_m = (float)right_delta_count * right_meters_per_count;

    /* Forward Kinematics Step 4: Calculate linear speeds per wheel in m/s */
    float left_velocity_m_s = left_distance_m / delta_time_s;
    float right_velocity_m_s = right_distance_m / delta_time_s;

    /* Forward Kinematics Step 5: Solve differential forward kinematics equations for chassis twist */
    out_twist->linear_x = (right_velocity_m_s + left_velocity_m_s) / 2.0f;
    /* half_track_width_m is L/2, so the track width in the denominator is 2 * half_track_width_m */
    out_twist->angular_z = (right_velocity_m_s - left_velocity_m_s) / (2.0f * s_ctx.half_track_width_m);

    return ESP_OK;
}

/**
 * @brief Reads the raw ADC current-sensor value of both wheels and returns them by reference.
 *
 * @param[out] left_current_raw  Raw ADC counts of the LEFT wheel current sensor. Must not be NULL.
 * @param[out] right_current_raw Raw ADC counts of the RIGHT wheel current sensor. Must not be NULL.
 *
 * @return esp_err_t ESP_OK, ESP_ERR_INVALID_STATE (not initialized, or a wheel without current sensor),
 *         ESP_ERR_INVALID_ARG (NULL pointer), or the ADC driver error of the first wheel that failed.
 */
esp_err_t diff_drive_get_current_raw(int *left_current_raw, int *right_current_raw)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");
    ESP_RETURN_ON_FALSE(left_current_raw && right_current_raw, ESP_ERR_INVALID_ARG, TAG, "Output pointer is NULL");

    /* Read both wheels before looking at any status, so a missing sensor on one does not hide the other */
    esp_err_t left_status = wheel_get_current_raw(s_ctx.left_wheel, left_current_raw);
    esp_err_t right_status = wheel_get_current_raw(s_ctx.right_wheel, right_current_raw);

    return (left_status != ESP_OK) ? left_status : right_status;
}

/**
 * @brief Reads the current of both wheels in milliamps and returns them by reference.
 *
 * @param[out] left_current_ma  Current of the LEFT wheel in mA. Must not be NULL.
 * @param[out] right_current_ma Current of the RIGHT wheel in mA. Must not be NULL.
 *
 * @return esp_err_t ESP_OK, ESP_ERR_INVALID_STATE (not initialized, factor not configured, or a wheel without
 *         current sensor), ESP_ERR_INVALID_ARG (NULL pointer), or the ADC driver error of the first wheel that failed.
 */
esp_err_t diff_drive_get_current_ma(float *left_current_ma, float *right_current_ma)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");
    ESP_RETURN_ON_FALSE(left_current_ma && right_current_ma, ESP_ERR_INVALID_ARG, TAG, "Output pointer is NULL");
    ESP_RETURN_ON_FALSE(s_ctx.current_ma_per_count > 0.0f, ESP_ERR_INVALID_STATE, TAG, "current_ma_per_count is not configured");

    int left_current_raw = 0;
    int right_current_raw = 0;

    /* Read both wheels before looking at any status, so a missing sensor on one does not hide the other */
    esp_err_t left_status = wheel_get_current_raw(s_ctx.left_wheel, &left_current_raw);
    esp_err_t right_status = wheel_get_current_raw(s_ctx.right_wheel, &right_current_raw);

    /* A wheel whose read failed reports 0 mA instead of an invalid value */
    *left_current_ma = (left_status == ESP_OK) ? (float)left_current_raw * s_ctx.current_ma_per_count : 0.0f;
    *right_current_ma = (right_status == ESP_OK) ? (float)right_current_raw * s_ctx.current_ma_per_count : 0.0f;

    return (left_status != ESP_OK) ? left_status : right_status;
}

/**
 * @brief Immediately engages active electronic braking on both motor drivers.
 *
 * @details Overrides any ongoing PWM output and commands the underlying driver channels into
 *          a high-side or low-side short circuit state (active brake / short brake).
 *
 *          It also discards the registered target twist and the ramp state, so the periodic control task
 *          keeps the wheels braked instead of re-energizing them on its next cycle: the platform stays
 *          stopped until the next @ref diff_drive_set_twist. The wheels are braked immediately; a control
 *          cycle that was already in flight can re-apply its previous power at most once, and the control
 *          task re-asserts the brake on its next cycle (at most `control_period_ms` later).
 *
 *          Both wheels are always commanded, even if the first one reports an error.
 *
 * @return esp_err_t
 *         - ESP_OK: Active braking engaged on left and right wheel drivers.
 *         - ESP_ERR_INVALID_STATE: Module not initialized.
 */
esp_err_t diff_drive_brake(void)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");

    /* Discard the target first, so no control cycle that starts from now on can ramp towards it, and ask the
     * control task (sole owner of the ramp state) to zero the ramp and keep the brake asserted. */
    portENTER_CRITICAL(&s_twist_spinlock);
    s_ctx.target_twist.linear_x = 0.0f;
    s_ctx.target_twist.angular_z = 0.0f;
    s_ctx.is_brake_requested = true;
    portEXIT_CRITICAL(&s_twist_spinlock);

    /* Brake both wheels immediately, and report the first failure only after trying both */
    esp_err_t left_status = wheel_brake(s_ctx.left_wheel);
    esp_err_t right_status = wheel_brake(s_ctx.right_wheel);
    ESP_RETURN_ON_ERROR(left_status, TAG, "Failed to apply brake to left wheel driver");
    ESP_RETURN_ON_ERROR(right_status, TAG, "Failed to apply brake to right wheel driver");

    return ESP_OK;
}

/**
 * @brief Checks if an emergency fault (overcurrent/stall) is latching the system shut down.
 *
 * @return true An emergency lockout is currently active. Motion commands will be rejected.
 * @return false Normal system state. Control commands are permitted.
 */
bool diff_drive_is_fault_active(void)
{
    if (!s_ctx.is_initialized) return false;

    portENTER_CRITICAL(&s_safety_flag_spinlock);
    bool is_fault_active = s_ctx.is_fault_active;
    portEXIT_CRITICAL(&s_safety_flag_spinlock);

    return is_fault_active;
}

/**
 * @brief Clears an active emergency lockout state, restoring normal motion command execution.
 *
 * @return esp_err_t 
 *         - ESP_OK: Fault flag cleared.
 *         - ESP_ERR_INVALID_STATE: Module not initialized.
 */
esp_err_t diff_drive_clear_fault(void)
{
    ESP_RETURN_ON_FALSE(s_ctx.is_initialized, ESP_ERR_INVALID_STATE, TAG, "Module is not initialized");

    portENTER_CRITICAL(&s_safety_flag_spinlock);
    s_ctx.is_fault_active = false;
    portEXIT_CRITICAL(&s_safety_flag_spinlock);
  
    ESP_LOGI(TAG, "Emergency overcurrent fault latch cleared successfully.");
    return ESP_OK;
}