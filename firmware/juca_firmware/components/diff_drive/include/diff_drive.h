/**
 * @file    diff_drive.h
 * @brief   High-level kinematic control, odometry estimation, and safety management library 
 *          for differential drive robotic platforms.
 *
 * @details This component provides an enterprise-grade control layer for two-wheeled differential 
 *          drive rovers operating within the ESP-IDF and FreeRTOS frameworks. It encapsulates 
 *          forward and inverse kinematics calculations, dynamic power-trimming calibration, and 
 *          real-time hardware protection tasks.
 *
 * @section features Key Features
 * - **Inverse Kinematics**: Maps ROS-style translational (v_x) and rotational (omega_z) 
 *   commands (Twist) directly into PWM control signals for dual motor channels.
 * - **Forward Kinematics (Odometry)**: Computes real-time chassis velocity telemetry from quadrature 
 *   encoder tick accumulation over a discrete sampling period (dt).
 * - **Dynamic Auto-Calibration**: Automatically measures maximum physical wheel velocities, 
 *   computing scaling trim factors (left_trim_factor, right_trim_factor) to eliminate 
 *   mechanical and electrical drift during straight-line trajectories.
 * - **Real-time Safety**: Spawns a dedicated FreeRTOS background task (`diff_safety`) that monitors 
 *   raw ADC current levels, triggering active software lockouts and emergency braking upon stall or overcurrent.
 *
 * @section architecture Architecture & Thread Safety
 * - **Design Pattern**: Singleton architecture. All internal state context is statically allocated 
 *   within the source file (`diff_drive.c`) to eliminate heap fragmentation and handle management overhead.
 * - **Concurrency**: Safe for non-blocking invocation from multiple FreeRTOS tasks. Safety state transitions 
 *   utilize atomic flag evaluations to guarantee immediate control-loop shutoff during fault conditions.
 *   Exception: @ref diff_drive_get_twist keeps the previous encoder sample between calls, so it must be
 *   called from a single task only.
 *
 * @section example Usage Example
 * @code{c}
 * // Initialize wheel handles (assuming 'wheel' module is pre-configured)
 * diff_drive_config_t config = {
 *     .left_wheel = left_wheel_h,
 *     .right_wheel = right_wheel_h,
 *     .half_track_width_m = 0.125f,       // L/2: half of the 25 cm track width (L = 0.25 m)
 *     .wheel_radius_m = 0.04f,            // 4 cm wheel radius
 *     .left_encoder_cpr = 1440,           // 1440 CPR in 4x quadrature mode
 *     .right_encoder_cpr = 1440,          // may differ from the left one
 *     .max_linear_velocity_m_s = 1.0f,    // 1.0 m/s maximum speed
 *     .max_angular_velocity_rad_s = 3.14f, // 3.14 rad/s maximum angular speed
 *     .max_wheel_rad_s = 25.0f,           // 25 rad/s maximum motor speed
 *     .max_adc_raw_threshold = 2800,      // Current sensor ADC threshold
 *     .monitor_period_ms = 50,            // Run safety check every 50ms
 *     .safety_task_priority = 5,
 *     .safety_task_stack_size = 3072
 * };
 *
 * if (diff_drive_init(&config) == ESP_OK) {
 *     // Drive forward at 0.5 m/s with no rotation
 *     diff_drive_twist_t cmd = { .linear_x = 0.5f, .angular_z = 0.0f };
 *     diff_drive_set_twist(&cmd);
 * }
 * @endcode
 *
 * @author  Matheus
 * @date    Jan 2025
 */

#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wheel.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *                                 CONSTANTS
 * ==========================================================================*/

/** @brief Shortest accepted test pulse for @ref diff_drive_calibrate_max_speed, in milliseconds. */
#define DIFF_DRIVE_CALIBRATION_MIN_DURATION_MS    500U

/**
 * @brief Longest accepted test pulse for @ref diff_drive_calibrate_max_speed, in milliseconds.
 *
 * @details The safety task is suspended while the pulse runs, so the time spent at constant power
 *          without overcurrent protection is bounded.
 */
#define DIFF_DRIVE_CALIBRATION_MAX_DURATION_MS    10000U

/* ============================================================================
 *                         DATA STRUCTURES AND TYPES
 * ==========================================================================*/

/**
 * @brief Velocity command and telemetry structure formatted according to the ROS/Twist standard.
 *
 * Encapsulates the two planar degrees of freedom (2DoF) applicable to a non-holonomic 
 * differential drive robot operating on a 2D surface.
 */
typedef struct {
    float linear_x;   /**< Translational linear velocity along the longitudinal X-axis in meters per second (m/s). Positive values denote forward movement, negative denote reverse. */
    float angular_z;  /**< Rotational angular velocity around the vertical Z-axis in radians per second (rad/s). Positive values denote counter-clockwise (yaw-left) rotation. */
} diff_drive_twist_t;

/**
 * @brief System-wide configuration structure for the differential drive module.
 *
 * Contains all geometric dimensions, physical hardware limitations, per-wheel encoder specifications, 
 * and operational parameters for the background safety thread. Must be fully populated 
 * before calling @ref diff_drive_init.
 */
typedef struct {
    wheel_handle_t left_wheel;         /**< Opaque handle to the initialized left wheel driver instance. Must not be NULL. */
    wheel_handle_t right_wheel;        /**< Opaque handle to the initialized right wheel driver instance. Must not be NULL. */

    float half_track_width_m;               /**< HALF of the track width (L/2), in meters (m): distance from the robot centerline to the ground contact of each drive wheel. The left-right wheel spacing is L = 2 * `half_track_width_m` (e.g. 0.15f for a 30 cm track). Must be > 0.0. */
    float wheel_radius_m;              /**< Effective rolling radius of the drive wheels (r) in meters (m). Must be > 0.0. */
    uint32_t left_encoder_cpr;         /**< Total counts per revolution (CPR) of the LEFT wheel encoder in 4x quadrature mode. May differ from the right one. Set to 0 if it is physically absent: odometry and calibration are then refused, as both need both encoders. */
    uint32_t right_encoder_cpr;        /**< Same for the RIGHT wheel encoder. */

    float max_linear_velocity_m_s;     /**< Software speed clamp for linear velocity in meters per second (m/s). Set to 0.0f to disable linear speed clamping. */
    float max_angular_velocity_rad_s;  /**< Software speed clamp for angular velocity in radians per second (rad/s). Set to 0.0f to disable angular speed clamping. */
    float max_wheel_rad_s;             /**< Maximum theoretical or empirical motor angular velocity in rad/s when driven at 100% PWM duty cycle. Must be > 0.0. It sets the duty-to-speed scale and, together with the wheels' `max_power_ticks`, the fastest twist the module will ask for (see @ref diff_drive_set_twist); @ref diff_drive_calibrate_max_speed replaces it with the measured value. */

    /* --- Overcurrent and Stall Protection Configurations --- */
    int max_adc_raw_threshold;         /**< Upper limit for raw ADC current sensor readings before triggering an emergency lock. Set to 0 to disable monitoring task. */
    float current_ma_per_count;        /**< Conversion factor from raw ADC counts to milliamps (mA per count), the same for both wheels. Used only by @ref diff_drive_get_current_ma. Set to 0.0f if unknown: that function then returns ESP_ERR_INVALID_STATE. Must be finite and >= 0.0. */
    uint32_t monitor_period_ms;        /**< Sampling and evaluation interval for the safety task in milliseconds (e.g., 50 ms). */
    UBaseType_t safety_task_priority;  /**< FreeRTOS priority level assigned to the background safety monitoring task (e.g., 5). */
    uint32_t safety_task_stack_size;   /**< Memory stack depth in bytes allocated for the safety task (e.g., 3072 bytes). */

    /* --- Periodic Motion Control Task Configuration --- */
    float max_accel_linear_m_s2;       /**< Maximum linear acceleration (m/s^2) enforced by the control task's rate limiter when ramping towards the commanded twist. Set to <= 0.0f to disable ramp limiting (instant application). */
    float max_accel_angular_rad_s2;    /**< Maximum angular acceleration (rad/s^2) enforced by the control task's rate limiter. Set to <= 0.0f to disable ramp limiting (instant application). */
    uint32_t control_period_ms;        /**< Execution period of the periodic motion control task in milliseconds. Set to 0 to use the default (20 ms / 50 Hz). */
    UBaseType_t control_task_priority; /**< FreeRTOS priority level assigned to the periodic control task. MUST be strictly lower than @ref safety_task_priority whenever the safety task is enabled (`max_adc_raw_threshold > 0`); @ref diff_drive_init returns ESP_ERR_INVALID_ARG otherwise. Set to 0 to use the default (4). */
    uint32_t control_task_stack_size;  /**< Memory stack depth in bytes allocated for the control task. Set to 0 to use the default (3072 bytes). */
} diff_drive_config_t;

/* ============================================================================
 *                             PUBLIC API
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
esp_err_t diff_drive_init(const diff_drive_config_t *config);

/**
 * @brief Queries the initialization status of the differential drive controller module.
 *
 * @return true The module is initialized and ready for execution.
 * @return false The module is uninitialized or shut down.
 */
bool diff_drive_is_initialized(void);

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
esp_err_t diff_drive_deinit(void);

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
esp_err_t diff_drive_calibrate_max_speed(uint32_t test_duration_ms, float *out_max_wheel_rad_s);

/**
 * @brief Registers a target platform velocity vector (v_x, omega_z) to be tracked by the periodic control task.
 *
 * @details This function does NOT drive the motors directly. It validates the request, clamps 
 *          v_x and omega_z to the limits defined in configuration, and atomically stores the 
 *          result as the new target twist. The background periodic control task (`diff_control`, 
 *          started by @ref diff_drive_init) is solely responsible for ramping the currently applied 
 *          twist towards this target (respecting `max_accel_linear_m_s2` / `max_accel_angular_rad_s2`) and 
 *          dispatching the resulting inverse-kinematics PWM values to the motor drivers on every cycle:
 *          0. **Reachable speed**: the faster wheel of a twist turns at (|v_x| + |omega_z| * L / 2) / r. The
 *             most a wheel can deliver is `max_wheel_rad_s` (the calibrated value after
 *             @ref diff_drive_calibrate_max_speed) times the allowed duty fraction
 *             (`max_power_ticks` of the wheels / WHEEL_PWM_DUTY_TICK_MAX). If a target needs more, v_x and
 *             omega_z are both multiplied by the same factor, so the robot follows the same curve more slowly.
 *             The limit is applied to the target before the ramp, so the ramp never climbs past what the wheels
 *             can reach. A target inside the limit is not changed. The limit follows the calibration: it
 *             is re-evaluated every control cycle.
 *          1. **Linear Wheel Speeds**:
 *             v_left  = v_x - omega_z * L / 2
 *             v_right = v_x + omega_z * L / 2
 *             where L = 2 * half_track_width_m is the full track width.
 *          2. **Angular Speeds**:
 *             omega_left = v_left / r,  omega_right = v_right / r   (r = wheel_radius_m)
 *          3. **Trimmed PWM Scaling**:
 *             power_left  = round(omega_left  * power_ticks_per_rad_s * left_trim_factor)
 *             power_right = round(omega_right * power_ticks_per_rad_s * right_trim_factor)
 *             where power_ticks_per_rad_s = WHEEL_PWM_DUTY_TICK_MAX / max_wheel_rad_s.
 *
 * @note The target registered here is the clamped one (configuration limits only); the reachable-speed
 *       scaling above is done by the control task, so it always uses the latest calibration.
 *
 * @note This function is fast, non-blocking, and safe to call from any FreeRTOS task at any rate 
 *       (e.g., from a ROS/micro-ROS `cmd_vel` subscriber callback). It does not need to be called 
 *       periodically; the control task will keep tracking the last registered target until a new 
 *       one is set.
 *
 * @param[in] twist Pointer to a @ref diff_drive_twist_t structure containing target linear and angular velocities.
 * 
 * @return esp_err_t 
 *         - ESP_OK: Target twist successfully validated and registered.
 *         - ESP_ERR_INVALID_STATE: Module not initialized or overcurrent fault state is active.
 *         - ESP_ERR_INVALID_ARG: Null pointer passed for `twist`, or a non-finite (NaN/Inf) component.
 */
esp_err_t diff_drive_set_twist(const diff_drive_twist_t *twist);

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
esp_err_t diff_drive_get_twist(float delta_time_s, diff_drive_twist_t *out_twist);

/**
 * @brief Reads the raw ADC current-sensor value of both wheels and returns them by reference.
 *
 * @details One conversion per wheel, in raw ADC counts (the same unit as `max_adc_raw_threshold`; no conversion
 *          to amperes). Both wheels are always read, even if the first read fails.
 *
 * @param[out] left_current_raw  Raw ADC counts of the LEFT wheel current sensor. Must not be NULL.
 * @param[out] right_current_raw Raw ADC counts of the RIGHT wheel current sensor. Must not be NULL.
 *
 * @return esp_err_t
 *         - ESP_OK: Both wheels were read.
 *         - ESP_ERR_INVALID_STATE: Module not initialized, or a wheel has no current sensor (its output is 0).
 *         - ESP_ERR_INVALID_ARG: A NULL pointer.
 *         - Other: error of the ADC driver (the output of that wheel is not valid).
 */
esp_err_t diff_drive_get_current_raw(int *left_current_raw, int *right_current_raw);

/**
 * @brief Reads the current of both wheels in milliamps and returns them by reference.
 *
 * @details Takes one conversion per wheel (as @ref diff_drive_get_current_raw) and multiplies each raw value by
 *          `current_ma_per_count` from the configuration. Both wheels are always read, even if the first read fails.
 *
 * @note The result is only as good as the factor you configured. On the Juca prototype it was calibrated on the
 *       left channel at 100% duty only: below 100% duty the PWM chops the current and the value is not the motor
 *       current, and a current under about 25 mA reads as 0 (amplifier offset).
 *
 * @param[out] left_current_ma  Current of the LEFT wheel in mA. Must not be NULL.
 * @param[out] right_current_ma Current of the RIGHT wheel in mA. Must not be NULL.
 *
 * @return esp_err_t
 *         - ESP_OK: Both wheels were read.
 *         - ESP_ERR_INVALID_STATE: Module not initialized, `current_ma_per_count` not configured (0), or a wheel has
 *           no current sensor (its output is 0).
 *         - ESP_ERR_INVALID_ARG: A NULL pointer.
 *         - Other: error of the ADC driver (the output of that wheel is 0).
 */
esp_err_t diff_drive_get_current_ma(float *left_current_ma, float *right_current_ma);

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
esp_err_t diff_drive_brake(void);

/**
 * @brief Checks if an emergency fault (overcurrent/stall) is latching the system shut down.
 *
 * @details This is a HARD LATCH: once tripped by `diff_safety`, it stays active -- and the 
 *          periodic control task keeps the wheels at zero -- until @ref diff_drive_clear_fault() is 
 *          called explicitly. It never clears itself, even after the overcurrent/stall condition 
 *          that originally caused it goes away.
 *
 * @return true An emergency lockout is currently active. Motion commands will be rejected.
 * @return false Normal system state. Control commands are permitted.
 */
bool diff_drive_is_fault_active(void);

/**
 * @brief Clears an active emergency lockout state, restoring normal motion command execution.
 *
 * @details The moment the fault latched, the previously registered target twist was discarded (reset 
 *          to zero) -- so clearing the fault by itself will NOT make the platform resume its previous 
 *          motion. Call @ref diff_drive_set_twist() again afterwards to command new motion. This is 
 *          intentional: it prevents the platform from automatically driving back into whatever caused 
 *          the stall the instant the fault is cleared.
 *
 * @warning Only call this after confirming it is actually safe to move again (e.g. the obstruction 
 *          has been cleared). Calling it in a tight loop / immediately after every trip without 
 *          addressing the underlying cause will simply cause the fault to re-trip repeatedly.
 *
 * @return esp_err_t 
 *         - ESP_OK: Fault flag cleared.
 *         - ESP_ERR_INVALID_STATE: Module not initialized.
 */
esp_err_t diff_drive_clear_fault(void);

#ifdef __cplusplus
}
#endif