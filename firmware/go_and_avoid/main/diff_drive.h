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
 * - **Inverse Kinematics**: Maps ROS-style translational ($v_x$) and rotational ($\omega_z$) 
 *   commands (Twist) directly into PWM control signals for dual motor channels.
 * - **Forward Kinematics (Odometry)**: Computes real-time chassis velocity telemetry from quadrature 
 *   encoder tick accumulation over a discrete sampling period ($\Delta t$).
 * - **Dynamic Auto-Calibration**: Automatically measures maximum physical wheel velocities, 
 *   computing scaling trim factors ($\text{left\_trim\_factor}, \text{right\_trim\_factor}$) to eliminate 
 *   mechanical and electrical drift during straight-line trajectories.
 * - **Real-time Safety**: Spawns a dedicated FreeRTOS background task (`diff_safety_tsk`) that monitors 
 *   raw ADC current levels, triggering active software lockouts and emergency braking upon stall or overcurrent.
 *
 * @section architecture Architecture & Thread Safety
 * - **Design Pattern**: Singleton architecture. All internal state context is statically allocated 
 *   within the source file (`diff_drive.c`) to eliminate heap fragmentation and handle management overhead.
 * - **Concurrency**: Safe for non-blocking invocation from multiple FreeRTOS tasks. Safety state transitions 
 *   utilize atomic flag evaluations to guarantee immediate control-loop shutoff during fault conditions.
 *
 * @section example Usage Example
 * @code{c}
 * // Initialize wheel handles (assuming 'wheel' module is pre-configured)
 * diffDriveConfig_t config = {
 *     .left_wheel = left_wheel_h,
 *     .right_wheel = right_wheel_h,
 *     .half_track_width = 0.25f,               // 25 cm track width
 *     .wheel_radius = 0.04f,              // 4 cm wheel radius
 *     .encoder_cpr = 1440,                // 1440 CPR in 4x quadrature mode
 *     .max_linear_velocity = 1.0f,        // 1.0 m/s maximum speed
 *     .max_angular_velocity = 3.14f,      // 3.14 rad/s maximum angular speed
 *     .max_wheel_rad_s = 25.0f,           // 25 rad/s maximum motor speed
 *     .max_adc_raw_threshold = 2800,      // Current sensor ADC threshold
 *     .monitor_period_ms = 50,            // Run safety check every 50ms
 *     .safety_task_priority = 5,
 *     .safety_task_stack_size = 3072
 * };

 * if (DiffDrive_Init(&config) == ESP_OK) {
 *     // Drive forward at 0.5 m/s with no rotation
 *     diffDriveTwist_t cmd = { .linear_x = 0.5f, .angular_z = 0.0f };
 *     DiffDrive_SetTwist(&cmd);
 * }
 * @endcode
 *
 * @author  Matheus
 * @date    Jan 2025
 */

#ifndef MAIN_DIFF_DRIVE_H_
#define MAIN_DIFF_DRIVE_H_

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
} diffDriveTwist_t;

/**
 * @brief System-wide configuration structure for the differential drive module.
 *
 * Contains all geometric dimensions, physical hardware limitations, encoder specifications, 
 * and operational parameters for the background safety thread. Must be fully populated 
 * before calling @ref DiffDrive_Init.
 */
typedef struct {
    wheelHandle_t left_wheel;          /**< Opaque handle to the initialized left wheel driver instance. Must not be NULL. */
    wheelHandle_t right_wheel;         /**< Opaque handle to the initialized right wheel driver instance. Must not be NULL. */

    float half_track_width;                 /**< Lateral distance between the centerlines of the left and right wheel ground contacts ($L$) in meters (m). Must be $> 0.0$. */
    float wheel_radius;                /**< Effective rolling radius of the drive wheels ($r$) in meters (m). Must be $> 0.0$. */
    uint32_t encoder_cpr;              /**< Total counts per revolution (CPR) of the encoder in 4x quadrature mode. Set to 0 if encoders are physically absent. */

    float max_linear_velocity;         /**< Software speed clamp for linear velocity in meters per second (m/s). Set to 0.0f to disable linear speed clamping. */
    float max_angular_velocity;        /**< Software speed clamp for angular velocity in radians per second (rad/s). Set to 0.0f to disable angular speed clamping. */
    float max_wheel_rad_s;             /**< Maximum theoretical or empirical motor angular velocity in rad/s when driven at 100% PWM duty cycle. Must be $> 0.0$. */

    /* --- Overcurrent and Stall Protection Configurations --- */
    int max_adc_raw_threshold;         /**< Upper limit for raw ADC current sensor readings before triggering an emergency lock. Set to 0 to disable monitoring task. */
    uint32_t monitor_period_ms;        /**< Sampling and evaluation interval for the safety task in milliseconds (e.g., 50 ms). */
    UBaseType_t safety_task_priority;  /**< FreeRTOS priority level assigned to the background safety monitoring task (e.g., 5). */
    uint32_t safety_task_stack_size;   /**< Memory stack depth in bytes allocated for the safety task (e.g., 3072 bytes). */
} diffDriveConfig_t;

/* ============================================================================
 *                             PUBLIC API
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
esp_err_t DiffDrive_Init(const diffDriveConfig_t *config);

/**
 * @brief Queries the initialization status of the differential drive controller module.
 *
 * @return true The module is initialized and ready for execution.
 * @return false The module is uninitialized or shut down.
 */
bool DiffDrive_IsInitialized(void);

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
esp_err_t DiffDrive_Deinit(void);

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
esp_err_t DiffDrive_CalibrateMaxSpeed(uint32_t test_duration_ms, float *out_max_wheel_rad_s);

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
esp_err_t DiffDrive_SetTwist(const diffDriveTwist_t *twist);

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
esp_err_t DiffDrive_GetTwist(float dt_seconds, diffDriveTwist_t *out_twist);

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
esp_err_t DiffDrive_Brake(void);

/**
 * @brief Checks if an emergency fault (overcurrent/stall) is latching the system shut down.
 *
 * @return true An emergency lockout is currently active. Motion commands will be rejected.
 * @return false Normal system state. Control commands are permitted.
 */
bool DiffDrive_IsFaultActive(void);

/**
 * @brief Clears an active emergency lockout state, restoring normal motion command execution.
 *
 * @return esp_err_t 
 *         - ESP_OK: Fault flag cleared.
 *         - ESP_ERR_INVALID_STATE: Module not initialized.
 */
esp_err_t DiffDrive_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_DIFF_DRIVE_H_ */