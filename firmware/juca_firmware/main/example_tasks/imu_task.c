/**
 * @file    imu_task.c
 * @brief   Example: read the orientation or the acceleration of the robot from the MPU6050.
 *
 * @details The MPU6050 has a small processor inside, the DMP (Digital Motion Processor), that fuses
 *          the gyroscope and the accelerometer and outputs the orientation as a quaternion. This task
 *          starts the sensor, calibrates it (the robot must be still and level) and then prints, every
 *          time there is new data, ONE of three things chosen at compile time with @ref IMU_TASK_OUTPUT:
 *
 *          | IMU_TASK_OUTPUT          | Prints                                   | Source                     |
 *          |--------------------------|------------------------------------------|----------------------------|
 *          | IMU_OUTPUT_QUATERNION    | Orientation quaternion (w, x, y, z)      | DMP, about 10 per second   |
 *          | IMU_OUTPUT_EULER         | Roll, pitch and yaw angles, in degrees   | DMP quaternion, converted  |
 *          | IMU_OUTPUT_ACCELERATION  | Acceleration on x, y, z, in m/s^2        | Accelerometer registers    |
 *
 *          Nothing moves. See README_imu.md in this folder.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "mpu6050init.h"
#include "imu_task.h"
#include "juca_board.h"

/* ============================================================================
 *                     WHAT TO PRINT (choose one, then rebuild)
 * ==========================================================================*/

/** @brief Print the orientation quaternion from the DMP. */
#define IMU_OUTPUT_QUATERNION      1
/** @brief Print roll, pitch and yaw in degrees, computed from the DMP quaternion. */
#define IMU_OUTPUT_EULER           2
/** @brief Print the acceleration of the three axes in m/s^2, read from the accelerometer. */
#define IMU_OUTPUT_ACCELERATION    3

/**
 * @brief What the task prints: one of the three IMU_OUTPUT_* values above.
 *
 * @details It is chosen at compile time, so only the code of the chosen output is built. It can also
 *          be set from the build, without editing this file: add `-DIMU_TASK_OUTPUT=2` to the
 *          compiler flags (for example with `target_compile_definitions` in main/CMakeLists.txt).
 */
#ifndef IMU_TASK_OUTPUT
#define IMU_TASK_OUTPUT            IMU_OUTPUT_EULER
#endif

#if (IMU_TASK_OUTPUT != IMU_OUTPUT_QUATERNION) && (IMU_TASK_OUTPUT != IMU_OUTPUT_EULER) && \
    (IMU_TASK_OUTPUT != IMU_OUTPUT_ACCELERATION)
#error "IMU_TASK_OUTPUT must be IMU_OUTPUT_QUATERNION, IMU_OUTPUT_EULER or IMU_OUTPUT_ACCELERATION"
#endif

/* ============================================================================
 *                                 SETTINGS
 * ==========================================================================*/

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG = "imu_task";

/** @brief Calibration iterations for the gyroscope and for the accelerometer (more = better offsets, slower). */
#define CALIBRATION_LOOPS          15

/** @brief How often the task asks the DMP for a new quaternion. The DMP produces about 10 per second,
 *         so polling every 40 ms never misses one; most polls simply find nothing new. */
#define POLL_PERIOD_MS             40

/** @brief How often the task prints the acceleration (it is read directly, so any period works). */
#define ACCELERATION_PERIOD_MS     100

/** @brief Accelerometer scale: the driver keeps the +/-2 g range, where 1 g is 16384 counts. */
#define ACCEL_COUNTS_PER_G         16384.0f

/** @brief Standard gravity, in m/s^2. */
#define STANDARD_GRAVITY_M_S2      9.80665f

/** @brief Radians to degrees. */
#define RAD_TO_DEG                 57.2957795f

/* ============================================================================
 *                                 OUTPUTS
 * ==========================================================================*/

#if IMU_TASK_OUTPUT == IMU_OUTPUT_QUATERNION

/** @brief Prints the quaternion as it comes from the DMP. */
static void print_output(const mpu6050_quaternion_t *q)
{
    ESP_LOGI(TAG, "Quat [w=%.3f x=%.3f y=%.3f z=%.3f]", q->w, q->x, q->y, q->z);
}

#elif IMU_TASK_OUTPUT == IMU_OUTPUT_EULER

/**
 * @brief Converts the quaternion to roll, pitch and yaw, and prints them in degrees.
 *
 * @details Aerospace convention (rotation order Z, Y, X):
 *          - roll  = rotation around x, from -180 to +180 degrees;
 *          - pitch = rotation around y, from -90 to +90 degrees;
 *          - yaw   = rotation around z (the heading on a flat floor), from -180 to +180 degrees.
 *
 *          All three are relative to the orientation the robot had at the end of the calibration.
 *          Which of roll and pitch is "forward" and the signs depend on how the sensor is mounted.
 *          Near pitch = +/-90 degrees roll and yaw are not defined (gimbal lock) and jump; a robot on
 *          the floor never gets there. Yaw drifts slowly: the sensor has no compass.
 */
static void print_output(const mpu6050_quaternion_t *q)
{
    const float roll_rad = atan2f(2.0f * (q->w * q->x + q->y * q->z),
                                  1.0f - 2.0f * (q->x * q->x + q->y * q->y));

    /* Clamp before asinf: rounding can push the argument slightly beyond +/-1, which would give NaN. */
    float sin_pitch = 2.0f * (q->w * q->y - q->z * q->x);
    if (sin_pitch > 1.0f)  { sin_pitch = 1.0f; }
    if (sin_pitch < -1.0f) { sin_pitch = -1.0f; }
    const float pitch_rad = asinf(sin_pitch);

    const float yaw_rad = atan2f(2.0f * (q->w * q->z + q->x * q->y),
                                 1.0f - 2.0f * (q->y * q->y + q->z * q->z));

    ESP_LOGI(TAG, "Roll %7.1f deg | Pitch %6.1f deg | Yaw %7.1f deg",
             roll_rad * RAD_TO_DEG, pitch_rad * RAD_TO_DEG, yaw_rad * RAD_TO_DEG);
}

#endif

/* ============================================================================
 *                                   TASK
 * ==========================================================================*/

portTASK_FUNCTION(imu_task, arg)
{
    (void)arg;

    /* I2C, DMP firmware, calibration and DMP enable in one call. The I2C port and pins come from the
     * board header. KEEP THE ROBOT STILL AND LEVEL until "Calibration complete!" appears. */
    esp_err_t init_status = mpu6050_complete_init(JUCA_I2C_PORT_IMU, JUCA_PIN_I2C_SDA, JUCA_PIN_I2C_SCL, CALIBRATION_LOOPS);
    if (init_status != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start the IMU: %s", esp_err_to_name(init_status));
        vTaskDelete(NULL);
        return;
    }

#if IMU_TASK_OUTPUT == IMU_OUTPUT_ACCELERATION

    /* The acceleration is read from the accelerometer registers, not from the DMP, so the DMP is
     * stopped: it is not needed, and its FIFO would fill up and overflow for nothing. The offsets
     * written by the calibration stay active, so at rest the reading is about (0, 0, +9.8). */
    mpu6050_dmp_enable(JUCA_I2C_PORT_IMU, false);
    ESP_LOGI(TAG, "Printing the acceleration in m/s^2 (it includes gravity: about +9.8 on the vertical axis at rest).");

    mpu6050_data_t data;
    for ( ; ; )
    {
        esp_err_t read_status = mpu6050_read_data(JUCA_I2C_PORT_IMU, &data);
        if (read_status == ESP_OK)
        {
            /* counts -> g -> m/s^2 */
            const float scale = STANDARD_GRAVITY_M_S2 / ACCEL_COUNTS_PER_G;
            ESP_LOGI(TAG, "Accel [x=%6.2f y=%6.2f z=%6.2f] m/s^2",
                     data.accel_x * scale, data.accel_y * scale, data.accel_z * scale);
        }
        else
        {
            ESP_LOGW(TAG, "Failed to read the accelerometer: %s", esp_err_to_name(read_status));
        }

        vTaskDelay(pdMS_TO_TICKS(ACCELERATION_PERIOD_MS));
    }

#else   /* IMU_OUTPUT_QUATERNION or IMU_OUTPUT_EULER: both come from the DMP quaternion */

    mpu6050_quaternion_t quaternion;
    for ( ; ; )
    {
        esp_err_t read_status = mpu6050_dmp_read_quaternion(JUCA_I2C_PORT_IMU, &quaternion);
        if (read_status == ESP_OK)
        {
            print_output(&quaternion);
        }
        else if (read_status != ESP_ERR_NOT_FOUND)
        {
            /* ESP_ERR_NOT_FOUND only means "no new packet yet" and is not worth a line. Anything else is
             * a real problem: a FIFO overflow (ESP_ERR_INVALID_STATE) or an I2C error. */
            ESP_LOGW(TAG, "Failed to read DMP quaternion: %s", esp_err_to_name(read_status));
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }

#endif
}
