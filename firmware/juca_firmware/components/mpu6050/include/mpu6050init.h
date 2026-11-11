/**
 * @file mpu6050init.h
 * @brief Board-specific MPU6050 initialization layer.
 *
 * Defines pins, I2C port and the full bring-up sequence
 * (I2C -> DMP -> calibration -> enable DMP).
 */
#ifndef MPU6050INIT_H
#define MPU6050INIT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run the full MPU6050 initialization sequence.
 *
 * Steps:
 *  -# @ref mpu6050_init (I2C + wake-up)
 *  -# @ref mpu6050_dmp_initialize
 *  -# Gyroscope and accelerometer calibration
 *  -# @ref mpu6050_dmp_enable
 *
 * @warning Keep the sensor still during calibration.
 * @note This function does not terminate the calling task on error;
 *       error handling is the caller's responsibility.
 *
 * @return ESP_OK on success, or the first error encountered.
 */
esp_err_t mpu6050_complete_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050INIT_H */
