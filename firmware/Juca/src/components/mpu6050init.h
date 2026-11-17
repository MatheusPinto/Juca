/**
 * @file    mpu6050init.h
 * @brief   One-call bring-up of the MPU6050 with the DMP.
 *
 * @details Runs, in order: I2C and wake-up, DMP firmware load, gyroscope and
 *          accelerometer calibration, DMP enable. The pins and the I2C port are
 *          arguments: this component does not know the board.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */
#ifndef MPU6050INIT_H
#define MPU6050INIT_H

#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run the full MPU6050 initialization sequence.
 *
 * Steps:
 *  -# @ref mpu6050_init (I2C + wake-up)
 *  -# @ref mpu6050_dmp_initialize
 *  -# @ref mpu6050_calibrate_gyro and @ref mpu6050_calibrate_accel
 *  -# @ref mpu6050_dmp_enable
 *
 * @warning Keep the sensor still and level (Z axis up) during the calibration,
 *          which takes a few seconds.
 * @note This function does not terminate the calling task on error;
 *       error handling is the caller's responsibility. After a failure of step
 *       2 or later the I2C driver stays installed.
 *
 * @param[in] i2c_num           I2C port to use.
 * @param[in] sda_gpio          SDA GPIO.
 * @param[in] scl_gpio          SCL GPIO.
 * @param[in] calibration_loops Calibration iterations for each sensor, 1 to
 *                              255 (15 is a good default).
 *
 * @return ESP_OK on success, or the first error encountered.
 */
esp_err_t mpu6050_complete_init(i2c_port_t i2c_num, gpio_num_t sda_gpio,
                                gpio_num_t scl_gpio, uint8_t calibration_loops);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050INIT_H */
