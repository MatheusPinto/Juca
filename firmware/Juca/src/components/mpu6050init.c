/**
 * @file    mpu6050init.c
 * @brief   One-call bring-up of the MPU6050 with the DMP.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */
#include "mpu6050init.h"
#include "esp_check.h"
#include "esp_log.h"
#include "juca_port.h"

static const char *TAG __attribute__((unused)) = "mpu6050_init"; /* unused when the Arduino log level hides every message */

esp_err_t mpu6050_complete_init(i2c_port_t i2c_num, gpio_num_t sda_gpio,
                                gpio_num_t scl_gpio, uint8_t calibration_loops)
{
    ESP_RETURN_ON_ERROR(mpu6050_init(i2c_num, sda_gpio, scl_gpio), TAG, "Failed to initialize MPU6050");
    ESP_RETURN_ON_ERROR(mpu6050_dmp_initialize(i2c_num), TAG, "Failed to initialize DMP");

    ESP_LOGI(TAG, "Calibrating MPU6050... keep the sensor still and level!");
    ESP_RETURN_ON_ERROR(mpu6050_calibrate_gyro(i2c_num, calibration_loops), TAG, "Gyro calibration failed");
    ESP_RETURN_ON_ERROR(mpu6050_calibrate_accel(i2c_num, calibration_loops), TAG, "Accel calibration failed");
    ESP_LOGI(TAG, "Calibration complete!");

    ESP_RETURN_ON_ERROR(mpu6050_dmp_enable(i2c_num, true), TAG, "Failed to enable DMP");
    return ESP_OK;
}
