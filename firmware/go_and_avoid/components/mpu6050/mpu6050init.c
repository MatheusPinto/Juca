/**
 * @file mpu6050init.c
 * @brief MPU6050 initialization sequence for this board.
 */
#include "mpu6050init.h"
#include "mpu6050.h"

/** @name Hardware configuration (override with -D if needed) */
/** @{ */
#ifndef MPU6050_SDA_GPIO
#define MPU6050_SDA_GPIO GPIO_NUM_40 /**< SDA GPIO (alternative: GPIO_NUM_21). */
#endif
#ifndef MPU6050_SCL_GPIO
#define MPU6050_SCL_GPIO GPIO_NUM_39 /**< SCL GPIO (alternative: GPIO_NUM_22). */
#endif
#ifndef MPU6050_I2C_PORT
#define MPU6050_I2C_PORT I2C_NUM_0   /**< I2C port in use. */
#endif
/** @} */

/** Number of calibration iterations (gyro and accel). */
#define MPU6050_CALIB_LOOPS 15

static const char *TAG = "MPU_INIT";

esp_err_t mpu6050_complete_init(void)
{
    esp_err_t ret = mpu6050_init(MPU6050_I2C_PORT, MPU6050_SDA_GPIO, MPU6050_SCL_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return ret;
    }

    ret = mpu6050_dmp_initialize(MPU6050_I2C_PORT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DMP");
        return ret;
    }

    ESP_LOGI(TAG, "Calibrating MPU6050... Keep Juca still!");
    ret = mpu6050_calibrate_gyro(MPU6050_I2C_PORT, MPU6050_CALIB_LOOPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Gyro calibration failed");
        return ret;
    }
    ret = mpu6050_calibrate_accel(MPU6050_I2C_PORT, MPU6050_CALIB_LOOPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Accel calibration failed");
        return ret;
    }
    ESP_LOGI(TAG, "Calibration complete!");

    ret = mpu6050_dmp_enable(MPU6050_I2C_PORT, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable DMP");
        return ret;
    }

    return ESP_OK;
}
