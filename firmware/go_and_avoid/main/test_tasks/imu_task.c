#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "esp_log.h"
//#include "webserver.h"
#include <math.h>
#include <stdio.h>

#define I2C_MASTER_SDA_GPIO GPIO_NUM_40
#define I2C_MASTER_SCL_GPIO GPIO_NUM_39
//#define I2C_MASTER_SDA_GPIO GPIO_NUM_21
//#define I2C_MASTER_SCL_GPIO GPIO_NUM_22
#define I2C_NUM I2C_NUM_0

static const char *TAG = "IMU";

portTASK_FUNCTION(IMU_Task, arg) 
{
    (void)arg;

    esp_err_t ret;

    // Inicializa o MPU6050
    ret = mpu6050_init(I2C_NUM, I2C_MASTER_SDA_GPIO, I2C_MASTER_SCL_GPIO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        vTaskDelete(NULL);
        return;
    }

    ret = mpu6050_dmp_initialize(I2C_NUM);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize DMP");
        vTaskDelete(NULL);
        return;
    }

    // CALIBRATION START
    ESP_LOGI(TAG, "Calibrating MPU6050... Keep Juca still!");
    // These functions usually take ~400-1000 samples and write to MPU registers
    mpu6050_calibrate_gyro(I2C_NUM, 15); // '15' is a common loop count for calibration
    mpu6050_calibrate_accel(I2C_NUM, 15);
    ESP_LOGI(TAG, "Calibration complete!");

    ret = mpu6050_dmp_enable(I2C_NUM, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable DMP");
        vTaskDelete(NULL);
        return;
    }

    mpu6050_quaternion_t quaternion;

    while (1)
    {
        ret = mpu6050_dmp_read_quaternion(I2C_NUM, &quaternion);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "Quat [w=%.3f x=%.3f y=%.3f z=%.3f]",
                     quaternion.w,
                     quaternion.x,
                     quaternion.y,
                     quaternion.z);
            //stream_quaternion_to_web((httpd_handle_t)arg, quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        }
        else
        {
            //ESP_LOGW(TAG, "Failed to read DMP quaternion: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(40));
    }
}