#include "freertos/Freertos.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_GPIO GPIO_NUM_40
#define I2C_MASTER_SCL_GPIO GPIO_NUM_39
#define I2C_NUM I2C_NUM_0

portTASK_FUNCTION(IMU_Task, arg) 
{
    esp_err_t ret;

    // Inicializa o MPU6050
    ret = mpu6050_init(I2C_NUM, I2C_MASTER_SDA_GPIO, I2C_MASTER_SCL_GPIO);
    if (ret != ESP_OK)
    {
        ESP_LOGE("MPU6050", "Failed to initialize MPU6050");
        return;
    }

    // Estrutura para armazenar os dados lidos
    mpu6050_data_t data;

    while (1)
    {
        // Lê os dados do sensor
        ret = mpu6050_read_data(I2C_NUM, &data);
        if (ret == ESP_OK)
        {
        	ESP_LOGI("IMU", "Gyro Z: %d", data.gyro_z);

            //ESP_LOGI("MPU6050", "Accel X: %d, Accel Y: %d, Accel Z: %d", data.accel_x, data.accel_y, data.accel_z);
            //ESP_LOGI("MPU6050", "Gyro X: %d, Gyro Y: %d, Gyro Z: %d", data.gyro_x, data.gyro_y, data.gyro_z);
        }
        else
        {
            ESP_LOGE("IMU", "Failed to read data");
        }

        vTaskDelay(pdMS_TO_TICKS(500));  // Atraso de 500
    }
}
