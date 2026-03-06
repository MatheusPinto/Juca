#include "mpu6050.h"

static const char *TAG = "MPU6050";

// Função para escrever um byte no MPU6050
esp_err_t mpu6050_write_register(i2c_port_t i2c_num, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_write_byte(cmd, data, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Função para ler múltiplos bytes do MPU6050
esp_err_t mpu6050_read_registers(i2c_port_t i2c_num, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Função de inicialização do MPU6050
esp_err_t mpu6050_init(i2c_port_t i2c_num, gpio_num_t sda_gpio, gpio_num_t scl_gpio) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000  // 100kHz
    };

    esp_err_t ret = i2c_param_config(i2c_num, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter config failed");
        return ret;
    }

    ret = i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return ret;
    }

    // Acorda o MPU6050 e inicializa o sensor
    ret = mpu6050_write_register(i2c_num, MPU6050_REG_PWR_MGMT_1, 0x00);  // Acorda o sensor
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }

    return ESP_OK;
}

// Função para ler dados de aceleração e giroscópio
esp_err_t mpu6050_read_data(i2c_port_t i2c_num, mpu6050_data_t *data) {
    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_registers(i2c_num, MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data");
        return ret;
    }

    // Aceleração
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Giroscópio
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}
