/**
 * @file mpu6050.c
 * @brief I2C access and data reading implementation for the MPU6050.
 */
#include "mpu6050.h"

static const char *TAG = "MPU6050";

/** I2C transaction timeout. */
#define MPU6050_I2C_TIMEOUT_MS 1000

esp_err_t mpu6050_write_register(i2c_port_t i2c_num, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(MPU6050_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_read_registers(i2c_port_t i2c_num, uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd); /* repeated start */
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(MPU6050_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_init(i2c_port_t i2c_num, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000, /* 100 kHz */
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

    /* Wake the sensor (leave sleep mode, use the internal oscillator). */
    ret = mpu6050_write_register(i2c_num, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        i2c_driver_delete(i2c_num); /* allows a later init retry */
        return ret;
    }

    return ESP_OK;
}

esp_err_t mpu6050_read_data(i2c_port_t i2c_num, mpu6050_data_t *data)
{
    /* Layout: AX AY AZ (0-5) | TEMP (6-7) | GX GY GZ (8-13), big-endian. */
    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_registers(i2c_num, MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data");
        return ret;
    }

    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}
