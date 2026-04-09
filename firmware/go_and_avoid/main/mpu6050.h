#ifndef MPU6050_H
#define MPU6050_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

// Endereço I2C do MPU6050
#define MPU6050_ADDR 0x68  // Endereço padrão de 7 bits do MPU6050

// Registradores do MPU6050
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

// Estrutura para armazenar dados de aceleração e giroscópio
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_data_t;

// Função para inicializar o sensor MPU6050
esp_err_t mpu6050_init(i2c_port_t i2c_num, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

// Função para ler dados de aceleração e giroscópio
esp_err_t mpu6050_read_data(i2c_port_t i2c_num, mpu6050_data_t *data);

#endif // MPU6050_H
