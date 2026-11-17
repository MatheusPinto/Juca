/**
 * @file    mpu6050_wire.cpp
 * @brief   I2C transport of the MPU6050 driver, over the Arduino Wire library.
 *
 * @details The ESP-IDF firmware talks to the sensor with the ESP-IDF I2C driver. In Arduino the bus
 *          belongs to `Wire`, and mixing the two drivers in one program is not allowed (the ESP-IDF
 *          aborts at start-up). So the three bus operations the driver needs are implemented here
 *          with Wire, and mpu6050.c calls them.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#include <Arduino.h>
#include <Wire.h>

#include "esp_err.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "soc/soc_caps.h"

/** @brief Largest block one transfer may carry. The Wire buffer is 128 bytes by default. */
#define MPU6050_WIRE_MAX_TRANSFER  120

/** @brief Returns the Wire object of an I2C port number: 0 is Wire, 1 is Wire1 (when the chip has two). */
static TwoWire &bus_of(i2c_port_t i2c_num)
{
#if (defined(SOC_I2C_NUM) && (SOC_I2C_NUM > 1)) || (defined(SOC_HP_I2C_NUM) && (SOC_HP_I2C_NUM > 1))
    if (i2c_num == I2C_NUM_1)
    {
        return Wire1;
    }
#endif
    (void)i2c_num;
    return Wire;
}

/** @brief Translates the result of Wire.endTransmission() into an ESP-IDF error code. */
static esp_err_t error_of(uint8_t wire_status)
{
    switch (wire_status)
    {
    case 0:  return ESP_OK;
    case 5:  return ESP_ERR_TIMEOUT;
    default: return ESP_FAIL;   /* 2 or 3: no acknowledge from the device; 1 or 4: bus error */
    }
}

extern "C" esp_err_t mpu6050_port_begin(i2c_port_t i2c_num, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t frequency_hz)
{
    /* If the sketch already started this bus, Wire keeps it and only reports success. */
    return bus_of(i2c_num).begin((int)sda_gpio, (int)scl_gpio, frequency_hz) ? ESP_OK : ESP_FAIL;
}

extern "C" esp_err_t mpu6050_port_write(i2c_port_t i2c_num, uint8_t address, uint8_t reg, const uint8_t *data, size_t len)
{
    if (len > MPU6050_WIRE_MAX_TRANSFER)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    TwoWire &bus = bus_of(i2c_num);
    bus.beginTransmission(address);
    bus.write(reg);
    bus.write(data, len);
    return error_of(bus.endTransmission());
}

extern "C" esp_err_t mpu6050_port_read(i2c_port_t i2c_num, uint8_t address, uint8_t reg, uint8_t *data, size_t len)
{
    if (len > MPU6050_WIRE_MAX_TRANSFER)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    TwoWire &bus = bus_of(i2c_num);

    /* Send the register address, keep the bus (repeated start), then read. */
    bus.beginTransmission(address);
    bus.write(reg);
    esp_err_t status = error_of(bus.endTransmission(false));
    if (status != ESP_OK)
    {
        return status;
    }
    if (bus.requestFrom(address, len, true) != len)
    {
        return ESP_FAIL;
    }
    bus.readBytes(data, len);
    return ESP_OK;
}
