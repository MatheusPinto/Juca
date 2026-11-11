/**
 * @file    mpu6050.c
 * @brief   I2C access, DMP firmware loading, calibration, and data reading
 * implementation for the MPU6050.
 *
 * @details Implements low-level I2C communication routines for MPU-6050 sensor
 * configuration, DMP (Digital Motion Processor) image flashing and FIFO
 * extraction (quaternion orientation), and closed-loop PID offset calibration
 * for gyroscopes and accelerometers.
 *
 * @note    DMP initialization and memory loading routines are based on the
 * I2Cdevlib MPU-6050 implementation by Jeff Rowberg.
 *
 * @date    2025
 * @author  Matheus
 */

#include "mpu6050.h"
#include "mpu6050_dmp_firmware.h"

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG = "MPU6050";

/* ============================================================================
 *                              DEFINITIONS
 * ==========================================================================*/

/** @brief Default I2C transaction timeout in milliseconds. */
#define MPU6050_I2C_TIMEOUT_MS 1000

/* ============================================================================
 *                     PRIVATE HELPER FUNCTIONS
 * ==========================================================================*/

/**
 * @brief Write a contiguous sequence of bytes to an MPU6050 register address
 * via I2C.
 *
 * @param[in] i2c_num I2C port number.
 * @param[in] reg     Target register start address.
 * @param[in] data    Pointer to buffer containing data to write.
 * @param[in] len     Number of bytes to write.
 * @return esp_err_t ESP_OK on success, or an ESP-IDF I2C driver error code.
 */
static esp_err_t mpu6050_write_registers(i2c_port_t i2c_num, uint8_t reg,
                                         const uint8_t *data, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd, reg, 1);
  i2c_master_write(cmd, (uint8_t *)data, len, 1);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief Update specific bit fields of a register using a bitmask.
 *
 * @param[in] i2c_num I2C port number.
 * @param[in] reg     Target register address.
 * @param[in] mask    Bitmask defining bits to be modified.
 * @param[in] value   New bit values to apply within the masked positions.
 * @return esp_err_t ESP_OK on success, or an error code from read/write
 * operations.
 */
static esp_err_t mpu6050_update_register_bits(i2c_port_t i2c_num, uint8_t reg,
                                              uint8_t mask, uint8_t value) {
  uint8_t current = 0;
  esp_err_t ret = mpu6050_read_registers(i2c_num, reg, &current, 1);
  if (ret != ESP_OK) {
    return ret;
  }

  current = (current & ~mask) | (value & mask);
  return mpu6050_write_register(i2c_num, reg, current);
}

/**
 * @brief Select the DMP firmware memory bank and addressing modes.
 *
 * @param[in] i2c_num          I2C port number.
 * @param[in] bank             Memory bank index (0-31).
 * @param[in] prefetch_enabled Enable DMP memory prefetch mode.
 * @param[in] user_bank        Enable user bank selection mode.
 * @return esp_err_t ESP_OK on success, or an I2C communication error code.
 */
static esp_err_t mpu6050_set_memory_bank(i2c_port_t i2c_num, uint8_t bank,
                                         bool prefetch_enabled,
                                         bool user_bank) {
  bank &= 0x1F;
  if (user_bank) {
    bank |= 0x20;
  }
  if (prefetch_enabled) {
    bank |= 0x40;
  }
  return mpu6050_write_register(i2c_num, MPU6050_REG_BANK_SEL, bank);
}

/**
 * @brief Set the starting memory byte offset within the selected DMP memory
 * bank.
 *
 * @param[in] i2c_num I2C port number.
 * @param[in] address Start address offset (0-255).
 * @return esp_err_t ESP_OK on success, or an I2C communication error code.
 */
static esp_err_t mpu6050_set_memory_start_address(i2c_port_t i2c_num,
                                                  uint8_t address) {
  return mpu6050_write_register(i2c_num, MPU6050_REG_MEM_START, address);
}

/**
 * @brief Read a 16-bit signed big-endian word from two consecutive registers.
 *
 * @param[in]  i2c_num I2C port number.
 * @param[in]  reg     Starting register address (MSB).
 * @param[out] value   Pointer to store decoded 16-bit signed integer.
 * @return esp_err_t ESP_OK on success, or an I2C communication error code.
 */
static esp_err_t mpu6050_read_word(i2c_port_t i2c_num, uint8_t reg,
                                   int16_t *value) {
  uint8_t raw[2] = {0};
  esp_err_t ret = mpu6050_read_registers(i2c_num, reg, raw, 2);
  if (ret != ESP_OK) {
    return ret;
  }
  *value = (int16_t)((raw[0] << 8) | raw[1]);
  return ESP_OK;
}

/**
 * @brief Write a 16-bit signed word in big-endian format to two consecutive
 * registers.
 *
 * @param[in] i2c_num I2C port number.
 * @param[in] reg     Starting register address (MSB).
 * @param[in] value   16-bit signed word to write.
 * @return esp_err_t ESP_OK on success, or an I2C communication error code.
 */
static esp_err_t mpu6050_write_word(i2c_port_t i2c_num, uint8_t reg,
                                    int16_t value) {
  uint8_t raw[2] = {(uint8_t)((value >> 8) & 0xFF), (uint8_t)(value & 0xFF)};
  return mpu6050_write_registers(i2c_num, reg, raw, 2);
}

/**
 * @brief Linearly map a floating-point value from an input range to an output
 * range.
 *
 * @param[in] val     Input value to map.
 * @param[in] in_min  Lower boundary of input range.
 * @param[in] in_max  Upper boundary of input range.
 * @param[in] out_min Lower boundary of output range.
 * @param[in] out_max Upper boundary of output range.
 * @return float Mapped value scaled to the output range.
 */
static float mpu6050_mapf(float val, float in_min, float in_max, float out_min,
                          float out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Closed-loop PID offset calibration routine for accelerometer or
 * gyroscope axes.
 *
 * @details Reads sensor axis measurements iteratively, computes error against
 * expected zero (or 1g on gravity axis for Z-accelerometer), and writes
 * adjusted hardware offsets.
 *
 * @param[in] i2c_num      I2C port number.
 * @param[in] read_address Starting register address for sensor data (accel or
 * gyro).
 * @param[in] kP           Proportional calibration gain.
 * @param[in] kI           Integral calibration gain.
 * @param[in] loops        Number of calibration iterations to execute.
 * @return esp_err_t ESP_OK on success, or an error code if I2C transaction
 * fails.
 */
static esp_err_t mpu6050_calibrate_pid(i2c_port_t i2c_num, uint8_t read_address,
                                       float kP, float kI, uint8_t loops) {
  const uint8_t save_address = (read_address == MPU6050_REG_ACCEL_XOUT_H)
                                   ? MPU6050_REG_XA_OFFS_H
                                   : MPU6050_REG_XG_OFFS_USRH;
  int16_t data = 0;
  int16_t bit_zero[3] = {0};
  float i_term[3] = {0.0f, 0.0f, 0.0f};

  for (int i = 0; i < 3; i++) {
    esp_err_t ret =
        mpu6050_read_word(i2c_num, (uint8_t)(save_address + (i * 2)), &data);
    if (ret != ESP_OK) {
      return ret;
    }
    if (save_address != MPU6050_REG_XG_OFFS_USRH) {
      bit_zero[i] = data & 1;
      i_term[i] = (float)data * 8.0f;
    } else {
      i_term[i] = (float)data * 4.0f;
    }
  }

  for (uint8_t l = 0; l < loops; l++) {
    int16_t e_sample = 0;

    for (int c = 0; c < 100; c++) {
      float e_sum = 0.0f;

      for (int i = 0; i < 3; i++) {
        esp_err_t ret = mpu6050_read_word(
            i2c_num, (uint8_t)(read_address + (i * 2)), &data);
        if (ret != ESP_OK) {
          return ret;
        }

        float reading = (float)data;
        if ((read_address == MPU6050_REG_ACCEL_XOUT_H) && (i == 2)) {
          reading -= 16384.0f;
        }

        const float error = -reading;
        e_sum += fabsf(reading);

        const float p_term = kP * error;
        i_term[i] += (error * 0.001f) * kI;

        if (save_address != MPU6050_REG_XG_OFFS_USRH) {
          data = (int16_t)lroundf((p_term + i_term[i]) / 8.0f);
          data = (int16_t)((data & 0xFFFE) | bit_zero[i]);
        } else {
          data = (int16_t)lroundf((p_term + i_term[i]) / 4.0f);
        }

        ret = mpu6050_write_word(i2c_num, (uint8_t)(save_address + (i * 2)),
                                 data);
        if (ret != ESP_OK) {
          return ret;
        }
      }

      if ((c == 99) && (e_sum > 1000.0f)) {
        c = 0;
      }

      if ((e_sum *
           ((read_address == MPU6050_REG_ACCEL_XOUT_H) ? 0.05f : 1.0f)) <
          5.0f) {
        e_sample++;
      }
      if ((e_sum < 100.0f) && (c > 10) && (e_sample >= 10)) {
        break;
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }

    kP *= 0.75f;
    kI *= 0.75f;

    for (int i = 0; i < 3; i++) {
      if (save_address != MPU6050_REG_XG_OFFS_USRH) {
        data = (int16_t)lroundf(i_term[i] / 8.0f);
        data = (int16_t)((data & 0xFFFE) | bit_zero[i]);
      } else {
        data = (int16_t)lroundf(i_term[i] / 4.0f);
      }

      esp_err_t ret =
          mpu6050_write_word(i2c_num, (uint8_t)(save_address + (i * 2)), data);
      if (ret != ESP_OK) {
        return ret;
      }
    }
  }

  esp_err_t ret = mpu6050_update_register_bits(i2c_num, MPU6050_REG_USER_CTRL,
                                               (1 << 2), (1 << 2));
  if (ret != ESP_OK) {
    return ret;
  }

  return mpu6050_update_register_bits(i2c_num, MPU6050_REG_USER_CTRL, (1 << 3),
                                      (1 << 3));
}

/**
 * @brief Write a block of firmware or configuration data to DMP memory across
 * bank boundaries.
 *
 * @param[in] i2c_num   I2C port number.
 * @param[in] data      Pointer to source byte array.
 * @param[in] data_size Number of bytes to transfer.
 * @param[in] bank      Initial DMP memory bank.
 * @param[in] address   Initial byte offset within the bank.
 * @return esp_err_t ESP_OK on success, or an ESP-IDF error code.
 */
static esp_err_t mpu6050_write_memory_block(i2c_port_t i2c_num,
                                            const uint8_t *data,
                                            uint16_t data_size, uint8_t bank,
                                            uint8_t address) {
  uint16_t i = 0;
  while (i < data_size) {
    uint16_t chunk_size = MPU6050_DMP_MEMORY_CHUNK_SIZE;
    if (i + chunk_size > data_size) {
      chunk_size = data_size - i;
    }
    if (chunk_size > (uint16_t)(256 - address)) {
      chunk_size = (uint16_t)(256 - address);
    }

    if (chunk_size == 0) {
      ESP_LOGE(TAG,
               "Invalid zero-length DMP memory chunk at bank %u address %u",
               bank, address);
      return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t ret = mpu6050_set_memory_bank(i2c_num, bank, false, false);
    if (ret != ESP_OK) {
      return ret;
    }

    ret = mpu6050_set_memory_start_address(i2c_num, address);
    if (ret != ESP_OK) {
      return ret;
    }

    ret = mpu6050_write_registers(i2c_num, MPU6050_REG_MEM_R_W, data + i,
                                  chunk_size);
    if (ret != ESP_OK) {
      return ret;
    }

    i += chunk_size;
    address += chunk_size;
    if (address == 0) {
      bank++;
    }
  }

  return ESP_OK;
}

/**
 * @brief Write a formatted DMP configuration set to the sensor memory and
 * registers.
 *
 * @param[in] i2c_num   I2C port number.
 * @param[in] data      Pointer to configuration byte array.
 * @param[in] data_size Total size of the configuration array in bytes.
 * @return esp_err_t ESP_OK on success, or an ESP-IDF error code.
 */
static esp_err_t mpu6050_write_dmp_config_set(i2c_port_t i2c_num,
                                              const uint8_t *data,
                                              uint16_t data_size) {
  uint16_t i = 0;
  while (i < data_size) {
    uint8_t bank = data[i++];
    uint8_t offset = data[i++];
    uint8_t length = data[i++];

    if (length > 0) {
      esp_err_t ret =
          mpu6050_write_memory_block(i2c_num, data + i, length, bank, offset);
      if (ret != ESP_OK) {
        return ret;
      }
      i += length;
    } else {
      uint8_t special = data[i++];
      if (special == 0x01) {
        esp_err_t ret =
            mpu6050_write_register(i2c_num, MPU6050_REG_INT_ENABLE, 0x32);
        if (ret != ESP_OK) {
          return ret;
        }
      }
    }
  }

  return ESP_OK;
}

/* ============================================================================
 *                         PUBLIC API IMPLEMENTATION
 * ==========================================================================*/

/**
 * @brief Write one byte to an MPU6050 register.
 *
 * @param[in] i2c_num I2C port number.
 * @param[in] reg     Register address.
 * @param[in] data    Value byte to write.
 * @return esp_err_t ESP_OK on success, or error from i2c_master_cmd_begin().
 */
esp_err_t mpu6050_write_register(i2c_port_t i2c_num, uint8_t reg,
                                 uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(MPU6050_I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief Read consecutive bytes starting at a register address using repeated
 * start.
 *
 * @param[in]  i2c_num I2C port number.
 * @param[in]  reg     First register to read.
 * @param[out] data    Destination buffer (must hold at least @p len bytes).
 * @param[in]  len     Number of bytes to read.
 * @return esp_err_t ESP_OK on success, or error from i2c_master_cmd_begin().
 */
esp_err_t mpu6050_read_registers(i2c_port_t i2c_num, uint8_t reg, uint8_t *data,
                                 size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd); /* repeated start */
  i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(MPU6050_I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief Configure and install the I2C driver, then wake the MPU6050 sensor.
 *
 * @param[in] i2c_num  I2C port to use.
 * @param[in] sda_gpio GPIO pin assigned to SDA line.
 * @param[in] scl_gpio GPIO pin assigned to SCL line.
 * @return esp_err_t ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t mpu6050_init(i2c_port_t i2c_num, gpio_num_t sda_gpio,
                       gpio_num_t scl_gpio) {
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

/**
 * @brief Read raw accelerometer and gyroscope measurements.
 *
 * @param[in]  i2c_num I2C port number.
 * @param[out] data    Pointer to @ref mpu6050_data_t structure to populate.
 * @return esp_err_t ESP_OK on success, or an I2C communication error code.
 */
esp_err_t mpu6050_read_data(i2c_port_t i2c_num, mpu6050_data_t *data) {
  /* Layout: AX AY AZ (0-5) | TEMP (6-7) | GX GY GZ (8-13), big-endian. */
  uint8_t buffer[14];
  esp_err_t ret = mpu6050_read_registers(i2c_num, MPU6050_REG_ACCEL_XOUT_H,
                                         buffer, sizeof(buffer));
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

/**
 * @brief Reset the MPU-6050 device through the power management register.
 *
 * @param[in] i2c_num I2C port number.
 * @return esp_err_t ESP_OK on success, or an I2C error code.
 */
esp_err_t mpu6050_reset(i2c_port_t i2c_num) {
  uint8_t reg_prev_data = 0;
  esp_err_t ret = mpu6050_read_registers(i2c_num, MPU6050_REG_PWR_MGMT_1,
                                         &reg_prev_data, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read power register");
    return ret;
  }
  reg_prev_data |= (1 << 7);
  ret = mpu6050_write_register(i2c_num, MPU6050_REG_PWR_MGMT_1, reg_prev_data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to reset MPU6050");
    return ret;
  }
  return ESP_OK;
}

/**
 * @brief Enable or disable sensor low-power sleep mode.
 *
 * @param[in] i2c_num I2C port number.
 * @param[in] enabled Set true to enter sleep mode, false to wake up.
 * @return esp_err_t ESP_OK on success, or an I2C error code.
 */
esp_err_t mpu6050_set_sleep_enabled(i2c_port_t i2c_num, bool enabled) {
  uint8_t reg_prev_data = 0;
  esp_err_t ret = mpu6050_read_registers(i2c_num, MPU6050_REG_PWR_MGMT_1,
                                         &reg_prev_data, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read power register");
    return ret;
  }

  if (enabled) {
    reg_prev_data |= (1 << 6);
  } else {
    reg_prev_data &= ~(1 << 6);
  }

  ret = mpu6050_write_register(i2c_num, MPU6050_REG_PWR_MGMT_1, reg_prev_data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set sleep");
    return ret;
  }
  return ESP_OK;
}

/**
 * @brief Load DMP firmware and configuration, and initialize FIFO for
 * quaternion packets.
 *
 * @param[in] i2c_num I2C port number.
 * @return esp_err_t ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t mpu6050_dmp_initialize(i2c_port_t i2c_num) {
  esp_err_t ret = mpu6050_reset(i2c_num);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to reset before DMP init");
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(30));

  ret = mpu6050_set_sleep_enabled(i2c_num, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable sleep before DMP init");
    return ret;
  }

  ret = mpu6050_set_memory_bank(i2c_num, 0x10, true, true);
  if (ret != ESP_OK) {
    return ret;
  }
  ret = mpu6050_set_memory_start_address(i2c_num, 0x06);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_set_memory_bank(i2c_num, 0x00, false, false);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_write_memory_block(i2c_num, dmpMemory, MPU6050_DMP_CODE_SIZE, 0,
                                   0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write DMP memory block");
    return ret;
  }

  ret =
      mpu6050_write_dmp_config_set(i2c_num, dmpConfig, MPU6050_DMP_CONFIG_SIZE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write DMP config");
    return ret;
  }

  ret =
      mpu6050_update_register_bits(i2c_num, MPU6050_REG_PWR_MGMT_1, 0x07, 0x03);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_write_register(i2c_num, MPU6050_REG_INT_ENABLE, 0x12);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_write_register(i2c_num, MPU6050_REG_SMPLRT_DIV, 0x04);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_update_register_bits(i2c_num, MPU6050_REG_CONFIG, 0x38, 0x08);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_update_register_bits(i2c_num, MPU6050_REG_CONFIG, 0x07, 0x03);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_update_register_bits(i2c_num, MPU6050_REG_GYRO_CONFIG, 0x18,
                                     0x18);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_write_register(i2c_num, MPU6050_REG_DMP_CFG_1, 0x03);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_write_register(i2c_num, MPU6050_REG_DMP_CFG_2, 0x00);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_write_register(i2c_num, MPU6050_REG_MOT_THR, 2);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_write_register(i2c_num, MPU6050_REG_ZRMOT_THR, 156);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_write_register(i2c_num, MPU6050_REG_MOT_DUR, 80);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_write_register(i2c_num, MPU6050_REG_ZRMOT_DUR, 0);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_update_register_bits(i2c_num, MPU6050_REG_USER_CTRL, (1 << 2),
                                     (1 << 2));
  if (ret != ESP_OK) {
    return ret;
  }
  ret = mpu6050_update_register_bits(i2c_num, MPU6050_REG_USER_CTRL, (1 << 3),
                                     (1 << 3));
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_update_register_bits(i2c_num, MPU6050_REG_USER_CTRL, (1 << 6),
                                     (1 << 6));
  if (ret != ESP_OK) {
    return ret;
  }
  ret =
      mpu6050_update_register_bits(i2c_num, MPU6050_REG_USER_CTRL, (1 << 7), 0);
  if (ret != ESP_OK) {
    return ret;
  }

  uint8_t int_status = 0;
  ret = mpu6050_read_registers(i2c_num, MPU6050_REG_INT_STATUS, &int_status, 1);
  if (ret != ESP_OK) {
    return ret;
  }

  return ESP_OK;
}

/**
 * @brief Enable or disable the Digital Motion Processor (DMP) feature.
 *
 * @param[in] i2c_num I2C port number.
 * @param[in] enabled Set true to enable DMP, false to disable.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t mpu6050_dmp_enable(i2c_port_t i2c_num, bool enabled) {
  uint8_t value = enabled ? (1 << 7) : 0;
  return mpu6050_update_register_bits(i2c_num, MPU6050_REG_USER_CTRL, (1 << 7),
                                      value);
}

/**
 * @brief Read a single DMP packet from the hardware FIFO buffer.
 *
 * @param[in]  i2c_num     I2C port number.
 * @param[out] fifo_packet Output buffer sized to at least @ref
 * MPU6050_DMP_PACKET_SIZE bytes.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE on FIFO overflow,
 *                   ESP_ERR_NOT_FOUND if packet not ready, or an I2C error.
 */
static esp_err_t mpu6050_dmp_read_fifo_packet(i2c_port_t i2c_num,
                                              uint8_t *fifo_packet) {
  uint8_t int_status = 0;
  uint8_t fifo_count_buffer[2] = {0};

  esp_err_t ret =
      mpu6050_read_registers(i2c_num, MPU6050_REG_INT_STATUS, &int_status, 1);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = mpu6050_read_registers(i2c_num, MPU6050_REG_FIFO_COUNTH,
                               fifo_count_buffer, 2);
  if (ret != ESP_OK) {
    return ret;
  }

  uint16_t fifo_count =
      ((uint16_t)fifo_count_buffer[0] << 8) | fifo_count_buffer[1];
  if ((int_status & 0x10) || fifo_count == 1024) {
    mpu6050_update_register_bits(i2c_num, MPU6050_REG_USER_CTRL, (1 << 2),
                                 (1 << 2));
    return ESP_ERR_INVALID_STATE;
  }

  if ((int_status & 0x02) == 0) {
    return ESP_ERR_NOT_FOUND;
  }

  while (fifo_count < MPU6050_DMP_PACKET_SIZE) {
    ret = mpu6050_read_registers(i2c_num, MPU6050_REG_FIFO_COUNTH,
                                 fifo_count_buffer, 2);
    if (ret != ESP_OK) {
      return ret;
    }
    fifo_count = ((uint16_t)fifo_count_buffer[0] << 8) | fifo_count_buffer[1];
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  ret = mpu6050_read_registers(i2c_num, MPU6050_REG_FIFO_R_W, fifo_packet,
                               MPU6050_DMP_PACKET_SIZE);
  if (ret != ESP_OK) {
    return ret;
  }

  return ESP_OK;
}

/**
 * @brief Read a quaternion packet from FIFO and decode normalized orientation
 * components.
 *
 * @param[in]  i2c_num I2C port number.
 * @param[out] q       Pointer to @ref mpu6050_quaternion_t structure to
 * populate.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if q is NULL, or an
 * error code.
 */
esp_err_t mpu6050_dmp_read_quaternion(i2c_port_t i2c_num,
                                      mpu6050_quaternion_t *q) {
  if (q == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t packet[MPU6050_DMP_PACKET_SIZE] = {0};
  esp_err_t ret = mpu6050_dmp_read_fifo_packet(i2c_num, packet);
  if (ret != ESP_OK) {
    return ret;
  }

  int16_t q_i[4];
  q_i[0] = (int16_t)((packet[0] << 8) | packet[1]);
  q_i[1] = (int16_t)((packet[4] << 8) | packet[5]);
  q_i[2] = (int16_t)((packet[8] << 8) | packet[9]);
  q_i[3] = (int16_t)((packet[12] << 8) | packet[13]);

  q->w = (float)q_i[0] / 16384.0f;
  q->x = (float)q_i[1] / 16384.0f;
  q->y = (float)q_i[2] / 16384.0f;
  q->z = (float)q_i[3] / 16384.0f;

  return ESP_OK;
}

/**
 * @brief Calibrate gyroscope sensor offsets using closed-loop tuning.
 *
 * @param[in] i2c_num I2C port number.
 * @param[in] loops   Number of tuning iterations.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if loops is 0, or an
 * error code.
 */
esp_err_t mpu6050_calibrate_gyro(i2c_port_t i2c_num, uint8_t loops) {
  if (loops == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  float kP = 0.3f;
  float kI = 90.0f;
  const float scale =
      (100.0f - mpu6050_mapf((float)loops, 1.0f, 5.0f, 20.0f, 0.0f)) * 0.01f;
  kP *= scale;
  kI *= scale;

  ESP_LOGI(TAG, "Calibrating gyro with %u loops", loops);
  return mpu6050_calibrate_pid(i2c_num, MPU6050_REG_GYRO_XOUT_H, kP, kI, loops);
}

/**
 * @brief Calibrate accelerometer sensor offsets using closed-loop tuning with
 * Z-axis gravity reference.
 *
 * @param[in] i2c_num I2C port number.
 * @param[in] loops   Number of tuning iterations.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if loops is 0, or an
 * error code.
 */
esp_err_t mpu6050_calibrate_accel(i2c_port_t i2c_num, uint8_t loops) {
  if (loops == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  float kP = 0.3f;
  float kI = 20.0f;
  const float scale =
      (100.0f - mpu6050_mapf((float)loops, 1.0f, 5.0f, 20.0f, 0.0f)) * 0.01f;
  kP *= scale;
  kI *= scale;

  ESP_LOGI(TAG, "Calibrating accel with %u loops", loops);
  return mpu6050_calibrate_pid(i2c_num, MPU6050_REG_ACCEL_XOUT_H, kP, kI,
                               loops);
}
