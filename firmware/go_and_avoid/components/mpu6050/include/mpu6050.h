/**
 * @file mpu6050.h
 * @brief MPU6050 inertial sensor driver (ESP-IDF, legacy I2C driver).
 *
 * Provides low-level I2C access, accelerometer/gyroscope reads and the
 * DMP (Digital Motion Processor) API for obtaining quaternions.
 *
 * @note This driver knows nothing about board pins or the board-specific
 *       bring-up sequence. That lives in @ref mpu6050init.h.
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup mpu6050_defs Definitions
 *  @{ */

/** 7-bit I2C address (AD0 pin tied to GND). Use 0x69 if AD0 is tied to VCC. */
#define MPU6050_ADDR 0x68

/** Size in bytes of one DMP packet read from the FIFO. */
#define MPU6050_DMP_PACKET_SIZE 42

/** @} */

/** @defgroup mpu6050_regs Registers
 *  @{ */

/** @name General configuration */
/** @{ */
#define MPU6050_REG_SMPLRT_DIV   0x19 /**< Sample rate divider. */
#define MPU6050_REG_CONFIG       0x1A /**< Digital low-pass filter (DLPF). */
#define MPU6050_REG_GYRO_CONFIG  0x1B /**< Gyroscope full-scale range. */
/** @} */

/** @name Motion detection */
/** @{ */
#define MPU6050_REG_MOT_THR      0x1F /**< Motion detection threshold. */
#define MPU6050_REG_MOT_DUR      0x20 /**< Motion detection duration. */
#define MPU6050_REG_ZRMOT_THR    0x21 /**< Zero-motion detection threshold. */
#define MPU6050_REG_ZRMOT_DUR    0x22 /**< Zero-motion detection duration. */
/** @} */

/** @name Calibration offsets */
/** @{ */
#define MPU6050_REG_XA_OFFS_H    0x06 /**< Accelerometer X offset (MSB). */
#define MPU6050_REG_YA_OFFS_H    0x08 /**< Accelerometer Y offset (MSB). */
#define MPU6050_REG_ZA_OFFS_H    0x0A /**< Accelerometer Z offset (MSB). */
#define MPU6050_REG_XG_OFFS_USRH 0x13 /**< Gyroscope X offset (MSB). */
#define MPU6050_REG_YG_OFFS_USRH 0x15 /**< Gyroscope Y offset (MSB). */
#define MPU6050_REG_ZG_OFFS_USRH 0x17 /**< Gyroscope Z offset (MSB). */
/** @} */

/** @name FIFO and interrupts */
/** @{ */
#define MPU6050_REG_FIFO_EN      0x23 /**< Selects which sensors feed the FIFO. */
#define MPU6050_REG_INT_ENABLE   0x38 /**< Interrupt enable. */
#define MPU6050_REG_INT_STATUS   0x3A /**< Interrupt status. */
#define MPU6050_REG_FIFO_COUNTH  0x72 /**< FIFO byte count (MSB). */
#define MPU6050_REG_FIFO_R_W     0x74 /**< FIFO read/write. */
/** @} */

/** @name Measurement data */
/** @{ */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B /**< Start of the 14-byte measurement block. */
#define MPU6050_REG_GYRO_XOUT_H  0x43 /**< Start of the gyroscope data. */
/** @} */

/** @name Control and DMP */
/** @{ */
#define MPU6050_REG_USER_CTRL    0x6A /**< User control (FIFO, DMP, resets). */
#define MPU6050_REG_PWR_MGMT_1   0x6B /**< Power management / reset / sleep. */
#define MPU6050_REG_BANK_SEL     0x6D /**< DMP memory bank select. */
#define MPU6050_REG_MEM_START    0x6E /**< DMP memory start address. */
#define MPU6050_REG_MEM_R_W      0x6F /**< DMP memory read/write. */
#define MPU6050_REG_DMP_CFG_1    0x70 /**< DMP configuration 1. */
#define MPU6050_REG_DMP_CFG_2    0x71 /**< DMP configuration 2. */
/** @} */

/** @} */ /* mpu6050_regs */

/** @defgroup mpu6050_types Types
 *  @{ */

/** @brief Raw (unit-less) sensor measurements. */
typedef struct {
    int16_t accel_x; /**< X-axis acceleration (LSB). */
    int16_t accel_y; /**< Y-axis acceleration (LSB). */
    int16_t accel_z; /**< Z-axis acceleration (LSB). */
    int16_t gyro_x;  /**< X-axis angular rate (LSB). */
    int16_t gyro_y;  /**< Y-axis angular rate (LSB). */
    int16_t gyro_z;  /**< Z-axis angular rate (LSB). */
} mpu6050_data_t;

/** @brief Orientation quaternion (normalized) provided by the DMP. */
typedef struct {
    float w; /**< Scalar component. */
    float x; /**< Imaginary component i. */
    float y; /**< Imaginary component j. */
    float z; /**< Imaginary component k. */
} mpu6050_quaternion_t;

/** @} */

/** @defgroup mpu6050_lowlevel Low-level I2C access
 *  @{ */

/**
 * @brief Write one byte to an MPU6050 register.
 *
 * @param[in] i2c_num I2C port (driver already installed).
 * @param[in] reg     Register address.
 * @param[in] data    Value to write.
 * @return ESP_OK on success, or the error returned by i2c_master_cmd_begin().
 */
esp_err_t mpu6050_write_register(i2c_port_t i2c_num, uint8_t reg, uint8_t data);

/**
 * @brief Read consecutive bytes starting at a register.
 *
 * Uses a repeated start between the address write and the read.
 *
 * @param[in]  i2c_num I2C port (driver already installed).
 * @param[in]  reg     First register to read.
 * @param[out] data    Destination buffer (at least @p len bytes).
 * @param[in]  len     Number of bytes to read.
 * @return ESP_OK on success, or the error returned by i2c_master_cmd_begin().
 */
esp_err_t mpu6050_read_registers(i2c_port_t i2c_num, uint8_t reg, uint8_t *data, size_t len);

/** @} */

/** @defgroup mpu6050_core Initialization and reading
 *  @{ */

/**
 * @brief Configure and install the I2C driver, then wake the MPU6050.
 *
 * Runs at 100 kHz with internal pull-ups enabled. If a failure occurs after
 * the driver has been installed, the driver is uninstalled before returning.
 *
 * @param[in] i2c_num  I2C port to use.
 * @param[in] sda_gpio SDA GPIO.
 * @param[in] scl_gpio SCL GPIO.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t mpu6050_init(i2c_port_t i2c_num, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Read accelerometer and gyroscope data (raw values).
 *
 * Reads the 14-byte block starting at @ref MPU6050_REG_ACCEL_XOUT_H. The
 * temperature bytes are discarded.
 *
 * @param[in]  i2c_num I2C port.
 * @param[out] data    Structure filled with the measurements.
 * @return ESP_OK on success, or an I2C error.
 */
esp_err_t mpu6050_read_data(i2c_port_t i2c_num, mpu6050_data_t *data);

/**
 * @brief Reset the device through PWR_MGMT_1.
 *
 * @param[in] i2c_num I2C port.
 * @return ESP_OK on success, or an I2C error.
 */
esp_err_t mpu6050_reset(i2c_port_t i2c_num);

/**
 * @brief Enable or disable sleep mode.
 *
 * @param[in] i2c_num I2C port.
 * @param[in] enabled true to enter sleep, false to wake up.
 * @return ESP_OK on success, or an I2C error.
 */
esp_err_t mpu6050_set_sleep_enabled(i2c_port_t i2c_num, bool enabled);

/** @} */

/** @defgroup mpu6050_dmp DMP (Digital Motion Processor)
 *  @{ */

/**
 * @brief Load the DMP firmware and configuration into the sensor.
 *
 * Must be called after @ref mpu6050_init and before @ref mpu6050_dmp_enable.
 *
 * @param[in] i2c_num I2C port.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t mpu6050_dmp_initialize(i2c_port_t i2c_num);

/**
 * @brief Turn the DMP (and its FIFO) on or off.
 *
 * @param[in] i2c_num I2C port.
 * @param[in] enabled true to enable, false to disable.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t mpu6050_dmp_enable(i2c_port_t i2c_num, bool enabled);

/**
 * @brief Read one packet from the FIFO and extract the quaternion.
 *
 * @param[in]  i2c_num I2C port.
 * @param[out] q       Output quaternion.
 * @return ESP_OK on success; an error if a full packet
 *         (@ref MPU6050_DMP_PACKET_SIZE bytes) is not available or on I2C failure.
 */
esp_err_t mpu6050_dmp_read_quaternion(i2c_port_t i2c_num, mpu6050_quaternion_t *q);

/** @} */

/** @defgroup mpu6050_calib Calibration
 *  @{ */

/**
 * @brief Calibrate the gyroscope offsets.
 *
 * The sensor must stay still during calibration.
 *
 * @param[in] i2c_num I2C port.
 * @param[in] loops   Number of tuning iterations (more = more accurate but slower).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t mpu6050_calibrate_gyro(i2c_port_t i2c_num, uint8_t loops);

/**
 * @brief Calibrate the accelerometer offsets.
 *
 * The sensor must be still, with the Z axis aligned with gravity.
 *
 * @param[in] i2c_num I2C port.
 * @param[in] loops   Number of tuning iterations.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t mpu6050_calibrate_accel(i2c_port_t i2c_num, uint8_t loops);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
