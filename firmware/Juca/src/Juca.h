/**
 * @file    Juca.h
 * @brief   Arduino library for the Juca differential-drive robot (ESP32-S3).
 *
 * @details One include gives a sketch two levels of access to the robot:
 *
 *          - The `Juca` object (class @ref JucaRobot): a few Arduino-style methods that set up each
 *            subsystem with the pins of the board and cover the common uses. Start here.
 *          - The component functions of the Juca ESP-IDF firmware (`wheel_*`, `diff_drive_*`,
 *            `threeeyes_*`, `ir_line_*`, `mpu6050_*`), for everything the object does not expose.
 *            They are the same sources as in the firmware, so its documentation applies.
 *
 *          Units are SI and are part of the names: meters, m/s, rad/s, cm, mA, ms.
 *          `linear` is positive forward and `angular` is positive counter-clockwise (to the left),
 *          as in a ROS Twist.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#pragma once

#include <Arduino.h>

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || (ESP_ARDUINO_VERSION_MAJOR < 3)
#error "The Juca library needs the Arduino-ESP32 core version 3.0 or newer (Boards Manager > esp32 by Espressif Systems)."
#endif

#include "juca_board.h"
#include "juca_platform.h"

#include "components/wheel.h"
#include "components/diff_drive.h"
#include "components/threeeyes.h"
#include "components/ir_line.h"
#include "components/mpu6050.h"
#include "components/mpu6050init.h"

/** @brief Distances above this, in cm, are reported as "nothing in range" (the range of the HC-SR04). */
#define JUCA_SONAR_MAX_CM       400.0f

/** @brief Value returned by @ref JucaRobot::readSonarCm when nothing is in range. */
#define JUCA_SONAR_NOTHING_CM   (-1.0f)

/** @brief Which ultrasonic sensor saw the nearest object. */
enum JucaSonarSide : int8_t {
    JUCA_SONAR_NONE = -1,    /**< No object in range. */
    JUCA_SONAR_LEFT = 0,
    JUCA_SONAR_MIDDLE = 1,
    JUCA_SONAR_RIGHT = 2,
};

/**
 * @brief The Juca robot: motors, odometry and sensors, with the pins of the board already filled in.
 *
 * @details Use the global object `Juca`. Every `begin...()` method sets up one subsystem and returns
 *          true on success; call only the ones the sketch needs. After a method returns false,
 *          @ref lastError tells why.
 */
class JucaRobot {
public:
    /* ---------------------------------------------------------------------
     *  Wheels (motor + encoder + current sensor), without the drive layer
     * ------------------------------------------------------------------- */

    /**
     * @brief Creates the two wheels. The motors start braked.
     *
     * @details Only needed alone for low-level tests with `wheel_set_power()`; @ref beginDrive calls it.
     *          Do not command the wheels directly after @ref beginDrive: the drive layer owns them.
     */
    bool beginWheels();

    /** @brief Handle of the left wheel for the `wheel_*` functions (NULL before @ref beginWheels). */
    wheel_handle_t leftWheel() const { return _leftWheel; }
    /** @brief Handle of the right wheel for the `wheel_*` functions (NULL before @ref beginWheels). */
    wheel_handle_t rightWheel() const { return _rightWheel; }

    /* ---------------------------------------------------------------------
     *  Differential drive
     * ------------------------------------------------------------------- */

    /**
     * @brief Returns the drive configuration used by @ref beginDrive when none is given.
     *
     * @details Geometry, encoders and current factor of the tested platform; no speed limit; ramps of
     *          0.5 m/s^2 and 1.5 rad/s^2; overcurrent threshold of 3000 ADC counts. Change the fields
     *          you need and pass the result to @ref beginDrive. The wheel handles are filled in there.
     */
    diff_drive_config_t defaultDriveConfig() const;

    /**
     * @brief Creates the wheels (if needed) and starts the differential drive. Nothing moves yet.
     * @param[in] config Optional configuration from @ref defaultDriveConfig, modified. NULL = defaults.
     */
    bool beginDrive(const diff_drive_config_t *config = nullptr);

    /**
     * @brief Measures the real top speed of the wheels and the left/right difference.
     *
     * @warning THE ROBOT MOVES at full power for the whole duration, and the overcurrent protection is
     *          paused. Give it 2 m of free floor, or lift the wheels.
     *
     * @param[in]  durationMs       Length of the test, 500 to 10000 ms.
     * @param[out] maxWheelRadS     Optional: receives the measured wheel speed at 100% duty, in rad/s.
     */
    bool calibrate(uint32_t durationMs = 4000, float *maxWheelRadS = nullptr);

    /**
     * @brief Sets the velocity the robot should have. Returns at once; the robot keeps it until the next call.
     * @param[in] linear_m_s    Forward speed in m/s (negative = backward).
     * @param[in] angular_rad_s Turn rate in rad/s (positive = to the left).
     * @return false if the command was refused (not started, or an overcurrent fault is active).
     */
    bool setTwist(float linear_m_s, float angular_rad_s);

    /**
     * @brief Reads the velocity the robot really had since the previous call (wheel odometry).
     *
     * @details Call it at a steady rate from ONE place, for example every 50 ms. The time between two
     *          calls is measured inside. A call less than 10 ms after the previous one returns false
     *          and changes nothing (too few encoder counts for a meaningful speed).
     *
     * @param[out] linear_m_s    Measured forward speed, m/s.
     * @param[out] angular_rad_s Measured turn rate, rad/s.
     */
    bool getTwist(float &linear_m_s, float &angular_rad_s);

    /** @brief Brakes both wheels now. The robot stays stopped until the next @ref setTwist. */
    bool brake();

    /** @brief true while the overcurrent protection keeps the robot stopped. */
    bool faultActive();

    /** @brief Releases the overcurrent protection. The robot stays stopped until the next @ref setTwist. */
    bool clearFault();

    /**
     * @brief Reads the motor currents, in mA.
     * @note  The factor was measured at 100% duty; at lower duty the value is below the real motor current.
     */
    bool getCurrentMilliamps(float &left_mA, float &right_mA);

    /* ---------------------------------------------------------------------
     *  Ultrasonic sensors (3 x HC-SR04)
     * ------------------------------------------------------------------- */

    /** @brief Sets up the three ultrasonic sensors. */
    bool beginSonar();

    /**
     * @brief Fires the three sensors and returns the distance of the NEAREST object.
     *
     * @details Blocks until the first echo: a few ms with an object near, about 40 ms with nothing in
     *          range, `timeoutMs` if no sensor answers. Call it from one place only (normally `loop()`).
     *
     * @param[out] side      Optional: which sensor saw the object.
     * @param[in]  timeoutMs Longest wait for an echo. Keep it at 60 or more.
     * @return Distance in cm, or @ref JUCA_SONAR_NOTHING_CM (negative) if nothing is in range.
     */
    float readSonarCm(JucaSonarSide *side = nullptr, uint32_t timeoutMs = 60);

    /* ---------------------------------------------------------------------
     *  Infrared line sensors (5)
     * ------------------------------------------------------------------- */

    /**
     * @brief Sets up the five line sensors.
     * @param[in] lineLevel Level (0 or 1) a sensor outputs over the line. Default: the board's.
     */
    bool beginLine(uint8_t lineLevel = JUCA_IR_LINE_LEVEL_ON_LINE);

    /** @brief One bit per sensor, bit 0 = far left, 1 = over the line. Compare with `IR_LINE_PATTERN_*`. */
    uint8_t readLineMask();

    /**
     * @brief Position of the line under the sensors: the average of the sensors that see it.
     * @param[out] position -2 (far left) ... 0 (centered) ... +2 (far right), in half steps.
     * @return false if no sensor sees the line (`position` is not changed).
     */
    bool readLinePosition(float &position);

    /* ---------------------------------------------------------------------
     *  IMU (MPU6050)
     * ------------------------------------------------------------------- */

    /**
     * @brief Starts the IMU and calibrates it. KEEP THE ROBOT STILL AND LEVEL: it takes a few seconds.
     * @param[in] calibrationLoops Calibration passes for each sensor (more = better and slower).
     */
    bool beginImu(uint8_t calibrationLoops = 15);

    /**
     * @brief Reads a new orientation quaternion, if the sensor has one (about 10 per second).
     * @return false if there is nothing new yet (normal: just call again later) or on an error.
     *         @ref lastError is ESP_ERR_NOT_FOUND in the first case.
     */
    bool readQuaternion(mpu6050_quaternion_t &quaternion);

    /* ---------------------------------------------------------------------
     *  Diagnostics
     * ------------------------------------------------------------------- */

    /** @brief Error of the last method that returned false. Print it with `esp_err_to_name()`. */
    esp_err_t lastError() const { return _lastError; }

private:
    bool check(esp_err_t status);

    wheel_handle_t _leftWheel = nullptr;
    wheel_handle_t _rightWheel = nullptr;
    uint32_t _lastTwistMicros = 0;
    esp_err_t _lastError = ESP_OK;
};

/** @brief The robot. */
extern JucaRobot Juca;
