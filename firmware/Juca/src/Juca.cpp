/**
 * @file    Juca.cpp
 * @brief   Implementation of the JucaRobot class: board pins and defaults on top of the components.
 *
 * @date    Nov 2026
 * @author  Matheus Leitezke Pinto
 */

#include "Juca.h"

JucaRobot Juca;

/** @brief Shortest time between two odometry readings, in microseconds. */
static const uint32_t MIN_TWIST_INTERVAL_US = 10000;

/** @brief Stack of the two background tasks of the drive layer, in bytes. */
static const uint32_t DRIVE_TASK_STACK_BYTES = 4096;

bool JucaRobot::check(esp_err_t status)
{
    _lastError = status;
    return status == ESP_OK;
}

/* ============================================================================
 *                                  WHEELS
 * ==========================================================================*/

bool JucaRobot::beginWheels()
{
    if (_leftWheel != nullptr && _rightWheel != nullptr)
    {
        return true;
    }

    /* Read each current sensor once here, so the Arduino core sets up the ADC and these two pins now,
     * from the sketch, and not later from the background task that watches the motor currents. */
    (void)analogRead(JUCA_PIN_CURRENT_LEFT);
    (void)analogRead(JUCA_PIN_CURRENT_RIGHT);

    wheel_config_t left = {};
    left.pwm_a_gpio = JUCA_PIN_MOTOR_LEFT_PWM_A;
    left.pwm_b_gpio = JUCA_PIN_MOTOR_LEFT_PWM_B;
    left.mcpwm_group_id = JUCA_MCPWM_GROUP_MOTORS;
    left.max_power_ticks = JUCA_MOTOR_MAX_POWER_TICKS;
    left.encoder_a_gpio = JUCA_PIN_ENCODER_LEFT_A;
    left.encoder_b_gpio = JUCA_PIN_ENCODER_LEFT_B;
    left.current_sense_gpio = JUCA_PIN_CURRENT_LEFT;

    wheel_config_t right = {};
    right.pwm_a_gpio = JUCA_PIN_MOTOR_RIGHT_PWM_A;
    right.pwm_b_gpio = JUCA_PIN_MOTOR_RIGHT_PWM_B;
    right.mcpwm_group_id = JUCA_MCPWM_GROUP_MOTORS;
    right.max_power_ticks = JUCA_MOTOR_MAX_POWER_TICKS;
    right.encoder_a_gpio = JUCA_PIN_ENCODER_RIGHT_A;
    right.encoder_b_gpio = JUCA_PIN_ENCODER_RIGHT_B;
    right.current_sense_gpio = JUCA_PIN_CURRENT_RIGHT;

    if (!check(wheel_create(&left, &_leftWheel)))
    {
        _leftWheel = nullptr;
        return false;
    }
    if (!check(wheel_create(&right, &_rightWheel)))
    {
        /* Do not keep half a robot: release the wheel that was created. */
        wheel_delete(_leftWheel);
        _leftWheel = nullptr;
        _rightWheel = nullptr;
        return false;
    }
    return true;
}

/* ============================================================================
 *                             DIFFERENTIAL DRIVE
 * ==========================================================================*/

diff_drive_config_t JucaRobot::defaultDriveConfig() const
{
    diff_drive_config_t config = {};

    /* Geometry and encoders of the tested platform (juca_platform.h). */
    config.half_track_width_m = JUCA_HALF_TRACK_WIDTH_M;
    config.wheel_radius_m = JUCA_WHEEL_RADIUS_M;
    config.left_encoder_cpr = JUCA_ENCODER_CPR_LEFT;
    config.right_encoder_cpr = JUCA_ENCODER_CPR_RIGHT;

    /* No speed limit of our own (0 = off). The wheels still limit what can be reached. */
    config.max_linear_velocity_m_s = 0.0f;
    config.max_angular_velocity_rad_s = 0.0f;
    /* First estimate of the wheel speed at 100% duty; calibrate() replaces it with the measured value. */
    config.max_wheel_rad_s = 20.0f;

    /* Overcurrent protection: raw ADC threshold (about 0.8 A at full duty), checked every 50 ms. */
    config.max_adc_raw_threshold = 3000;
    config.current_ma_per_count = JUCA_CURRENT_MA_PER_COUNT;
    config.monitor_period_ms = 50;
    config.safety_task_priority = 5;
    config.safety_task_stack_size = DRIVE_TASK_STACK_BYTES;

    /* Acceleration ramps, and the control task (every 20 ms, below the safety task). */
    config.max_accel_linear_m_s2 = 0.5f;
    config.max_accel_angular_rad_s2 = 1.5f;
    config.control_period_ms = 20;
    config.control_task_priority = 4;
    config.control_task_stack_size = DRIVE_TASK_STACK_BYTES;

    return config;
}

bool JucaRobot::beginDrive(const diff_drive_config_t *config)
{
    if (!beginWheels())
    {
        return false;
    }

    diff_drive_config_t driveConfig = (config != nullptr) ? *config : defaultDriveConfig();
    driveConfig.left_wheel = _leftWheel;
    driveConfig.right_wheel = _rightWheel;

    _lastTwistMicros = micros();
    return check(diff_drive_init(&driveConfig));
}

bool JucaRobot::calibrate(uint32_t durationMs, float *maxWheelRadS)
{
    bool ok = check(diff_drive_calibrate_max_speed(durationMs, maxWheelRadS));
    /* The calibration re-seeds the odometry: measure the next interval from now. */
    _lastTwistMicros = micros();
    return ok;
}

bool JucaRobot::setTwist(float linear_m_s, float angular_rad_s)
{
    const diff_drive_twist_t twist = { linear_m_s, angular_rad_s };
    return check(diff_drive_set_twist(&twist));
}

bool JucaRobot::getTwist(float &linear_m_s, float &angular_rad_s)
{
    const uint32_t nowMicros = micros();
    const uint32_t elapsedMicros = nowMicros - _lastTwistMicros;   /* stays correct when micros() wraps */
    if (elapsedMicros < MIN_TWIST_INTERVAL_US)
    {
        _lastError = ESP_ERR_INVALID_STATE;
        return false;
    }

    diff_drive_twist_t twist = {};
    if (!check(diff_drive_get_twist((float)elapsedMicros * 1e-6f, &twist)))
    {
        return false;
    }
    _lastTwistMicros = nowMicros;
    linear_m_s = twist.linear_x;
    angular_rad_s = twist.angular_z;
    return true;
}

bool JucaRobot::brake()
{
    return check(diff_drive_brake());
}

bool JucaRobot::faultActive()
{
    return diff_drive_is_fault_active();
}

bool JucaRobot::clearFault()
{
    return check(diff_drive_clear_fault());
}

bool JucaRobot::getCurrentMilliamps(float &left_mA, float &right_mA)
{
    return check(diff_drive_get_current_ma(&left_mA, &right_mA));
}

/* ============================================================================
 *                             ULTRASONIC SENSORS
 * ==========================================================================*/

bool JucaRobot::beginSonar()
{
    threeeyes_config_t config = {};
    config.trig_gpio = JUCA_PIN_HCSR04_TRIG;
    config.echo_left_gpio = JUCA_PIN_HCSR04_ECHO_LEFT;
    config.echo_middle_gpio = JUCA_PIN_HCSR04_ECHO_MIDDLE;
    config.echo_right_gpio = JUCA_PIN_HCSR04_ECHO_RIGHT;
    config.mcpwm_group_id = JUCA_MCPWM_GROUP_HCSR04;
    return check(threeeyes_init(&config));
}

float JucaRobot::readSonarCm(JucaSonarSide *side, uint32_t timeoutMs)
{
    if (side != nullptr)
    {
        *side = JUCA_SONAR_NONE;
    }
    if (!check(threeeyes_trigger_and_wait(pdMS_TO_TICKS(timeoutMs))))
    {
        return JUCA_SONAR_NOTHING_CM;
    }

    threeeyes_value_t values[3];
    threeeyes_read_values(&values[0], &values[1], &values[2]);

    /* Among the sensors that answered, keep the shortest echo. */
    int nearest = -1;
    for (int i = 0; i < 3; i++)
    {
        if (values[i].is_updated && (nearest < 0 || values[i].tof_ticks < values[nearest].tof_ticks))
        {
            nearest = i;
        }
    }
    if (nearest < 0)
    {
        return JUCA_SONAR_NOTHING_CM;
    }

    /* A sensor with nothing in front of it answers with a long echo (about 650 cm): not an object. */
    const float distanceCm = threeeyes_convert_ticks_to_cm(values[nearest].tof_ticks);
    if (distanceCm > JUCA_SONAR_MAX_CM)
    {
        return JUCA_SONAR_NOTHING_CM;
    }
    if (side != nullptr)
    {
        *side = (JucaSonarSide)nearest;
    }
    return distanceCm;
}

/* ============================================================================
 *                               LINE SENSORS
 * ==========================================================================*/

bool JucaRobot::beginLine(uint8_t lineLevel)
{
    ir_line_config_t config = {};
    config.sensor_gpios[0] = JUCA_PIN_IR_LINE_FAR_LEFT;
    config.sensor_gpios[1] = JUCA_PIN_IR_LINE_LEFT;
    config.sensor_gpios[2] = JUCA_PIN_IR_LINE_MIDDLE;
    config.sensor_gpios[3] = JUCA_PIN_IR_LINE_RIGHT;
    config.sensor_gpios[4] = JUCA_PIN_IR_LINE_FAR_RIGHT;
    config.line_level = lineLevel;
    return check(ir_line_init(&config));
}

uint8_t JucaRobot::readLineMask()
{
    uint8_t mask = 0;
    check(ir_line_read_mask(&mask));
    return mask;
}

bool JucaRobot::readLinePosition(float &position)
{
    const uint8_t mask = readLineMask();
    int sum = 0;
    int count = 0;
    for (int i = 0; i < IR_LINE_SENSOR_COUNT; i++)
    {
        if (mask & (1U << i))
        {
            sum += i - 2;   /* bit 0 is the far left sensor: position -2 */
            count++;
        }
    }
    if (count == 0)
    {
        return false;
    }
    position = (float)sum / (float)count;
    return true;
}

/* ============================================================================
 *                                    IMU
 * ==========================================================================*/

bool JucaRobot::beginImu(uint8_t calibrationLoops)
{
    return check(mpu6050_complete_init(JUCA_I2C_PORT_IMU, JUCA_PIN_I2C_SDA, JUCA_PIN_I2C_SCL, calibrationLoops));
}

bool JucaRobot::readQuaternion(mpu6050_quaternion_t &quaternion)
{
    return check(mpu6050_dmp_read_quaternion(JUCA_I2C_PORT_IMU, &quaternion));
}
