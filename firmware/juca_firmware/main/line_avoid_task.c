/**
 * @file    line_avoid_task.c
 * @brief   Follows a line with the 5 infrared sensors and goes around the obstacles seen by the 3 HC-SR04.
 *
 * @details The robot is driven through the `diff_drive` component: this task only decides a velocity
 *          (a "twist": `linear_x` in m/s and `angular_z` in rad/s) and sends it with
 *          `diff_drive_set_twist()`. It runs a small state machine at 20 Hz.
 *
 *          FOLLOW_LINE
 *            A line is normally seen by one or two neighbouring sensors. The position of the line is the
 *            average of the sensors that see it, from -2 (far left) to +2 (far right), with half steps
 *            when two sensors see it (9 positions). A proportional controller turns the robot towards
 *            the line. If no sensor sees the line, the robot spins towards the side where it saw it last.
 *
 *          AVOID (all the other states)
 *            When an ultrasonic sensor sees an object closer than `OBSTACLE_CM`, the robot leaves the
 *            line, goes around the object on a square path and comes back to the line. The turns and the
 *            distances are measured with the wheel odometry (the measured velocity from
 *            `diff_drive_get_twist()` integrated over time), not by timing the motors.
 *
 *            Detour to the right (seen from above, the robot drives to the right):
 *
 *              line  =====>====[ obstacle ]=============>=====
 *                          |                       ^
 *            1 TURN_OUT    |                       |  5 TURN_IN
 *            2 SIDE_STEP   |                       |  6 FIND_LINE
 *                          v                       |  7 ALIGN
 *                          +-----------------------+
 *                          3 TURN_PARALLEL, 4 PASS
 *
 *          The robot is not calibrated (`diff_drive_calibrate_max_speed()` is not called): the line
 *          sensors close the loop, so a small left/right difference between the motors is corrected on
 *          the line. Put the robot on the line before it starts: it waits `START_DELAY_MS` first.
 *
 *          All the numbers that can be tuned are the `#define`s below.
 *
 *          Limits of this simple version:
 *          - The drive is open loop and the motors do not move below about 24% of duty, so the speeds
 *            cannot be set very low, and the real speed is not the commanded one. Tune `FOLLOW_SPEED_M_S`
 *            and the turn speeds on the robot.
 *          - A lost line is searched by spinning in place. That only finds a line that the sensor bar can
 *            reach, a few centimeters from the wheel axle.
 *          - The detour uses fixed distances (`AVOID_*`) that assume an obstacle up to about 20 cm wide and
 *            15 cm deep, one at a time. The ultrasonic sensors point forward, so the obstacle is not seen while
 *            the robot is beside it: the pass is measured with the odometry, not sensed.
 *
 * @date    Nov 2026
 * @author  Matheus
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_private/esp_clk.h"

#include "diff_drive.h"
#include "threeeyes.h"
#include "ir_line.h"
#include "juca_board.h"
#include "juca_platform.h"
#include "line_avoid_task.h"

static const char *TAG = "line_avoid";

/* ============================================================================
 *                          TUNING (change these first)
 * ==========================================================================*/

/* ---- Following the line ---- */

/**
 * Forward speed on a straight line, in m/s. Do not go much lower: the drive is open loop and the motors
 * only start to turn above about 24% of duty, so a wheel commanded below about 0.15 m/s does not move.
 */
#define FOLLOW_SPEED_M_S        0.25f
/** Steering gain: rad/s of turn for each unit of line position (one unit = the distance between two sensors). */
#define LINE_KP_RAD_S           1.2f
/** Fraction of the speed removed when the line is at the edge of the sensors (0 = never slow down). */
#define LINE_SLOWDOWN           0.5f
/** Spin speed used to look for the line when no sensor sees it, in rad/s. */
#define SEARCH_TURN_RAD_S       1.5f

/* ---- Seeing obstacles ---- */

/**
 * An object closer than this (cm) starts the detour. The robot turns in place at the start of the detour, so
 * this distance plus the 0.10 m from the axle to the sensors must be larger than the radius of the robot
 * (about 0.2 m to the front corners on the tested platform), with a margin.
 */
#define OBSTACLE_CM             20.0f
/** The way is free again when the nearest object is farther than this (cm). */
#define CLEAR_CM                35.0f
/** While moving forward during the detour, the robot waits if something is closer than this (cm). */
#define STOP_CM                 8.0f
/** Consecutive readings needed to start the detour (filters a wrong echo). */
#define OBSTACLE_CONFIRM_READS  2
/** Consecutive free readings needed to say the obstacle was passed. */
#define CLEAR_CONFIRM_READS     3

/* ---- Going around ---- */

/** Side of the detour when the object is straight ahead: -1 = right, +1 = left. */
#define AVOID_DEFAULT_SIDE      (-1)
/** Forward speed during the detour, in m/s (same remark as FOLLOW_SPEED_M_S about the dead zone). */
#define AVOID_SPEED_M_S         0.22f
/**
 * How far the robot moves away from the line, in meters. It must be larger than half the width of the
 * robot (about 0.17 m on the tested platform: 0.15 m to the center of a wheel plus its thickness; measure
 * yours) plus half the width of the obstacle, plus a margin.
 */
#define AVOID_SIDE_STEP_M       0.35f
/**
 * Distance driven beside the obstacle, in meters. Count from the moment the detour starts: about 0.30 m to
 * the near face of the obstacle (OBSTACLE_CM plus the 0.10 m from the axle to the ultrasonic sensors), then
 * the depth of the obstacle, then half the width of the robot (about 0.17 m) so that it can turn back
 * towards the line without touching it, plus a margin. 0.70 m covers obstacles up to about 15 cm deep.
 */
#define AVOID_PASS_M            0.70f
/** Distance from the wheel axle to the line sensors, in meters. Measure it on your robot. */
#define SENSOR_BAR_AHEAD_M      0.06f
/** Slowest turn, in rad/s. It must be high enough to move the motors (about 1.2 rad/s on the tested platform). */
#define TURN_MIN_RAD_S          1.5f
/** Fastest turn, in rad/s. */
#define TURN_MAX_RAD_S          2.5f
/** The turn slows down proportionally to the remaining angle (rad/s per rad). */
#define TURN_KP                 3.0f
/** A turn ends when the remaining angle is smaller than this, in radians (0.08 rad is about 4.6 degrees). */
#define TURN_MARGIN_RAD         0.08f
/** Safety: a detour state that lasts longer than this gives up and goes back to FOLLOW_LINE. */
#define STEP_TIMEOUT_MS         8000

/* ---- Loop and sensors ---- */

/** Period of the main loop, in ms (20 Hz). */
#define LOOP_PERIOD_MS          50
/** Time to place the robot on the line before it starts moving, in ms. */
#define START_DELAY_MS          3000
/** Maximum wait for an ultrasonic echo, in ms. */
#define SONAR_TIMEOUT_MS        60
/** Echoes shorter than this (cm) are noise. */
#define SONAR_MIN_CM            2.0f
/** Distance reported when no sensor heard an echo, in cm. */
#define SONAR_NO_ECHO_CM        400.0f
/** After an overcurrent fault the robot waits this long (ms) before it is released again. */
#define FAULT_RETRY_MS          3000

#define HALF_PI_RAD             1.5707963f
#define IR_SENSOR_COUNT         5

/* ============================================================================
 *                              LINE SENSORS
 * ==========================================================================*/

/* From the far left to the far right. The index minus 2 is the position of the sensor: -2 ... +2. */
static const gpio_num_t s_ir_pins[IR_SENSOR_COUNT] = {
    INFRA_RED_LEFT_GPIO,
    INFRA_RED_CENTER_LEFT_GPIO,
    INFRA_RED_MIDDLE_GPIO,
    INFRA_RED_CENTER_RIGHT_GPIO,
    INFRA_RED_RIGHT_GPIO,
};

/* Level read by a sensor that is over the line. `IR_LINE_IS_BLACK` is defined in ir_line.h. */
#ifdef IR_LINE_IS_BLACK
#define IR_LEVEL_ON_LINE 0
#else
#define IR_LEVEL_ON_LINE 1
#endif

static void line_sensors_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = INFRA_RED_GPIO_PINS_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
}

/**
 * @brief Finds where the line is under the sensors.
 *
 * @details A line is normally seen by one sensor, or by two neighbours when it is between them. The
 *          position is the average of the sensors that see it: one sensor gives -2, -1, 0, +1 or +2, and two
 *          neighbours give the half steps between them (-1.5, -0.5, +0.5, +1.5). That is 9 positions from 5
 *          sensors.
 *
 * @param[out] position Position of the line from -2 (far left) to +2 (far right), 0 = centered.
 * @return true  A line was found and `position` is valid.
 * @return false No sensor sees the line.
 */
static bool line_read_position(float *position)
{
    int count = 0;
    int sum = 0;

    for (int i = 0; i < IR_SENSOR_COUNT; i++)
    {
        if (gpio_get_level(s_ir_pins[i]) == IR_LEVEL_ON_LINE)
        {
            sum += i - 2;       /* index 0 is the far left sensor: position -2 */
            count++;
        }
    }

    if (count == 0)
    {
        return false;
    }
    *position = (float)sum / (float)count;
    return true;
}

/* ============================================================================
 *                           ULTRASONIC SENSORS
 * ==========================================================================*/

typedef enum {
    OBSTACLE_NONE = 0,
    OBSTACLE_LEFT,      /* The order matches the sensors: left, middle, right. */
    OBSTACLE_MIDDLE,
    OBSTACLE_RIGHT,
} obstacle_side_t;

typedef struct {
    float distance_cm;      /* Distance to the nearest object, or SONAR_NO_ECHO_CM. */
    obstacle_side_t side;   /* Which sensor saw it. */
} obstacle_t;

/**
 * @brief Triggers the three HC-SR04 and returns the nearest object they saw.
 *
 * @details `ThreeEyes_TrigAndWait()` returns as soon as the first echo arrives, which is the nearest
 *          object, so only that sensor has `isUpdated` set. This is exactly what the task needs: the
 *          distance of the nearest object and the side where it is. It blocks for about 40 ms when
 *          nothing is in range (an HC-SR04 with no echo still answers with a 38 ms pulse).
 *
 *          If no sensor answers at all (a wire is off), the wait ends by timeout after `SONAR_TIMEOUT_MS`
 *          and the library keeps the flags and the values of the previous reading. They are old data, so
 *          they are ignored: the result is "nothing seen" (the library prints its own error each time).
 */
static obstacle_t sonar_read_nearest(void)
{
    ultrasonic_value_t sensor[3];

    TickType_t start_tick = xTaskGetTickCount();
    ThreeEyes_TrigAndWait(pdMS_TO_TICKS(SONAR_TIMEOUT_MS));
    bool timed_out = (xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(SONAR_TIMEOUT_MS);
    ThreeEyes_Read(&sensor[0], &sensor[1], &sensor[2]);

    obstacle_t nearest = { .distance_cm = SONAR_NO_ECHO_CM, .side = OBSTACLE_NONE };
    for (int i = 0; i < 3; i++)
    {
        if (timed_out || sensor[i].isUpdated != pdTRUE)
        {
            continue;
        }
        /* Same conversion as threeeyes_task.c: ticks -> microseconds -> centimeters (58 us per cm). */
        float cm = (sensor[i].tof_ticks * (1000000.0f / esp_clk_apb_freq())) / 58.0f;
        if (cm >= SONAR_MIN_CM && cm < nearest.distance_cm)
        {
            nearest.distance_cm = cm;
            nearest.side = (obstacle_side_t)(OBSTACLE_LEFT + i);
        }
    }
    return nearest;
}

/* ============================================================================
 *                                  DRIVE
 * ==========================================================================*/

/**
 * @brief Creates the ADC unit, the two wheels and starts the differential drive.
 *
 * @details Same sequence as diff_drive_task.c (read it for the explanation of each field), without the
 *          calibration. The pins and the platform constants come from juca_board.h and juca_platform.h.
 */
static void drive_init(void)
{
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = JUCA_ADC_UNIT_CURRENT,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));

    adc_oneshot_chan_cfg_t adc_channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JUCA_ADC_CHANNEL_CURRENT_LEFT, &adc_channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JUCA_ADC_CHANNEL_CURRENT_RIGHT, &adc_channel_config));

    wheel_config_t left_config = {
        .pwm_a_gpio = JUCA_PIN_MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = JUCA_PIN_MOTOR_LEFT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_ticks = JUCA_MOTOR_MAX_POWER_TICKS,
        .encoder_a_gpio = JUCA_PIN_ENCODER_LEFT_A,
        .encoder_b_gpio = JUCA_PIN_ENCODER_LEFT_B,
        .adc_handle = adc_handle,
        .adc_channel = JUCA_ADC_CHANNEL_CURRENT_LEFT,
    };
    wheel_config_t right_config = {
        .pwm_a_gpio = JUCA_PIN_MOTOR_RIGHT_PWM_A,
        .pwm_b_gpio = JUCA_PIN_MOTOR_RIGHT_PWM_B,
        .mcpwm_group_id = 0,
        .max_power_ticks = JUCA_MOTOR_MAX_POWER_TICKS,
        .encoder_a_gpio = JUCA_PIN_ENCODER_RIGHT_A,
        .encoder_b_gpio = JUCA_PIN_ENCODER_RIGHT_B,
        .adc_handle = adc_handle,
        .adc_channel = JUCA_ADC_CHANNEL_CURRENT_RIGHT,
    };

    wheel_handle_t left_wheel = NULL;
    wheel_handle_t right_wheel = NULL;
    ESP_ERROR_CHECK(wheel_create(&left_config, &left_wheel));
    ESP_ERROR_CHECK(wheel_create(&right_config, &right_wheel));

    diff_drive_config_t drive_config = {
        .left_wheel = left_wheel,
        .right_wheel = right_wheel,

        .half_track_width_m = JUCA_HALF_TRACK_WIDTH_M,
        .wheel_radius_m = JUCA_WHEEL_RADIUS_M,
        .left_encoder_cpr = JUCA_ENCODER_CPR_LEFT,
        .right_encoder_cpr = JUCA_ENCODER_CPR_RIGHT,

        .max_linear_velocity_m_s = 0.4f,
        .max_angular_velocity_rad_s = 3.0f,
        .max_wheel_rad_s = 20.0f,

        .max_adc_raw_threshold = 3000,
        .current_ma_per_count = JUCA_CURRENT_MA_PER_COUNT,
        .monitor_period_ms = 50,
        .safety_task_priority = 5,
        .safety_task_stack_size = configMINIMAL_STACK_SIZE * 4,

        /* A line follower needs quick steering: the angular ramp is faster than in diff_drive_task.c. */
        .max_accel_linear_m_s2 = 1.0f,
        .max_accel_angular_rad_s2 = 8.0f,
        .control_period_ms = 20,
        .control_task_priority = 4,
        .control_task_stack_size = 3072,
    };
    ESP_ERROR_CHECK(diff_drive_init(&drive_config));
}

/* ============================================================================
 *                              STATE MACHINE
 * ==========================================================================*/

typedef enum {
    STATE_FOLLOW_LINE,
    STATE_TURN_OUT,         /* 1. turn away from the obstacle (90 degrees) */
    STATE_SIDE_STEP,        /* 2. move sideways, away from the line */
    STATE_TURN_PARALLEL,    /* 3. turn back to the direction of the line */
    STATE_PASS,             /* 4. drive beside the obstacle until it is behind */
    STATE_TURN_IN,          /* 5. turn towards the line (90 degrees) */
    STATE_FIND_LINE,        /* 6. drive until the sensors see the line (and a little more) */
    STATE_ALIGN,            /* 7. turn back to the direction of the line */
} state_t;

static const char *const s_state_names[] = {
    "FOLLOW_LINE", "TURN_OUT", "SIDE_STEP", "TURN_PARALLEL", "PASS", "TURN_IN", "FIND_LINE", "ALIGN",
};

/* Everything the state machine remembers between two loops. */
typedef struct {
    state_t state;
    int detour_side;            /* +1 = detour to the left, -1 = to the right */
    float heading_rad;          /* Turned angle since the detour began, + = counter-clockwise (odometry) */
    float travelled_m;          /* Distance driven since the current state began (odometry) */
    int count;                  /* Debounce counter of the current state */
    bool passed;                /* The current state is in its second phase */
    TickType_t state_start_tick;
    int last_line_side;         /* Side where the line was seen last: -1 = left, +1 = right */
} app_t;

/* What the sensors say in this loop. */
typedef struct {
    obstacle_t obstacle;
    bool line_found;
    float line_position;
} sensors_t;

static void enter_state(app_t *app, state_t next)
{
    if (next != app->state)
    {
        ESP_LOGI(TAG, "%s -> %s", s_state_names[app->state], s_state_names[next]);
    }
    app->state = next;
    app->count = 0;
    app->passed = false;
    app->travelled_m = 0.0f;
    app->state_start_tick = xTaskGetTickCount();
}

/* Forward speed of the detour, but stopped while something is very close in front. */
static float avoid_speed(const sensors_t *in)
{
    return (in->obstacle.distance_cm < STOP_CM) ? 0.0f : AVOID_SPEED_M_S;
}

/**
 * @brief Turns in place towards a heading, slowing down when it gets close.
 *
 * @param[in]  target_rad  Wanted heading, relative to the heading the robot had when the detour began.
 * @param[in]  heading_rad Current heading, from the odometry.
 * @param[out] cmd         Receives the turn command.
 * @return true The robot is within `TURN_MARGIN_RAD` of the target, so the turn is done.
 */
static bool turn_to(float target_rad, float heading_rad, diff_drive_twist_t *cmd)
{
    float error = target_rad - heading_rad;
    if (fabsf(error) < TURN_MARGIN_RAD)
    {
        cmd->angular_z = 0.0f;
        return true;
    }

    float speed = fabsf(TURN_KP * error);
    if (speed < TURN_MIN_RAD_S) { speed = TURN_MIN_RAD_S; }
    if (speed > TURN_MAX_RAD_S) { speed = TURN_MAX_RAD_S; }
    cmd->angular_z = copysignf(speed, error);
    return false;
}

/* FOLLOW_LINE: steer towards the line, or look for it. */
static void follow_line(const sensors_t *in, app_t *app, diff_drive_twist_t *cmd)
{
    if (in->line_found)
    {
        /* Line on the right (positive position) -> turn right, which is a negative angular_z. */
        cmd->angular_z = -LINE_KP_RAD_S * in->line_position;
        cmd->linear_x = FOLLOW_SPEED_M_S * (1.0f - LINE_SLOWDOWN * fabsf(in->line_position) / 2.0f);

        if (in->line_position < -0.01f)
        {
            app->last_line_side = -1;
        }
        else if (in->line_position > 0.01f)
        {
            app->last_line_side = +1;
        }
    }
    else
    {
        /* Lost the line: stop and spin towards the side where it was seen last. */
        cmd->linear_x = 0.0f;
        cmd->angular_z = -(float)app->last_line_side * SEARCH_TURN_RAD_S;
    }
}

/**
 * @brief One step of the state machine: reads the sensors and decides the velocity command.
 *
 * @param[in,out] app State of the machine.
 * @param[in]     in  Sensor readings of this loop.
 * @param[out]    cmd Velocity command (it starts as "stop").
 */
static void state_machine_step(app_t *app, const sensors_t *in, diff_drive_twist_t *cmd)
{
    /* A detour that takes too long is abandoned: back to following the line. */
    if (app->state != STATE_FOLLOW_LINE &&
        (xTaskGetTickCount() - app->state_start_tick) * portTICK_PERIOD_MS > STEP_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "%s took too long, giving up the detour.", s_state_names[app->state]);
        enter_state(app, STATE_FOLLOW_LINE);
    }

    switch (app->state)
    {
    case STATE_FOLLOW_LINE:
        app->count = (in->obstacle.distance_cm < OBSTACLE_CM) ? app->count + 1 : 0;
        if (app->count >= OBSTACLE_CONFIRM_READS)
        {
            /* An object on the left: go around on the right, and the other way round. */
            if (in->obstacle.side == OBSTACLE_LEFT)       { app->detour_side = -1; }
            else if (in->obstacle.side == OBSTACLE_RIGHT) { app->detour_side = +1; }
            else                                          { app->detour_side = AVOID_DEFAULT_SIDE; }

            ESP_LOGW(TAG, "Obstacle at %.0f cm (sensor %d): detour to the %s.", in->obstacle.distance_cm,
                     (int)in->obstacle.side, (app->detour_side > 0) ? "left" : "right");
            app->heading_rad = 0.0f;
            enter_state(app, STATE_TURN_OUT);
        }
        else
        {
            follow_line(in, app, cmd);
        }
        break;

    case STATE_TURN_OUT:
        if (turn_to(app->detour_side * HALF_PI_RAD, app->heading_rad, cmd))
        {
            enter_state(app, STATE_SIDE_STEP);
        }
        break;

    case STATE_SIDE_STEP:
        cmd->linear_x = avoid_speed(in);
        if (app->travelled_m >= AVOID_SIDE_STEP_M)
        {
            enter_state(app, STATE_TURN_PARALLEL);
        }
        break;

    case STATE_TURN_PARALLEL:
        if (turn_to(0.0f, app->heading_rad, cmd))
        {
            enter_state(app, STATE_PASS);
        }
        break;

    case STATE_PASS:
        /* Drive at least AVOID_PASS_M, and keep going while the sensors still see the obstacle. */
        cmd->linear_x = avoid_speed(in);
        app->count = (in->obstacle.distance_cm > CLEAR_CM) ? app->count + 1 : 0;
        if (app->travelled_m >= AVOID_PASS_M && app->count >= CLEAR_CONFIRM_READS)
        {
            enter_state(app, STATE_TURN_IN);
        }
        break;

    case STATE_TURN_IN:
        /* The line is on the side opposite to the detour. */
        if (turn_to(-app->detour_side * HALF_PI_RAD, app->heading_rad, cmd))
        {
            enter_state(app, STATE_FIND_LINE);
        }
        break;

    case STATE_FIND_LINE:
        /* Phase 1: drive until a sensor sees the line. Phase 2: drive on until the wheel axle is over it. */
        cmd->linear_x = avoid_speed(in);
        if (!app->passed)
        {
            if (in->line_found)
            {
                app->passed = true;
                app->travelled_m = 0.0f;
            }
        }
        else if (app->travelled_m >= SENSOR_BAR_AHEAD_M)
        {
            cmd->linear_x = 0.0f;
            enter_state(app, STATE_ALIGN);
        }
        break;

    case STATE_ALIGN:
        if (turn_to(0.0f, app->heading_rad, cmd))
        {
            enter_state(app, STATE_FOLLOW_LINE);
        }
        break;
    }
}

/* ============================================================================
 *                                  TASK
 * ==========================================================================*/

/**
 * @brief Main task: reads the sensors, runs the state machine and drives the robot, 20 times per second.
 *
 * @param[in] arg FreeRTOS task argument (unused).
 */
portTASK_FUNCTION(line_avoid_task, arg)
{
    (void)arg;

    line_sensors_init();
    ThreeEyes_Init();
    drive_init();

    ESP_LOGW(TAG, "Put the robot on the line. It starts moving in %d s.", START_DELAY_MS / 1000);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    app_t app = { .state = STATE_FOLLOW_LINE, .last_line_side = +1 };
    diff_drive_twist_t measured = {0};
    bool fault_logged = false;
    TickType_t fault_tick = 0;
    int log_counter = 0;

    /* The first odometry reading would include everything that happened since init: discard it. */
    (void)diff_drive_get_twist(0.001f, &measured);

    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t previous_tick = last_wake_tick;

    for ( ; ; )
    {
        /* ---- Overcurrent protection tripped: stay stopped, then release and start again ---- */
        if (diff_drive_is_fault_active())
        {
            if (!fault_logged)
            {
                ESP_LOGE(TAG, "Overcurrent fault: the robot is stopped. Retrying in %d s.", FAULT_RETRY_MS / 1000);
                fault_logged = true;
                fault_tick = xTaskGetTickCount();
            }
            else if ((xTaskGetTickCount() - fault_tick) * portTICK_PERIOD_MS >= FAULT_RETRY_MS)
            {
                diff_drive_clear_fault();
                fault_logged = false;
                enter_state(&app, STATE_FOLLOW_LINE);
            }
            vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(LOOP_PERIOD_MS));
            continue;
        }

        /* ---- 1. Read the sensors. The ultrasonic reading blocks first, so the line is read fresh. ---- */
        sensors_t in = { .line_position = 0.0f };
        in.obstacle = sonar_read_nearest();
        in.line_found = line_read_position(&in.line_position);

        /* ---- 2. Odometry: integrate the measured velocity to know the turned angle and the distance ---- */
        TickType_t now_tick = xTaskGetTickCount();
        float delta_time_s = (float)(now_tick - previous_tick) * portTICK_PERIOD_MS / 1000.0f;
        previous_tick = now_tick;
        if (delta_time_s > 0.0f && diff_drive_get_twist(delta_time_s, &measured) == ESP_OK)
        {
            app.heading_rad += measured.angular_z * delta_time_s;
            app.travelled_m += fabsf(measured.linear_x) * delta_time_s;
        }

        /* ---- 3. Decide and drive ---- */
        diff_drive_twist_t cmd = {0};
        state_machine_step(&app, &in, &cmd);
        if (diff_drive_set_twist(&cmd) != ESP_OK && !diff_drive_is_fault_active())
        {
            /* (A fault that latched just now also rejects the command; the next loop reports it.) */
            ESP_LOGW(TAG, "Velocity command rejected.");
        }

        /* ---- 4. Telemetry twice per second ---- */
        if (++log_counter >= 10)
        {
            log_counter = 0;
            if (in.line_found)
            {
                ESP_LOGI(TAG, "%-13s line %+.1f | obstacle %3.0f cm | cmd %.2f m/s %+.2f rad/s",
                         s_state_names[app.state], in.line_position, in.obstacle.distance_cm,
                         cmd.linear_x, cmd.angular_z);
            }
            else
            {
                ESP_LOGI(TAG, "%-13s line lost | obstacle %3.0f cm | cmd %.2f m/s %+.2f rad/s",
                         s_state_names[app.state], in.obstacle.distance_cm, cmd.linear_x, cmd.angular_z);
            }
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}
