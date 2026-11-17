/*
 * Juca - LineAvoid
 *
 * Follows a line with the 5 infrared sensors and goes around the obstacles seen by the 3 HC-SR04.
 * Port of line_avoid_task.c of the Juca ESP-IDF firmware. Serial Monitor at 115200 baud.
 *
 * THE ROBOT MOVES 3 s AFTER START-UP. Put it on the line first.
 * THIS EXAMPLE WAS NOT TESTED ON THE ROBOT: every number below is a starting value. Run the
 * LineSensors, Sonar and WheelTest examples first, and be ready to cut the motor power.
 *
 * It runs a small state machine at 20 Hz and only decides a velocity ("twist": linear_x in m/s,
 * angular_z in rad/s), which the differential drive of the library executes.
 *
 * FOLLOW_LINE
 *   The position of the line is the average of the sensors that see it, from -2 (far left) to +2
 *   (far right). A proportional controller turns the robot towards the line. If no sensor sees the
 *   line, the robot spins towards the side where it saw it last.
 *
 * The other states: the detour
 *   When an ultrasonic sensor sees an object closer than OBSTACLE_CM, the robot leaves the line,
 *   goes around the object on a square path and comes back. Turns and distances are measured with
 *   the wheel odometry, not by timing the motors.
 *
 *     line  =====>====[ obstacle ]=============>=====
 *                 |                       ^
 *   1 TURN_OUT    |                       |  5 TURN_IN
 *   2 SIDE_STEP   |                       |  6 FIND_LINE
 *                 v                       |  7 ALIGN
 *                 +-----------------------+
 *                 3 TURN_PARALLEL, 4 PASS
 *
 * Limits of this simple version:
 *   - The drive is open loop and the motors do not move below about 24% of duty: the speeds cannot
 *     be set very low, and the real speed is not the commanded one.
 *   - A lost line is searched by spinning in place.
 *   - The detour uses fixed distances that assume one obstacle up to about 20 cm wide and 15 cm deep.
 *   - At these speeds the overcurrent protection does NOT detect a blocked wheel.
 */

#include <Juca.h>

/* ============================================================================
 *                          TUNING (change these first)
 * ==========================================================================*/

/* ---- Following the line ---- */

/** Forward speed on a straight line, in m/s. Do not go much lower (dead zone of the motors). */
#define FOLLOW_SPEED_M_S        0.25f
/** Steering gain: rad/s of turn for each unit of line position (one unit = the distance between two sensors). */
#define LINE_KP_RAD_S           1.2f
/** Fraction of the speed removed when the line is at the edge of the sensors (0 = never slow down). */
#define LINE_SLOWDOWN           0.5f
/** Spin speed used to look for the line when no sensor sees it, in rad/s. */
#define SEARCH_TURN_RAD_S       1.5f

/* ---- Seeing obstacles ---- */

/** An object closer than this (cm) starts the detour. */
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
/** Forward speed during the detour, in m/s. */
#define AVOID_SPEED_M_S         0.22f
/** How far the robot moves away from the line, in meters (half the robot + half the obstacle + margin). */
#define AVOID_SIDE_STEP_M       0.35f
/** Distance driven beside the obstacle, in meters. 0.70 m covers obstacles up to about 15 cm deep. */
#define AVOID_PASS_M            0.70f
/** Distance from the wheel axle to the line sensors, in meters. Measure it on your robot. */
#define SENSOR_BAR_AHEAD_M      0.06f
/** Slowest turn, in rad/s. It must be high enough to move the motors. */
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
/** Distance used when nothing is in range, in cm. */
#define SONAR_NO_ECHO_CM        400.0f
/** After an overcurrent fault the robot waits this long (ms) before it is released again. */
#define FAULT_RETRY_MS          3000

#define HALF_PI_RAD             1.5707963f

/* ============================================================================
 *                                  TYPES
 * ==========================================================================*/
/* (All the types are declared before the first function: the Arduino builder needs that.) */

/* What the ultrasonic sensors say. */
typedef struct {
    float distance_cm;      /* Distance to the nearest object, or SONAR_NO_ECHO_CM. */
    JucaSonarSide side;     /* Which sensor saw it. */
} obstacle_t;

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

/* Everything the state machine remembers between two loops. */
typedef struct {
    state_t state;
    int detour_side;            /* +1 = detour to the left, -1 = to the right */
    float heading_rad;          /* Turned angle since the detour began, + = counter-clockwise (odometry) */
    float travelled_m;          /* Distance driven since the current state began (odometry) */
    int count;                  /* Debounce counter of the current state */
    bool passed;                /* The current state is in its second phase */
    uint32_t state_start_ms;
    int last_line_side;         /* Side where the line was seen last: -1 = left, +1 = right */
} app_t;

/* What the sensors say in this loop. */
typedef struct {
    obstacle_t obstacle;
    bool line_found;
    float line_position;
} sensors_t;

/* ============================================================================
 *                                  STATE
 * ==========================================================================*/

static const char *const s_state_names[] = {
    "FOLLOW_LINE", "TURN_OUT", "SIDE_STEP", "TURN_PARALLEL", "PASS", "TURN_IN", "FIND_LINE", "ALIGN",
};

static app_t s_app = { STATE_FOLLOW_LINE, 0, 0.0f, 0.0f, 0, false, 0, +1 };
static diff_drive_twist_t s_measured = { 0.0f, 0.0f };
static bool s_fault_logged = false;
static uint32_t s_fault_ms = 0;
static uint32_t s_previous_ms = 0;
static int s_log_counter = 0;

/* ============================================================================
 *                                 SENSORS
 * ==========================================================================*/

/**
 * Triggers the three HC-SR04 and returns the nearest object they saw.
 * It blocks for about 40 ms when nothing is in range, and SONAR_TIMEOUT_MS if no sensor answers.
 */
static obstacle_t sonar_read_nearest(void)
{
    obstacle_t nearest = { SONAR_NO_ECHO_CM, JUCA_SONAR_NONE };

    JucaSonarSide side = JUCA_SONAR_NONE;
    float cm = Juca.readSonarCm(&side, SONAR_TIMEOUT_MS);
    if (cm >= SONAR_MIN_CM)     /* a negative value means "nothing in range" */
    {
        nearest.distance_cm = cm;
        nearest.side = side;
    }
    return nearest;
}

/* ============================================================================
 *                              STATE MACHINE
 * ==========================================================================*/

static void enter_state(app_t *app, state_t next)
{
    if (next != app->state)
    {
        Serial.printf("%s -> %s\n", s_state_names[app->state], s_state_names[next]);
    }
    app->state = next;
    app->count = 0;
    app->passed = false;
    app->travelled_m = 0.0f;
    app->state_start_ms = millis();
}

/* Forward speed of the detour, but stopped while something is very close in front. */
static float avoid_speed(const sensors_t *in)
{
    return (in->obstacle.distance_cm < STOP_CM) ? 0.0f : AVOID_SPEED_M_S;
}

/**
 * Turns in place towards a heading, slowing down when it gets close.
 * Returns true when the robot is within TURN_MARGIN_RAD of the target (the turn is done).
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

/* One step of the state machine: from the sensor readings to the velocity command (it starts as "stop"). */
static void state_machine_step(app_t *app, const sensors_t *in, diff_drive_twist_t *cmd)
{
    /* A detour that takes too long is abandoned: back to following the line. */
    if (app->state != STATE_FOLLOW_LINE && (millis() - app->state_start_ms) > STEP_TIMEOUT_MS)
    {
        Serial.printf("%s took too long, giving up the detour.\n", s_state_names[app->state]);
        enter_state(app, STATE_FOLLOW_LINE);
    }

    switch (app->state)
    {
    case STATE_FOLLOW_LINE:
        app->count = (in->obstacle.distance_cm < OBSTACLE_CM) ? app->count + 1 : 0;
        if (app->count >= OBSTACLE_CONFIRM_READS)
        {
            /* An object on the left: go around on the right, and the other way round. */
            if (in->obstacle.side == JUCA_SONAR_LEFT)       { app->detour_side = -1; }
            else if (in->obstacle.side == JUCA_SONAR_RIGHT) { app->detour_side = +1; }
            else                                            { app->detour_side = AVOID_DEFAULT_SIDE; }

            Serial.printf("Obstacle at %.0f cm: detour to the %s.\n", in->obstacle.distance_cm,
                          (app->detour_side > 0) ? "left" : "right");
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
 *                              SETUP AND LOOP
 * ==========================================================================*/

static void halt(const char *what)
{
    Serial.printf("%s failed to start: %s\n", what, esp_err_to_name(Juca.lastError()));
    while (true) { delay(1000); }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);   /* time to open the Serial Monitor */

    if (!Juca.beginLine())  { halt("Line sensors"); }
    if (!Juca.beginSonar()) { halt("Sonar"); }

    /* The drive of the library with a configuration for line following: speed limits, and quicker
     * ramps than the default (a line follower needs quick steering). The robot is not calibrated:
     * the line sensors close the loop, so a small left/right difference is corrected on the line. */
    diff_drive_config_t config = Juca.defaultDriveConfig();
    config.max_linear_velocity_m_s = 0.4f;
    config.max_angular_velocity_rad_s = 3.0f;
    config.max_accel_linear_m_s2 = 1.0f;
    config.max_accel_angular_rad_s2 = 8.0f;
    if (!Juca.beginDrive(&config)) { halt("Drive"); }

    Serial.printf("Put the robot on the line. It starts moving in %d s.\n", START_DELAY_MS / 1000);
    delay(START_DELAY_MS);

    /* The first odometry reading would include everything that happened since the start: discard it.
     * (This sketch integrates the odometry itself, so it uses diff_drive_get_twist() directly, with
     * its own time step, instead of Juca.getTwist().) */
    (void)diff_drive_get_twist(0.001f, &s_measured);
    s_previous_ms = millis();
    s_app.state_start_ms = s_previous_ms;
}

void loop()
{
    uint32_t loop_start_ms = millis();

    /* ---- Overcurrent protection tripped: stay stopped, then release and start again ---- */
    if (Juca.faultActive())
    {
        if (!s_fault_logged)
        {
            Serial.printf("Overcurrent fault: the robot is stopped. Retrying in %d s.\n", FAULT_RETRY_MS / 1000);
            s_fault_logged = true;
            s_fault_ms = millis();
        }
        else if (millis() - s_fault_ms >= FAULT_RETRY_MS)
        {
            Juca.clearFault();
            s_fault_logged = false;
            enter_state(&s_app, STATE_FOLLOW_LINE);
        }
        delay(LOOP_PERIOD_MS);
        return;
    }

    /* ---- 1. Read the sensors. The ultrasonic reading blocks first, so the line is read fresh. ---- */
    sensors_t in = { { SONAR_NO_ECHO_CM, JUCA_SONAR_NONE }, false, 0.0f };
    in.obstacle = sonar_read_nearest();
    in.line_found = Juca.readLinePosition(in.line_position);

    /* ---- 2. Odometry: integrate the measured velocity to know the turned angle and the distance ---- */
    uint32_t now_ms = millis();
    float delta_time_s = (float)(now_ms - s_previous_ms) / 1000.0f;
    s_previous_ms = now_ms;
    if (delta_time_s > 0.0f && diff_drive_get_twist(delta_time_s, &s_measured) == ESP_OK)
    {
        s_app.heading_rad += s_measured.angular_z * delta_time_s;
        s_app.travelled_m += fabsf(s_measured.linear_x) * delta_time_s;
    }

    /* ---- 3. Decide and drive ---- */
    diff_drive_twist_t cmd = { 0.0f, 0.0f };
    state_machine_step(&s_app, &in, &cmd);
    if (!Juca.setTwist(cmd.linear_x, cmd.angular_z) && !Juca.faultActive())
    {
        /* (A fault that latched just now also rejects the command; the next loop reports it.) */
        Serial.println("Velocity command rejected.");
    }

    /* ---- 4. Telemetry twice per second ---- */
    if (++s_log_counter >= 10)
    {
        s_log_counter = 0;
        if (in.line_found)
        {
            Serial.printf("%-13s line %+.1f | obstacle %3.0f cm | cmd %.2f m/s %+.2f rad/s\n",
                          s_state_names[s_app.state], in.line_position, in.obstacle.distance_cm,
                          cmd.linear_x, cmd.angular_z);
        }
        else
        {
            Serial.printf("%-13s line lost | obstacle %3.0f cm | cmd %.2f m/s %+.2f rad/s\n",
                          s_state_names[s_app.state], in.obstacle.distance_cm, cmd.linear_x, cmd.angular_z);
        }
    }

    /* ---- 5. Keep the 20 Hz rhythm: wait for what is left of the period ---- */
    uint32_t spent_ms = millis() - loop_start_ms;
    if (spent_ms < LOOP_PERIOD_MS)
    {
        delay(LOOP_PERIOD_MS - spent_ms);
    }
}
