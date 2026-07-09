/**
 * @file    wheel.c
 * @brief   Differential drive wheel control module.
 *
 * This module is responsible for:
 *  - Configuring and driving the left/right DC motors (via MCPWM);
 *  - Reading the quadrature encoders (via the PCNT peripheral) to track
 *    wheel rotation;
 *  - Reading motor current through the ADC to detect wheel stall
 *    conditions and disable PWM output when a stall is detected;
 *  - Exposing a simple API to set raw wheel speed/direction commands.
 *
 * @date    Jan 9, 2025
 * @author  Matheus
 */

#include "wheel.h"
#include <stdbool.h>

/** Log tag used by ESP_LOG* calls in this module. */
const static char *WHEEL_TAG = "Wheels";

/** Raw ADC samples buffer for the left motor current sensor. */
static int adc_left_raw[2][10];
/** Raw ADC samples buffer for the right motor current sensor. */
static int adc_right_raw[2][10];

/* ============================================================================
 *                              ADC CONFIGURATION
 * ==========================================================================*/

/** General ADC channel configuration (bit width and attenuation) shared by
 *  both the left and right current-sensing channels. */
adc_oneshot_chan_cfg_t adc_config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
};

/** ADC oneshot unit handle used to perform current readings. */
adc_oneshot_unit_handle_t adc_handle;

/** ADC oneshot unit initialization configuration. */
adc_oneshot_unit_init_cfg_t adc_init_config = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};

/* ============================================================================
 *                        GENERAL MOTOR CONTEXT STRUCTS
 * ==========================================================================*/

/** Left motor control context (motor handle + associated encoder unit). */
static motor_control_context_t motor_left_ctrl_ctx = {
    .pcnt_encoder = NULL,
};

/** Right motor control context (motor handle + associated encoder unit). */
static motor_control_context_t motor_right_ctrl_ctx = {
    .pcnt_encoder = NULL,
};

/** Left motor MCPWM GPIO / frequency configuration. */
bdc_motor_config_t motor_left_config = {
    .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    .pwma_gpio_num = BDC_LEFT_MCPWM_GPIO_A,
    .pwmb_gpio_num = BDC_LEFT_MCPWM_GPIO_B,
};

/** Right motor MCPWM GPIO / frequency configuration. */
bdc_motor_config_t motor_right_config = {
    .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    .pwma_gpio_num = BDC_RIGHT_MCPWM_GPIO_A,
    .pwmb_gpio_num = BDC_RIGHT_MCPWM_GPIO_B,
};

/* ============================================================================
 *                          PWM (MCPWM) DRIVER CONFIG
 * ==========================================================================*/

/** MCPWM group/resolution configuration shared by both motors. */
bdc_motor_mcpwm_config_t mcpwm_config = {
    .group_id = 0,
    .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
};

/* ============================================================================
 *                              MOTOR HANDLES
 * ==========================================================================*/

/** Left motor driver handle. */
bdc_motor_handle_t motor_left = NULL;
/** Right motor driver handle. */
bdc_motor_handle_t motor_right = NULL;

/* ============================================================================
 *                    ENCODER PULSE COUNTER (PCNT) CONFIG
 * ==========================================================================*/

/** General pulse counter unit configuration, shared by both encoders. */
pcnt_unit_config_t pcnt_unit_config = {
    .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
    .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
    .flags.accum_count = true, /**< Enable counter accumulation. */
};

/** Glitch filter configuration applied to both encoder PCNT units. */
pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 1000,
};

/* ============================================================================
 *                       ENCODER PULSE COUNTER HANDLES
 * ==========================================================================*/

/** Pulse counter unit handle for the left wheel encoder. */
pcnt_unit_handle_t pcnt_left_unit = NULL;
/** Pulse counter unit handle for the right wheel encoder. */
pcnt_unit_handle_t pcnt_right_unit = NULL;

/* ============================================================================
 *                      ENCODER PULSE COUNTER CHANNELS
 * ==========================================================================*/

/** Left encoder - channel A GPIO configuration (edge/level pins). */
pcnt_chan_config_t chan_a_left_config = {
    .edge_gpio_num = BDC_ENCODER_LEFT_GPIO_A,
    .level_gpio_num = BDC_ENCODER_LEFT_GPIO_B,
};

/** Right encoder - channel A GPIO configuration (edge/level pins). */
pcnt_chan_config_t chan_a_right_config = {
    .edge_gpio_num = BDC_ENCODER_RIGHT_GPIO_A,
    .level_gpio_num = BDC_ENCODER_RIGHT_GPIO_B,
};

/** Left encoder - channel A handle. */
pcnt_channel_handle_t pcnt_chan_a_left = NULL;
/** Right encoder - channel A handle. */
pcnt_channel_handle_t pcnt_chan_a_right = NULL;

/** Left encoder - channel B GPIO configuration (edge/level pins). */
pcnt_chan_config_t chan_b_left_config = {
    .edge_gpio_num = BDC_ENCODER_LEFT_GPIO_B,
    .level_gpio_num = BDC_ENCODER_LEFT_GPIO_A,
};

/** Right encoder - channel B GPIO configuration (edge/level pins). */
pcnt_chan_config_t chan_b_right_config = {
    .edge_gpio_num = BDC_ENCODER_RIGHT_GPIO_B,
    .level_gpio_num = BDC_ENCODER_RIGHT_GPIO_A,
};

/** Left encoder - channel B handle. */
pcnt_channel_handle_t pcnt_chan_b_left = NULL;
/** Right encoder - channel B handle. */
pcnt_channel_handle_t pcnt_chan_b_right = NULL;

/* ============================================================================
 *                            INTERNAL STATE
 * ==========================================================================*/

/** Current wheel command (direction + PWM duty for each wheel). */
static wheel_cmd_t current_cmd = {0};

/** Global flag that enables/disables PWM output (used by the stall
 *  protection logic). Access must be protected by #pwm_mux. */
static volatile bool pwm_enabled = true;

/** Spinlock protecting concurrent access to #pwm_enabled. */
static portMUX_TYPE pwm_mux = portMUX_INITIALIZER_UNLOCKED;

/* ============================================================================
 *                          FORWARD DECLARATIONS
 * ==========================================================================*/

static void wheel_apply(void);

/* ============================================================================
 *                          HELPER / INLINE FUNCTIONS
 * ==========================================================================*/

/**
 * @brief Clamp a PWM value to the maximum allowed duty cycle.
 *
 * @param val Raw PWM value to clamp.
 * @return uint32_t Value clamped to [0, PWM_MAX].
 */
static inline uint32_t clamp_pwm(uint32_t val)
{
    if (val > PWM_MAX) return PWM_MAX;
    return val;
}

/**
 * @brief Thread-safe setter for the global PWM-enabled flag.
 *
 * @param val New state: true to allow PWM output, false to force it off.
 */
static inline void set_pwm_enabled(bool val)
{
    portENTER_CRITICAL(&pwm_mux);
    pwm_enabled = val;
    portEXIT_CRITICAL(&pwm_mux);
}

/* ============================================================================
 *                              PUBLIC API
 * ==========================================================================*/

/**
 * @brief Check whether PWM output is currently enabled.
 *
 * PWM output is automatically disabled by the stall-detection task
 * (#power_tracker) when a wheel stall condition is detected, and is
 * re-enabled once the condition clears.
 *
 * @return true  PWM output is enabled (motors may spin).
 * @return false PWM output is disabled (motors are forced to brake/stop).
 */
bool power_manager_is_enabled(void)
{
    return pwm_enabled;
}

/**
 * @brief Apply a raw PWM duty cycle to the right motor.
 *
 * @param pwm Duty cycle value to apply to the right motor driver.
 */
void apply_pwm_right(uint32_t pwm)
{
    ESP_ERROR_CHECK(bdc_motor_set_speed(motor_right, pwm));
}

/**
 * @brief Apply a raw PWM duty cycle to the left motor.
 *
 * @param pwm Duty cycle value to apply to the left motor driver.
 */
void apply_pwm_left(uint32_t pwm)
{
    ESP_ERROR_CHECK(bdc_motor_set_speed(motor_left, pwm));
}

/**
 * @brief Set the raw direction and PWM duty cycle for both wheels.
 *
 * The provided PWM values are clamped to the maximum allowed duty cycle,
 * stored as the current command, and immediately applied to the motors.
 *
 * @param dir_left  Desired direction for the left wheel.
 * @param pwm_left  Desired PWM duty cycle for the left wheel.
 * @param dir_right Desired direction for the right wheel.
 * @param pwm_right Desired PWM duty cycle for the right wheel.
 */
void wheel_SetRawSpeed(
    wheel_dir_t dir_left, uint32_t pwm_left,
    wheel_dir_t dir_right, uint32_t pwm_right)
{
    current_cmd.dir_left  = dir_left;
    current_cmd.dir_right = dir_right;

    current_cmd.pwm_left  = clamp_pwm(pwm_left);
    current_cmd.pwm_right = clamp_pwm(pwm_right);

    wheel_apply();
}

/* ============================================================================
 *                          INTERNAL IMPLEMENTATION
 * ==========================================================================*/

/**
 * @brief Apply the current wheel command to the motor drivers.
 *
 * Reads a local snapshot of #current_cmd, forces both PWM duty cycles to
 * zero if PWM output is globally disabled (stall protection active), and
 * then drives each motor: brakes it if its duty cycle is zero or its
 * direction is #WHEEL_STOP, otherwise sets the requested direction and
 * applies the PWM duty cycle.
 */
static void wheel_apply(void)
{
    wheel_cmd_t cmd = current_cmd;

    if (!power_manager_is_enabled())
    {
        cmd.pwm_left = 0;
        cmd.pwm_right = 0;
    }

    /* Left wheel */
    if (cmd.pwm_left == 0 || cmd.dir_left == WHEEL_STOP)
    {
        bdc_motor_brake(motor_left);
    }
    else
    {
        if (cmd.dir_left == WHEEL_FORWARD)
            bdc_motor_forward(motor_left);
        else
            bdc_motor_reverse(motor_left);

        apply_pwm_left(cmd.pwm_left);
    }

    /* Right wheel */
    if (cmd.pwm_right == 0 || cmd.dir_right == WHEEL_STOP)
    {
        bdc_motor_brake(motor_right);
    }
    else
    {
        if (cmd.dir_right == WHEEL_FORWARD)
            bdc_motor_forward(motor_right);
        else
            bdc_motor_reverse(motor_right);

        apply_pwm_right(cmd.pwm_right);
    }
}

/* ============================================================================
 *                                  TASKS
 * ==========================================================================*/

/**
 * @brief FreeRTOS task that monitors motor current and detects wheel stall.
 *
 * Periodically reads the left/right motor current (via wheel_GetPower())
 * and:
 *  - Adaptively speeds up its own polling period when current is high, and
 *    slows back down (up to 200 ms) when current is low, in order to react
 *    quickly to a developing stall while saving CPU time otherwise;
 *  - Maintains a hysteresis counter that increments while current stays
 *    above #LIMITE_STALL and decrements otherwise;
 *  - Declares a stall once the counter reaches #TEMPO_STALL consecutive
 *    high-current cycles: PWM output is disabled and both motors are
 *    immediately braked;
 *  - Clears the stall condition (re-enabling PWM output) once current
 *    drops below #LIMITE_LIBERACAO and the stall counter has fully
 *    decayed to zero.
 *
 * @param arg Unused FreeRTOS task argument.
 */
portTASK_FUNCTION(power_tracker, arg)
{
    uint32_t power_left_wheel, power_right_wheel;

    int stall_counter = 0;
    bool active_stall = false;

    /* Thresholds (raw ADC current reading) */
    const int LIMITE_STALL = 2500;
    const int LIMITE_LIBERACAO = 1500;

    /* Detection parameters */
    const int TEMPO_STALL = 10; /* number of cycles */

    /* Adaptive polling period */
    int ms_period = 200;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        wheel_GetPower(&power_left_wheel, &power_right_wheel);

        bool high_current = (power_left_wheel > LIMITE_STALL ||
                             power_right_wheel > LIMITE_STALL);

        /* ============================
         * Dynamic period adjustment
         * ============================ */
        if (high_current)
        {
            if (ms_period > 10)
            {
                ms_period /= 2;
                if (ms_period < 10)
                    ms_period = 10;

                /* Avoid drift when changing the period too fast */
                last_wake = xTaskGetTickCount();
            }
        }
        else
        {
            if (ms_period < 200)
            {
                ms_period += 10;
                if (ms_period > 200)
                    ms_period = 200;
            }
        }

        /* ============================
         * Stall counter
         * ============================ */
        if (high_current)
        {
            if (stall_counter < TEMPO_STALL)
                stall_counter++;
        }
        else
        {
            if (stall_counter > 0)
                stall_counter--;
        }

        /* ============================
         * Stall detection
         * ============================ */
        if (!active_stall && stall_counter >= TEMPO_STALL)
        {
            active_stall = true;
            set_pwm_enabled(false);

            /* Ensure the wheels stop immediately */
            bdc_motor_brake(motor_left);
            bdc_motor_brake(motor_right);

            ESP_LOGI(WHEEL_TAG, "Motor Stall!!!! Wheels blocked.");
        }

        /* ============================
         * Release (with hysteresis)
         * ============================ */
        bool low_current = (power_left_wheel < LIMITE_LIBERACAO &&
                              power_right_wheel < LIMITE_LIBERACAO);

        if (active_stall && low_current && stall_counter == 0)
        {
            active_stall = false;
            set_pwm_enabled(true);

            ESP_LOGI(WHEEL_TAG, "Wheels unblocked.");
        }

        /* ============================
         * Deterministic delay
         * ============================ */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ms_period));
    }
}

/**
 * @brief FreeRTOS task that periodically re-applies the current wheel
 *        command to the motor drivers at a fixed 50 Hz rate.
 *
 * Takes a snapshot of #current_cmd on every cycle, forces both PWM duty
 * cycles to zero if PWM output is globally disabled, and drives each
 * motor accordingly (brake or forward/reverse with the requested duty
 * cycle).
 *
 * @param arg Unused FreeRTOS task argument.
 */
portTASK_FUNCTION(speed_ctrl, arg)
{
    wheel_cmd_t cmd_local;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        /* Snapshot of the current command */
        cmd_local = current_cmd;

        /* Global protection */
        if (!power_manager_is_enabled())
        {
            cmd_local.pwm_left  = 0;
            cmd_local.pwm_right = 0;
        }

        /* =========================
         * Left motor
         * ========================= */
        if (cmd_local.pwm_left == 0 || cmd_local.dir_left == WHEEL_STOP)
        {
            bdc_motor_brake(motor_left);
        }
        else
        {
            if (cmd_local.dir_left == WHEEL_FORWARD)
                bdc_motor_forward(motor_left);
            else
                bdc_motor_reverse(motor_left);

            apply_pwm_left(cmd_local.pwm_left);
        }

        /* =========================
         * Right motor
         * ========================= */
        if (cmd_local.pwm_right == 0 || cmd_local.dir_right == WHEEL_STOP)
        {
            bdc_motor_brake(motor_right);
        }
        else
        {
            if (cmd_local.dir_right == WHEEL_FORWARD)
                bdc_motor_forward(motor_right);
            else
                bdc_motor_reverse(motor_right);

            apply_pwm_right(cmd_local.pwm_right);
        }

        /* Update rate */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20)); /* 50 Hz */
    }
}

/* ============================================================================
 *                          INITIALIZATION / API
 * ==========================================================================*/

/**
 * @brief Initialize the wheel subsystem.
 *
 * Performs full initialization of the module:
 *  - Configures the ADC unit and channels used for current sensing;
 *  - Creates the left/right MCPWM-based DC motor drivers;
 *  - Creates and configures the left/right PCNT units and their A/B
 *    channels to decode the quadrature encoder signals;
 *  - Enables the encoders and clears their counters;
 *  - Enables both motors, sets their initial direction to forward and
 *    their speed to zero;
 *  - Starts the #power_tracker task, which monitors current and protects
 *    the motors against stall conditions.
 *
 * @return int Always returns 1 on success (errors abort execution via
 *             ESP_ERROR_CHECK).
 */
int wheel_Init( void )
{
    /* ---------------------------------------------------------------
     * ADC initialization (current sensing)
     * --------------------------------------------------------------- */
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));
    //ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc_right_motor_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_8, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_1, &adc_config));

    //ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_right_motor_handle, ADC_CHANNEL_8, &config));

    /* ---------------------------------------------------------------
     * General motor config structs (declared statically above)
     * --------------------------------------------------------------- */
    //   ESP_LOGI(TAG, "Create DC motor Left");
    //   ESP_LOGI(TAG, "Create DC motor Right");

    /* ---------------------------------------------------------------
     * Create motor handles
     * --------------------------------------------------------------- */
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_left_config, &mcpwm_config, &motor_left));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_right_config, &mcpwm_config, &motor_right));
    motor_left_ctrl_ctx.motor = motor_left;
    motor_right_ctrl_ctx.motor = motor_right;

    /* ---------------------------------------------------------------
     * Encoder pulse counter config structs (declared statically above)
     * --------------------------------------------------------------- */
    // ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");

    /* ---------------------------------------------------------------
     * Create encoder pulse counter unit handles
     * --------------------------------------------------------------- */
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_left_unit));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_left_unit, &filter_config));
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_right_unit));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_right_unit, &filter_config));

    /* ---------------------------------------------------------------
     * Create encoder pulse counter channels
     *
     * Each encoder has two channels (A and B). Each channel below (A and
     * B) controls the pulse counter in a different way. Both use the
     * BDC_ENCODER_<LEFT/RIGHT>_GPIO_A and BDC_ENCODER_<LEFT/RIGHT>_GPIO_B
     * pins.
     *  - If the motor spins counter-clockwise, channel A decrements the
     *    counter on each BDC_ENCODER_<LEFT/RIGHT>_GPIO_A edge.
     *  - If the motor spins clockwise, channel B increments the counter
     *    on each BDC_ENCODER_<LEFT/RIGHT>_GPIO_A edge.
     * --------------------------------------------------------------- */
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_left_unit, &chan_a_left_config, &pcnt_chan_a_left));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_right_unit, &chan_a_right_config, &pcnt_chan_a_right));

    /*
     * When counter-clockwise, each edge of BDC_ENCODER_GPIO_A decrements
     * the counter.
     *        ____        ____        ____        ____
     *       |    |      |    |      |    |      |    |
     * ______|    |______|    |______|    |______|    |___      BDC_ENCODER_GPIO_B (LEVEL)
     *     ____        ____        ____        ____
     *    |    |      |    |      |    |      |    |
     * ___|    |______|    |______|    |______|    |___         BDC_ENCODER_GPIO_A  (EDGE)
     *  [-1]  [-2]  [-3]  [-4]  [-5]  [-6]   [-7]  [-8]         Counter Value
     *
     */
    // Decrease the counter on rising edge at LOW level, increase the counter on falling edge at HIGH level (will never happen!)
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_left, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_right, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    // Keep the counting mode when the control signal is at HIGH level (decrement), and reverse the counting mode when it is at LOW level (decrement)
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_left_unit, &chan_b_left_config, &pcnt_chan_b_left));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_right_unit, &chan_b_right_config, &pcnt_chan_b_right));

    /*
     * When clockwise, each edge of BDC_ENCODER_GPIO_B increments the
     * counter.
     *     ____        ____        ____        ____
     *    |    |      |    |      |    |      |    |
     * ___|    |______|    |______|    |______|    |___      BDC_ENCODER_GPIO_A (LEVEL)
     *        ____        ____        ____        ____
     *       |    |      |    |      |    |      |    |
     * ______|    |______|    |______|    |______|    |___   BDC_ENCODER_GPIO_B  (EDGE)
     *      [1]  [2]    [3]  [4]    [5]  [6]    [7]  [8]     Counter Value
     *
     */
    // Increase the counter on rising edge at HIGH level, decrease the counter on falling edge at HIGH level (will never happen!)
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_left, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_right, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    // Keep the counting mode when the control signal is at HIGH level (increment), and reverse the counting mode when it is at LOW level (increment)
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* ---------------------------------------------------------------
     * Configure pulse counter watch points
     * --------------------------------------------------------------- */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_left_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_left_unit, BDC_ENCODER_PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_right_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_right_unit, BDC_ENCODER_PCNT_LOW_LIMIT));

    /* ---------------------------------------------------------------
     * Enable and start the pulse counter modules
     * --------------------------------------------------------------- */
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_left_unit));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_right_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_left_unit));  // Always required before starting
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_right_unit)); // Always required before starting
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_left_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_right_unit));
    motor_left_ctrl_ctx.pcnt_encoder = pcnt_left_unit;
    motor_right_ctrl_ctx.pcnt_encoder = pcnt_right_unit;

    /* ---------------------------------------------------------------
     * Enable and start the motors
     * --------------------------------------------------------------- */
    ESP_LOGI(WHEEL_TAG, "Enable motors");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_left));
    ESP_ERROR_CHECK(bdc_motor_enable(motor_right));
    ESP_LOGI(WHEEL_TAG, "Forward motors");
    ESP_ERROR_CHECK(bdc_motor_forward(motor_left));
    ESP_ERROR_CHECK(bdc_motor_forward(motor_right));
    //ESP_LOGI(TAG, "Start motor speed loop");
    //ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

    bdc_motor_set_speed(motor_left, 0);
    bdc_motor_set_speed(motor_right, 0);

    /* ---------------------------------------------------------------
     * Start the stall-protection monitoring task
     * --------------------------------------------------------------- */
    xTaskCreate(
        power_tracker,
        "pw_trck",
        configMINIMAL_STACK_SIZE*3,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL
    );

    return 1;
}

/**
 * @brief Read the current motor power (raw ADC current readings).
 *
 * @param[out] pL Pointer where the left motor's raw ADC reading is stored.
 * @param[out] pR Pointer where the right motor's raw ADC reading is stored.
 */
void wheel_GetPower( uint32_t *pL, uint32_t *pR )
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_1, &adc_right_raw[1][0]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_8, &adc_left_raw[1][0]));
    *pL = adc_left_raw[1][0];
    *pR = adc_right_raw[1][0];
    //printf("Left ADC: %d; \t Right ADC: %d.\n", adc_left_raw[1][0], adc_right_raw[1][0]);
    //printf("Left ADC: %d\n", adc_left_raw[1][0]);
}

/**
 * @brief Read the current encoder pulse counts for both wheels.
 *
 * @param[out] pL Pointer where the left encoder pulse count is stored.
 * @param[out] pR Pointer where the right encoder pulse count is stored.
 */
void wheel_GetEndoderPulses( int *pL, int *pR )
{
    pcnt_unit_get_count(pcnt_left_unit, pL);
    pcnt_unit_get_count(pcnt_right_unit, pR);
    //printf("Left encoder: %d\tRight encoder: %d\r\n", cur_left_pulse_count, cur_right_pulse_count);
}