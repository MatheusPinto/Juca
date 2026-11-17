/**
 * @file      diff_drive_task.c
 * @brief     Step-by-step example: driving the Juca robot with the `diff_drive` component from the serial console.
 * @details   This module implements the main control task (`diff_drive_ctrl_task`) and an asynchronous
 *            interactive console task (`console_input_task`). It initializes the hardware peripherals
 *            (MCPWM, Quadrature Encoders, ADC Oneshot), sets up the differential drive controller
 *            (kinematics, acceleration ramp and overcurrent protection), executes automatic max-speed
 *            calibration, and handles non-blocking velocity commands alongside automatic safety fault monitoring.
 *
 * ============================================================================
 *  HOW TO READ THIS FILE (for students)
 * ============================================================================
 *  The file is written to be read from top to bottom, like a tutorial. Every block of code has a
 *  comment that says WHAT it does and, more importantly, WHY it is there. The order of the file is:
 *
 *    1. Includes and global variables.
 *    2. `init_usb_cdc_stdio()`    - makes `printf()` / `fgets()` work over the USB cable.
 *    3. `parse_twist_line()`      - turns the text "0.2 0.5" into two numbers.
 *    4. `console_input_task()`    - the task that reads what you type.
 *    5. The speed PID             - the small controller that makes the measured speed match the command.
 *    6. `diff_drive_ctrl_task()`  - the main task. It has 5 steps:
 *         Step 1  create the ADC (the "current meter" of the motors),
 *         Step 2  create the two wheels (PWM, encoder, current sensor),
 *         Step 3  fill in the configuration of the differential drive and start it,
 *         Step 4  calibrate (the robot moves by itself for 4 seconds!),
 *         Step 5  the endless loop: read commands, recover from faults, read the odometry,
 *                 run the speed PID and print the result.
 *
 * ============================================================================
 *  HOW TO RUN IT
 * ============================================================================
 *  1. In `main/main.c`, enable `#define DIFF_DRIVE_TASK` (remove the `//`), and comment out the other
 *     examples so that their log lines do not mix with the ones of this program.
 *  2. Put the robot on the floor with at least 2 meters of free path in front of it (or lift the wheels
 *     off the ground). The calibration in Step 4 makes both wheels spin at the maximum allowed duty
 *     (87.5% with `JUCA_MOTOR_MAX_POWER_TICKS` of juca_platform.h) for 4 seconds.
 *  3. Build, flash and open the serial monitor. Wait for the calibration to finish. Then type a command
 *     and press ENTER. A command is two numbers separated by a space: "<linear_x> <angular_z>".
 *
 *       0.2 0      -> go forward at 0.2 m/s, no rotation
 *      -0.2 0      -> go backward at 0.2 m/s
 *       0 1        -> spin in place, counter-clockwise (to the left), at 1 rad/s
 *       0.3 0.5    -> move forward at 0.3 m/s while turning left (a curve)
 *       0 0        -> stop
 *
 *     `linear_x` is in meters per second (m/s) and `angular_z` is in radians per second (rad/s).
 *     A line that is not two valid numbers is treated as "0 0" (stop), which is the safest reaction.
 *
 * ============================================================================
 *  THE BIG PICTURE: WHO TALKS TO WHOM
 * ============================================================================
 *
 *    you (serial monitor)
 *          |  type "0.2 0" + ENTER
 *          v
 *    console_input_task          priority 1 (lowest): only waits for text
 *          |  s_command_queue    a mailbox that holds up to 5 commands
 *          v
 *    diff_drive_ctrl_task        THIS file's main task: sets everything up, then loops at 20 Hz
 *          |  diff_drive_set_twist() / diff_drive_get_twist() / diff_drive_clear_fault()
 *          v
 *    diff_drive component        starts its own two tasks inside `diff_drive_init()`:
 *          |                       - diff_control (priority 4, every 20 ms): ramp + inverse kinematics + PWM
 *          |                       - diff_safety  (priority 5, every 50 ms): reads the motor currents
 *          v
 *    wheel (left) / wheel (right)  ->  motor driver (PWM), encoder (pulse counter), current sensor (ADC)
 *
 *  Notice that THIS file never sets a PWM value by hand. It only says "where do I want to go" (a velocity
 *  command, called a "twist") and "how fast did I really go" (odometry). The `diff_drive` component does the
 *  rest. That separation is the whole point of using a library.
 *
 *  The `diff_drive` component is OPEN LOOP: it turns a speed into motor power and never checks the result.
 *  This file closes the loop on top of it (see "SPEED PID" below): every 50 ms it compares the speed you
 *  asked for with the speed the encoders measured, and corrects the command it sends to the library.
 *
 * @date      Jan 2025 / Refactored
 * @author    Matheus
 */

/* ============================================================================
 *                                  INCLUDES
 * ==========================================================================*/

/* Standard C library: printf/sscanf/fgets/setvbuf (stdio), strcspn (string), isfinite (math). */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* `sdkconfig.h` holds every CONFIG_* option chosen in `idf.py menuconfig`. We use it below to find out
 * whether the console of this board is the USB cable (USB-Serial-JTAG) or a classic UART. */
#include "sdkconfig.h"

/* FreeRTOS: the real-time operating system that runs our tasks.
 *   task.h   -> create tasks, delays, task notifications
 *   queue.h  -> queues (mailboxes) used to pass commands between tasks */
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ESP-IDF logging (ESP_LOGI / ESP_LOGW / ESP_LOGE) and GPIO/UART driver headers. */
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/gpio_types.h"

/* ADC "oneshot" driver: reads the analog voltage of the motor current sensors, one sample at a time. */
#include "esp_adc/adc_oneshot.h"

/* The ESP32-S3 can talk to the computer through the USB cable itself (USB-Serial-JTAG) or through a UART
 * chip. Each one has its own driver header, so we pick the right one at COMPILE time with #if. */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#else
#include "driver/uart_vfs.h"
#endif

/* Our own components:
 *   diff_drive.h      -> the differential drive library (twist commands, odometry, safety)
 *   diff_drive_task.h -> the declaration of the task defined in this file
 *   juca_board.h      -> pins, ADC channels and fixed parameters of the Juca board
 *   juca_platform.h   -> chassis and motor parameters of the tested platform (they change with the hardware) */
#include "diff_drive.h"
#include "diff_drive_task.h"
#include "juca_board.h"
#include "juca_platform.h"

/**
 * @brief Logging tag used across ESP-IDF system log macros.
 *
 * @details Every `ESP_LOGI(TAG, ...)` line printed on the monitor starts with this name, so you can tell
 *          which part of the program wrote it (for example "I (3120) diff_drive_task: ...").
 */
static const char *TAG = "diff_drive_task";

/* Pins, ADC channels and the current conversion factor of the board: see juca_board.h.
 * Geometry, encoder resolution and motor power cap of the tested platform: see juca_platform.h. */

/* ============================================================================
 *                                 SPEED PID
 * ==========================================================================*/

/**
 * @brief 1 = closed loop (the speed PID corrects the command), 0 = open loop (the command goes straight
 *        to the library, as in the first version of this example).
 *
 * @details Run the same commands with 0 and with 1 and compare the `[Odometry]` lines: that difference is
 *          what a feedback controller buys you.
 */
#define SPEED_PID_ENABLE              1

/**
 * @name Gains of the two controllers (one for `linear_x`, one for `angular_z`)
 * @brief Starting values chosen with a simple simulation of the motors. They were NOT tuned on the robot.
 *
 * @details The controller output is a CORRECTION added to the command, in the same unit as the command.
 *          - KP: correction per unit of error, right now. Higher reacts faster; too high oscillates.
 *          - KI: correction per unit of error per second. It is what removes the steady error (for example
 *            the dead zone of the motors); too high overshoots.
 *          - KD: correction per unit of error change per second. 0 here: the encoder speed is noisy at
 *            20 Hz and a derivative would amplify that noise.
 * @{
 */
#define SPEED_PID_LINEAR_KP           0.5f    /**< (m/s) per (m/s). */
#define SPEED_PID_LINEAR_KI           2.0f    /**< (m/s) per (m/s * s). */
#define SPEED_PID_LINEAR_KD           0.0f    /**< (m/s) per (m/s / s). */
#define SPEED_PID_ANGULAR_KP          0.5f    /**< (rad/s) per (rad/s). */
#define SPEED_PID_ANGULAR_KI          2.0f    /**< (rad/s) per (rad/s * s). */
#define SPEED_PID_ANGULAR_KD          0.0f    /**< (rad/s) per (rad/s / s). */
/** @} */

/** @brief Largest correction the linear controller may add to the command, in m/s (either sign). */
#define SPEED_PID_LINEAR_LIMIT_M_S    0.4f
/** @brief Largest correction the angular controller may add to the command, in rad/s (either sign). */
#define SPEED_PID_ANGULAR_LIMIT_RAD_S 2.5f

/** @brief Linear acceleration limit, in m/s^2: the speed changes by at most this much every second. */
#define ACCEL_LINEAR_M_S2             0.5f
/** @brief Angular acceleration limit, in rad/s^2. */
#define ACCEL_ANGULAR_RAD_S2          1.5f

#if SPEED_PID_ENABLE   /* The controller code below is only compiled when the PID is in use. */

/**
 * @brief State of one PID controller.
 *
 * @details The gains and the limit are set once; `integral` and `previous_error` are the memory of the
 *          controller between two calls.
 */
typedef struct {
    float kp;               /**< Proportional gain. */
    float ki;               /**< Integral gain, per second. */
    float kd;               /**< Derivative gain, times seconds. */
    float output_limit;     /**< The output, and the integral part alone, stay within +/- this value. */
    float integral;         /**< Accumulated integral part of the output (already multiplied by ki). */
    float previous_error;   /**< Error of the previous call, for the derivative. */
} speed_pid_t;

/**
 * @brief Forgets the past of a controller (used when the robot stops and after a fault).
 *
 * @param[in,out] pid Controller to reset.
 */
static void speed_pid_reset(speed_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

/**
 * @brief One step of the PID: from the error to the correction.
 *
 * @details output = kp * error + integral + kd * (error - previous_error) / dt, where the integral grows
 *          by ki * error * dt at every call.
 *
 *          The integral is limited to +/- `output_limit` ("anti-windup"): if the wheels cannot deliver
 *          what is asked (a blocked wheel, a command above the top speed) the error never goes to zero,
 *          and without the limit the integral would grow without end and take a long time to come back.
 *
 * @param[in,out] pid          Controller.
 * @param[in]     error        Wanted value minus measured value.
 * @param[in]     delta_time_s Time since the previous call, in seconds (> 0).
 * @return float Correction to add to the command, within +/- `output_limit`.
 */
static float speed_pid_step(speed_pid_t *pid, float error, float delta_time_s)
{
    pid->integral += pid->ki * error * delta_time_s;
    if (pid->integral > pid->output_limit)  { pid->integral = pid->output_limit; }
    if (pid->integral < -pid->output_limit) { pid->integral = -pid->output_limit; }

    float output = pid->kp * error + pid->integral + pid->kd * (error - pid->previous_error) / delta_time_s;
    pid->previous_error = error;

    if (output > pid->output_limit)  { output = pid->output_limit; }
    if (output < -pid->output_limit) { output = -pid->output_limit; }
    return output;
}

/**
 * @brief Moves a value towards a target by at most `max_step` (the acceleration ramp of the reference).
 *
 * @param[in] current  Present value.
 * @param[in] target   Value to reach.
 * @param[in] max_step Largest change allowed in this call (>= 0).
 * @return float New value.
 */
static float ramp_towards(float current, float target, float max_step)
{
    float delta = target - current;
    if (delta > max_step)  { delta = max_step; }
    if (delta < -max_step) { delta = -max_step; }
    return current + delta;
}

/**
 * @brief Brings a command inside the speeds the wheels can really reach, keeping its direction.
 *
 * @details The faster wheel of a twist (v, w) turns at |v| + |w| * L/2 (in m/s at the wheel surface).
 *          The most a wheel can do is `max_wheel_rad_s * r * (duty cap / full duty)`. If the command asks
 *          for more, BOTH components are multiplied by the same factor: same curve, lower speed. This is
 *          the same rule the library applies inside (`scale_twist_to_wheel_limit` in diff_drive.c).
 *
 *          WHY HERE TOO: the library limits what it SENDS to the motors, but not the reference of this
 *          PID. Without this step, a command of 50 m/s would make the reference ramp up to 50 m/s (it
 *          only stops there), the error would stay large forever, and after a "0 0" the reference would
 *          need 100 s (50 m/s at 0.5 m/s^2) to come back down: the robot would keep running at full
 *          speed for all that time. This is the "wind-up" of the reference.
 *
 * @param[in,out] twist           Command to limit.
 * @param[in]     usable_wheel_m_s Fastest speed a wheel surface can reach, in m/s (> 0).
 * @return true if the command was reduced.
 */
static bool limit_to_reachable(diff_drive_twist_t *twist, float usable_wheel_m_s)
{
    const float demand_m_s = fabsf(twist->linear_x) + fabsf(twist->angular_z) * JUCA_HALF_TRACK_WIDTH_M;
    if (demand_m_s <= usable_wheel_m_s)
    {
        return false;
    }
    const float scale = usable_wheel_m_s / demand_m_s;
    twist->linear_x *= scale;
    twist->angular_z *= scale;
    return true;
}

/**
 * @brief Turns a typed command into the target of the PID reference, and says so when it was reduced.
 *
 * @param[in]  command          Command as typed.
 * @param[out] target           Command inside the reachable speeds.
 * @param[in]  usable_wheel_m_s See @ref limit_to_reachable.
 */
static void set_pid_target(const diff_drive_twist_t *command, diff_drive_twist_t *target, float usable_wheel_m_s)
{
    *target = *command;
    if (limit_to_reachable(target, usable_wheel_m_s))
    {
        ESP_LOGW(TAG, "Command above what the wheels can reach: limited to %.3f m/s, %.3f rad/s.",
                 target->linear_x, target->angular_z);
    }
}

#endif /* SPEED_PID_ENABLE */

/* ============================================================================
 *                    COMMUNICATION AND CONSOLE GLOBALS
 * ==========================================================================*/

/**
 * @brief Global queue handle for passing velocity commands (`diff_drive_twist_t`)
 *        from the console reader task to the main control task.
 *
 * @details A FreeRTOS queue is a thread-safe mailbox. One task puts items in (`xQueueSend`) and another task
 *          takes them out (`xQueueReceive`), in the same order, without the two ever touching the same
 *          memory at the same time. That is the safe way to hand data from one task to another.
 *          The queue is created at the beginning of `diff_drive_ctrl_task()`; until then the handle is NULL.
 */
static QueueHandle_t s_command_queue = NULL;

/**
 * @brief Configures standard I/O streams for non-buffered, truly blocking read operations.
 *
 * @details By default, the ESP-IDF standard console VFS operates in non-blocking mode,
 *          causing functions like `fgets()` or `scanf()` to return immediately with an empty string
 *          when no input is available. This helper initializes the underlying hardware driver
 *          (USB-Serial JTAG or UART) and installs it into the VFS layer with blocking line-ending
 *          configurations.
 *
 *          "VFS" means "virtual file system": ESP-IDF lets you treat the serial port as if it were a file,
 *          so `printf()` writes to it and `fgets(..., stdin)` reads from it. This function only prepares that
 *          "file" so that reading from it WAITS until you press ENTER.
 *
 * @note Automatically detects target configurations via `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`
 *       at compile time.
 */
static void init_usb_cdc_stdio(void)
{
    /* `_IONBF` = "no buffering". Characters are sent/received immediately instead of waiting in a buffer
     * until a full line is ready. Without it, text printed with `printf()` could appear late on the monitor. */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    /* ---- Case A: the console is the USB cable of the ESP32-S3 itself ---- */

    /* Install the USB-Serial-JTAG driver with its default settings (buffer sizes etc.).
     * ESP_ERROR_CHECK stops the program with an error message if the call fails. */
    usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

    /* Connect the driver to the VFS, which is what makes `fgets()` wait for data instead of returning at once. */
    usb_serial_jtag_vfs_use_driver();

    /* Line endings. When you press ENTER the terminal sends a CR character ('\r'); we ask the VFS to turn
     * it into the '\n' that `fgets()` expects. In the opposite direction, every '\n' we print is sent as
     * CR+LF ("\r\n"), so the next line starts at the left edge of the terminal instead of "staircasing". */
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
#else
    /* ---- Case B: the console is a classic UART (a USB-to-serial chip on the board) ---- */

    /* Install the UART driver only if nobody did it before (installing twice is an error). */
    if (!uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM))
    {
        /* Serial port settings: speed from menuconfig, 8 data bits, no parity, 1 stop bit ("8N1"),
         * the default clock source. */
        const uart_config_t uart_config = {
            .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .source_clk = UART_SCLK_DEFAULT,
        };
        /* 256 = size in bytes of the receive buffer; the other zeros mean: no transmit buffer, no event queue. */
        ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));
    }

    /* Same two ideas as in Case A: blocking reads through the VFS, and CR/CRLF line endings. */
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
#endif
}

/**
 * @brief Parses a console line of the form "<linear_x> <angular_z>".
 *
 * @details `sscanf("%f")` also accepts "nan" and "inf", which are not valid velocities, so non-finite
 *          values are rejected here.
 *
 *          `sscanf` works like `scanf`, but it reads from a string instead of the keyboard. The format
 *          `"%f %f"` means "a decimal number, some spaces, another decimal number". It returns how many
 *          numbers it managed to read, so a result different from 2 means the line was malformed
 *          (for example "abc", "0.2" with a missing value, or an empty line).
 *
 * @param[in]  line      NUL-terminated input line.
 * @param[out] out_twist Receives the parsed command; left untouched when parsing fails.
 * @return true  Both values were parsed and are finite.
 * @return false The line is malformed or contains NaN/Inf.
 */
static bool parse_twist_line(const char *line, diff_drive_twist_t *out_twist)
{
    /* Read into local variables first, so `out_twist` is never half-written when the line is bad. */
    float linear_x = 0.0f;
    float angular_z = 0.0f;

    /* Reject the line if (a) we did not get exactly two numbers, or (b) any of them is NaN or infinity.
     * `isfinite()` is true for every normal number and false for NaN ("not a number") and +/-infinity. */
    if (sscanf(line, "%f %f", &linear_x, &angular_z) != 2 || !isfinite(linear_x) || !isfinite(angular_z))
    {
        return false;
    }

    /* Only now that we know the values are good, copy them to the caller. */
    out_twist->linear_x = linear_x;
    out_twist->angular_z = angular_z;
    return true;
}

/**
 * @brief FreeRTOS task dedicated to reading user input from the console asynchronously.
 *
 * @details Suspends execution until notified by the control task. Upon notification,
 *          it prompts the operator for target linear and angular velocities (`linear_x` and `angular_z`),
 *          blocks until input is submitted via stdin, parses the input, and pushes the parsed command
 *          into `s_command_queue`. The control task notifies it again after consuming each command, so
 *          any number of commands can be typed. A malformed or non-finite line is sent as a stop
 *          command (0 0); an end-of-file/read error sends nothing and keeps waiting for a line.
 *
 *          WHY A SEPARATE TASK? `fgets()` BLOCKS: it does not return until you press ENTER, which can take
 *          minutes. If the main control task called it directly, the main loop (odometry, fault handling)
 *          would freeze while it waited. So the waiting is done here, in its own low-priority task, and the
 *          result is delivered to the main task through a queue. This is a very common pattern in embedded
 *          systems: "one task per thing that waits".
 *
 *          THE HANDSHAKE. The task only prompts when the main task "taps its shoulder" with a task
 *          notification (`xTaskNotify`). The main task does that when it is ready for a new command.
 *          This keeps the prompt from appearing in the middle of the calibration logs.
 *
 * @param[in] arg Pointer to task parameter list (unused).
 */
static void console_input_task(void *arg)
{
    /* Tell the compiler we know `arg` is unused (avoids a warning). */
    (void)arg;

    /* Buffer for one typed line (at most 63 characters + the terminating '\0'). */
    char line[64];

    /* The command that will be sent to the main task. `{0}` starts both fields at 0.0 (= "stop"). */
    diff_drive_twist_t new_command = {0};

    /* Initialize USB Serial/JTAG VFS drivers */
    init_usb_cdc_stdio();

    printf("\n--- ESP32-S3 Console Ready ---\n");

    /* Receives the notification value. We do not use the value (the main task sends "no value"),
     * but the function needs a place to store it. */
    uint32_t notified_bits;

    while (1)
    {
        /* Wait for notification from main controller before asking for input.
         *   - First argument  (0x00):       do not clear any notification bit when entering the wait.
         *   - Second argument (0xFFFFFFFF): clear all bits when leaving it, so the next wait really waits.
         *   - portMAX_DELAY: wait forever. The task sleeps and uses no CPU until the notification arrives. */
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &notified_bits, portMAX_DELAY);

        printf("Enter target velocity (linear_x angular_z): \n");
        fflush(stdout); /* Flush output buffer immediately */

        /* Block until the user submits input with ENTER. End-of-file or an I/O error yields no line:
         * there is no command to send, and resending the previous one would make the control task apply
         * it again (even to clear a fault) -- so keep waiting instead. */
        while (fgets(line, sizeof(line), stdin) == NULL)
        {
            /* Nothing to read right now: sleep 100 ms (so other tasks can run) and try again. */
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        /* Strip trailing carriage return and newline characters.
         * `strcspn(line, "\r\n")` returns the position of the first '\r' or '\n' in the text; writing a 0
         * (the end-of-string marker) there cuts the line right before the line ending. */
        line[strcspn(line, "\r\n")] = 0;

        if (parse_twist_line(line, &new_command))
        {
            /* Good line: echo back what we understood, so you can confirm it on the monitor. */
            printf("Accepted target -> linear_x = %.2f, angular_z = %.2f\n",
                   new_command.linear_x, new_command.angular_z);
        }
        else
        {
            /* Bad line: do NOT guess what the person meant. Send "0 0" (stop) and say so. */
            new_command.linear_x = 0;
            new_command.angular_z = 0;
            printf("Invalid input! Received: '%s'\n", line);
        }

        /* Hand the command to the main task. The queue COPIES the structure, so reusing `new_command`
         * on the next lap of the loop is safe. `portMAX_DELAY` means "if the queue is full, wait". */
        if (s_command_queue != NULL)
        {
            xQueueSend(s_command_queue, &new_command, portMAX_DELAY);
        }

        /* A short pause (50 ms) so that a burst of input cannot flood the main task. */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ============================================================================
 *                          TASK IMPLEMENTATIONS
 * ==========================================================================*/

/**
 * @brief Main differential drive management task.
 *
 * @details Executes setup routines, initializes hardware interfaces, calibrates motor thresholds,
 *          and enters a 20 Hz periodic loop. In each cycle, it:
 *          1. Monitors and resolves safety fault states.
 *          2. Consumes non-blocking velocity command updates from the console queue.
 *          3. Computes forward kinematics odometry and outputs real-time telemetry.
 *
 *          `portTASK_FUNCTION(name, arg)` is just a FreeRTOS macro that expands to
 *          `void name(void *arg)`: the standard shape of a task function. A task function normally
 *          NEVER returns; it ends with an endless loop (see Step 5).
 *
 * @param[in] arg FreeRTOS task argument (unused).
 */
portTASK_FUNCTION(diff_drive_ctrl_task, arg)
{
    /* ----------------------------------------------------------------
     * Step 0: the mailbox and the console task
     * ---------------------------------------------------------------- */

    /* Create the queue that carries commands from the console task to this task.
     * It holds up to 5 items, and each item is one `diff_drive_twist_t` (two floats = 8 bytes). */
    s_command_queue = xQueueCreate(5, sizeof(diff_drive_twist_t));
    if (s_command_queue == NULL)
    {
        /* Not enough memory to create the queue: nothing else can work, so log the error and
         * delete THIS task (passing NULL means "the task that is calling"). */
        ESP_LOGE(TAG, "Failed to create command queue!");
        vTaskDelete(NULL);
        return;
    }

    TaskHandle_t console_input_task_handle = NULL;
    /* Create console input task with priority 1 and adequate stack depth.
     * Arguments: function, a name (shows up in debug tools), stack size in bytes, parameter (none),
     * priority (higher number = more important), and a place to store the new task's handle.
     * We keep the handle because we need it later to send notifications to this task. */
    xTaskCreate(console_input_task, "console_input", 3072, NULL, 1, &console_input_task_handle);

    /* ----------------------------------------------------------------
     * 1. Initialize Shared ADC Oneshot Unit
     * ---------------------------------------------------------------- */
    /* WHAT: the ADC (analog-to-digital converter) is the part of the chip that measures a voltage and
     *       gives back a number. Each motor has a current sensor that turns "motor current" into a voltage,
     *       so reading the ADC tells us how much current the motor is drawing (a stalled motor draws a lot).
     * WHY HERE: the ADC unit is created ONCE and then shared by both wheels (each wheel just uses a
     *       different channel of it). That is why we create it here instead of inside each wheel. */
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        /* Which ADC unit. The board header picks ADC1 on purpose: ADC2 cannot be used while Wi-Fi is on. */
        .unit_id = JUCA_ADC_UNIT_CURRENT,
        /* Default clock source for the converter. */
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));

    /* Settings of each ADC channel (the same for the left and the right sensor). */
    adc_oneshot_chan_cfg_t adc_channel_config = {
        /* Resolution: the default of the chip (12 bits on the ESP32-S3, so readings go from 0 to 4095). */
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        /* Attenuation 12 dB lets the ADC measure up to about 3.1 V. Our sensor output stays under that. */
        .atten = ADC_ATTEN_DB_12,
    };
    /* Tell the driver which channel belongs to which sensor (the channel numbers are in juca_board.h). */
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JUCA_ADC_CHANNEL_CURRENT_LEFT, &adc_channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JUCA_ADC_CHANNEL_CURRENT_RIGHT, &adc_channel_config));

    /* ----------------------------------------------------------------
     * 2. Configure Wheel Instances
     * ---------------------------------------------------------------- */
    /* A "wheel" (see components/wheel) bundles everything that belongs to ONE side of the robot:
     *   - the motor driver, fed by two PWM signals (one per H-bridge input, so the motor can go both ways),
     *   - the quadrature encoder, which counts pulses as the wheel turns (A and B signals give direction),
     *   - the current sensor, read through the ADC channel.
     * Here we only FILL IN the configuration structures; `wheel_create()` below does the real work. */
    wheel_config_t left_config = {
        /* The two GPIO pins that drive the left motor's H-bridge (pins live in juca_board.h). */
        .pwm_a_gpio = JUCA_PIN_MOTOR_LEFT_PWM_A,
        .pwm_b_gpio = JUCA_PIN_MOTOR_LEFT_PWM_B,
        /* Which MCPWM peripheral group generates the PWM. The ESP32-S3 has two; both wheels use group 0. */
        .mcpwm_group_id = 0,
        /* Duty cap of this motor, in PWM ticks (full scale is 400). 350 means "never more than 87.5%". */
        .max_power_ticks = JUCA_MOTOR_MAX_POWER_TICKS,
        /* The two encoder signals of this wheel. */
        .encoder_a_gpio = JUCA_PIN_ENCODER_LEFT_A,
        .encoder_b_gpio = JUCA_PIN_ENCODER_LEFT_B,
        /* The shared ADC unit from Step 1 and the channel of THIS wheel's current sensor.
         * (Passing NULL as `adc_handle` would simply disable current sensing for this wheel.) */
        .adc_handle = adc_handle,
        .adc_channel = JUCA_ADC_CHANNEL_CURRENT_LEFT,
    };

    /* Same thing for the right wheel: only the pins and the ADC channel change. */
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

    /* A `wheel_handle_t` is an "opaque handle": a ticket that we pass to the wheel functions to say
     * WHICH wheel we mean. We cannot (and do not need to) look inside it. */
    wheel_handle_t left_wheel = NULL;
    wheel_handle_t right_wheel = NULL;

    /* Allocate the hardware (PWM, pulse counter) and get the handles. The motors start BRAKED (stopped). */
    ESP_ERROR_CHECK(wheel_create(&left_config, &left_wheel));
    ESP_ERROR_CHECK(wheel_create(&right_config, &right_wheel));

    ESP_LOGI(TAG, "Wheel instances created successfully.");

    /* ----------------------------------------------------------------
     * 3. Populate Differential Drive Configuration Structure
     * ---------------------------------------------------------------- */
    /* Now we tell the `diff_drive` library about the robot. The library copies this structure, so it
     * could even be discarded after `diff_drive_init()`. Every field has a unit in its name:
     *   _m = meters, _m_s = meters per second, _rad_s = radians per second, _m_s2 / _rad_s2 = acceleration,
     *   _ms = milliseconds. */
    diff_drive_config_t drive_config = {
        /* The two wheels created in Step 2. */
        .left_wheel = left_wheel,
        .right_wheel = right_wheel,

        /* ---- Geometry (needed to convert between robot speed and wheel speed) ---- */

        /* HALF of the distance between the two wheels (L/2). 0.15 m means the wheels are 0.30 m apart. */
        .half_track_width_m = JUCA_HALF_TRACK_WIDTH_M,
        /* Radius of the wheels. */
        .wheel_radius_m = JUCA_WHEEL_RADIUS_M,
        /* Encoder resolution: how many counts the encoder gives per full turn of the WHEEL
         * (after the gearbox). Each wheel has its own value because the two encoders could differ. */
        .left_encoder_cpr = JUCA_ENCODER_CPR_LEFT,
        .right_encoder_cpr = JUCA_ENCODER_CPR_RIGHT,

        /* ---- Speed limits ---- */

        /* Software speed limits (a policy: "this robot must never go faster than this").
         * A command above them is reduced to the limit. 0 would disable the limit. */
        .max_linear_velocity_m_s =  0.0f,
        .max_angular_velocity_rad_s = 0.0f,
        /* First estimate of how fast one wheel spins at 100% duty. It sets the scale "how many PWM ticks
         * for 1 rad/s". The calibration in Step 4 REPLACES it with the measured value. Until then it also
         * limits the top speed: 20 rad/s x 0.03 m x (350/400) is about 0.52 m/s. */
        .max_wheel_rad_s = 20.0f,

        /* ---- Overcurrent (stall) protection ---- */

        /* If the raw ADC reading of a wheel stays above 3000 counts, the safety task stops the robot.
         * With the Juca factor below this is about 0.8 A. Setting 0 would turn the protection off. */
        .max_adc_raw_threshold = 3000,
        /* Conversion factor from ADC counts to milliamps, measured on the Juca. It is only used by
         * `diff_drive_get_current_ma()`; the safety check above works directly in raw counts. */
        .current_ma_per_count = JUCA_CURRENT_MA_PER_COUNT,
        /* The safety task looks at the currents every 50 ms. */
        .monitor_period_ms = 50,
        /* The safety task must be MORE important than the control task (5 > 4 below). The library refuses
         * to start otherwise, because a busy control task must never be able to starve the safety check. */
        .safety_task_priority = 5,
        /* Stack size of that task, in bytes (four times FreeRTOS' minimum). */
        .safety_task_stack_size = configMINIMAL_STACK_SIZE * 4,

        /* ---- Smooth motion (acceleration ramp) ---- */

        /* The speed is not allowed to jump: it changes by at most 0.5 m/s every second (linear) and
         * 1.5 rad/s every second (angular). Going from 0 to 0.4 m/s therefore takes 0.8 s, which protects
         * the gearbox, the battery and the traction of the wheels. A value <= 0 would disable the ramp.
         *
         * With the speed PID this ramp is done HERE, in this task (on the reference the PID follows), so
         * the ramp of the library is turned off: two ramps in a row would delay the corrections of the
         * PID and make it overshoot. */
#if SPEED_PID_ENABLE
        .max_accel_linear_m_s2 = 0.0f,
        .max_accel_angular_rad_s2 = 0.0f,
#else
        .max_accel_linear_m_s2 = ACCEL_LINEAR_M_S2,
        .max_accel_angular_rad_s2 = ACCEL_ANGULAR_RAD_S2,
#endif

        /* ---- The control task that lives inside the library ---- */

        /* It runs every 20 ms (50 times per second) and sends the PWM values to the motors. */
        .control_period_ms = 20,
        /* Priority 4: lower than the safety task (5), as required. */
        .control_task_priority = 4,
        /* Stack size in bytes. */
        .control_task_stack_size = 3072
    };

    /* Initialize differential drive controller.
     * This validates the configuration and starts the library's two background tasks
     * (`diff_control` and `diff_safety`). From now on the robot is "armed", but it stays stopped until
     * we send a command. ESP_ERROR_CHECK stops the program if the configuration is invalid. */
    ESP_ERROR_CHECK(diff_drive_init(&drive_config));

    /* ----------------------------------------------------------------
     * 4. Perform Dynamic Maximum Speed Calibration
     * ---------------------------------------------------------------- */
    /* WHY CALIBRATE? Two motors of the same model are never identical, and the battery voltage changes.
     * If we send the same power to both, one wheel spins faster and the robot drifts to one side.
     * The calibration makes both wheels spin at the SAME power for a few seconds and measures their real
     * speeds with the encoders. From that, the library computes:
     *   - a "trim factor" that slows down the faster wheel, so equal commands drive straight, and
     *   - the true maximum wheel speed (`max_wheel_rad_s`), so that "0.4 m/s" means about 0.4 m/s.
     * The results live only in RAM: they are lost at every reset, so we calibrate again at every boot. */
    ESP_LOGI(TAG, "Starting calibration. Ensure robot path is clear!");

    /* Give the person 2 seconds to step away (or lift the robot) before it starts moving. */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* ROBOT MOVES HERE: both wheels at the same, maximum allowed duty (350 of 400 ticks = 87.5% here)
     * for 4000 ms (the allowed range is 0.5 s to 10 s).
     * While this runs, the safety task is paused: there is NO overcurrent protection until it returns.
     * The function returns the measured maximum wheel speed through the pointer we pass (`&...`). */
    float measured_max_rad_s = 0.0f;
    esp_err_t calibration_status = diff_drive_calibrate_max_speed(4000, &measured_max_rad_s);
    if (calibration_status == ESP_OK)
    {
        /* The value is what the wheel would reach at 100% duty. On the Juca it came out around 26 to 28 rad/s
         * in real runs, and it changes a little from boot to boot (battery charge, floor, temperature). */
        ESP_LOGI(TAG, "Auto-calibration successful! Max wheel speed: %.2f rad/s", measured_max_rad_s);
    }
    else
    {
        /* The calibration can fail (for example an encoder not counting, or a fault already latched).
         * The robot still works, using the estimate from the configuration and no trim. */
        ESP_LOGW(TAG, "Calibration skipped or failed (%s). Using fallback configuration.", esp_err_to_name(calibration_status));
    }

    /* The command that is currently being executed. It starts at (0, 0) = stopped. */
    diff_drive_twist_t command_twist = {0};

#if SPEED_PID_ENABLE
    /* The two speed controllers, and the REFERENCE they follow: the command after the acceleration ramp.
     * Following the ramped reference (instead of the raw command) keeps the error small while the robot
     * accelerates, so the integral does not build up during the ramp. */
    speed_pid_t linear_pid = {
        .kp = SPEED_PID_LINEAR_KP, .ki = SPEED_PID_LINEAR_KI, .kd = SPEED_PID_LINEAR_KD,
        .output_limit = SPEED_PID_LINEAR_LIMIT_M_S,
    };
    speed_pid_t angular_pid = {
        .kp = SPEED_PID_ANGULAR_KP, .ki = SPEED_PID_ANGULAR_KI, .kd = SPEED_PID_ANGULAR_KD,
        .output_limit = SPEED_PID_ANGULAR_LIMIT_RAD_S,
    };
    diff_drive_twist_t reference_twist = {0};

    /* Fastest speed a wheel surface can reach, in m/s: the calibrated wheel speed at 100% duty (or the
     * estimate of the configuration if the calibration failed) times the wheel radius, times the duty cap.
     * Commands are brought inside it before the ramp (see limit_to_reachable). */
    const float max_wheel_rad_s = (calibration_status == ESP_OK) ? measured_max_rad_s : drive_config.max_wheel_rad_s;
    const float usable_wheel_m_s = max_wheel_rad_s * JUCA_WHEEL_RADIUS_M *
                                   ((float)JUCA_MOTOR_MAX_POWER_TICKS / (float)WHEEL_PWM_DUTY_TICK_MAX);
    ESP_LOGI(TAG, "Speed PID on. Reachable: %.2f m/s straight, %.2f rad/s in place.",
             usable_wheel_m_s, usable_wheel_m_s / JUCA_HALF_TRACK_WIDTH_M);

    /* The command after limit_to_reachable(): what the reference ramps towards. */
    diff_drive_twist_t target_twist = {0};
#endif

    /* Request first target command from user console.
     * `xTaskNotify(..., eNoAction)` only WAKES the console task (it carries no value). The console task
     * then prints its prompt and waits for you to type. */
    xTaskNotify(console_input_task_handle, 0, eNoAction);

    /* Block execution until initial command is received.
     * `portMAX_DELAY` = wait as long as it takes. The robot stays stopped meanwhile. */
    xQueueReceive(s_command_queue, &command_twist, portMAX_DELAY);

    /* Register the first command. IMPORTANT: `diff_drive_set_twist()` does NOT move the motors directly.
     * It only checks the values, applies the speed limits and stores the result as the new TARGET. The
     * library's control task then ramps towards that target and drives the PWM. This call returns
     * immediately and is safe to call from any task.
     *
     * With the speed PID the command is not sent here: Part D of the loop sends the reference plus the
     * correction, starting from zero. */
#if SPEED_PID_ENABLE
    set_pid_target(&command_twist, &target_twist, usable_wheel_m_s);
#else
    ESP_ERROR_CHECK(diff_drive_set_twist(&command_twist));
#endif

    ESP_LOGI(TAG, "First valid command applied (Linear=%.3f, Angular=%.3f). Restoring log verbosity.",
             command_twist.linear_x, command_twist.angular_z);

    /* The console task only prompts when notified: ask for the next command right away */
    xTaskNotify(console_input_task_handle, 0, eNoAction);

    /* Where the odometry (the speed the robot REALLY has, measured by the encoders) will be stored. */
    diff_drive_twist_t estimated_twist = {0};

    ESP_LOGI(TAG, "Application loop (odometry + telemetry) running at 20 Hz...");

    /* ----------------------------------------------------------------
     * Timing variables for the main loop
     * ---------------------------------------------------------------- */

    /* The loop period: 50 ms, which is 20 Hz. `pdMS_TO_TICKS` converts milliseconds into RTOS "ticks"
     * (the RTOS clock; with the default setting one tick is 10 ms, so 50 ms is 5 ticks). */
    const TickType_t task_period_ticks = pdMS_TO_TICKS(50); // 20 Hz

    /* `last_wake_tick` is the reference for `vTaskDelayUntil()`: the moment this loop last "woke up". */
    TickType_t last_wake_tick = xTaskGetTickCount();

    /* `previous_tick` is the time of the previous odometry sample, used to compute the real elapsed time (dt). */
    TickType_t previous_tick = last_wake_tick;

    /* After a safety fault, wait at least 3 s between two recovery attempts. This stops a stalled robot
     * from being re-energized again and again in a tight loop. */
    const TickType_t fault_retry_cooldown_ticks = pdMS_TO_TICKS(3000);
    TickType_t last_fault_retry_tick = 0;

    /* ----------------------------------------------------------------
     * 5. Main Execution Loop
     * ---------------------------------------------------------------- */
    /* This loop runs forever, once every 50 ms. Each lap has up to four parts:
     *   A. if a safety fault is latched: try to recover, then skip the rest of the lap;
     *   B. measure the real time elapsed since the previous lap (dt);
     *   C. check whether you typed a new command, and register it;
     *   D. read the odometry and print it; then (D2) run the speed PID and send the corrected command.
     * `for ( ; ; )` is an endless loop, the same as `while (1)`. */
    for ( ; ; )
    {
        /* ---- Part A. Check safety fault status ---- */

        /* The `diff_safety` task latches a fault when the motor current stays too high (a stall, for
         * example when the robot is blocked against a wall). While it is latched the library keeps the
         * wheels braked and REFUSES new motion commands, until we explicitly clear the fault. */
        if (diff_drive_is_fault_active())
        {
            TickType_t now_tick = xTaskGetTickCount();

            /* Only try to recover if enough time has passed since the last attempt. Subtracting two tick
             * counters like this stays correct even when the counter wraps around. */
            if ((now_tick - last_fault_retry_tick) >= fault_retry_cooldown_ticks)
            {
                /* Ask the console task to prompt us. (If it is already waiting for a line, the extra
                 * notification is harmless.) */
                xTaskNotify(console_input_task_handle, 0, eNoAction);

                /* Non-blocking check (timeout 0): did you type something? */
                if (xQueueReceive(s_command_queue, &command_twist, 0) == pdTRUE)
                {
                    /* You typed a command, which we take as "yes, I looked at the robot, it is safe
                     * to go on". Release the latch... */
                    diff_drive_clear_fault();

                    /* ...and register the command again. Clearing the fault does NOT resume the previous
                     * motion (the target was discarded when the fault latched, on purpose), so the new
                     * command is what makes the robot move again. */
#if SPEED_PID_ENABLE
                    /* With the PID the command is not sent to the library here (it has no ramp of its own
                     * in this mode, so the robot would jump to the full command at once): it becomes the
                     * new target, and Part D2 ramps the reference up from zero. The controllers forget
                     * what they had accumulated while the wheel was blocked. */
                    set_pid_target(&command_twist, &target_twist, usable_wheel_m_s);
                    reference_twist = (diff_drive_twist_t){0};
                    speed_pid_reset(&linear_pid);
                    speed_pid_reset(&angular_pid);
                    ESP_LOGW(TAG, "Safety lock active! Cleared fault and set a new target twist.");
#else
                    esp_err_t resume_status = diff_drive_set_twist(&command_twist);
                    if (resume_status != ESP_OK)
                    {
                        ESP_LOGW(TAG, "Resume command rejected (%s) -- retrying after cooldown.",
                                 esp_err_to_name(resume_status));
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Safety lock active! Cleared fault and re-issued target twist.");
                    }
#endif

                    /* Remember when this attempt happened, to enforce the cooldown. */
                    last_fault_retry_tick = now_tick;

                    /* Ask for the next command */
                    xTaskNotify(console_input_task_handle, 0, eNoAction);
                }
            }

            /* While faulted we do not measure odometry. Restart the dt reference so that the first sample
             * after the recovery does not see the whole faulted period as elapsed time. (That first sample
             * still covers everything the wheels moved since the previous call, so read it with care.) */
            previous_tick = xTaskGetTickCount();

            /* Wait until the next 50 ms boundary, then go back to the top of the loop (`continue`),
             * skipping parts B, C and D. */
            vTaskDelayUntil(&last_wake_tick, task_period_ticks);
            continue;
        }

        /* ---- Part B. Calculate actual delta time (dt) in seconds ---- */

        /* The odometry needs the REAL time since the previous sample, not the nominal 50 ms, because
         * the loop can be delayed a little by other tasks. Speed = distance / time, so a wrong time gives
         * a wrong speed. The tick difference times the tick duration (ms per tick) gives milliseconds;
         * dividing by 1000 gives seconds. */
        TickType_t current_tick = xTaskGetTickCount();
        float delta_time_s = (float)(current_tick - previous_tick) * portTICK_PERIOD_MS / 1000.0f;
        previous_tick = current_tick;

        /* ---- Part C. Non-blocking check for updated velocity commands from console ---- */

        /* Timeout 0 means "do not wait": if the queue is empty we move on immediately, so the loop keeps
         * its 20 Hz rhythm even when you are not typing. */
        diff_drive_twist_t new_command_from_console;
        if (xQueueReceive(s_command_queue, &new_command_from_console, 0) == pdTRUE)
        {
            /* A new command arrived: keep a copy. In open loop it is given to the library at once as the
             * new target; with the speed PID, Part D sends it (ramped and corrected) on every lap. */
            command_twist = new_command_from_console;
#if SPEED_PID_ENABLE
            set_pid_target(&command_twist, &target_twist, usable_wheel_m_s);
#endif
#if !SPEED_PID_ENABLE
            esp_err_t set_twist_status = diff_drive_set_twist(&command_twist);
            if (set_twist_status != ESP_OK)
            {
                /* The library can refuse a command (for example if a fault became active meanwhile). */
                ESP_LOGW(TAG, "New command rejected: %s", esp_err_to_name(set_twist_status));
            }
#endif

            /* Ask for the next command */
            xTaskNotify(console_input_task_handle, 0, eNoAction);
        }

        /* ---- Part D. Retrieve platform odometry estimation via Forward Kinematics ---- */

        /* ODOMETRY = estimating how the robot is really moving. The library reads the two encoders,
         * checks how many counts each wheel advanced since the last call, converts that into distances
         * and divides by `delta_time_s` to get the speed of each wheel. From the two wheel speeds it
         * computes the speed of the whole robot: `linear_x` (average of both wheels) and `angular_z`
         * (how much faster the right wheel is than the left one, divided by the track width).
         * The guard avoids calling it with a zero or negative dt, which the library rejects. */
        if (delta_time_s > 0.0f)
        {
            esp_err_t odometry_status = diff_drive_get_twist(delta_time_s, &estimated_twist);
            if (odometry_status == ESP_OK)
            {
                /* Compare this line with the command you typed: with no load, the measured speed is
                 * usually close to the command at high speed and lower at low speed (the motors need a
                 * minimum duty to start turning). To also print the motor currents, call
                 * `diff_drive_get_current_ma(&left_ma, &right_ma)` here. */
                ESP_LOGI(TAG, "[Odometry] Linear X: %.3f m/s | Angular Z: %.3f rad/s",
                         estimated_twist.linear_x, estimated_twist.angular_z);

#if SPEED_PID_ENABLE
                /* ---- Part D2. Speed PID: correct the command with what was just measured ---- */

                /* 1. The reference moves towards your command (already brought inside the reachable speeds),
                 *    limited by the acceleration. */
                reference_twist.linear_x = ramp_towards(reference_twist.linear_x, target_twist.linear_x,
                                                        ACCEL_LINEAR_M_S2 * delta_time_s);
                reference_twist.angular_z = ramp_towards(reference_twist.angular_z, target_twist.angular_z,
                                                         ACCEL_ANGULAR_RAD_S2 * delta_time_s);

                diff_drive_twist_t output_twist = {0};
                if (reference_twist.linear_x == 0.0f && reference_twist.angular_z == 0.0f)
                {
                    /* 2a. Asked to stand still: send exactly zero (the library brakes the wheels) and clear
                     *     the controllers, so encoder noise at rest cannot slowly wind them up. */
                    speed_pid_reset(&linear_pid);
                    speed_pid_reset(&angular_pid);
                }
                else
                {
                    /* 2b. error = wanted - measured. What is sent is the reference (what open loop would
                     *     send) PLUS the correction of each controller. */
                    output_twist.linear_x = reference_twist.linear_x +
                        speed_pid_step(&linear_pid, reference_twist.linear_x - estimated_twist.linear_x, delta_time_s);
                    output_twist.angular_z = reference_twist.angular_z +
                        speed_pid_step(&angular_pid, reference_twist.angular_z - estimated_twist.angular_z, delta_time_s);
                }

                /* 3. Give the corrected command to the library. It can refuse it if a fault latched just
                 *    now; the next lap handles the fault in Part A. */
                esp_err_t set_twist_status = diff_drive_set_twist(&output_twist);
                if (set_twist_status != ESP_OK && !diff_drive_is_fault_active())
                {
                    ESP_LOGW(TAG, "Command rejected: %s", esp_err_to_name(set_twist_status));
                }

                /* Reference = what the robot should be doing now; Sent = what the PID asked the library for.
                 * Sent above Reference means the PID is pushing harder to make up for a slow robot. */
                ESP_LOGI(TAG, "[PID] Reference: %.3f m/s %.3f rad/s | Sent: %.3f m/s %.3f rad/s",
                         reference_twist.linear_x, reference_twist.angular_z,
                         output_twist.linear_x, output_twist.angular_z);
#endif
            }
        }

        /* Maintain 20 Hz periodic timing.
         * `vTaskDelayUntil` sleeps until `last_wake_tick + task_period_ticks` and updates `last_wake_tick`.
         * Unlike a plain `vTaskDelay(50 ms)`, it does not accumulate the time spent doing the work above,
         * so the loop really runs 20 times per second instead of drifting. */
        vTaskDelayUntil(&last_wake_tick, task_period_ticks);
    }
}