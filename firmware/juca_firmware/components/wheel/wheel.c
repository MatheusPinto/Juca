/**
 * @file    wheel.c
 * @brief   snake_case handle-based implementation for wheel motor, optional encoder, and optional ADC.
 *
 * @details Implements low-level control routines for H-Bridge motor driving (via MCPWM BDC),
 *          quadrature encoder pulse accumulation (via PCNT), and current sensor readings (via ADC Oneshot).
 *
 * @date    Sep 2025
 * @author  Matheus Leitezke Pinto
 */

#include "wheel.h"
#include <stdlib.h>
#include <math.h>
#include "esp_check.h"

/** @brief Logging tag used across ESP-IDF system log macros. */
static const char *TAG = "wheel";

/**
 * @brief Internal concrete structure representing the wheel context handle @ref wheel_handle_t.
 */
struct wheel_t {
    /* --- Peripherals --- */
    bdc_motor_handle_t motor;              /**< ESP-IDF BDC Motor driver handle based on MCPWM. */
    pcnt_unit_handle_t pcnt_unit;          /**< PCNT hardware unit handle (NULL if unused). */
    pcnt_channel_handle_t pcnt_channel_a;  /**< PCNT channel A handle. */
    pcnt_channel_handle_t pcnt_channel_b;  /**< PCNT channel B handle. */
    adc_oneshot_unit_handle_t adc_handle;  /**< Injected shared ADC unit handle (NULL if unused). */
    adc_channel_t adc_channel;             /**< Dedicated ADC channel mapped to motor current sensor. */

    /* --- Limits & Hardware Properties --- */
    uint32_t max_power_ticks;               /**< Maximum power input magnitude allowed for PWM duty mapping. */

    /* --- Current Motor State --- */
    int32_t current_power_ticks;           /**< Currently applied signed power command. */
};

/* ============================================================================
 *                     COMPILE-TIME STATIC ASSERTS
 * ==========================================================================*/

/* Verifies that PCNT limits and PWM configuration parameters are mathematically valid at compile time. */
static_assert(CONFIG_WHEEL_ENCODER_PCNT_HIGH_LIMIT > 0, "Encoder pulse overflow high limit must be bigger than zero!");
static_assert(WHEEL_ENCODER_PCNT_LOW_LIMIT < 0, "Encoder pulse overflow low limit must be less than zero!");
static_assert(CONFIG_WHEEL_TIMER_RESOLUTION_HZ > 0, "PWM timer resolution must be bigger than zero!");
static_assert(CONFIG_WHEEL_PWM_FREQ_HZ > 0, "PWM frequency must be bigger than zero!");

/* ============================================================================
 *                     PRIVATE HELPER / INLINE FUNCTIONS
 * ==========================================================================*/

/**
 * @brief Clamp raw PWM tick values to instance specific allowable boundaries.
 *
 * @param[in] wheel Target wheel handle.
 * @param[in] value_ticks Requested raw PWM value.
 * @return uint32_t Clamped value bounded within [0, max_power_ticks].
 */
static inline uint32_t clamp_pwm(wheel_handle_t wheel, uint32_t value_ticks)
{
    if (value_ticks > wheel->max_power_ticks) 
    {
        return wheel->max_power_ticks;
    }
    return value_ticks;
}

/**
 * @brief Apply current command variables directly to motor driver hardware.
 *
 * @details Evaluates sign and magnitude of `current_power_ticks`:
 *          - If `current_power_ticks == 0`, commands active brake.
 *          - If `current_power_ticks > 0`, commands FORWARD direction on H-Bridge.
 *          - If `current_power_ticks < 0`, commands REVERSE direction on H-Bridge.
 *          After applying direction, sets speed duty ticks to absolute magnitude of `current_power_ticks`.
 *
 * @param[in] wheel Target wheel handle.
 * @return esp_err_t ESP_OK on success or underlying ESP-IDF driver error code.
 */
static esp_err_t apply_hardware(wheel_handle_t wheel)
{
    /* Active brake immediately if power command is zero */
    if (wheel->current_power_ticks == 0) 
    {
        return bdc_motor_brake(wheel->motor);
    }

    /* Set H-Bridge direction signals */
    if (wheel->current_power_ticks > 0) 
    {
        ESP_RETURN_ON_ERROR(bdc_motor_forward(wheel->motor), TAG, "Failed setting forward direction");
    } 
    else 
    {
        ESP_RETURN_ON_ERROR(bdc_motor_reverse(wheel->motor), TAG, "Failed setting reverse direction");
    }

    /* Apply power magnitude as speed duty ticks */
    return bdc_motor_set_speed(wheel->motor, (uint32_t)abs(wheel->current_power_ticks));
}

/* ============================================================================
 *                              PUBLIC API IMPLEMENTATION
 * ==========================================================================*/

esp_err_t wheel_create(const wheel_config_t *config, wheel_handle_t *out_wheel)
{
    /* Input argument validation. Everything that can be rejected is checked BEFORE allocating,
     * so an invalid configuration cannot leak the handle context (and a negative limit cannot
     * wrap around into a huge unsigned one). */
    ESP_RETURN_ON_FALSE(config && out_wheel, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments provided");
    ESP_RETURN_ON_FALSE((config->max_power_ticks >= 0) && (config->max_power_ticks <= WHEEL_PWM_DUTY_TICK_MAX),
                        ESP_ERR_INVALID_ARG, TAG, "max_power_ticks must be within [0, WHEEL_PWM_DUTY_TICK_MAX]");

    /* Allocate memory for wheel handle context on heap (zero-initialized via calloc) */
    wheel_handle_t wheel = (wheel_handle_t)calloc(1, sizeof(struct wheel_t));
    ESP_RETURN_ON_FALSE(wheel, ESP_ERR_NO_MEM, TAG, "Failed allocating memory for wheel handle");

    /* ----------------------------------------------------------------
     * 1. Store Instance Dynamic Parameters
     * ---------------------------------------------------------------- */
    wheel->max_power_ticks = (uint32_t)config->max_power_ticks;

    /* Store ADC references (can be NULL if current sensing is disabled) */
    wheel->adc_handle = config->adc_handle;
    wheel->adc_channel = config->adc_channel;

    esp_err_t ret = ESP_OK; // Variable used by ESP_GOTO_ON_ERROR macro

    /* ----------------------------------------------------------------
     * 2. MCPWM Motor Driver Initialization (Mandatory)
     * ---------------------------------------------------------------- */
    bdc_motor_config_t motor_config = {
        .pwma_gpio_num = config->pwm_a_gpio,
        .pwmb_gpio_num = config->pwm_b_gpio,
        .pwm_freq_hz = CONFIG_WHEEL_PWM_FREQ_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = config->mcpwm_group_id,
        .resolution_hz = CONFIG_WHEEL_TIMER_RESOLUTION_HZ,
    };
    /* Instantiate BDC motor device using MCPWM hardware peripheral */
    ESP_GOTO_ON_ERROR(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &wheel->motor), 
                       err, TAG, "Failed creating MCPWM motor device");

    /* ----------------------------------------------------------------
     * 3. PCNT Quadrature Encoder Initialization (Optional)
     * ---------------------------------------------------------------- */
    /* Only configure PCNT peripheral if both encoder pins are valid GPIO numbers */
    bool has_encoder = (config->encoder_a_gpio != GPIO_NUM_NC) && 
                       (config->encoder_b_gpio != GPIO_NUM_NC);

    if (has_encoder) 
    {
        /* Configure PCNT hardware counting unit */
        pcnt_unit_config_t pcnt_config = {
            .high_limit = CONFIG_WHEEL_ENCODER_PCNT_HIGH_LIMIT,
            .low_limit = WHEEL_ENCODER_PCNT_LOW_LIMIT,
            .flags.accum_count = true, /* Driver-side accumulator across limit overflows; needs the two watch points added below */
        };
        ESP_GOTO_ON_ERROR(pcnt_new_unit(&pcnt_config, &wheel->pcnt_unit), err, TAG, "Failed creating PCNT unit");

        /* Configure glitch filter to ignore spurious noise pulses under 1000 ns (1 us) */
        pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
        ESP_GOTO_ON_ERROR(pcnt_unit_set_glitch_filter(wheel->pcnt_unit, &filter_config), err, TAG, "Failed configuring glitch filter");

        /* --- Quadrature Decoding - Channel A (4x Mode) --- */
        pcnt_chan_config_t channel_a_config = {
            .edge_gpio_num = config->encoder_a_gpio,
            .level_gpio_num = config->encoder_b_gpio,
        };
        ESP_GOTO_ON_ERROR(pcnt_new_channel(wheel->pcnt_unit, &channel_a_config, &wheel->pcnt_channel_a), err, TAG, "Failed creating PCNT Channel A");
        /* Define edge and level signal actions for Channel A */
        ESP_GOTO_ON_ERROR(pcnt_channel_set_edge_action(wheel->pcnt_channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE), err, TAG, "Edge action A failed");
        ESP_GOTO_ON_ERROR(pcnt_channel_set_level_action(wheel->pcnt_channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), err, TAG, "Level action A failed");

        /* --- Quadrature Decoding - Channel B (4x Mode) --- */
        pcnt_chan_config_t channel_b_config = {
            .edge_gpio_num = config->encoder_b_gpio,
            .level_gpio_num = config->encoder_a_gpio,
        };
        ESP_GOTO_ON_ERROR(pcnt_new_channel(wheel->pcnt_unit, &channel_b_config, &wheel->pcnt_channel_b), err, TAG, "Failed creating PCNT Channel B");
        /* Define edge and level signal actions for Channel B */
        ESP_GOTO_ON_ERROR(pcnt_channel_set_edge_action(wheel->pcnt_channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE), err, TAG, "Edge action B failed");
        ESP_GOTO_ON_ERROR(pcnt_channel_set_level_action(wheel->pcnt_channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), err, TAG, "Level action B failed");

        /* Watch points at the counter limits: required for accum_count to work. Without them the hardware
         * counter silently wraps to 0 at +/- the limit and those counts are lost. */
        ESP_GOTO_ON_ERROR(pcnt_unit_add_watch_point(wheel->pcnt_unit, CONFIG_WHEEL_ENCODER_PCNT_HIGH_LIMIT), err, TAG, "Failed adding PCNT high-limit watch point");
        ESP_GOTO_ON_ERROR(pcnt_unit_add_watch_point(wheel->pcnt_unit, WHEEL_ENCODER_PCNT_LOW_LIMIT), err, TAG, "Failed adding PCNT low-limit watch point");

        /* Enable, clear, and start hardware pulse counting */
        ESP_GOTO_ON_ERROR(pcnt_unit_enable(wheel->pcnt_unit), err, TAG, "Failed enabling PCNT unit");
        ESP_GOTO_ON_ERROR(pcnt_unit_clear_count(wheel->pcnt_unit), err, TAG, "Failed clearing PCNT count");
        ESP_GOTO_ON_ERROR(pcnt_unit_start(wheel->pcnt_unit), err, TAG, "Failed starting PCNT unit");
    } else {
        wheel->pcnt_unit = NULL; /* Ensure handle remains NULL when disabled */
    }

    /* ----------------------------------------------------------------
     * 4. Motor Driver Activation & Initial Safety State
     * ---------------------------------------------------------------- */
    ESP_GOTO_ON_ERROR(bdc_motor_enable(wheel->motor), err, TAG, "Failed enabling motor driver");
    ESP_GOTO_ON_ERROR(bdc_motor_brake(wheel->motor), err, TAG, "Failed setting motor brake state");

    *out_wheel = wheel;
    return ESP_OK;

err:
    /* Rollback mechanism: Clean up allocated resources if initialization fails midway */
    wheel_delete(wheel);
    return ret;
}

esp_err_t wheel_set_power(wheel_handle_t wheel, int32_t power_ticks)
{
    ESP_RETURN_ON_FALSE(wheel, ESP_ERR_INVALID_ARG, TAG, "Invalid wheel handle");

    int32_t max_power_ticks_signed = (int32_t)wheel->max_power_ticks;

    /* Symmetrically clamp power command within [-max_power_ticks, +max_power_ticks] */
    if (power_ticks > max_power_ticks_signed) 
    {
        power_ticks = max_power_ticks_signed;
    } 
    else if (power_ticks < -max_power_ticks_signed) 
    {
        power_ticks = -max_power_ticks_signed;
    }

    /* Update internal state and apply to physical hardware */
    wheel->current_power_ticks = power_ticks;

    return apply_hardware(wheel);
}

esp_err_t wheel_brake(wheel_handle_t wheel)
{
    ESP_RETURN_ON_FALSE(wheel, ESP_ERR_INVALID_ARG, TAG, "Invalid wheel handle");

    /* Reset power state and engage active brake via BDC driver */
    wheel->current_power_ticks = 0;

    return bdc_motor_brake(wheel->motor);
}

esp_err_t wheel_get_encoder_count(wheel_handle_t wheel, int *out_count)
{
    ESP_RETURN_ON_FALSE(wheel && out_count, ESP_ERR_INVALID_ARG, TAG, "Invalid parameters");

    /* Safety guard: Return error if encoder feature is disabled on this instance */
    if (!wheel->pcnt_unit) 
    {
        *out_count = 0;
        return ESP_ERR_INVALID_STATE;
    }

    /* Read accumulated pulse count from PCNT hardware unit */
    return pcnt_unit_get_count(wheel->pcnt_unit, out_count);
}

esp_err_t wheel_clear_encoder_count(wheel_handle_t wheel)
{
    ESP_RETURN_ON_FALSE(wheel, ESP_ERR_INVALID_ARG, TAG, "Invalid wheel handle");

    /* Safety guard: Return error if encoder feature is disabled on this instance */
    if (!wheel->pcnt_unit) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* Reset PCNT counter back to zero */
    return pcnt_unit_clear_count(wheel->pcnt_unit);
}

esp_err_t wheel_get_current_raw(wheel_handle_t wheel, int *out_raw_adc)
{
    ESP_RETURN_ON_FALSE(wheel && out_raw_adc, ESP_ERR_INVALID_ARG, TAG, "Invalid parameters");

    /* Safety guard: Return error if ADC channel was not assigned to this instance */
    if (!wheel->adc_handle) 
    {
        *out_raw_adc = 0;
        return ESP_ERR_INVALID_STATE;
    }

    /* Perform oneshot analog conversion on mapped ADC channel */
    return adc_oneshot_read(wheel->adc_handle, wheel->adc_channel, out_raw_adc);
}

esp_err_t wheel_get_max_power_ticks(wheel_handle_t wheel, uint32_t *out_max_power_ticks)
{
    ESP_RETURN_ON_FALSE(wheel && out_max_power_ticks, ESP_ERR_INVALID_ARG, TAG, "Invalid parameters");

    *out_max_power_ticks = wheel->max_power_ticks;
    return ESP_OK;
}

esp_err_t wheel_delete(wheel_handle_t wheel)
{
    /* Return immediately if handle is already NULL */
    if (!wheel) return ESP_OK;

    /* 1. Disable and release BDC Motor hardware device */
    if (wheel->motor) 
    {
        bdc_motor_disable(wheel->motor);
        bdc_motor_del(wheel->motor);
    }

    /* 2. Stop, disable, and delete PCNT hardware unit and channels (if allocated) */
    if (wheel->pcnt_unit) 
    {
        pcnt_unit_stop(wheel->pcnt_unit);
        pcnt_unit_disable(wheel->pcnt_unit);
        if (wheel->pcnt_channel_a) pcnt_del_channel(wheel->pcnt_channel_a);
        if (wheel->pcnt_channel_b) pcnt_del_channel(wheel->pcnt_channel_b);
        pcnt_del_unit(wheel->pcnt_unit);
    }

    /* Note: Shared ADC unit handle (wheel->adc_handle) is NOT destroyed here 
     * as it is managed externally by the main application. */

    /* 3. Free allocated context memory */
    free(wheel);
    return ESP_OK;
}
