/**
 * @file    wheel.c
 * @brief   PascalCase handle-based implementation for wheel motor, optional encoder, and optional ADC.
 *
 * Implements hardware control abstractions for BDC motor driving via MCPWM,
 * optional pulse accumulation via PCNT peripheral, and optional raw current monitoring via ADC.
 *
 * @date    Jan 2025
 * @author  Matheus
 */

#include "wheel.h"
#include <stdlib.h>
#include <math.h>
#include "esp_check.h"

/** Logging tag used across this module. */
static const char *TAG = "wheel";

/**
 * @brief Internal concrete representation of the wheel context handle.
 */
struct Wheel_t {
    /* --- Peripherals --- */
    bdc_motor_handle_t motor;              /**< ESP-IDF BDC Motor driver handle. */
    pcnt_unit_handle_t pcnt_unit;          /**< PCNT hardware unit handle (NULL if unused). */
    pcnt_channel_handle_t pcnt_chan_a;     /**< PCNT channel A handle. */
    pcnt_channel_handle_t pcnt_chan_b;     /**< PCNT channel B handle. */
    adc_oneshot_unit_handle_t adc_handle;  /**< Shared ADC unit handle (NULL if unused). */
    adc_channel_t adc_channel;             /**< Dedicated ADC current channel. */

    /* --- Dynamic Hardware Properties --- */
    uint32_t mcpwm_resolution_hz;          /**< Configured MCPWM timer resolution. */
    uint32_t pwm_freq_hz;                  /**< Configured PWM switching frequency. */
    uint32_t max_duty_ticks;               /**< Maximum PWM duty cycle calculated in ticks. */
    int32_t max_power_limit;               /**< Maximum allowable signed power magnitude. */

    /* --- Command State --- */
    Wheel_Dir_t current_dir;               /**< Currently commanded direction state. */
    uint32_t current_pwm;                  /**< Currently applied PWM duty tick value. */
    int32_t current_power;                 /**< Currently applied signed power value. */
};

/* ============================================================================
 *                     PRIVATE HELPER / INLINE FUNCTIONS
 * ==========================================================================*/

/**
 * @brief Clamp raw PWM tick values to instance specific allowable boundaries.
 */
static inline uint32_t Wheel_ClampPwm(Wheel_Handle_t wheel, uint32_t val)
{
    if (val > wheel->max_duty_ticks) {
        return wheel->max_duty_ticks;
    }
    return val;
}

/**
 * @brief Apply current command variables directly to motor driver hardware.
 */
static esp_err_t Wheel_ApplyHardware(Wheel_Handle_t wheel)
{
    if (wheel->current_pwm == 0 || wheel->current_dir == WHEEL_STOP) {
        return bdc_motor_brake(wheel->motor);
    }

    if (wheel->current_dir == WHEEL_FORWARD) {
        ESP_RETURN_ON_ERROR(bdc_motor_forward(wheel->motor), TAG, "Failed setting forward direction");
    } else {
        ESP_RETURN_ON_ERROR(bdc_motor_reverse(wheel->motor), TAG, "Failed setting reverse direction");
    }

    return bdc_motor_set_speed(wheel->motor, wheel->current_pwm);
}

/* ============================================================================
 *                              PUBLIC API IMPLEMENTATION
 * ==========================================================================*/

esp_err_t Wheel_New(const Wheel_Config_t *config, Wheel_Handle_t *ret_wheel)
{
    ESP_RETURN_ON_FALSE(config && ret_wheel, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments provided");

    Wheel_Handle_t wheel = (Wheel_Handle_t)calloc(1, sizeof(struct Wheel_t));
    ESP_RETURN_ON_FALSE(wheel, ESP_ERR_NO_MEM, TAG, "Failed allocating memory for wheel handle");

    /* ----------------------------------------------------------------
     * 1. Store & Calculate Instance Dynamic Parameters
     * ---------------------------------------------------------------- */
    wheel->mcpwm_resolution_hz = (config->mcpwm_resolution_hz > 0) ? 
                                 config->mcpwm_resolution_hz : WHEEL_TIMER_RESOLUTION_HZ;

    wheel->pwm_freq_hz = (config->pwm_freq_hz > 0) ? 
                         config->pwm_freq_hz : WHEEL_PWM_FREQ_HZ;

    wheel->max_duty_ticks = wheel->mcpwm_resolution_hz / wheel->pwm_freq_hz;

    wheel->max_power_limit = (config->max_power_limit > 0) ? 
                             config->max_power_limit : (int32_t)wheel->max_duty_ticks;

    wheel->adc_handle = config->adc_handle;
    wheel->adc_channel = config->adc_channel;

    esp_err_t ret = ESP_OK;

    /* ----------------------------------------------------------------
     * 2. MCPWM Motor Driver Initialization (Mandatory)
     * ---------------------------------------------------------------- */
    bdc_motor_config_t motor_config = {
        .pwma_gpio_num = config->pwm_a_gpio,
        .pwmb_gpio_num = config->pwm_b_gpio,
        .pwm_freq_hz = wheel->pwm_freq_hz,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = config->mcpwm_group_id,
        .resolution_hz = wheel->mcpwm_resolution_hz,
    };
    ESP_GOTO_ON_ERROR(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &wheel->motor), 
                       err, TAG, "Failed creating MCPWM motor device");

    /* ----------------------------------------------------------------
     * 3. PCNT Quadrature Encoder Initialization (Optional)
     * ---------------------------------------------------------------- */
    bool has_encoder = (config->encoder_a_gpio != GPIO_NUM_NC) && 
                       (config->encoder_b_gpio != GPIO_NUM_NC);

    if (has_encoder) {
        pcnt_unit_config_t pcnt_config = {
            .high_limit = (config->pcnt_high_limit != 0) ? config->pcnt_high_limit : INT16_MAX,
            .low_limit = (config->pcnt_low_limit != 0) ? config->pcnt_low_limit : INT16_MIN,
            .flags.accum_count = true,
        };
        ESP_GOTO_ON_ERROR(pcnt_new_unit(&pcnt_config, &wheel->pcnt_unit), err, TAG, "Failed creating PCNT unit");

        pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
        ESP_GOTO_ON_ERROR(pcnt_unit_set_glitch_filter(wheel->pcnt_unit, &filter_config), err, TAG, "Failed configuring glitch filter");

        /* Quadrature Decoding - Channel A */
        pcnt_chan_config_t chan_a_cfg = {
            .edge_gpio_num = config->encoder_a_gpio,
            .level_gpio_num = config->encoder_b_gpio,
        };
        ESP_GOTO_ON_ERROR(pcnt_new_channel(wheel->pcnt_unit, &chan_a_cfg, &wheel->pcnt_chan_a), err, TAG, "Failed creating PCNT Channel A");
        ESP_GOTO_ON_ERROR(pcnt_channel_set_edge_action(wheel->pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE), err, TAG, "Edge action A failed");
        ESP_GOTO_ON_ERROR(pcnt_channel_set_level_action(wheel->pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), err, TAG, "Level action A failed");

        /* Quadrature Decoding - Channel B */
        pcnt_chan_config_t chan_b_cfg = {
            .edge_gpio_num = config->encoder_b_gpio,
            .level_gpio_num = config->encoder_a_gpio,
        };
        ESP_GOTO_ON_ERROR(pcnt_new_channel(wheel->pcnt_unit, &chan_b_cfg, &wheel->pcnt_chan_b), err, TAG, "Failed creating PCNT Channel B");
        ESP_GOTO_ON_ERROR(pcnt_channel_set_edge_action(wheel->pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE), err, TAG, "Edge action B failed");
        ESP_GOTO_ON_ERROR(pcnt_channel_set_level_action(wheel->pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), err, TAG, "Level action B failed");

        ESP_GOTO_ON_ERROR(pcnt_unit_enable(wheel->pcnt_unit), err, TAG, "Failed enabling PCNT unit");
        ESP_GOTO_ON_ERROR(pcnt_unit_clear_count(wheel->pcnt_unit), err, TAG, "Failed clearing PCNT count");
        ESP_GOTO_ON_ERROR(pcnt_unit_start(wheel->pcnt_unit), err, TAG, "Failed starting PCNT unit");
    } else {
        wheel->pcnt_unit = NULL;
    }

    /* ----------------------------------------------------------------
     * 4. Motor Driver Activation & Initial State
     * ---------------------------------------------------------------- */
    ESP_GOTO_ON_ERROR(bdc_motor_enable(wheel->motor), err, TAG, "Failed enabling motor driver");
    ESP_GOTO_ON_ERROR(bdc_motor_brake(wheel->motor), err, TAG, "Failed setting motor brake state");

    *ret_wheel = wheel;
    return ESP_OK;

err:
    Wheel_Del(wheel);
    return ret;
}

esp_err_t Wheel_SetDutyCycle(Wheel_Handle_t wheel, Wheel_Dir_t dir, uint32_t pwm_duty)
{
    ESP_RETURN_ON_FALSE(wheel, ESP_ERR_INVALID_ARG, TAG, "Invalid wheel handle");

    wheel->current_dir = dir;
    wheel->current_pwm = Wheel_ClampPwm(wheel, pwm_duty);

    return Wheel_ApplyHardware(wheel);
}

esp_err_t Wheel_SetPower(Wheel_Handle_t wheel, int32_t power)
{
    ESP_RETURN_ON_FALSE(wheel, ESP_ERR_INVALID_ARG, TAG, "Invalid wheel handle");

    /* Bounded to [-max_power_limit, +max_power_limit] */
    if (power > wheel->max_power_limit) {
        power = wheel->max_power_limit;
    } else if (power < -wheel->max_power_limit) {
        power = -wheel->max_power_limit;
    }

    wheel->current_power = power;
    wheel->current_dir = (power >= 0) ? WHEEL_FORWARD : WHEEL_REVERSE;
    
    /* Map input power magnitude to dynamic max duty ticks */
    uint32_t abs_power = (uint32_t)abs(power);
    if (wheel->max_power_limit != (int32_t)wheel->max_duty_ticks && wheel->max_power_limit > 0) {
        wheel->current_pwm = (abs_power * wheel->max_duty_ticks) / (uint32_t)wheel->max_power_limit;
    } else {
        wheel->current_pwm = abs_power;
    }

    return Wheel_ApplyHardware(wheel);
}

esp_err_t Wheel_Brake(Wheel_Handle_t wheel)
{
    ESP_RETURN_ON_FALSE(wheel, ESP_ERR_INVALID_ARG, TAG, "Invalid wheel handle");

    wheel->current_pwm = 0;
    wheel->current_dir = WHEEL_STOP;
    wheel->current_power = 0;

    return bdc_motor_brake(wheel->motor);
}

esp_err_t Wheel_GetEncoderCount(Wheel_Handle_t wheel, int *out_count)
{
    ESP_RETURN_ON_FALSE(wheel && out_count, ESP_ERR_INVALID_ARG, TAG, "Invalid parameters");

    if (!wheel->pcnt_unit) {
        *out_count = 0;
        return ESP_ERR_INVALID_STATE;
    }

    return pcnt_unit_get_count(wheel->pcnt_unit, out_count);
}

esp_err_t Wheel_ClearEncoderCount(Wheel_Handle_t wheel)
{
    ESP_RETURN_ON_FALSE(wheel, ESP_ERR_INVALID_ARG, TAG, "Invalid wheel handle");

    if (!wheel->pcnt_unit) {
        return ESP_ERR_INVALID_STATE;
    }

    return pcnt_unit_clear_count(wheel->pcnt_unit);
}

esp_err_t Wheel_GetCurrentRaw(Wheel_Handle_t wheel, int *out_raw_adc)
{
    ESP_RETURN_ON_FALSE(wheel && out_raw_adc, ESP_ERR_INVALID_ARG, TAG, "Invalid parameters");

    if (!wheel->adc_handle) {
        *out_raw_adc = 0;
        return ESP_ERR_INVALID_STATE;
    }

    return adc_oneshot_read(wheel->adc_handle, wheel->adc_channel, out_raw_adc);
}

esp_err_t Wheel_Del(Wheel_Handle_t wheel)
{
    if (!wheel) return ESP_OK;

    /* Disable and release BDC Motor device */
    if (wheel->motor) {
        bdc_motor_disable(wheel->motor);
        bdc_motor_del_mcpwm_device(wheel->motor);
    }

    /* Disable and release PCNT unit and channels if initialized */
    if (wheel->pcnt_unit) {
        pcnt_unit_stop(wheel->pcnt_unit);
        pcnt_unit_disable(wheel->pcnt_unit);
        if (wheel->pcnt_chan_a) pcnt_del_channel(wheel->pcnt_chan_a);
        if (wheel->pcnt_chan_b) pcnt_del_channel(wheel->pcnt_chan_b);
        pcnt_del_unit(wheel->pcnt_unit);
    }

    free(wheel);
    return ESP_OK;
}