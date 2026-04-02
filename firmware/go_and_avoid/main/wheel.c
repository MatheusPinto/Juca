/*
 * wheel.c
 *
 *  Created on: 9 de jan. de 2025
 *      Author: Matheus
 */

#include "wheel.h"
#include <stdbool.h>
const static char *WHEEL_TAG = "Wheels";

static int adc_left_raw[2][10];
static int adc_right_raw[2][10];

//-------------ADC general Config---------------//
adc_oneshot_chan_cfg_t adc_config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
};

//-------------ADC general Init variables---------------//
adc_oneshot_unit_handle_t adc_handle;
adc_oneshot_unit_init_cfg_t adc_init_config = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};

 /***********************Create general motors config structs********************************/

static motor_control_context_t motor_left_ctrl_ctx = {
    .pcnt_encoder = NULL,
};
static motor_control_context_t motor_right_ctrl_ctx = {
    .pcnt_encoder = NULL,
};

    bdc_motor_config_t motor_left_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_LEFT_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_LEFT_MCPWM_GPIO_B,
    };
    
    
        bdc_motor_config_t motor_right_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_RIGHT_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_RIGHT_MCPWM_GPIO_B,
    };

/*******************************************************************************************/

    /*****************************Create PWM motors config structs******************************/
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    /*******************************************************************************************/

 /**********************************Create motors handle*************************************/
    bdc_motor_handle_t motor_left = NULL;
    bdc_motor_handle_t motor_right = NULL;
        /*******************************************************************************************/


    /**********************Create encoders pulse counter config structs*************************/
    // Set the general Pulse counter module configuration
    pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_glitch_filter_config_t filter_config = {
           .max_glitch_ns = 1000,
       };
    /*******************************************************************************************/


    /*************************Create encoders pulse counter handles****************************/
    pcnt_unit_handle_t pcnt_left_unit = NULL;
    pcnt_unit_handle_t pcnt_right_unit = NULL;
    /*******************************************************************************************/


    /*************************Create encoders pulse counter channels****************************/


    // Pulse counter channel A configuration for left encoder
    pcnt_chan_config_t chan_a_left_config = {
        .edge_gpio_num = BDC_ENCODER_LEFT_GPIO_A,
        .level_gpio_num = BDC_ENCODER_LEFT_GPIO_B,
    };
    // Pulse counter channel A configuration for right encoder
    pcnt_chan_config_t chan_a_right_config = {
   		.edge_gpio_num = BDC_ENCODER_RIGHT_GPIO_A,
		.level_gpio_num = BDC_ENCODER_RIGHT_GPIO_B,
    };

    // Pulse counter channel A handles
    pcnt_channel_handle_t pcnt_chan_a_left = NULL;
    pcnt_channel_handle_t pcnt_chan_a_right = NULL;

   // Pulse counter channel B configuration for left encoder
    pcnt_chan_config_t chan_b_left_config = {
        .edge_gpio_num = BDC_ENCODER_LEFT_GPIO_B,
        .level_gpio_num = BDC_ENCODER_LEFT_GPIO_A,
    };
    // Pulse counter channel B configuration for right encoder
    pcnt_chan_config_t chan_b_right_config = {
        .edge_gpio_num = BDC_ENCODER_RIGHT_GPIO_B,
        .level_gpio_num = BDC_ENCODER_RIGHT_GPIO_A,
    };

    // Pulse counter channel B handles
    pcnt_channel_handle_t pcnt_chan_b_left = NULL;
    pcnt_channel_handle_t pcnt_chan_b_right = NULL;

    /*******************************************************************************************/





static wheel_cmd_t current_cmd = {0};

#define PWM_MAX BDC_MCPWM_DUTY_TICK_MAX  // ou o valor real do seu driver

static void wheel_apply(void);

static inline uint32_t clamp_pwm(uint32_t val)
{
    if (val > PWM_MAX) return PWM_MAX;
    return val;
}

    // ================================
// Controle global de PWM (binário)
// ================================
static volatile bool pwm_enabled = true;
static portMUX_TYPE pwm_mux = portMUX_INITIALIZER_UNLOCKED;

static inline void set_pwm_enabled(bool val)
{
    portENTER_CRITICAL(&pwm_mux);
    pwm_enabled = val;
    portEXIT_CRITICAL(&pwm_mux);
}

bool power_manager_is_enabled(void)
{
    return pwm_enabled;
}


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

static void wheel_apply(void)
{
    wheel_cmd_t cmd = current_cmd;

    if (!power_manager_is_enabled())
    {
        cmd.pwm_left = 0;
        cmd.pwm_right = 0;
    }

    // Esquerdo
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

        aplicar_pwm_left(cmd.pwm_left);
    }

    // Direito
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

        aplicar_pwm_right(cmd.pwm_right);
    }
}

// ================================
// Task de monitoramento
// ================================
portTASK_FUNCTION(power_tracker, arg)
{
    uint32_t power_left_wheel, power_right_wheel;

    int contador_stall = 0;
    bool stall_ativo = false;

    // Limiares
    const int LIMITE_STALL = 4000;
    const int LIMITE_LIBERACAO = 3000;

    // Parâmetros de detecção
    const int TEMPO_STALL = 10; // número de ciclos

    // Período adaptativo
    int periodo_ms = 200;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        wheel_GetPower(&power_left_wheel, &power_right_wheel);

        bool corrente_alta = (power_left_wheel > LIMITE_STALL ||
                             power_right_wheel > LIMITE_STALL);

        // ============================
        // Ajuste dinâmico do período
        // ============================
        if (corrente_alta)
        {
            if (periodo_ms > 10)
            {
                periodo_ms /= 2;
                if (periodo_ms < 10)
                    periodo_ms = 10;

                // evita drift ao mudar muito rápido
                last_wake = xTaskGetTickCount();
            }
        }
        else
        {
            if (periodo_ms < 200)
            {
                periodo_ms += 10;
                if (periodo_ms > 200)
                    periodo_ms = 200;
            }
        }

        // ============================
        // Contador de stall
        // ============================
        if (corrente_alta)
        {
            if (contador_stall < TEMPO_STALL)
                contador_stall++;
        }
        else
        {
            if (contador_stall > 0)
                contador_stall--;
        }

        // ============================
        // Detecta stall
        // ============================
        if (!stall_ativo && contador_stall >= TEMPO_STALL)
        {
            stall_ativo = true;
            set_pwm_enabled(false);
            wheel_SetVel(0, 0); // garante parada imediata

            ESP_LOGI(WHEEL_TAG, "Motor Stall!!!! Wheels blocked.");
        }

        // ============================
        // Liberação (com histerese)
        // ============================
        bool corrente_baixa = (power_left_wheel < LIMITE_LIBERACAO &&
                              power_right_wheel < LIMITE_LIBERACAO);

        if (stall_ativo && corrente_baixa && contador_stall == 0)
        {
            stall_ativo = false;
            set_pwm_enabled(true);

            ESP_LOGI(WHEEL_TAG, "Wheels unblocked.");
        }

        // ============================
        // Delay determinístico
        // ============================
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(periodo_ms));
    }
}

portTASK_FUNCTION(speed_ctrl, arg)
{
    wheel_cmd_t cmd_local;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        // Copia comando (snapshot)
        cmd_local = current_cmd;

        // Proteção global
        if (!power_manager_is_enabled())
        {
            cmd_local.pwm_left  = 0;
            cmd_local.pwm_right = 0;
        }

        // =========================
        // Motor esquerdo
        // =========================
        if (cmd_local.pwm_left == 0 || cmd_local.dir_left == WHEEL_STOP)
        {
            bdc_motor_stop(motor_left);
        }
        else
        {
            if (cmd_local.dir_left == WHEEL_FORWARD)
                bdc_motor_forward(motor_left);
            else
                bdc_motor_reverse(motor_left);

            aplicar_pwm_left(cmd_local.pwm_left);
        }

        // =========================
        // Motor direito
        // =========================
        if (cmd_local.pwm_right == 0 || cmd_local.dir_right == WHEEL_STOP)
        {
            bdc_motor_stop(motor_right);
        }
        else
        {
            if (cmd_local.dir_right == WHEEL_FORWARD)
                bdc_motor_forward(motor_right);
            else
                bdc_motor_reverse(motor_right);

            aplicar_pwm_right(cmd_local.pwm_right);
        }

        // Frequência de atualização
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20)); // 50 Hz
    }
}


int wheel_Init( void )
{

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));
	//ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc_right_motor_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_8, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_1, &adc_config));

	//ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_right_motor_handle, ADC_CHANNEL_8, &config));
	
	

    /***********************Create general motors config structs********************************/
 //   ESP_LOGI(TAG, "Create DC motor Left");

 //   ESP_LOGI(TAG, "Create DC motor Right");

    /*******************************************************************************************/



    /**********************************Create motors handle*************************************/

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_left_config, &mcpwm_config, &motor_left));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_right_config, &mcpwm_config, &motor_right));
    motor_left_ctrl_ctx.motor = motor_left;
    motor_right_ctrl_ctx.motor = motor_right;
    /*******************************************************************************************/

    /**********************Create encoders pulse counter config structs*************************/
   // ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");

    /*******************************************************************************************/

    /*************************Create encoders pulse counter handles****************************/
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_left_unit));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_left_unit, &filter_config));
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_right_unit));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_right_unit, &filter_config));
    /*******************************************************************************************/

    /*************************Create encoders pulse counter channels****************************/

    // Each encoder has two channels (A and B).
    // Each channel below (A and B) will controlls the pulse counter in a different way.
    // Both of then will use the BDC_ENCODER_<LEFT/RIGHT>_GPIO_A and BDC_ENCODER_<LEFT/RIGHT>_GPIO_B pins.
    // If motor is counter-clockwise, then channel A will decrement the counter each BDC_ENCODER_<LEFT/RIGHT>_GPIO_A edge.
    // If motor is clockwise, then channel B will increment the counter each BDC_ENCODER_<LEFT/RIGHT>_GPIO_A edge.

    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_left_unit, &chan_a_left_config, &pcnt_chan_a_left));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_right_unit, &chan_a_right_config, &pcnt_chan_a_right));

    /*
     * When counter clock-wise, each edge of BDC_ENCODER_GPIO_A, will decrement the counter.
     *        ____        ____        ____        ____
     *       |    |      |    |      |    |      |    |
     * ______|    |______|    |______|    |______|    |___      BDC_ENCODER_GPIO_B (LEVEL)
     *     ____        ____        ____        ____
     *    |    |      |    |      |    |      |    |
     * ___|    |______|    |______|    |______|    |___         BDC_ENCODER_GPIO_A  (EDGE)
     *  [-1]  [-2]  [-3]  [-4]  [-5]  [-6]   [-7]  [-8]         Counter Value
     *
     *
     * */
    // decrease the counter on rising edge on LOW LEVEL, increase the counter on falling edge on HIGH LEVEL (Will never happens!)
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_left, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_right, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    // keep the counting mode when the control signal is high level (decrement), and reverse the counting mode when the control signal is low level (decrement)
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

 
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_left_unit, &chan_b_left_config, &pcnt_chan_b_left));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_right_unit, &chan_b_right_config, &pcnt_chan_b_right));

    /*
      * When clock-wise, each edge of BDC_ENCODER_GPIO_B, will increment the counter.
      *     ____        ____        ____        ____
      *    |    |      |    |      |    |      |    |
      * ___|    |______|    |______|    |______|    |___      BDC_ENCODER_GPIO_A (LEVEL)
      *        ____        ____        ____        ____
      *       |    |      |    |      |    |      |    |
      * ______|    |______|    |______|    |______|    |___   BDC_ENCODER_GPIO_B  (EDGE)
      *      [1]  [2]    [3]  [4]    [5]  [6]    [7]  [8]     Counter Value
      *
      *
      * */
    // increase the counter on rising edge on HIGH LEVEL, decrease the counter on falling edge on HIGH LEVEL (Will never happens!)
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_left, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_right, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    // keep the counting mode when the control signal is high level (increment), and reverse the counting mode when the control signal is low level (increment)
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    /*******************************************************************************************/

    /*************************Configure Pulse counter Watch Points****************************/
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_left_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_left_unit, BDC_ENCODER_PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_right_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_right_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    /*******************************************************************************************/

    /************************Enable and starts Pulse Counter modules**************************/
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_left_unit));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_right_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_left_unit)); // Always necessary before starts
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_right_unit)); // Always necessary before starts
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_left_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_right_unit));
    motor_left_ctrl_ctx.pcnt_encoder = pcnt_left_unit;
    motor_right_ctrl_ctx.pcnt_encoder = pcnt_right_unit;
    /*******************************************************************************************/

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


void wheel_GetPower( uint32_t *pL, uint32_t *pR )
{
	ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_1, &adc_right_raw[1][0]));
	ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_8, &adc_left_raw[1][0]));
	*pL = adc_left_raw[1][0];
	*pR = adc_right_raw[1][0];
	//printf("Left ADC: %d; \t Right ADC: %d.\n", adc_left_raw[1][0], adc_right_raw[1][0]);
	//printf("Left ADC: %d\n", adc_left_raw[1][0]);
}

void wheel_GetEndoderPulses( int *pL, int *pR )
{
	pcnt_unit_get_count(pcnt_left_unit, pL);
    pcnt_unit_get_count(pcnt_right_unit, pR);
    //printf("Left encoder: %d\tRight encoder: %d\r\n", cur_left_pulse_count, cur_right_pulse_count);

}


