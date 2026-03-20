#include "threeeyes.h"
#include "freertos/semphr.h"
#include <inttypes.h>
//#include "rom/ets_sys.h" // Para printf dentro de ISR

const static char *TAG = "TreeEyes";

ultrasonic_sensor_t sensors[3]; // Estrutura para os 3 sensores
static TaskHandle_t actualTaskHandle;

static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    ultrasonic_sensor_t *sensor = (ultrasonic_sensor_t *)user_data;

    //static uint32_t start_time = 0;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) // Detecção da borda de subida (início do sinal ECHO)
    {
        sensor->start_time = edata->cap_value;
    } 
    else if (edata->cap_edge == MCPWM_CAP_EDGE_NEG) // Detecção da borda de descida (fim do sinal ECHO)
    {
        //if (sensor->notify_bit == 0) {
             // Use ets_printf pois printf comum trava dentro de ISR
         //    ets_printf("ERRO ISR: sensor->notify_bit esta ZERO!\n");
        //}
        uint32_t end_time = edata->cap_value;
        sensor->value.tof_ticks = end_time - sensor->start_time;
        // Notifica a tarefa associada ao sensor
        BaseType_t high_task_wakeup = pdFALSE;
        if (actualTaskHandle != NULL) // Por algum motivo, a biblioteca pegou um handle nulo?
        {
            xTaskNotifyFromISR(actualTaskHandle, sensor->notify_bit, eSetBits, &high_task_wakeup);
            return high_task_wakeup == pdTRUE;
        }
    }

    return false;
}

void ThreeEyes_TrigAndWait( TickType_t xTicksToWait )
{
    actualTaskHandle = xTaskGetCurrentTaskHandle();
    
    // 1. Limpa qualquer notificação antiga antes de começar
    ulTaskNotifyValueClear(NULL, 0xFFFFFFFF);

    // 2. Dispara o pulso TRIG
    gpio_set_level(HC_SR04_TRIG_GPIO, 1);
    esp_rom_delay_us(10); 
    gpio_set_level(HC_SR04_TRIG_GPIO, 0);

    uint32_t notifiedBits = 0;
    
    // 3. Espera uma única vez por QUALQUER um dos bits
    // Usamos 0x00 no clear on exit para podermos analisar os bits manualmente
    // while ( sensors[0].value.isUpdated != pdTRUE && sensors[1].value.isUpdated != pdTRUE && sensors[2].value.isUpdated != pdTRUE )
    if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedBits, xTicksToWait) == pdTRUE) 
    {
        if (notifiedBits == 0) {
            printf("AVISO: Acordei mas os bits vieram ZERO!\n");
        }
        // 4. Mapeia quais sensores responderam
        sensors[0].value.isUpdated = (notifiedBits & NOTIFY_BIT_LEFT_SENSOR)   ? pdTRUE : pdFALSE;
        sensors[1].value.isUpdated = (notifiedBits & NOTIFY_BIT_MIDDLE_SENSOR) ? pdTRUE : pdFALSE;
        sensors[2].value.isUpdated = (notifiedBits & NOTIFY_BIT_RIGHT_SENSOR)  ? pdTRUE : pdFALSE;
    }
    else {
        printf("ERRO: Saí por Timeout mesmo com portMAX_DELAY (Isso não deveria ocorrer!)\n");
    }
    actualTaskHandle = NULL;
}

void ThreeEyes_Read( ultrasonic_value_t *left, ultrasonic_value_t *middle, ultrasonic_value_t *right)
{
    left->tof_ticks = sensors[0].value.tof_ticks;
    left->isUpdated = sensors[0].value.isUpdated;

    middle->tof_ticks = sensors[1].value.tof_ticks;
    middle->isUpdated = sensors[1].value.isUpdated;

    right->tof_ticks = sensors[2].value.tof_ticks;
    right->isUpdated = sensors[2].value.isUpdated;
}

void ThreeEyes_Init( void )
{
    ESP_LOGI(TAG, "Configurando Timer de Captura");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_timer_config = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_config, &cap_timer));

    ESP_LOGI(TAG, "Configurando pino TRIG");
    gpio_config_t trig_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&trig_config));
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    ESP_LOGI(TAG, "Iniciando Timer de Captura");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    gpio_num_t echo_gpios[3] = {HC_SR04_ECHO_GPIO_LEFT, HC_SR04_ECHO_GPIO_MIDDLE, HC_SR04_ECHO_GPIO_RIGHT};

    sensors[0].notify_bit = NOTIFY_BIT_LEFT_SENSOR;
    sensors[1].notify_bit = NOTIFY_BIT_MIDDLE_SENSOR;
    sensors[2].notify_bit = NOTIFY_BIT_RIGHT_SENSOR;
    for (int i = 0; i < 3; i++) 
    {
        ESP_LOGI(TAG, "Configurando canal de captura para o Sensor %d", i + 1);

        mcpwm_capture_channel_config_t channel_config = {
            .gpio_num = echo_gpios[i],
            .prescale = 1,
            .flags.neg_edge = true,
            .flags.pos_edge = true,
            .flags.pull_up = false, // ou false?
        };

        ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &channel_config, &sensors[i].channel));

        mcpwm_capture_event_callbacks_t callbacks = {
            .on_cap = hc_sr04_echo_callback,
        };

        ESP_LOGI(TAG, "Registrando callback para o Sensor %d", i + 1);
        ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(sensors[i].channel, &callbacks, &sensors[i]));

        ESP_LOGI(TAG, "Habilitando canal de captura para o Sensor %d", i + 1);
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(sensors[i].channel));
        sensors[i].echo_gpio_num = echo_gpios[i];
        sensors[i].value.isUpdated = pdFALSE;
    }
}

void ThreeEyes_DisableLeft(void)
{
    ESP_LOGI(TAG, "Habilitando canal de captura para o Sensor Esquerdo");
    ESP_ERROR_CHECK(mcpwm_capture_channel_disable(sensors[0].channel));
}

void ThreeEyes_DisableMiddle(void)
{
    ESP_LOGI(TAG, "Habilitando canal de captura para o Sensor do Meio");
    ESP_ERROR_CHECK(mcpwm_capture_channel_disable(sensors[1].channel));
}

void ThreeEyes_DisableRight(void)
{
    ESP_LOGI(TAG, "Habilitando canal de captura para o Sensor Direito");
    ESP_ERROR_CHECK(mcpwm_capture_channel_disable(sensors[2].channel));
}