#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "wheel.h"

const static char *TAG = "wheels";

portTASK_FUNCTION(wheel_ctrl, arg)
{
  Wheel_Config_t minimal_wheel_cfg = {
      .pwm_a_gpio = GPIO_NUM_12,
      .pwm_b_gpio = GPIO_NUM_13,
      .mcpwm_group_id = 0,

      /* Encoders desativados */
      .encoder_a_gpio = GPIO_NUM_NC,
      .encoder_b_gpio = GPIO_NUM_NC,

      /* ADC desativado (demais parâmetros podem ficar omissos ou 0) */
      .adc_handle = NULL,
  };

  Wheel_Handle_t wheel_basic = NULL;
  ESP_ERROR_CHECK(Wheel_New(&minimal_wheel_cfg, &wheel_basic));



	int dir = 0;
	int count = 0;
	while(1)
	{
          if (count == 6) 
          {
			  count = 0;
            if (dir == 0) 
            {
                /* O motor funciona normalmente via PWM */
              Wheel_SetPower(wheel_basic, WHEEL_PWM_DUTY_TICK_MAX);
              dir = 1;
            } 
            else 
            {
              Wheel_SetPower(wheel_basic, -WHEEL_PWM_DUTY_TICK_MAX);
              dir = 0;
            }
          }
          
          vTaskDelay(pdMS_TO_TICKS(500));
	}
	
}
