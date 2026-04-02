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
	wheel_Init();
	//wheel_SetVel(BDC_MCPWM_DUTY_TICK_MAX, BDC_MCPWM_DUTY_TICK_MAX);

	uint32_t power_left_wheel, power_right_wheel; 

	int dir = 0;
	int count = 0;
	int pL = 0, pR = 0;
	while(1)
	{
          if (count == 6) 
          {
			  count = 0;
            if (dir == 0) 
            {
              //wheel_GoForward
              wheel_SetRawSpeed(WHEEL_FORWARD, BDC_MCPWM_DUTY_TICK_MAX, WHEEL_FORWARD, BDC_MCPWM_DUTY_TICK_MAX);
              dir = 1;
            } 
            else 
            {
              //wheel_GoBackward();
              wheel_SetRawSpeed(WHEEL_BACKWARD, BDC_MCPWM_DUTY_TICK_MAX, WHEEL_BACKWARD, BDC_MCPWM_DUTY_TICK_MAX);
              dir = 0;
            }
          }
          
          wheel_GetEndoderPulses(&pL, &pR);
          //ESP_LOGI(TAG, "Left encoder: %d\tRight encoder: %d\r\n", pL, pR);
          
          
          wheel_GetPower(&power_left_wheel, &power_right_wheel);
          printf("Left ADC: %" PRIu32 "; \t Right ADC: %" PRIu32 ".\n", power_left_wheel, power_right_wheel);

		  //printf("Left ADC: %d\n", adc_left_raw[1][0]);
          
          count++;
          vTaskDelay(pdMS_TO_TICKS(500));
	}
	
}
