#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"
#include "ir_line.h"

static const char *TAG = "IR_Line";

portTASK_FUNCTION(ir_line_follow, arg)
{
	gpio_config_t ir_line_config = {
			.pin_bit_mask = INFRA_RED_GPIO_PINS_MASK,
			.mode = GPIO_MODE_INPUT,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.intr_type = GPIO_INTR_DISABLE
	};

	gpio_config( &ir_line_config );

	uint64_t gpioValue;

	while (1) {
		gpioValue = (uint64_t)gpio_get_level(INFRA_RED_LEFT_GPIO) << INFRA_RED_LEFT_GPIO |
				    (uint64_t)gpio_get_level(INFRA_RED_CENTER_LEFT_GPIO) << INFRA_RED_CENTER_LEFT_GPIO|
				    (uint64_t)gpio_get_level(INFRA_RED_MIDDLE_GPIO) << INFRA_RED_MIDDLE_GPIO|
				    (uint64_t)gpio_get_level(INFRA_RED_CENTER_RIGHT_GPIO) << INFRA_RED_CENTER_RIGHT_GPIO|
				    (uint64_t)gpio_get_level(INFRA_RED_RIGHT_GPIO) << INFRA_RED_RIGHT_GPIO;

		#ifdef IR_LINE_IS_BLACK
		gpioValue = gpioValue | INFRA_RED_OUT_GPIO_MASK;
		#else
		gpioValue = gpioValue & INFRA_RED_OUT_GPIO_MASK;
		#endif

        //ESP_LOGI(TAG, "gpio value: %llu", gpioValue);
		//ESP_LOGI(TAG, "gpio_value: %x", gpioValue);
		//gpioValue = (REG_READ(GPIO_IN_REG) & INFRA_RED_OUT_GPIO_MASK);
		switch(gpioValue)
		{
		case IR_LINE_VERY_VERY_LEFT:
			ESP_LOGI(TAG, "GPIO VERY VERY LEFT!");
			break;
		case IR_LINE_VERY_LEFT:
			ESP_LOGI(TAG, "GPIO VERY LEFT!");
			break;
		case IR_LINE_LEFT:
			ESP_LOGI(TAG, "GPIO LEFT!");
			break;
		case IR_LINE_LEFT_MIDDLE:
			ESP_LOGI(TAG, "GPIO LEFT MIDDLE!");
			break;
		case IR_LINE_MIDDLE:
			ESP_LOGI(TAG, "GPIO MIDDLE!");
			break;
		case IR_LINE_RIGHT_MIDDLE:
			ESP_LOGI(TAG, "GPIO RIGHT MIDDLE!");
			break;
		case IR_LINE_RIGHT:
			ESP_LOGI(TAG, "GPIO RIGHT!");
			break;
		case IR_LINE_VERY_RIGHT:
			ESP_LOGI(TAG, "GPIO VERY RIGHT!");
			break;
		case IR_LINE_VERY_VERY_RIGHT:
			ESP_LOGI(TAG, "GPIO VERY VERY RIGHT!");
			break;
		default:
			ESP_LOGI(TAG, "NOTHING!");
			break;
		}
        vTaskDelay( 500 / portTICK_PERIOD_MS);
    }
}
