#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
//#include "led_strip.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"

static const char *TAG = "IR_Line";

#define INFRA_RED_LEFT_GPIO  GPIO_NUM_15
#define INFRA_RED_CENTER_LEFT_GPIO  GPIO_NUM_16
#define INFRA_RED_MIDDLE_GPIO  GPIO_NUM_17
#define INFRA_RED_CENTER_RIGHT_GPIO  GPIO_NUM_18
#define INFRA_RED_RIGHT_GPIO GPIO_NUM_8

#define IR_LINE_VERY_VERY_LEFT (1ULL << INFRA_RED_LEFT_GPIO)
#define IR_LINE_VERY_LEFT ((1ULL << INFRA_RED_LEFT_GPIO) | (1ULL << INFRA_RED_CENTER_LEFT_GPIO))
#define IR_LINE_LEFT (1ULL << INFRA_RED_CENTER_LEFT_GPIO)
#define IR_LINE_LEFT_MIDDLE ((1ULL << INFRA_RED_CENTER_LEFT_GPIO) | (1ULL << INFRA_RED_MIDDLE_GPIO))
#define IR_LINE_MIDDLE (1ULL << INFRA_RED_MIDDLE_GPIO)
#define IR_LINE_RIGHT_MIDDLE ((1ULL << INFRA_RED_MIDDLE_GPIO) | (1ULL << INFRA_RED_CENTER_RIGHT_GPIO))
#define IR_LINE_RIGHT (1ULL << INFRA_RED_CENTER_RIGHT_GPIO)
#define IR_LINE_VERY_RIGHT ((1ULL << INFRA_RED_CENTER_RIGHT_GPIO) | (1ULL << INFRA_RED_RIGHT_GPIO))
#define IR_LINE_VERY_VERY_RIGHT (1ULL << INFRA_RED_RIGHT_GPIO)

#define INFRA_RED_OUT_GPIO_MASK ((uint64_t)((1ULL << INFRA_RED_LEFT_GPIO) | (1ULL << INFRA_RED_CENTER_LEFT_GPIO) | (1ULL << INFRA_RED_MIDDLE_GPIO) | (1ULL << INFRA_RED_CENTER_RIGHT_GPIO) | (1ULL << INFRA_RED_RIGHT_GPIO)))

void app_main(void)
{
	gpio_config_t ir_line_config = {
			.pin_bit_mask = INFRA_RED_OUT_GPIO_MASK,
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

		gpioValue &= INFRA_RED_OUT_GPIO_MASK;

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
