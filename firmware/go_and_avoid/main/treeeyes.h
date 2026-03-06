#ifndef TREE_EYES_LIB
#define  TREE_EYES_LIB

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "hal/gpio_types.h"


#define HC_SR04_TRIG_GPIO  GPIO_NUM_42
#define HC_SR04_ECHO_GPIO_LEFT  GPIO_NUM_5
#define HC_SR04_ECHO_GPIO_MIDDLE  GPIO_NUM_4
#define HC_SR04_ECHO_GPIO_RIGHT  GPIO_NUM_1
#define NOTIFY_BIT_LEFT_SENSOR ( 1UL << 0UL ) // 0x01
#define NOTIFY_BIT_MIDDLE_SENSOR ( 1UL << 1UL ) // 0x02
#define NOTIFY_BIT_RIGHT_SENSOR ( 1UL << 2UL ) // 0x04

typedef struct {
    uint32_t tof_ticks;
    BaseType_t isUpdated;
} ultrasonic_value_t;

typedef struct {
    mcpwm_cap_channel_handle_t channel;
    gpio_num_t echo_gpio_num;
    uint32_t start_time;
    uint32_t notify_bit;
    ultrasonic_value_t value;
} ultrasonic_sensor_t;

void TreeEyes_Init( void );

void TreeEyes_TrigAndWait( TickType_t xTicksToWait );

void TreeEyes_Read( ultrasonic_value_t *left, ultrasonic_value_t *middle, ultrasonic_value_t *right);

void TreeEyes_DisableLeft(void);

void TreeEyes_DisableMiddle(void);

void TreeEyes_DisableRight(void);

#endif // TREE_EYES_LIB