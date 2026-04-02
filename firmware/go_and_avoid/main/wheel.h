/*
 * wheel.h
 *
 *  Created on: 9 de jan. de 2025
 *      Author: Matheus
 */

#ifndef MAIN_WHEEL_H_
#define MAIN_WHEEL_H_

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_LEFT_MCPWM_GPIO_A              GPIO_NUM_11
#define BDC_LEFT_MCPWM_GPIO_B              GPIO_NUM_10
#define BDC_RIGHT_MCPWM_GPIO_A              GPIO_NUM_12
#define BDC_RIGHT_MCPWM_GPIO_B              GPIO_NUM_13

#define BDC_ENCODER_LEFT_GPIO_A            GPIO_NUM_14
#define BDC_ENCODER_LEFT_GPIO_B            GPIO_NUM_21
#define BDC_ENCODER_RIGHT_GPIO_A            GPIO_NUM_7
#define BDC_ENCODER_RIGHT_GPIO_B            GPIO_NUM_6
#define BDC_ENCODER_PCNT_HIGH_LIMIT   7000 // esses valores de limite de contagem podem ser usados para reiniciar a contagem, e/ou para gerar interrupção quando chegar ao limite. Aqui esses valores são irrelevantes, pois não são usados.
#define BDC_ENCODER_PCNT_LOW_LIMIT    -7000

#define BDC_PID_LOOP_PERIOD_MS        100   // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED          400  // expected motor speed, in the pulses counted by the rotary encoder

#define WHEEL_AXIS_LENGHT_2 10 
#define WHEELS_ENCODER_PPR 900
#define WHELL_RADIUS 0.033 //33cm


typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;


typedef enum {
    WHEEL_STOP = 0,
    WHEEL_FORWARD,
    WHEEL_REVERSE
} wheel_dir_t;

typedef struct {
    wheel_dir_t dir_left;
    wheel_dir_t dir_right;
    uint32_t pwm_left;
    uint32_t pwm_right;
} wheel_cmd_t;

int wheel_Init( void );

void wheel_SetRawSpeed(
    wheel_dir_t dir_left, uint32_t pwm_left,
    wheel_dir_t dir_right, uint32_t pwm_right);

void wheel_GetPower( uint32_t *pL, uint32_t *pR );

void wheel_GetEndoderPulses( int *pL, int *pR );

#endif /* MAIN_WHEEL_H_ */
