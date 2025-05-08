/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp32/rom/ets_sys.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include <inttypes.h>
#include "pins.h"

float get_distance_cm(uint32_t trig, uint32_t echo)
{
    // Send trigger pulse
    gpio_set_level(trig, 0);
    ets_delay_us(2);
    gpio_set_level(trig, 1);
    ets_delay_us(10);
    gpio_set_level(trig, 0);

    // Wait for echo to go HIGH
    uint32_t timeout = 30000; // 30ms timeout
    uint32_t start = esp_timer_get_time();
    while (gpio_get_level(echo) == 0) {
        if ((esp_timer_get_time() - start) > timeout) return -1;
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(echo) == 1) {
        if ((esp_timer_get_time() - echo_start) > timeout) return -1;
    }
    int64_t echo_end = esp_timer_get_time();

    float distance = (echo_end - echo_start) * 0.034 / 2.0;
    return distance;
}

void init_mecanum_motors() {
    // Set all IN2 (DIR) pins as digital outputs
    gpio_set_direction(FL_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(FR_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(RL_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(RR_IN2, GPIO_MODE_OUTPUT);

    // Configure LEDC PWM on IN1 pins
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t motor_channels[] = {
        {.channel = LEDC_CHANNEL_0, .gpio_num = FL_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_1, .gpio_num = FR_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_2, .gpio_num = RL_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_3, .gpio_num = RR_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_1},
    };

    for (int i = 0; i < 4; ++i) {
        ledc_channel_config(&motor_channels[i]);
    }
}

void set_motor(gpio_num_t in1, gpio_num_t in2, bool forward, uint32_t speed, ledc_channel_t pwm_channel) {
    if (forward) {
        gpio_set_level(in1, 1);  // HIGH
        gpio_set_level(in2, 0);  // LOW
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    }

    if(!forward){
        speed = 255 - speed; // Invert speed for backward movement
    }
    // Apply PWM to IN1
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel);
}

// Wheels Helpers
void move_forward(uint32_t speed) {
    set_motor(FL_IN1, FL_IN2, false, speed, LEDC_CHANNEL_0);
    set_motor(FR_IN1, FR_IN2, false, speed, LEDC_CHANNEL_1);
    set_motor(RL_IN1, RL_IN2, false, speed , LEDC_CHANNEL_2);
    set_motor(RR_IN1, RR_IN2, false, speed, LEDC_CHANNEL_3);
}

void move_backward(uint32_t speed) {
    set_motor(FL_IN1, FL_IN2, true, speed, LEDC_CHANNEL_0);
    set_motor(FR_IN1, FR_IN2, true, speed, LEDC_CHANNEL_1);
    set_motor(RL_IN1, RL_IN2, true, speed, LEDC_CHANNEL_2);
    set_motor(RR_IN1, RR_IN2, true, speed, LEDC_CHANNEL_3);
}

void move_right(uint32_t speed) {
    set_motor(FL_IN1, FL_IN2, false, speed, LEDC_CHANNEL_0);
    set_motor(FR_IN1, FR_IN2, false, speed, LEDC_CHANNEL_1);
    set_motor(RL_IN1, RL_IN2, true, speed, LEDC_CHANNEL_2);
    set_motor(RR_IN1, RR_IN2, true, speed, LEDC_CHANNEL_3);
}

void move_left(uint32_t speed) {
    set_motor(FL_IN1, FL_IN2, true, speed, LEDC_CHANNEL_0);
    set_motor(FR_IN1, FR_IN2, true, speed, LEDC_CHANNEL_1);
    set_motor(RL_IN1, RL_IN2, false, speed, LEDC_CHANNEL_2);
    set_motor(RR_IN1, RR_IN2, false, speed, LEDC_CHANNEL_3);
}

void rotate_counterclockwise(uint32_t speed) {
    set_motor(FL_IN1, FL_IN2, true, speed, LEDC_CHANNEL_0);
    set_motor(FR_IN1, FR_IN2, false, speed, LEDC_CHANNEL_1);
    set_motor(RL_IN1, RL_IN2, true, speed, LEDC_CHANNEL_2);
    set_motor(RR_IN1, RR_IN2, false, speed, LEDC_CHANNEL_3);
}

void rotate_clockwise(uint32_t speed) {
    set_motor(FL_IN1, FL_IN2, false, speed, LEDC_CHANNEL_0);
    set_motor(FR_IN1, FR_IN2, true, speed, LEDC_CHANNEL_1);
    set_motor(RL_IN1, RL_IN2, false, speed, LEDC_CHANNEL_2);
    set_motor(RR_IN1, RR_IN2, true, speed, LEDC_CHANNEL_3);
}

void stop_all_motors() {
    for (int ch = LEDC_CHANNEL_0; ch <= LEDC_CHANNEL_3; ch++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    }
    gpio_set_level(FL_IN1, 0);  // HIGH
    gpio_set_level(FL_IN2, 0);  // LOW
    gpio_set_level(FR_IN1, 0);  // HIGH
    gpio_set_level(FR_IN2, 0);  // LOW
    gpio_set_level(RL_IN1, 0);  // HIGH
    gpio_set_level(RL_IN2, 0);  // LOW
    gpio_set_level(RR_IN1, 0);  // HIGH
    gpio_set_level(RR_IN2, 0);  // LOW
}

void blink_led(uint8_t s_led_state)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(13, s_led_state);
}