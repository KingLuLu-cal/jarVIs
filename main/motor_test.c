// #include <stdio.h>
// #include "string.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/uart.h"
// #include "driver/gpio.h"
// #include "sdkconfig.h"
// #include "esp_log.h"
// #include "esp32/rom/ets_sys.h"
// #include "esp_timer.h"
// #include "driver/ledc.h"

// // Pin definitions
// #define FL_IN1 12
// // #define FL_IN2 21

// #define FR_IN1 22
// #define FR_IN2 27

// #define RL_IN1 32
// #define RL_IN2 33

// #define RR_IN1 14
// #define RR_IN2 15

// #define TEST_DURATION_MS 2000
// #define PWM_DUTY 128  // 50% of 255 (8-bit resolution)

// void set_motor(gpio_num_t in1, gpio_num_t in2, bool forward, uint32_t speed, ledc_channel_t pwm_channel) {
//     gpio_set_level(in1, forward ? 1 : 0);
//     gpio_set_level(in2, forward ? 0 : 1);
//     ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, speed);
//     ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel);
// }

// void stop_all_motors() {
//     for (int ch = LEDC_CHANNEL_0; ch <= LEDC_CHANNEL_3; ch++) {
//         ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
//         ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
//     }
// }

// // void app_main(void) {


// //     // Set all direction pins as outputs
// //     gpio_set_direction(FL_IN2, GPIO_MODE_OUTPUT);
// //     gpio_set_direction(FR_IN2, GPIO_MODE_OUTPUT);
// //     gpio_set_direction(RL_IN2, GPIO_MODE_OUTPUT);
// //     gpio_set_direction(RR_IN2, GPIO_MODE_OUTPUT);

// //     // PWM timer setup
// //     ledc_timer_config_t pwm_timer = {
// //         .speed_mode       = LEDC_LOW_SPEED_MODE,
// //         .timer_num        = LEDC_TIMER_1,
// //         .duty_resolution  = LEDC_TIMER_8_BIT,
// //         .freq_hz          = 20000,
// //         .clk_cfg          = LEDC_AUTO_CLK
// //     };
// //     ledc_timer_config(&pwm_timer);

// //     // PWM channel setup
// //     ledc_channel_config_t motor_channels[] = {
// //         {.channel = LEDC_CHANNEL_0, .gpio_num = FL_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_1},
// //         {.channel = LEDC_CHANNEL_1, .gpio_num = FR_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_1},
// //         {.channel = LEDC_CHANNEL_2, .gpio_num = RL_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_1},
// //         {.channel = LEDC_CHANNEL_3, .gpio_num = RR_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_1}
// //     };

// //     for (int i = 0; i < 4; i++) {
// //         ledc_channel_config(&motor_channels[i]);
// //     }

// //     // Test motors one by one
// //     // DOES NOT MOVE
// //     printf("Testing Front Left (FL) motor...\n");
// //     set_motor(FL_IN1, FL_IN2, true, PWM_DUTY, LEDC_CHANNEL_0);
// //     vTaskDelay(TEST_DURATION_MS / portTICK_PERIOD_MS);
// //     stop_all_motors();

// //     // DOES NOT MOVE
// //     printf("Testing Front Right (FR) motor...\n");
// //     set_motor(FR_IN1, FR_IN2, true, PWM_DUTY, LEDC_CHANNEL_1);
// //     vTaskDelay(TEST_DURATION_MS / portTICK_PERIOD_MS);
// //     stop_all_motors();

// //     printf("Testing Rear Left (RL) motor...\n");
// //     set_motor(RL_IN1, RL_IN2, true, PWM_DUTY, LEDC_CHANNEL_2);
// //     vTaskDelay(TEST_DURATION_MS / portTICK_PERIOD_MS);
// //     stop_all_motors();

// //     // printf("Testing Rear Right (RR) motor...\n");
// //     // set_motor(RR_IN1, RR_IN2, true, PWM_DUTY, LEDC_CHANNEL_3);
// //     // vTaskDelay(TEST_DURATION_MS / portTICK_PERIOD_MS);
// //     // stop_all_motors();

// //     printf("Motor test complete.\n");
// // }
