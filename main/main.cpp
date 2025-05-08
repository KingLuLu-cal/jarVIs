/* UART Echo Example with Mecanum Wheel Robot

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
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

// top sensor
#define TRIG_PIN 25 // represent A1 in code
#define ECHO_PIN 26 // represent A0 in code
// front sensor
#define TRIG_FRONT 4 // A5
#define ECHO_FRONT 36 // A4

// Motor pin definitions
// For FL Motor (Front Left)
#define FL_IN1 12  // Ain1
#define FL_IN2 21  // Ain2

// For FR Motor (Front Right)
#define FR_IN1 22  // Ain1
#define FR_IN2 27  // Ain2

// For RL Motor (Rear Left)
#define RL_IN1 32  // Bin1
#define RL_IN2 33  // Bin2

// For RR Motor (Rear Right)
#define RR_IN1 14  // Bin1
#define RR_IN2 20  // Bin2

// UART configuration
#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define DEFAULT_PERIOD 1000
#define MAX_DISTANCE 300
#define MIN_DISTANCE 5
#define BUF_SIZE (1024)

// Global variables
static uint8_t s_led_state = 1;
static uint32_t flash_period = DEFAULT_PERIOD;
static float distance = MAX_DISTANCE;
static uint32_t flash_period_dec = DEFAULT_PERIOD/10;

// Task handle
TaskHandle_t myTaskHandle = NULL;

// Function prototypes
static float get_distance_cm(uint32_t trig, uint32_t echo);
static void blink_led(void);
static void blink_task(void *arg);
static void read_distance(void *arg);
static void motor_control(void *arg);
static void echo_task(void *arg);
void init_mecanum_motors();
void set_motor(gpio_num_t in1, gpio_num_t in2, bool forward, uint32_t speed, ledc_channel_t pwm_channel);

// Mecanum movement functions
void move_forward(uint32_t speed);
void move_backward(uint32_t speed);
void move_left(uint32_t speed);
void move_right(uint32_t speed);
void rotate_clockwise(uint32_t speed);
void rotate_counterclockwise(uint32_t speed);
void stop_all_motors(void);

// Function implementations
static void blink_led(void)
{
    // Set the GPIO level according to the state (LOW or HIGH)
    gpio_set_level(13, s_led_state);
}

static void blink_task(void *arg)
{
    while(1)
    {
        s_led_state = !s_led_state;
        blink_led();
        vTaskDelay(flash_period / portTICK_PERIOD_MS);
    }
}

static void read_distance(void *arg)
{
    while(1)
    {
        distance = get_distance_cm(TRIG_PIN, ECHO_PIN);
        vTaskDelay(17 / portTICK_PERIOD_MS);
    }
}

static void motor_control(void *arg)
{
    while (1)
    {
        float top = get_distance_cm(TRIG_PIN, ECHO_PIN);         // top sensor
        float front = get_distance_cm(TRIG_FRONT, ECHO_FRONT);   // front sensor
        char msg[128];

        // Case 1: Obstacle in front
        if (front > 0 && front < 20) {
            if (top > MIN_DISTANCE && top < 50) {
                // Top object detected → avoid front obstacle → go left
                move_left(250);
                snprintf(msg, sizeof(msg), "Top: %.2f cm, Front: %.2f cm -> GO LEFT\n", top, front);
            } else {
                // No top object → stop
                stop_all_motors();
                snprintf(msg, sizeof(msg), "Top: %.2f cm, Front: %.2f cm -> STOP\n", top, front);
            }
        } else {
            // No obstacle in front
            if (top > MIN_DISTANCE && top < 50) {
                // Top object detected → go forward (scale speed by distance)
                uint32_t duty = (MAX_DISTANCE - top) / (MAX_DISTANCE - MIN_DISTANCE) * 255;
                move_forward(duty);
                snprintf(msg, sizeof(msg), "Top: %.2f cm, Front: %.2f cm -> FORWARD at %" PRIu32 "\n", top, front, duty);
            } else {
                // No top detection → forward at slow speed
                move_forward(180);
                snprintf(msg, sizeof(msg), "Top: %.2f cm, Front: %.2f cm -> EXPLORING\n", top, front);
            }
        }

        uart_write_bytes(ECHO_UART_PORT_NUM, msg, strlen(msg));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void echo_task(void *arg)
{
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Buffer for incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint8_t speed = 0;
    
    uart_write_bytes(ECHO_UART_PORT_NUM, "Commands Ready\n", strlen("Commands Ready\n"));
    
    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        
        // Write data back to the UART
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
        
        if (len)
        {
            data[len] = '\0';
            switch(data[0])
            {
                case 'I':
                    s_led_state = 1;
                    blink_led();
                    uart_write_bytes(ECHO_UART_PORT_NUM, "ESP32\n", strlen("ESP32\n"));
                    break;
                case 'T':
                    flash_period -= flash_period_dec;
                    if(flash_period <= flash_period_dec) flash_period = flash_period_dec;
                    break;
                case 'B':
                    vTaskSuspend(myTaskHandle);
                    break;
                case 'R':
                    flash_period = DEFAULT_PERIOD;
                    break;
                // Mecanum movement commands
                case 'W':
                    move_forward(speed);
                    speed = (speed + 1) % 255;
                    char msg3[64];
                    snprintf(msg3, sizeof(msg3), "Speed: %" PRIu8"\n", speed);
                    uart_write_bytes(ECHO_UART_PORT_NUM, msg3, strlen(msg3));
                    uart_write_bytes(ECHO_UART_PORT_NUM, "Moving Forward\n", strlen("Moving Forward\n"));
                    break;
                case 'S':
                    move_backward(250);
                    uart_write_bytes(ECHO_UART_PORT_NUM, "Moving Backward\n", strlen("Moving Backward\n"));
                    break;
                case 'A':
                    move_left(250);
                    uart_write_bytes(ECHO_UART_PORT_NUM, "Strafing Left\n", strlen("Strafing Left\n"));
                    break;
                case 'D':
                    move_right(250);
                    uart_write_bytes(ECHO_UART_PORT_NUM, "Strafing Right\n", strlen("Strafing Right\n"));
                    break;
                case 'Q':
                    rotate_counterclockwise(250);
                    uart_write_bytes(ECHO_UART_PORT_NUM, "Rotating CCW\n", strlen("Rotating CCW\n"));
                    break;
                case 'E':
                    rotate_clockwise(250);
                    uart_write_bytes(ECHO_UART_PORT_NUM, "Rotating CW\n", strlen("Rotating CW\n"));
                    break;
                case 'X':
                    stop_all_motors();
                    uart_write_bytes(ECHO_UART_PORT_NUM, "Motors Stopped\n", strlen("Motors Stopped\n"));
                    break;
                case 'O': 
                    {
                        float distance_2 = get_distance_cm(TRIG_PIN, ECHO_PIN);
                        char msg2[64];
                        if (distance_2 < 0) {
                            snprintf(msg2, sizeof(msg2), "Out of range\n");
                        } else {
                            snprintf(msg2, sizeof(msg2), "Distance: %.2f cm\n", distance_2);
                        }
                        uart_write_bytes(ECHO_UART_PORT_NUM, msg2, strlen(msg2));
                    }
                    break;
                case 'F': 
                    {
                        float distance = get_distance_cm(TRIG_FRONT, ECHO_FRONT);
                        char msg[64];
                        if (distance < 0) {
                            snprintf(msg, sizeof(msg), "Out of range\n");
                        } else {
                            snprintf(msg, sizeof(msg), "Distance: %.2f cm\n", distance);
                        }
                        uart_write_bytes(ECHO_UART_PORT_NUM, msg, strlen(msg));
                    }
                    break;
                default:
                    break;  
            }
        }
    }
}

static float get_distance_cm(uint32_t trig, uint32_t echo)
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
    // Set all IN2 pins as digital outputs
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
        gpio_set_level(in1, 0);  // LOW
        gpio_set_level(in2, 1);  // HIGH
    }

    if(!forward){
        speed = 255 - speed; // Invert speed for backward movement
    }
    
    // Apply PWM to IN1
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel);
}

// Mecanum wheel movement implementations
void move_forward(uint32_t speed) {
    set_motor(FL_IN1, FL_IN2, false, speed, LEDC_CHANNEL_0);
    set_motor(FR_IN1, FR_IN2, false, speed, LEDC_CHANNEL_1);
    set_motor(RL_IN1, RL_IN2, false, speed, LEDC_CHANNEL_2);
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
    
    // Set all motor control pins to LOW
    gpio_set_level(FL_IN1, 0);
    gpio_set_level(FL_IN2, 0);
    gpio_set_level(FR_IN1, 0);
    gpio_set_level(FR_IN2, 0);
    gpio_set_level(RL_IN1, 0);
    gpio_set_level(RL_IN2, 0);
    gpio_set_level(RR_IN1, 0);
    gpio_set_level(RR_IN2, 0);
}

// Main application entry point wrapped properly for C++
extern "C" void app_main(void)
{
    // Initialize LED
    gpio_reset_pin(13);
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    blink_led();
    xTaskCreate(blink_task, "blink_LED", 1024, NULL, 5, &myTaskHandle);

    // Initialize ultrasonic sensors
    // TOP sensor
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    // FRONT sensor
    gpio_set_direction(TRIG_FRONT, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_FRONT, GPIO_MODE_INPUT);
    xTaskCreate(read_distance, "read_distance", 2048, NULL, 5, NULL);

    // Initialize motors and create tasks
    init_mecanum_motors();
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(motor_control, "motor", 2048, NULL, 5, NULL);
}