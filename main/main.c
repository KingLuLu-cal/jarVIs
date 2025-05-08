/* UART Echo Example with Multitasking

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp32/rom/ets_sys.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include <inttypes.h>
#include "pins.h"
#include "manual.h"
#include "handles.h"
#include "drivers.h"



TaskHandle_t blink_task_handle;
TaskHandle_t motor_control_task_handle;
TaskHandle_t top_sensor_task_handle;
TaskHandle_t front_sensor_task_handle;
TaskHandle_t echo_task_handle;
TaskHandle_t bt_task_handle;

static uint8_t s_led_state = 1;
static uint32_t flash_period = DEFAULT_PERIOD;
static uint32_t flash_period_dec = DEFAULT_PERIOD/10;

// Global variables to store sensor readings
static float top_distance = MAX_DISTANCE;
static float front_distance = MAX_DISTANCE;

// Mutex for protecting access to the shared sensor data
SemaphoreHandle_t sensor_mutex = NULL;

// Flag for manual intervention
static volatile bool manual_control_enabled = false;


#define BUF_SIZE (1024)
#define TAG "ROBOT_CONTROL"

static void blink_task(void *arg)
{
    while(1)
    {
        s_led_state = !s_led_state;
        blink_led(s_led_state);
        vTaskDelay(flash_period / portTICK_PERIOD_MS);
    }
}

static void top_sensor_task(void *arg)
{
    ESP_LOGI(TAG, "Top sensor task started");
    while(1)
    {
        float current_distance = get_distance_cm(TRIG_PIN, ECHO_PIN);
        
        // Update the shared variable with mutex protection
        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            top_distance = current_distance;
            xSemaphoreGive(sensor_mutex);
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);  // 50ms sampling rate
    }
}

static void front_sensor_task(void *arg)
{
    ESP_LOGI(TAG, "Front sensor task started");
    while(1)
    {
        float current_distance = get_distance_cm(TRIG__front, ECHO__front);
        
        // Update the shared variable with mutex protection
        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            front_distance = current_distance;
            xSemaphoreGive(sensor_mutex);
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);  // 50ms sampling rate
    }
}

static void motor_control_task(void *arg)
{
    ESP_LOGI(TAG, "Motor control task started");
    float top = -1.0; 
    float front = -1.0;
    char msg[128];
    
    while (1)
    {
        // Check if manual control is enabled
        if (manual_control_enabled) {
            // Just wait and check again
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        
        // Get current sensor readings with mutex protection
        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            top = top_distance;
            front = front_distance;
            xSemaphoreGive(sensor_mutex);
        }
        
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
                // No top detection → forward at slow
                move_forward(180);
                snprintf(msg, sizeof(msg), "Top: %.2f cm, Front: %.2f cm -> EXPLORING\n", top, front);
            }
        }

        uart_write_bytes(ECHO_UART_PORT_NUM, msg, strlen(msg));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Initialize mutex for sensor data protection
    sensor_mutex = xSemaphoreCreateMutex();
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // Install UART driver and create UART event queue
    uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, NULL, 0);
    uart_param_config(ECHO_UART_PORT_NUM, &uart_config);
    uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    
    // LED setup
    gpio_reset_pin(13);
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    blink_led(s_led_state);
    
    // Sensor pins setup
    // TOP sensor
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    // Front sensor
    gpio_set_direction(TRIG__front, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO__front, GPIO_MODE_INPUT);
    
    // Initialize motors
    init_mecanum_motors();
    
    // Create tasks
    xTaskCreate(blink_task, "blink_LED", 1024, NULL, 5, &blink_task_handle);
    xTaskCreate(top_sensor_task, "top_sensor", 2048, NULL, 6, &top_sensor_task_handle);
    xTaskCreate(front_sensor_task, "front_sensor", 2048, NULL, 6, &front_sensor_task_handle);
    xTaskCreate(motor_control_task, "motor_control", 2048, NULL, 5, &motor_control_task_handle);
    xTaskCreate(echo_task, "uart_event", 2048, NULL, 7, &echo_task_handle);
    start_bt(); // Start Bluetooth task
    ESP_LOGI(TAG, "All tasks created successfully");
}