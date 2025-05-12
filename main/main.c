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
#include "drivers.h"
#include "globals.h"

// BLE includes
#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "drivers.h"
#include "globals.h"

// Define all the global variables declared in globals.h
TaskHandle_t blink_task_handle = NULL;
TaskHandle_t motor_control_task_handle = NULL;
TaskHandle_t top_sensor_task_handle = NULL;
TaskHandle_t front_sensor_task_handle = NULL;
TaskHandle_t echo_task_handle = NULL;
TaskHandle_t bt_task_handle = NULL;

uint8_t s_led_state = 1;
uint32_t flash_period = DEFAULT_PERIOD;
uint32_t flash_period_dec = DEFAULT_PERIOD/10;

// Global variables to store sensor readings
float top_distance = MAX_DISTANCE;
float front_distance = MAX_DISTANCE;

// Mutex for protecting access to the shared sensor data
SemaphoreHandle_t sensor_mutex = NULL;

// Flag for manual intervention

#define BUF_SIZE (1024)
#define TAG "ROBOT_CONTROL"

volatile bool manual_control_enabled = true;

static void blink_task(void *arg)
{
    while(1)
    {
        s_led_state = !s_led_state;
        blink_led(s_led_state);
        vTaskDelay(flash_period / portTICK_PERIOD_MS);
    }
}


// Top sensor
volatile int64_t echo_start_top = 0;
volatile int64_t echo_end_top = 0;
volatile bool distance_ready_top = false;

// Front sensor
volatile int64_t echo_start_front = 0;
volatile int64_t echo_end_front = 0;
volatile bool distance_ready_front = false;

static void IRAM_ATTR echo_isr_handler(void* arg)
{
    uint32_t pin = (uint32_t) arg;
    int64_t now = esp_timer_get_time();

    if (pin == ECHO_PIN) {
        if (gpio_get_level(ECHO_PIN)) {
            echo_start_top = now;
        } else {
            echo_end_top = now;
            distance_ready_top = true;
        }
    } else if (pin == ECHO__front) {
        if (gpio_get_level(ECHO__front)) {
            echo_start_front = now;
        } else {
            echo_end_front = now;
            distance_ready_front = true;
        }
    }
}

void setup_echo_interrupts()
{
    // Configure ECHO pins as inputs with interrupts
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .pin_bit_mask = (1ULL << ECHO_PIN) | (1ULL << ECHO__front),
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO_PIN, echo_isr_handler, (void*) ECHO_PIN);
    gpio_isr_handler_add(ECHO__front, echo_isr_handler, (void*) ECHO__front);
}

static void top_sensor_task(void *arg)
{
    ESP_LOGI(TAG, "Top sensor task started");

    while (1) {
        distance_ready_top = false;

        // Trigger ultrasonic pulse
        gpio_set_level(TRIG_PIN, 0);
        ets_delay_us(2);
        gpio_set_level(TRIG_PIN, 1);
        ets_delay_us(10);
        gpio_set_level(TRIG_PIN, 0);

        // Wait until echo ISR signals that distance is ready
        int timeout_us = 30000;
        int64_t start = esp_timer_get_time();
        while (!distance_ready_top) {
            if ((esp_timer_get_time() - start) > timeout_us) {
                ESP_LOGW(TAG, "Top sensor timeout");
                break;
            }
        }

        float distance = -1;
        if (distance_ready_top) {
            int64_t duration = echo_end_top - echo_start_top;
            distance = duration * 0.034 / 2.0;
        }

        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            top_distance = distance;
            xSemaphoreGive(sensor_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(17));  // 50 ms delay
    }
}

static void front_sensor_task(void *arg)
{
    ESP_LOGI(TAG, "Front sensor task started");

    while (1) {
        distance_ready_front = false;

        gpio_set_level(TRIG__front, 0);
        ets_delay_us(2);
        gpio_set_level(TRIG__front, 1);
        ets_delay_us(10);
        gpio_set_level(TRIG__front, 0);

        int timeout_us = 30000;
        int64_t start = esp_timer_get_time();
        while (!distance_ready_front) {
            if ((esp_timer_get_time() - start) > timeout_us) {
                ESP_LOGW(TAG, "Front sensor timeout");
                break;
            }
        }

        float distance = -1;
        if (distance_ready_front) {
            int64_t duration = echo_end_front - echo_start_front;
            distance = duration * 0.034 / 2.0;
        }

        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            front_distance = distance;
            xSemaphoreGive(sensor_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(17));
    }
}


motor_state_t state = AE_STOP;

static void motor_control_task(void *arg)
{
    #define MOTOR_TAG "MOTOR_CTRL"
    ESP_LOGI(MOTOR_TAG, "Motor logic task started");

    float top = -1.0;
    float front = -1.0;

    while (1)
    {
        if (manual_control_enabled) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        // Acquire sensor values
        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            top = top_distance;
            front = front_distance;
            xSemaphoreGive(sensor_mutex);
        }

        // Determine state
        if (front > 0 && front < 40) {
            state = (top > MIN_DISTANCE && top < 50) ? AE_SCURRY : AE_STOP;
        } else {
            state = (top > MIN_DISTANCE && top < 50) ? AE_SPEED_AWAY : AE_SLOW;
        }

        // Act on state
        switch (state)
        {
            case AE_SCURRY:
                rotate_counterclockwise(250);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;

            case AE_STOP:
                stop_all_motors();
                break;

            case AE_SPEED_AWAY: {
                uint32_t duty = (MAX_DISTANCE - top) / (MAX_DISTANCE - MIN_DISTANCE) * 255;
                move_forward(duty);
                break;
            }

            case AE_SLOW:
            default:
                move_forward(current_speed);
                ESP_LOGI(MOTOR_TAG, "Moving forward at speed %lu", current_speed);
                break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


void app_main(void)
{
    // Initialize NVS for BLE
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // Initialize mutex for sensor data protection
    sensor_mutex = xSemaphoreCreateMutex();
    setup_echo_interrupts();

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
    xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 5, &motor_control_task_handle);
    // xTaskCreate(echo_task, "uart_event", 4096, NULL, 7, &echo_task_handle);
    
    // Start Bluetooth task
    start_bt();
    // Enable manual mode by default
    ESP_LOGI(TAG, "Manual control enabled by default");
    ESP_LOGI(TAG, "All tasks created successfully");
}
