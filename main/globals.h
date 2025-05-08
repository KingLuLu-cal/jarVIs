#ifndef GLOBALS_H
#define GLOBALS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "pins.h"

// Task handles
extern TaskHandle_t blink_task_handle;
extern TaskHandle_t motor_control_task_handle;
extern TaskHandle_t top_sensor_task_handle;
extern TaskHandle_t front_sensor_task_handle;
extern TaskHandle_t echo_task_handle;
extern TaskHandle_t bt_task_handle;

// Shared variables
extern uint8_t s_led_state;
extern uint32_t flash_period;
extern uint32_t flash_period_dec;

// Sensor readings
extern float top_distance;
extern float front_distance;

// Mutex for protecting access to the shared sensor data
extern SemaphoreHandle_t sensor_mutex;

// Queue for BT commands
extern QueueHandle_t bt_cmd_queue;
#define BT_CMD_QUEUE_SIZE 10
#define BT_CMD_MAX_LEN 32

// Flag for manual intervention
extern volatile bool manual_control_enabled;

#endif // GLOBALS_H