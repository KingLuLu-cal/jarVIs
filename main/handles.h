#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t blink_task_handle;
extern TaskHandle_t motor_control_task_handle;
extern TaskHandle_t top_sensor_task_handle;
extern TaskHandle_t front_sensor_task_handle;
extern TaskHandle_t echo_task_handle;
extern TaskHandle_t bt_task_handle;