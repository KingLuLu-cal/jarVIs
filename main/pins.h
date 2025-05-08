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

// For FL Motor
#define FL_IN1 12  // Ain1
#define FL_IN2 21  // Ain2

// For FR Motor
#define FR_IN1 22  // Ain1
#define FR_IN2 27  // Ain2

// // For RL Motor
#define RL_IN1 32  // Bin1
#define RL_IN2 33  // Bin2

// For RR Motor
#define RR_IN1 14  // Bin1
#define RR_IN2 20  // Bin2

// top
#define TRIG_PIN 25 // represent A1 in code
#define ECHO_PIN 26 // represent A0 in code

// front
#define TRIG__front 4 // A5
#define ECHO__front 36 // A4