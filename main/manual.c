/* UART and BLE Example with Multitasking

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
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp32/rom/ets_sys.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include <inttypes.h>
#include "pins.h"

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

// Initialize the BT command queue
QueueHandle_t bt_cmd_queue = NULL;

#define BUF_SIZE (1024)
#define TAG "ROBOT_CONTROL"

char *BLE_TAG = "BLE-Server";
uint8_t ble_addr_type;
void ble_app_advertise(void);

// Write data to ESP32 defined as server
int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char bt_data[BT_CMD_MAX_LEN];
    
    // Copy the received data with null termination
    if (ctxt->om->om_len < BT_CMD_MAX_LEN) {
        memcpy(bt_data, ctxt->om->om_data, ctxt->om->om_len);
        bt_data[ctxt->om->om_len] = '\0';
        
        // Send the command to the BT command processing queue
        if (bt_cmd_queue != NULL) {
            xQueueSend(bt_cmd_queue, bt_data, portMAX_DELAY);
            ESP_LOGI(BLE_TAG, "BT command queued: %s", bt_data);
        }
    } else {
        ESP_LOGW(BLE_TAG, "BT command too long, ignored");
    }
    
    return 0;
}

// Read data from ESP32 defined as server
static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Read sensor data with mutex protection
    float top = -1.0, front = -1.0;
    if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
        top = top_distance;
        front = front_distance;
        xSemaphoreGive(sensor_mutex);
    }
    
    // Format sensor data for response
    char response[64];
    snprintf(response, sizeof(response), "Top: %.2f cm, Front: %.2f cm", top, front);
    
    os_mbuf_append(ctxt->om, response, strlen(response));
    return 0;
}

// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xFEF4),           // Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT ADV COMPLETE");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite BLE host task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

// Task to process Bluetooth commands
static void bt_task(void *arg)
{
    ESP_LOGI(TAG, "BT command processing task started");
    char bt_data[BT_CMD_MAX_LEN];
    uint8_t speed = 0;
    
    while (1) {
        // Wait for a command from the queue
        if (xQueueReceive(bt_cmd_queue, &bt_data, portMAX_DELAY)) {
            ESP_LOGI(TAG, "BT command received: %s", bt_data);
            
            // Process special commands first
            if (strcmp(bt_data, "MANUAL_START") == 0) {
                ESP_LOGI(TAG, "Manual control enabled via BT");
                manual_control_enabled = true;
                vTaskSuspend(motor_control_task_handle);
                continue;
            } 
            else if (strcmp(bt_data, "MANUAL_STOP") == 0) {
                ESP_LOGI(TAG, "Manual control disabled via BT");
                manual_control_enabled = false;
                vTaskResume(motor_control_task_handle);
                continue;
            }
            
            // For manual movement commands
            if (manual_control_enabled) {
                // Handle the same commands as in the UART task
                if (strcmp(bt_data, "LIGHT ON") == 0) {
                    s_led_state = 1;
                    blink_led(s_led_state);
                    ESP_LOGI(TAG, "BT: Light turned ON");
                }
                else if (strcmp(bt_data, "LIGHT OFF") == 0) {
                    s_led_state = 0;
                    blink_led(s_led_state);
                    ESP_LOGI(TAG, "BT: Light turned OFF");
                }
                else if (strcmp(bt_data, "FAN ON") == 0) {
                    // For demonstration, we'll reuse this command to resume the blink task
                    vTaskResume(blink_task_handle);
                    ESP_LOGI(TAG, "BT: Blink task resumed");
                }
                else if (strcmp(bt_data, "FAN OFF") == 0) {
                    // For demonstration, we'll reuse this command to suspend the blink task
                    vTaskSuspend(blink_task_handle);
                    ESP_LOGI(TAG, "BT: Blink task suspended");
                }
                // Movement commands
                else if (strcmp(bt_data, "FORWARD") == 0) {
                    move_forward(250);
                    ESP_LOGI(TAG, "BT: Moving Forward");
                }
                else if (strcmp(bt_data, "BACKWARD") == 0) {
                    move_backward(250);
                    ESP_LOGI(TAG, "BT: Moving Backward");
                }
                else if (strcmp(bt_data, "LEFT") == 0) {
                    move_left(250);
                    ESP_LOGI(TAG, "BT: Strafing Left");
                }
                else if (strcmp(bt_data, "RIGHT") == 0) {
                    move_right(250);
                    ESP_LOGI(TAG, "BT: Strafing Right");
                }
                else if (strcmp(bt_data, "ROTATE_CCW") == 0) {
                    rotate_counterclockwise(250);
                    ESP_LOGI(TAG, "BT: Rotating CCW");
                }
                else if (strcmp(bt_data, "ROTATE_CW") == 0) {
                    rotate_clockwise(250);
                    ESP_LOGI(TAG, "BT: Rotating CW");
                }
                else if (strcmp(bt_data, "STOP") == 0) {
                    stop_all_motors();
                    ESP_LOGI(TAG, "BT: Mecanum Motors Stopped");
                }
                // Sensor readings
                else if (strcmp(bt_data, "READ_TOP") == 0) {
                    float distance = -1.0;
                    if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
                        distance = top_distance;
                        xSemaphoreGive(sensor_mutex);
                    }
                    ESP_LOGI(TAG, "BT: Top sensor reading: %.2f cm", distance);
                }
                else if (strcmp(bt_data, "READ_FRONT") == 0) {
                    float distance = -1.0;
                    if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
                        distance = front_distance;
                        xSemaphoreGive(sensor_mutex);
                    }
                    ESP_LOGI(TAG, "BT: Front sensor reading: %.2f cm", distance);
                }
                else {
                    ESP_LOGW(TAG, "BT: Unknown command: %s", bt_data);
                }
            }
        }
    }
}

void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
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

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint8_t speed = 0;
    uart_write_bytes(ECHO_UART_PORT_NUM, "Commands\n", strlen("Commands\n"));
    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        
        if (len)
        {
            data[len] = '\0';
            // Echo back for confirmation
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
            
            switch(data[0])
            {
                case 'I':
                    s_led_state = 1;
                    blink_led(s_led_state);
                    uart_write_bytes(ECHO_UART_PORT_NUM, "ESP32\n", strlen("ESP32\n"));
                    break;
                case 'T':
                    flash_period -= flash_period_dec;
                    if(flash_period <= flash_period_dec) flash_period = flash_period_dec;
                    break;
                case 'B':
                    vTaskSuspend(blink_task_handle);
                    break;
                case 'R':
                    flash_period = DEFAULT_PERIOD;
                    break;
                case 'M':  // Manual control toggle
                    if (manual_control_enabled) {
                        manual_control_enabled = false;
                        vTaskResume(motor_control_task_handle);
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Auto mode enabled\n", strlen("Auto mode enabled\n"));
                    } else {
                        manual_control_enabled = true;
                        vTaskSuspend(motor_control_task_handle);
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Manual mode enabled\n", strlen("Manual mode enabled\n"));
                    }
                    break;
                // === Mecanum movement ===
                case 'W':
                    if (manual_control_enabled) {
                        move_forward(speed);
                        speed = (speed + 10) % 255;
                        if (speed < 100) speed = 100;  // Minimum speed for movement
                        char msg3[64];
                        snprintf(msg3, sizeof(msg3), "Speed: %" PRIu8 "\n", speed);
                        uart_write_bytes(ECHO_UART_PORT_NUM, msg3, strlen(msg3));
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Moving Forward\n", strlen("Moving Forward\n"));
                    } else {
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Enable manual mode first\n", 
                                        strlen("Enable manual mode first\n"));
                    }
                    break;
                case 'S':
                    if (manual_control_enabled) {
                        move_backward(250);
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Moving Backward\n", strlen("Moving Backward\n"));
                    } else {
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Enable manual mode first\n", 
                                        strlen("Enable manual mode first\n"));
                    }
                    break;
                case 'A':
                    if (manual_control_enabled) {
                        move_left(250);
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Strafing Left\n", strlen("Strafing Left\n"));
                    } else {
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Enable manual mode first\n", 
                                        strlen("Enable manual mode first\n"));
                    }
                    break;
                case 'D':
                    if (manual_control_enabled) {
                        move_right(250);
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Strafing Right\n", strlen("Strafing Right\n"));
                    } else {
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Enable manual mode first\n", 
                                        strlen("Enable manual mode first\n"));
                    }
                    break;
                case 'Q':
                    if (manual_control_enabled) {
                        rotate_counterclockwise(250);
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Rotating CCW\n", strlen("Rotating CCW\n"));
                    } else {
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Enable manual mode first\n", 
                                        strlen("Enable manual mode first\n"));
                    }
                    break;
                case 'E':
                    if (manual_control_enabled) {
                        rotate_clockwise(250);
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Rotating CW\n", strlen("Rotating CW\n"));
                    } else {
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Enable manual mode first\n", 
                                        strlen("Enable manual mode first\n"));
                    }
                    break;
                case 'X':
                    if (manual_control_enabled) {
                        stop_all_motors();
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Motors Stopped\n", strlen("Motors Stopped\n"));
                    } else {
                        uart_write_bytes(ECHO_UART_PORT_NUM, "Enable manual mode first\n", 
                                        strlen("Enable manual mode first\n"));
                    }
                    break;
                case 'O': 
                    {
                        float dist = -1.0;
                        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
                            dist = top_distance;
                            xSemaphoreGive(sensor_mutex);
                        }
                        char msg2[64];
                        if (dist < 0) {
                            snprintf(msg2, sizeof(msg2), "Top sensor: Out of range\n");
                        } else {
                            snprintf(msg2, sizeof(msg2), "Top sensor: %.2f cm\n", dist);
                        }
                        uart_write_bytes(ECHO_UART_PORT_NUM, msg2, strlen(msg2));
                    }
                    break;
                case 'F': 
                    {
                        float dist = -1.0;
                        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
                            dist = front_distance;
                            xSemaphoreGive(sensor_mutex);
                        }
                        char msg[64];
                        if (dist < 0) {
                            snprintf(msg, sizeof(msg), "Front sensor: Out of range\n");
                        } else {
                            snprintf(msg, sizeof(msg), "Front sensor: %.2f cm\n", dist);
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

void start_bt() {
    // Create queue for BT commands
    bt_cmd_queue = xQueueCreate(BT_CMD_QUEUE_SIZE, BT_CMD_MAX_LEN);
    
    // Initialize BLE
    nimble_port_init();                        // Initialize the host stack
    ble_svc_gap_device_name_set("ESP32-Robot"); // Set BLE device name
    ble_svc_gap_init();                        // Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // Initialize application
    
    // Create BT command processing task
    xTaskCreate(bt_task, "bt_cmd_task", 2048, NULL, 5, &bt_task_handle);
    
    // Start the BLE host task
    nimble_port_freertos_init(host_task);    
}