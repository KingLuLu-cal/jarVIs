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

// Define the current speed as a global variable
uint32_t current_speed = 180;  // Default speed (midpoint of 120-250)


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

static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Choose sensor based on `arg`
    const char* sensor_type = (const char*)arg;

    float value = -1.0;
    if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
        if (strcmp(sensor_type, "TOP") == 0) {
            value = top_distance;
        } else if (strcmp(sensor_type, "FRONT") == 0) {
            value = front_distance;
        }
        xSemaphoreGive(sensor_mutex);
    }

    char response[64];
    snprintf(response, sizeof(response), "%s: %.2f cm", sensor_type, value);
    os_mbuf_append(ctxt->om, response, strlen(response));
    return 0;
}

// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180), // Custom service
     .characteristics = (struct ble_gatt_chr_def[]){
        {
            .uuid = BLE_UUID128_DECLARE(0x12,0x34,0x56,0x78,0x90,0xab,0xcd,0xef,0xfe,0xdc,0xba,0x09,0x87,0x65,0x43,0x21),
            .flags = BLE_GATT_CHR_F_READ,
            .access_cb = device_read,
            .arg = (void*)"TOP"
        },
        {
            .uuid = BLE_UUID128_DECLARE(0x21,0x43,0x65,0x87,0x09,0xba,0xdc,0xfe,0xef,0xcd,0xab,0x90,0x78,0x56,0x34,0x12),
            .flags = BLE_GATT_CHR_F_READ,
            .access_cb = device_read,
            .arg = (void*)"FRONT"
         },
        {
            .uuid = BLE_UUID16_DECLARE(0xDEAD),
            .flags = BLE_GATT_CHR_F_WRITE,
            .access_cb = device_write
        },
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

            // Handle speed update command
            if (bt_data[0] == 'V') {  // Speed update command
                uint32_t new_speed = atoi(&bt_data[1]);  // Extract speed value
                if (new_speed >= 120 && new_speed <= 250) {
                    current_speed = new_speed;
                    ESP_LOGI(TAG, "BT: Speed updated to %lu", current_speed);
                } else {
                    ESP_LOGW(TAG, "BT: Invalid speed value: %lu", new_speed);
                }
                continue;
            }

            // For manual movement commands
            if (manual_control_enabled) {
                if (strcmp(bt_data, "FORWARD") == 0) {
                    move_forward(current_speed);
                    ESP_LOGI(TAG, "BT: Moving Forward");
                }
                else if (strcmp(bt_data, "BACKWARD") == 0) {
                    move_backward(current_speed);
                    ESP_LOGI(TAG, "BT: Moving Backward");
                }
                else if (strcmp(bt_data, "LEFT") == 0) {
                    move_left(current_speed);
                    ESP_LOGI(TAG, "BT: Strafing Left");
                }
                else if (strcmp(bt_data, "RIGHT") == 0) {
                    move_right(current_speed);
                    ESP_LOGI(TAG, "BT: Strafing Right");
                }
                else if (strcmp(bt_data, "ROTATE_CCW") == 0) {
                    rotate_counterclockwise(current_speed);
                    ESP_LOGI(TAG, "BT: Rotating CCW");
                }
                else if (strcmp(bt_data, "ROTATE_CW") == 0) {
                    rotate_clockwise(current_speed);
                    ESP_LOGI(TAG, "BT: Rotating CW");
                }
                else if (strcmp(bt_data, "STOP") == 0) {
                    stop_all_motors();
                    ESP_LOGI(TAG, "BT: Mecanum Motors Stopped");
                }
                else {
                    ESP_LOGW(TAG, "BT: Unknown command: %s", bt_data);
                }
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