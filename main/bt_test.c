/*
 * ESP32 Minimal Bluetooth Test with Improved Keepalive
 * Designed for better PC compatibility
 */

 #include <stdio.h>
 #include <string.h>
 #include <inttypes.h>  // For PRIu32 format specifier
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/timers.h"
 #include "esp_system.h"
 #include "esp_log.h"
 #include "nvs_flash.h"
 #include "esp_bt.h"
 #include "esp_bt_main.h"
 #include "esp_bt_device.h"
 #include "esp_gap_bt_api.h"
 #include "esp_spp_api.h"
 #include "driver/gpio.h"
 
 #define TAG "BT_TEST"
 #define LED_PIN 13  // Built-in LED on most ESP32 boards
 
 // Global variables to maintain connection
 static uint32_t spp_handle = 0;
 static bool is_connected = false;
 static TimerHandle_t keepalive_timer = NULL;
 
 // Keepalive timer callback
 static void keepalive_callback(TimerHandle_t xTimer) {
     if (is_connected && spp_handle > 0) {
         // Send a non-visible character as keepalive
         uint8_t keepalive = 0xFF;
         esp_spp_write(spp_handle, 1, &keepalive);
         ESP_LOGD(TAG, "Sent keepalive packet");
     }
 }
 
 // GAP callback function
 static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
     switch (event) {
         case ESP_BT_GAP_AUTH_CMPL_EVT:
             if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                 ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
             } else {
                 ESP_LOGE(TAG, "Authentication failed, status: %" PRIu8, param->auth_cmpl.stat);
             }
             break;
             
         default:
             break;
     }
 }
 
 // SPP callback function
 static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
     switch (event) {
         case ESP_SPP_INIT_EVT:
             ESP_LOGI(TAG, "SPP initialized");
             esp_bt_gap_set_device_name("ESP32_BT_TEST");
             esp_bt_gap_register_callback(esp_bt_gap_cb);
             esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
             esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP_TEST");
             gpio_set_level(LED_PIN, 1);  // Turn on LED when ready
             break;
             
         case ESP_SPP_DATA_IND_EVT:
             if (param->data_ind.len > 0) {
                 // Ignore keepalive packets (0xFF)
                 if (param->data_ind.data[0] != 0xFF) {
                     ESP_LOGI(TAG, "Data received, len=%" PRIu16 ", data[0]=0x%02x", 
                              param->data_ind.len, param->data_ind.data[0]);
                     
                     // Echo back the received data (helps with connection stability)
                     esp_spp_write(param->data_ind.handle, param->data_ind.len, param->data_ind.data);
                     
                     // If 'P' is received, respond with "Hello I am ESP32"
                     if (param->data_ind.data[0] == 'P') {
                         const char *response = "Hello I am ESP32\n";
                         esp_spp_write(param->data_ind.handle, strlen(response), (uint8_t *)response);
                         
                         // Toggle LED for visual feedback
                         static bool led_state = true;
                         led_state = !led_state;
                         gpio_set_level(LED_PIN, led_state);
                         
                         ESP_LOGI(TAG, "Sent: Hello I am ESP32");
                     }
                     
                     // Reset keepalive timer when we receive data
                     if (keepalive_timer != NULL) {
                         xTimerReset(keepalive_timer, 0);
                     }
                 }
             }
             break;
             
         case ESP_SPP_WRITE_EVT:
             // Reset keepalive timer when we write data
             if (keepalive_timer != NULL && is_connected) {
                 xTimerReset(keepalive_timer, 0);
             }
             break;
             
         case ESP_SPP_SRV_OPEN_EVT:
             ESP_LOGI(TAG, "Client connected, handle=%" PRIu32, param->srv_open.handle);
             spp_handle = param->srv_open.handle;
             is_connected = true;
             
             // Send an immediate welcome packet to establish the connection
             const char *welcome = "ESP32 Connected\n";
             esp_spp_write(param->srv_open.handle, strlen(welcome), (uint8_t *)welcome);
             
             // Create and start keepalive timer if not already created
             if (keepalive_timer == NULL) {
                 keepalive_timer = xTimerCreate(
                     "keepalive",
                     pdMS_TO_TICKS(500),  // Send keepalive every 500ms (faster)
                     pdTRUE,              // Auto reload
                     NULL,
                     keepalive_callback
                 );
             }
             
             if (keepalive_timer != NULL) {
                 xTimerStart(keepalive_timer, 0);
                 ESP_LOGI(TAG, "Started keepalive timer (500ms)");
             }
             
             // Blink the LED to indicate connection
             for (int i = 0; i < 3; i++) {
                 gpio_set_level(LED_PIN, 0);
                 vTaskDelay(100 / portTICK_PERIOD_MS);
                 gpio_set_level(LED_PIN, 1);
                 vTaskDelay(100 / portTICK_PERIOD_MS);
             }
             break;
             
         case ESP_SPP_CLOSE_EVT:
             ESP_LOGI(TAG, "Client disconnected, reason=%" PRIu8, param->close.status);
             spp_handle = 0;
             is_connected = false;
             
             // Stop keepalive timer
             if (keepalive_timer != NULL) {
                 xTimerStop(keepalive_timer, 0);
                 ESP_LOGI(TAG, "Stopped keepalive timer");
             }
             
             // Blink LED to indicate disconnection
             gpio_set_level(LED_PIN, 0);
             vTaskDelay(300 / portTICK_PERIOD_MS);
             gpio_set_level(LED_PIN, 1);  // Turn back on to show ready for connection
             break;
             
         case ESP_SPP_START_EVT:
             ESP_LOGI(TAG, "SPP server started");
             break;
             
         default:
             break;
     }
 }
 
 void app_main(void)
 {
     // Set up LED
     gpio_reset_pin(LED_PIN);
     gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
     gpio_set_level(LED_PIN, 0);  // Start with LED off
     
     ESP_LOGI(TAG, "Starting minimal BT test with improved keepalive");
     
     // Initialize NVS
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK(ret);
     
     // Initialize controller with dual mode
     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
     bt_cfg.mode = ESP_BT_MODE_BTDM;
     bt_cfg.bt_max_acl_conn = 3;       // Increase max connections
     ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
     ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));
     
     // Initialize Bluedroid
     ESP_ERROR_CHECK(esp_bluedroid_init());
     ESP_ERROR_CHECK(esp_bluedroid_enable());
     
     // Register SPP callback
     ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
     
     // Use enhanced SPP initialization
     esp_spp_cfg_t spp_cfg = {
         .mode = ESP_SPP_MODE_CB,
         .enable_l2cap_ertm = false,
         .tx_buffer_size = ESP_SPP_MAX_TX_BUFFER_SIZE,
     };
     ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));
     
     // Print Bluetooth device address
     const uint8_t *bd_addr = esp_bt_dev_get_address();
     if (bd_addr) {
         ESP_LOGI(TAG, "Bluetooth device address: %02x:%02x:%02x:%02x:%02x:%02x",
                bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
     }
     
     ESP_LOGI(TAG, "BT initialized, device name: ESP32_BT_TEST");
     ESP_LOGI(TAG, "Send 'P' character to get response");
     
     // Blink LED in a pattern to show initialization complete
     for (int i = 0; i < 3; i++) {
         gpio_set_level(LED_PIN, 1);
         vTaskDelay(300 / portTICK_PERIOD_MS);
         gpio_set_level(LED_PIN, 0);
         vTaskDelay(300 / portTICK_PERIOD_MS);
     }
     
     // Leave LED on to show device is ready
     gpio_set_level(LED_PIN, 1);
 }