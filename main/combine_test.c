// /*
//  * ESP32 Bluetooth Mecanum Motor Control
//  * Combined code for controlling robot via Bluetooth
//  */

// #include <stdio.h>
// #include <string.h>
// #include <inttypes.h>  // For PRIu32 format specifier
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/timers.h"
// #include "esp_system.h"
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "esp_bt.h"
// #include "esp_bt_main.h"
// #include "esp_bt_device.h"
// #include "esp_gap_bt_api.h"
// #include "esp_spp_api.h"
// #include "driver/gpio.h"
// #include "driver/ledc.h"
// #include "esp32/rom/ets_sys.h"
// #include "esp_timer.h"

// // ------ TAG for logs ------
// #define TAG "ESP32_BT_ROBOT"

// // ------ LED Configuration ------
// #define LED_PIN 13  // Built-in LED on most ESP32 boards

// // ------ Ultrasonic Sensor Configuration ------
// // Top sensor
// #define TRIG_PIN 25
// #define ECHO_PIN 26
// // Front sensor
// #define TRIG_FRONT 4
// #define ECHO_FRONT 36

// // ------ Motor Configuration ------
// // For FL Motor (Front Left)
// #define FL_IN1 12
// #define FL_IN2 21

// // For FR Motor (Front Right)
// #define FR_IN1 22
// #define FR_IN2 27

// // For RL Motor (Rear Left)
// #define RL_IN1 32
// #define RL_IN2 33

// // For RR Motor (Rear Right)
// #define RR_IN1 14
// #define RR_IN2 20

// // ------ Distance Configuration ------
// #define DEFAULT_PERIOD 1000
// #define MAX_DISTANCE 300
// #define MIN_DISTANCE 5

// // ------ Global Variables ------
// // Bluetooth related
// static uint32_t spp_handle = 0;
// static bool is_connected = false;
// static TimerHandle_t keepalive_timer = NULL;

// // LED related
// static uint8_t s_led_state = 1;

// // Distance related
// static float distance = MAX_DISTANCE;
// static float front_distance = MAX_DISTANCE;

// // ------ Function Prototypes ------
// // Bluetooth functions
// static void keepalive_callback(TimerHandle_t xTimer);
// static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
// static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

// // Sensor functions
// static float get_distance_cm(uint32_t trig, uint32_t echo);
// static void read_distance_task(void *arg);

// // Motor functions
// void init_mecanum_motors(void);
// void set_motor(gpio_num_t in1, gpio_num_t in2, bool forward, uint32_t speed, ledc_channel_t pwm_channel);
// void move_forward(uint32_t speed);
// void move_backward(uint32_t speed);
// void move_left(uint32_t speed);
// void move_right(uint32_t speed);
// void rotate_clockwise(uint32_t speed);
// void rotate_counterclockwise(uint32_t speed);
// void stop_all_motors(void);

// // LED functions
// static void blink_led(void);
// static void blink_task(void *arg);

// // ---------- LED Functions ----------
// static void blink_led(void) {
//     gpio_set_level(LED_PIN, s_led_state);
// }

// static void blink_task(void *arg) {
//     while(1) {
//         s_led_state = !s_led_state;
//         blink_led();
//         vTaskDelay(DEFAULT_PERIOD / portTICK_PERIOD_MS);
//     }
// }

// // ---------- Bluetooth Functions ----------
// static void keepalive_callback(TimerHandle_t xTimer) {
//     if (is_connected && spp_handle > 0) {
//         // Send a non-visible character as keepalive
//         uint8_t keepalive = 0xFF;
//         esp_spp_write(spp_handle, 1, &keepalive);
//         ESP_LOGD(TAG, "Sent keepalive packet");
//     }
// }

// static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
//     switch (event) {
//         case ESP_BT_GAP_AUTH_CMPL_EVT:
//             if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
//                 ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
//             } else {
//                 ESP_LOGE(TAG, "Authentication failed, status: %" PRIu8, param->auth_cmpl.stat);
//             }
//             break;
            
//         default:
//             break;
//     }
// }

// // Handle Bluetooth received data
// static void process_command(uint8_t command) {
//     uint32_t speed = 200; // Default motor speed
//     char response[64];

//     switch(command) {
//         case 'W': // Forward
//             move_forward(speed);
//             snprintf(response, sizeof(response), "Moving Forward\n");
//             break;
//         case 'S': // Backward
//             move_backward(speed);
//             snprintf(response, sizeof(response), "Moving Backward\n");
//             break;
//         case 'A': // Left
//             move_left(speed);
//             snprintf(response, sizeof(response), "Strafing Left\n");
//             break;
//         case 'D': // Right
//             move_right(speed);
//             snprintf(response, sizeof(response), "Strafing Right\n");
//             break;
//         case 'Q': // Rotate Counter-Clockwise
//             rotate_counterclockwise(speed);
//             snprintf(response, sizeof(response), "Rotating CCW\n");
//             break;
//         case 'E': // Rotate Clockwise
//             rotate_clockwise(speed);
//             snprintf(response, sizeof(response), "Rotating CW\n");
//             break;
//         case 'X': // Stop
//             stop_all_motors();
//             snprintf(response, sizeof(response), "Motors Stopped\n");
//             break;
//         case 'P': // Ping command from BT test
//             snprintf(response, sizeof(response), "Hello I am ESP32\n");
//             // Blink LED for visual feedback
//             s_led_state = !s_led_state;
//             blink_led();
//             break;
//         case 'I': // Info request
//             s_led_state = 1;
//             blink_led();
//             snprintf(response, sizeof(response), "ESP32 Robot\n");
//             break;
//         case 'O': // Top sensor reading
//             snprintf(response, sizeof(response), "Top sensor: %.2f cm\n", distance);
//             break;
//         case 'F': // Front sensor reading
//             snprintf(response, sizeof(response), "Front sensor: %.2f cm\n", front_distance);
//             break;
//         case 'V': // View all sensor data
//             snprintf(response, sizeof(response), "Top: %.2f cm, Front: %.2f cm\n", 
//                 distance, front_distance);
//             break;
//         default:
//             return; // Don't send response for unrecognized commands
//     }
    
//     // Send response via Bluetooth
//     if (is_connected && spp_handle > 0) {
//         esp_spp_write(spp_handle, strlen(response), (uint8_t *)response);
//     }
// }

// static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
//     switch (event) {
//         case ESP_SPP_INIT_EVT:
//             ESP_LOGI(TAG, "SPP initialized");
//             esp_bt_gap_set_device_name("ESP32_BT_ROBOT");
//             esp_bt_gap_register_callback(esp_bt_gap_cb);
//             esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
//             esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "ESP32_ROBOT");
//             gpio_set_level(LED_PIN, 1);  // Turn on LED when ready
//             break;
            
//         case ESP_SPP_DATA_IND_EVT:
//             if (param->data_ind.len > 0) {
//                 // Ignore keepalive packets (0xFF)
//                 if (param->data_ind.data[0] != 0xFF) {
//                     ESP_LOGI(TAG, "Data received, len=%" PRIu16 ", data[0]=0x%02x", 
//                             param->data_ind.len, param->data_ind.data[0]);
                    
//                     // Process the received command
//                     process_command(param->data_ind.data[0]);
                    
//                     // Reset keepalive timer when we receive data
//                     if (keepalive_timer != NULL) {
//                         xTimerReset(keepalive_timer, 0);
//                     }
//                 }
//             }
//             break;
            
//         case ESP_SPP_WRITE_EVT:
//             // Reset keepalive timer when we write data
//             if (keepalive_timer != NULL && is_connected) {
//                 xTimerReset(keepalive_timer, 0);
//             }
//             break;
            
//         case ESP_SPP_SRV_OPEN_EVT:
//             ESP_LOGI(TAG, "Client connected, handle=%" PRIu32, param->srv_open.handle);
//             spp_handle = param->srv_open.handle;
//             is_connected = true;
            
//             // Send an immediate welcome packet to establish the connection
//             const char *welcome = "ESP32 Robot Connected\n";
//             esp_spp_write(param->srv_open.handle, strlen(welcome), (uint8_t *)welcome);
            
//             // Create and start keepalive timer if not already created
//             if (keepalive_timer == NULL) {
//                 keepalive_timer = xTimerCreate(
//                     "keepalive",
//                     pdMS_TO_TICKS(500),  // Send keepalive every 500ms
//                     pdTRUE,              // Auto reload
//                     NULL,
//                     keepalive_callback
//                 );
//             }
            
//             if (keepalive_timer != NULL) {
//                 xTimerStart(keepalive_timer, 0);
//                 ESP_LOGI(TAG, "Started keepalive timer (500ms)");
//             }
            
//             // Blink the LED to indicate connection
//             for (int i = 0; i < 3; i++) {
//                 gpio_set_level(LED_PIN, 0);
//                 vTaskDelay(100 / portTICK_PERIOD_MS);
//                 gpio_set_level(LED_PIN, 1);
//                 vTaskDelay(100 / portTICK_PERIOD_MS);
//             }
//             break;
            
//         case ESP_SPP_CLOSE_EVT:
//             ESP_LOGI(TAG, "Client disconnected, reason=%" PRIu8, param->close.status);
//             spp_handle = 0;
//             is_connected = false;
        
//             // Stop keepalive timer
//             if (keepalive_timer != NULL) {
//                 xTimerStop(keepalive_timer, 0);
//                 xTimerDelete(keepalive_timer, 0);
//                 keepalive_timer = NULL;
//                 ESP_LOGI(TAG, "Stopped and deleted keepalive timer");
//             }
        
//             // Stop all motors when connection is lost for safety
//             stop_all_motors();
        
//             // Reset Bluetooth stack to ensure clean reconnection
//             esp_bluedroid_disable();
//             esp_bluedroid_enable();
        
//             // Blink LED to indicate disconnection
//             gpio_set_level(LED_PIN, 0);
//             vTaskDelay(300 / portTICK_PERIOD_MS);
//             gpio_set_level(LED_PIN, 1);  // Turn back on to show ready for connection
//             break;
            
//         case ESP_SPP_START_EVT:
//             ESP_LOGI(TAG, "SPP server started");
//             break;
            
//         default:
//             break;
//     }
// }

// // ---------- Sensor Functions ----------
// static float get_distance_cm(uint32_t trig, uint32_t echo) {
//     // Send trigger pulse
//     gpio_set_level(trig, 0);
//     ets_delay_us(2);
//     gpio_set_level(trig, 1);
//     ets_delay_us(10);
//     gpio_set_level(trig, 0);

//     // Wait for echo to go HIGH
//     uint32_t timeout = 30000; // 30ms timeout
//     uint32_t start = esp_timer_get_time();
//     while (gpio_get_level(echo) == 0) {
//         if ((esp_timer_get_time() - start) > timeout) return -1;
//     }

//     int64_t echo_start = esp_timer_get_time();
//     while (gpio_get_level(echo) == 1) {
//         if ((esp_timer_get_time() - echo_start) > timeout) return -1;
//     }
//     int64_t echo_end = esp_timer_get_time();

//     float distance = (echo_end - echo_start) * 0.034 / 2.0;
//     return distance;
// }

// static void read_distance_task(void *arg) {
//     while(1) {
//         // Update both sensor readings
//         distance = get_distance_cm(TRIG_PIN, ECHO_PIN);
//         front_distance = get_distance_cm(TRIG_FRONT, ECHO_FRONT);
        
//         // Send periodic sensor data if connected
//         if (is_connected && spp_handle > 0) {
//             static uint8_t counter = 0;
//             if (++counter >= 30) {  // Send every ~1.5 seconds (30 * 50ms)
//                 counter = 0;
//                 char sensor_data[64];
//                 snprintf(sensor_data, sizeof(sensor_data), "S:%.1f,%.1f\n", distance, front_distance);
//                 esp_spp_write(spp_handle, strlen(sensor_data), (uint8_t *)sensor_data);
//             }
//         }
        
//         vTaskDelay(50 / portTICK_PERIOD_MS);  // 50ms sampling rate
//     }
// }

// // ---------- Motor Functions ----------
// void init_mecanum_motors() {
//     // Set all IN2 pins as digital outputs
//     gpio_set_direction(FL_IN2, GPIO_MODE_OUTPUT);
//     gpio_set_direction(FR_IN2, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RL_IN2, GPIO_MODE_OUTPUT);
//     gpio_set_direction(RR_IN2, GPIO_MODE_OUTPUT);

//     // Configure LEDC PWM on IN1 pins
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .duty_resolution = LEDC_TIMER_8_BIT,
//         .timer_num = LEDC_TIMER_1,
//         .freq_hz = 20000,
//         .clk_cfg = LEDC_AUTO_CLK
//     };
//     ledc_timer_config(&ledc_timer);

//     ledc_channel_config_t motor_channels[] = {
//         {.channel = LEDC_CHANNEL_0, .gpio_num = FL_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_1},
//         {.channel = LEDC_CHANNEL_1, .gpio_num = FR_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_1},
//         {.channel = LEDC_CHANNEL_2, .gpio_num = RL_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_1},
//         {.channel = LEDC_CHANNEL_3, .gpio_num = RR_IN1, .duty = 0, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_1},
//     };

//     for (int i = 0; i < 4; ++i) {
//         ledc_channel_config(&motor_channels[i]);
//     }
// }

// void set_motor(gpio_num_t in1, gpio_num_t in2, bool forward, uint32_t speed, ledc_channel_t pwm_channel) {
//     if (forward) {
//         gpio_set_level(in1, 1);  // HIGH
//         gpio_set_level(in2, 0);  // LOW
//     } else {
//         gpio_set_level(in1, 0);
//         gpio_set_level(in2, 1);
//     }

//     if(!forward){
//         speed = 255 - speed; // Invert speed for backward movement
//     }
//     // Apply PWM to IN1
//     ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, speed);
//     ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel);
// }

// void move_forward(uint32_t speed) {
//     set_motor(FL_IN1, FL_IN2, false, speed, LEDC_CHANNEL_0);
//     set_motor(FR_IN1, FR_IN2, false, speed, LEDC_CHANNEL_1);
//     set_motor(RL_IN1, RL_IN2, false, speed, LEDC_CHANNEL_2);
//     set_motor(RR_IN1, RR_IN2, false, speed, LEDC_CHANNEL_3);
// }

// void move_backward(uint32_t speed) {
//     set_motor(FL_IN1, FL_IN2, true, speed, LEDC_CHANNEL_0);
//     set_motor(FR_IN1, FR_IN2, true, speed, LEDC_CHANNEL_1);
//     set_motor(RL_IN1, RL_IN2, true, speed, LEDC_CHANNEL_2);
//     set_motor(RR_IN1, RR_IN2, true, speed, LEDC_CHANNEL_3);
// }

// void move_right(uint32_t speed) {
//     set_motor(FL_IN1, FL_IN2, false, speed, LEDC_CHANNEL_0);
//     set_motor(FR_IN1, FR_IN2, false, speed, LEDC_CHANNEL_1);
//     set_motor(RL_IN1, RL_IN2, true, speed, LEDC_CHANNEL_2);
//     set_motor(RR_IN1, RR_IN2, true, speed, LEDC_CHANNEL_3);
// }

// void move_left(uint32_t speed) {
//     set_motor(FL_IN1, FL_IN2, true, speed, LEDC_CHANNEL_0);
//     set_motor(FR_IN1, FR_IN2, true, speed, LEDC_CHANNEL_1);
//     set_motor(RL_IN1, RL_IN2, false, speed, LEDC_CHANNEL_2);
//     set_motor(RR_IN1, RR_IN2, false, speed, LEDC_CHANNEL_3);
// }

// void rotate_counterclockwise(uint32_t speed) {
//     set_motor(FL_IN1, FL_IN2, true, speed, LEDC_CHANNEL_0);
//     set_motor(FR_IN1, FR_IN2, false, speed, LEDC_CHANNEL_1);
//     set_motor(RL_IN1, RL_IN2, true, speed, LEDC_CHANNEL_2);
//     set_motor(RR_IN1, RR_IN2, false, speed, LEDC_CHANNEL_3);
// }

// void rotate_clockwise(uint32_t speed) {
//     set_motor(FL_IN1, FL_IN2, false, speed, LEDC_CHANNEL_0);
//     set_motor(FR_IN1, FR_IN2, true, speed, LEDC_CHANNEL_1);
//     set_motor(RL_IN1, RL_IN2, false, speed, LEDC_CHANNEL_2);
//     set_motor(RR_IN1, RR_IN2, true, speed, LEDC_CHANNEL_3);
// }

// void stop_all_motors() {
//     for (int ch = LEDC_CHANNEL_0; ch <= LEDC_CHANNEL_3; ch++) {
//         ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
//         ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
//     }
//     gpio_set_level(FL_IN1, 0);
//     gpio_set_level(FL_IN2, 0);
//     gpio_set_level(FR_IN1, 0);
//     gpio_set_level(FR_IN2, 0);
//     gpio_set_level(RL_IN1, 0);
//     gpio_set_level(RL_IN2, 0);
//     gpio_set_level(RR_IN1, 0);
//     gpio_set_level(RR_IN2, 0);
// }

// // ---------- Main Function ----------
// void app_main(void) {
//     // Initialize LED
//     gpio_reset_pin(LED_PIN);
//     gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_level(LED_PIN, 0);  // Start with LED off
    
//     ESP_LOGI(TAG, "Starting ESP32 Bluetooth Robot Control");
    
//     // Initialize NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);
    
//     // Initialize Bluetooth controller
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     bt_cfg.mode = ESP_BT_MODE_BTDM;
//     bt_cfg.bt_max_acl_conn = 3;  // Increase max connections
//     ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
//     ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));
    
//     // Initialize Bluedroid
//     ESP_ERROR_CHECK(esp_bluedroid_init());
//     ESP_ERROR_CHECK(esp_bluedroid_enable());
    
//     // Register SPP callback
//     ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
    
//     // Enhanced SPP initialization
//     esp_spp_cfg_t spp_cfg = {
//         .mode = ESP_SPP_MODE_CB,
//         .enable_l2cap_ertm = false,
//         .tx_buffer_size = ESP_SPP_MAX_TX_BUFFER_SIZE,
//     };
//     ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));
    
//     // Print Bluetooth device address
//     const uint8_t *bd_addr = esp_bt_dev_get_address();
//     if (bd_addr) {
//         ESP_LOGI(TAG, "Bluetooth device address: %02x:%02x:%02x:%02x:%02x:%02x",
//                bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
//     }
    
//     // Initialize sensors
//     gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
//     gpio_set_direction(TRIG_FRONT, GPIO_MODE_OUTPUT);
//     gpio_set_direction(ECHO_FRONT, GPIO_MODE_INPUT);
    
//     // Initialize motors
//     init_mecanum_motors();
    
//     // Create tasks
//     xTaskCreate(blink_task, "blink_LED", 1024, NULL, 5, NULL);
//     xTaskCreate(read_distance_task, "read_sensors", 2048, NULL, 5, NULL);
    
//     // Signal initialization complete with LED pattern
//     for (int i = 0; i < 3; i++) {
//         gpio_set_level(LED_PIN, 1);
//         vTaskDelay(300 / portTICK_PERIOD_MS);
//         gpio_set_level(LED_PIN, 0);
//         vTaskDelay(300 / portTICK_PERIOD_MS);
//     }
    
//     // Leave LED on to show device is ready
//     gpio_set_level(LED_PIN, 1);
    
//     ESP_LOGI(TAG, "ESP32 Bluetooth Robot Control initialized");
//     ESP_LOGI(TAG, "Ready to accept Bluetooth connections");
// }