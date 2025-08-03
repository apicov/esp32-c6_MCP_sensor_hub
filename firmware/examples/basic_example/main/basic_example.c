/**
 * @file basic_example.c
 * @brief Enhanced example application demonstrating MCP Bridge library usage
 * 
 * This example shows how to:
 * - Initialize the MCP Bridge with enhanced configuration
 * - Register sensors and actuators with metadata
 * - Handle enhanced events and errors
 * - Use metrics and batch operations
 * - Configure TLS and QoS settings
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_mcp_bridge.h"

static const char *TAG = "MCP_EXAMPLE";

// Hardware configuration
#define LED_GPIO    GPIO_NUM_2
#define BUTTON_GPIO GPIO_NUM_0
#define TEMP_ADC_CHANNEL ADC1_CHANNEL_0
#define HUMIDITY_ADC_CHANNEL ADC1_CHANNEL_1

// Global state
static bool led_state = false;
static float last_temperature = 25.0;
static float last_humidity = 50.0;
static uint32_t motion_events = 0;
static uint32_t counter_value = 0;

/**
 * @brief Temperature sensor read callback
 */
static esp_err_t temperature_read_cb(const char *sensor_id, float *value, void *user_data) {
    // Read ADC
    int adc_reading = adc1_get_raw(TEMP_ADC_CHANNEL);
    if (adc_reading < 0) {
        ESP_LOGE(TAG, "Failed to read temperature ADC");
        return MCP_BRIDGE_ERR_SENSOR_FAILED;
    }
    
    // Convert to temperature (simplified - replace with actual sensor logic)
    // For TMP36: T = (Vout - 500mV) / 10mV/°C
    float voltage = adc_reading * (3300.0 / 4095.0); // Convert to mV
    *value = (voltage - 500.0) / 10.0;
    
    // Add some noise simulation
    *value += ((float)(esp_random() % 100) - 50) / 100.0;
    
    last_temperature = *value;
    ESP_LOGD(TAG, "Temperature read: %.2f°C", *value);
    
    return ESP_OK;
}

/**
 * @brief Humidity sensor read callback
 */
static esp_err_t humidity_read_cb(const char *sensor_id, float *value, void *user_data) {
    // Read ADC for humidity sensor
    int adc_reading = adc1_get_raw(HUMIDITY_ADC_CHANNEL);
    if (adc_reading < 0) {
        ESP_LOGE(TAG, "Failed to read humidity ADC");
        return MCP_BRIDGE_ERR_SENSOR_FAILED;
    }
    
    // Convert to humidity percentage (0-100%)
    *value = (adc_reading / 4095.0) * 100.0;
    
    // Add some variation
    *value += ((float)(esp_random() % 50) - 25) / 10.0;
    if (*value < 0) *value = 0;
    if (*value > 100) *value = 100;
    
    last_humidity = *value;
    ESP_LOGD(TAG, "Humidity read: %.2f%%", *value);
    
    return ESP_OK;
}

/**
 * @brief Button sensor read callback
 */
static esp_err_t button_read_cb(const char *sensor_id, float *value, void *user_data) {
    // Read button state (inverted due to pull-up)
    *value = gpio_get_level(BUTTON_GPIO) ? 0.0 : 1.0;
    return ESP_OK;
}

/**
 * @brief Motion sensor read callback (simulated)
 */
static esp_err_t motion_read_cb(const char *sensor_id, float *value, void *user_data) {
    // Simulate motion detection
    static uint32_t last_motion_time = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if (current_time - last_motion_time > 30000) { // Every 30 seconds
        if ((esp_random() % 100) < 15) { // 15% chance of motion
            *value = 1.0;
            last_motion_time = current_time;
            motion_events++;
        } else {
            *value = 0.0;
        }
    } else {
        *value = 0.0;
    }
    
    return ESP_OK;
}

/**
 * @brief Counter sensor read callback (test sensor)
 */
static esp_err_t counter_read_cb(const char *sensor_id, float *value, void *user_data) {
    // Increment counter each time it's read
    counter_value++;
    *value = (float)counter_value;
    
    ESP_LOGD(TAG, "Counter read: %lu", counter_value);
    
    return ESP_OK;
}

/**
 * @brief LED actuator control callback
 */
static esp_err_t led_control_cb(const char *actuator_id, const char *action, 
                               const void *value, void *user_data) {
    ESP_LOGI(TAG, "LED control: action=%s", action);
    
    if (strcmp(action, "toggle") == 0) {
        led_state = !led_state;
    } else if (strcmp(action, "write") == 0 && value != NULL) {
        // Value could be boolean, string, or number
        const char *str_value = (const char *)value;
        if (strcmp(str_value, "on") == 0 || strcmp(str_value, "true") == 0 || strcmp(str_value, "1") == 0) {
            led_state = true;
        } else if (strcmp(str_value, "off") == 0 || strcmp(str_value, "false") == 0 || strcmp(str_value, "0") == 0) {
            led_state = false;
        } else {
            ESP_LOGE(TAG, "Invalid LED value: %s", str_value);
            return MCP_BRIDGE_ERR_ACTUATOR_FAILED;
        }
    } else if (strcmp(action, "read") == 0) {
        // Just report current state
    } else {
        ESP_LOGE(TAG, "Unknown LED action: %s", action);
        return MCP_BRIDGE_ERR_ACTUATOR_FAILED;
    }
    
    // Apply state to hardware
    gpio_set_level(LED_GPIO, led_state);
    
    // Publish new status
    mcp_bridge_publish_actuator_status("led", led_state ? "on" : "off");
    
    ESP_LOGI(TAG, "LED is now %s", led_state ? "ON" : "OFF");
    return ESP_OK;
}

/**
 * @brief Enhanced event handler for MCP Bridge events
 */
static void event_handler(const mcp_event_t *event, void *user_data) {
    switch (event->type) {
        case MCP_EVENT_WIFI_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected");
            break;
            
        case MCP_EVENT_WIFI_DISCONNECTED:
            ESP_LOGW(TAG, "WiFi disconnected");
            break;
            
        case MCP_EVENT_MQTT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected - device online");
            break;
            
        case MCP_EVENT_MQTT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
            
        case MCP_EVENT_COMMAND_RECEIVED:
            ESP_LOGI(TAG, "Command received for %s: %s at %lu", 
                    event->data.command.actuator_id,
                    event->data.command.action,
                    event->data.command.timestamp);
            break;
            
        case MCP_EVENT_SENSOR_READ_ERROR:
            ESP_LOGE(TAG, "Sensor read error for %s: %s (%d)", 
                    event->data.sensor_error.sensor_id,
                    event->data.sensor_error.error_message,
                    event->data.sensor_error.error_code);
            break;
            
        case MCP_EVENT_ACTUATOR_ERROR:
            ESP_LOGE(TAG, "Actuator error for %s: %s (%d)", 
                    event->data.actuator_error.actuator_id,
                    event->data.actuator_error.error_message,
                    event->data.actuator_error.error_code);
            break;
            
        case MCP_EVENT_LOW_MEMORY:
            ESP_LOGW(TAG, "Low memory warning: %lu bytes free (threshold: %lu)", 
                    event->data.low_memory.free_heap,
                    event->data.low_memory.threshold);
            break;
            
        case MCP_EVENT_TLS_ERROR:
            ESP_LOGE(TAG, "TLS connection error");
            break;
            
        case MCP_EVENT_AUTH_ERROR:
            ESP_LOGE(TAG, "Authentication error");
            break;
            
        case MCP_EVENT_ERROR:
            ESP_LOGE(TAG, "General error: %s - %s (severity: %d)", 
                    event->data.error.error_type,
                    event->data.error.message,
                    event->data.error.severity);
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown event type: %d", event->type);
            break;
    }
}

/**
 * @brief Batch sensor reading task
 */
static void batch_sensor_task(void *pvParameters) {
    mcp_sensor_reading_t readings[4];  // Increased from 3 to 4
    uint32_t reading_time;
    
    ESP_LOGI(TAG, "Batch sensor task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000)); // Every minute
        
        reading_time = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
        
        // Prepare batch readings
        readings[0] = (mcp_sensor_reading_t){
            .sensor_id = "temperature",
            .sensor_type = "temperature", 
            .value = last_temperature,
            .unit = "°C",
            .timestamp = reading_time,
            .quality = 95.0
        };
        
        readings[1] = (mcp_sensor_reading_t){
            .sensor_id = "humidity",
            .sensor_type = "humidity",
            .value = last_humidity,
            .unit = "%",
            .timestamp = reading_time,
            .quality = 90.0
        };
        
        readings[2] = (mcp_sensor_reading_t){
            .sensor_id = "motion_events",
            .sensor_type = "counter",
            .value = (float)motion_events,
            .unit = "count",
            .timestamp = reading_time,
            .quality = 100.0
        };
        
        readings[3] = (mcp_sensor_reading_t){
            .sensor_id = "counter",
            .sensor_type = "counter",
            .value = (float)counter_value,
            .unit = "count",
            .timestamp = reading_time,
            .quality = 100.0
        };
        
        // Publish batch
        esp_err_t ret = mcp_bridge_publish_sensor_batch(readings, 4);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Batch sensor data published successfully");
        } else {
            ESP_LOGE(TAG, "Failed to publish batch sensor data: %s", esp_err_to_name(ret));
        }
    }
}

/**
 * @brief Metrics monitoring task
 */
static void metrics_task(void *pvParameters) {
    mcp_bridge_metrics_t metrics;
    
    ESP_LOGI(TAG, "Metrics monitoring task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(120000)); // Every 2 minutes
        
        esp_err_t ret = mcp_bridge_get_metrics(&metrics);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "=== Bridge Metrics ===");
            ESP_LOGI(TAG, "Messages sent: %lu", metrics.messages_sent);
            ESP_LOGI(TAG, "Messages received: %lu", metrics.messages_received);
            ESP_LOGI(TAG, "Connection failures: %lu", metrics.connection_failures);
            ESP_LOGI(TAG, "Sensor errors: %lu", metrics.sensor_read_errors);
            ESP_LOGI(TAG, "Actuator errors: %lu", metrics.actuator_errors);
            ESP_LOGI(TAG, "Uptime: %lu seconds", metrics.uptime_seconds);
            ESP_LOGI(TAG, "Free heap: %lu bytes (min: %lu)", 
                    metrics.free_heap_size, metrics.min_free_heap_size);
            ESP_LOGI(TAG, "WiFi reconnections: %lu", metrics.wifi_reconnections);
            ESP_LOGI(TAG, "MQTT reconnections: %lu", metrics.mqtt_reconnections);
            ESP_LOGI(TAG, "===================");
        }
    }
}

/**
 * @brief Initialize hardware
 */
static void init_hardware(void) {
    // Configure LED GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    // Configure button GPIO with pull-up
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TEMP_ADC_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(HUMIDITY_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

void app_main(void) {
    ESP_LOGI(TAG, "Enhanced MCP Bridge Example Starting...");
    
    // Initialize hardware
    init_hardware();
    
    // Configure enhanced MCP Bridge
    mcp_bridge_config_t config = {
        .wifi_ssid = "YourWiFiSSID",
        .wifi_password = "YourWiFiPassword",
        .mqtt_broker_uri = "mqtts://your-secure-broker.com:8883", // Using secure MQTT
        .mqtt_username = "your_device_username",
        .mqtt_password = "your_device_password",
        .device_id = NULL,  // Auto-generate
        .sensor_publish_interval_ms = 10000,  // 10 seconds
        .command_timeout_ms = 5000,           // 5 second timeout
        .enable_watchdog = true,
        .enable_device_auth = true,
        .log_level = 3,  // INFO level
        
        // Configure QoS levels
        .qos_config = {
            .sensor_qos = 0,     // Best effort for sensor data
            .actuator_qos = 1,   // At least once for commands
            .status_qos = 1,     // At least once for status
            .error_qos = 2       // Exactly once for errors
        },
        
        // Configure TLS (uncomment and configure for production)
        .tls_config = {
            .enable_tls = true,
            .ca_cert_pem = NULL,    // Load your CA certificate here
            .client_cert_pem = NULL, // Load your client certificate here
            .client_key_pem = NULL,  // Load your client key here
            .skip_cert_verification = true, // Set to false in production!
            .alpn_protocols = {"mqtt", NULL}
        }
    };
    
    ESP_ERROR_CHECK(mcp_bridge_init(&config));
    
    // Register enhanced event handler
    ESP_ERROR_CHECK(mcp_bridge_register_event_handler(event_handler, NULL));
    
    // Register temperature sensor with detailed metadata
    mcp_sensor_metadata_t temp_metadata = {
        .min_range = -40.0,
        .max_range = 85.0,
        .accuracy = 0.5,
        .update_interval_ms = 10000,
        .description = "TMP36 analog temperature sensor",
        .calibration_required = true,
        .calibration_interval_s = 86400 // Daily calibration
    };
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("temperature", "temperature", "°C", 
                                              &temp_metadata, temperature_read_cb, NULL));
    
    // Register humidity sensor
    mcp_sensor_metadata_t humidity_metadata = {
        .min_range = 0.0,
        .max_range = 100.0,
        .accuracy = 2.0,
        .update_interval_ms = 10000,
        .description = "Analog humidity sensor",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("humidity", "humidity", "%",
                                              &humidity_metadata, humidity_read_cb, NULL));
    
    // Register button sensor
    mcp_sensor_metadata_t button_metadata = {
        .min_range = 0.0,
        .max_range = 1.0,
        .accuracy = 1.0,
        .update_interval_ms = 0,  // Event-driven
        .description = "Built-in button (GPIO0)",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("button", "button", NULL,
                                              &button_metadata, button_read_cb, NULL));
    
    // Register motion sensor
    mcp_sensor_metadata_t motion_metadata = {
        .min_range = 0.0,
        .max_range = 1.0,
        .accuracy = 1.0,
        .update_interval_ms = 5000,
        .description = "Simulated motion sensor",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("motion", "motion", NULL,
                                              &motion_metadata, motion_read_cb, NULL));
    
    // Register counter sensor (test sensor)
    mcp_sensor_metadata_t counter_metadata = {
        .min_range = 0.0,
        .max_range = 4294967295.0,  // Max uint32_t value
        .accuracy = 1.0,
        .update_interval_ms = 2000,  // Read every 2 seconds
        .description = "Test counter sensor that increments on each read",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("counter", "counter", "count",
                                              &counter_metadata, counter_read_cb, NULL));
    
    // Register LED actuator with enhanced metadata
    const char *led_actions[] = {"read", "write", "toggle", NULL};
    mcp_actuator_metadata_t led_metadata = {
        .value_type = "boolean",
        .description = "Built-in LED (GPIO2) with on/off control",
        .supported_actions = led_actions,
        .min_value = NULL,
        .max_value = NULL,
        .response_time_ms = 100,
        .requires_confirmation = false
    };
    ESP_ERROR_CHECK(mcp_bridge_register_actuator("led", "led",
                                                &led_metadata, led_control_cb, NULL));
    
    // Start the bridge
    ESP_ERROR_CHECK(mcp_bridge_start());
    
    // Create enhanced tasks
    xTaskCreate(batch_sensor_task, "batch_sensor", 3072, NULL, 5, NULL);
    xTaskCreate(metrics_task, "metrics", 2048, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "All tasks created, entering main loop...");
    
    // Enhanced main loop with automatic error recovery
    uint32_t error_count = 0;
    uint32_t last_reconnect = 0;
    
    while (1) {
        bool wifi_connected, mqtt_connected;
        mcp_bridge_get_status(&wifi_connected, &mqtt_connected);
        
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        ESP_LOGI(TAG, "System Status - WiFi: %s, MQTT: %s, Device: %s", 
                wifi_connected ? "Connected" : "Disconnected",
                mqtt_connected ? "Connected" : "Disconnected",
                mcp_bridge_get_device_id());
        
        // Check for high temperature alert
        if (mqtt_connected && last_temperature > 30.0) {
            mcp_bridge_publish_error("high_temp", "Temperature exceeds 30°C threshold", 1);
        }
        
        // Check for excessive humidity
        if (mqtt_connected && last_humidity > 80.0) {
            mcp_bridge_publish_error("high_humidity", "Humidity exceeds 80% threshold", 1);
        }
        
        // Auto-recovery mechanism
        if (!wifi_connected || !mqtt_connected) {
            error_count++;
            if (error_count > 5 && (current_time - last_reconnect) > 60000) {
                ESP_LOGW(TAG, "Connection issues detected, forcing reconnection...");
                mcp_bridge_reconnect();
                last_reconnect = current_time;
                error_count = 0;
            }
        } else {
            error_count = 0;
        }
        
        // Enable high-frequency streaming for temperature if it's changing rapidly
        static float prev_temp = 0;
        if (abs(last_temperature - prev_temp) > 2.0) {
            ESP_LOGI(TAG, "Rapid temperature change detected, enabling streaming");
            mcp_bridge_set_sensor_streaming("temperature", true, 2000); // 2 second interval
        } else {
            mcp_bridge_set_sensor_streaming("temperature", false, 0);
        }
        prev_temp = last_temperature;
        
        vTaskDelay(pdMS_TO_TICKS(30000));  // Check every 30 seconds
    }
}
