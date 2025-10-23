/**
 * @file esp_mcp_bridge.c
 * @brief ESP32 MQTT-MCP Bridge Library Implementation
 * 
 * This library provides a bridge between ESP32 IoT devices and MCP (Model Context Protocol)
 * servers using MQTT as the transport layer.
 */

#include "esp_mcp_bridge.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "cJSON.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "MCP_BRIDGE";

/* ==================== CONSTANTS ==================== */

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MQTT_CONNECTED_BIT BIT0
#define MQTT_FAIL_BIT      BIT1

#define MCP_BRIDGE_DEVICE_ID_LEN 32
#define MCP_BRIDGE_MAX_TOPIC_LEN 128
#define MCP_BRIDGE_MAX_MESSAGE_LEN 1024
#define MCP_BRIDGE_MAX_SENSORS 16
#define MCP_BRIDGE_MAX_ACTUATORS 16
#define MCP_BRIDGE_COMMAND_QUEUE_SIZE 10
#define MCP_BRIDGE_RECONNECT_DELAY_MS 5000
#define MCP_BRIDGE_WATCHDOG_TIMEOUT_S 300

/* ==================== INTERNAL STRUCTURES ==================== */

/**
 * @brief Registered sensor structure
 */
typedef struct sensor_node {
    char *sensor_id;
    char *type;
    char *unit;
    mcp_sensor_metadata_t metadata;
    mcp_sensor_read_cb_t read_cb;
    mcp_sensor_read_multi_cb_t read_multi_cb;  // For multi-value sensors
    bool is_multi_value;                        // Flag to indicate multi-value sensor
    mcp_sensor_field_metadata_t *field_metadata; // Field metadata for multi-value sensors
    size_t field_count;                         // Number of fields
    void *user_data;
    uint32_t last_read_time;
    float last_value;
    struct sensor_node *next;
} sensor_node_t;

/**
 * @brief Registered actuator structure
 */
typedef struct actuator_node {
    char *actuator_id;
    char *type;
    mcp_actuator_metadata_t metadata;
    mcp_actuator_control_cb_t control_cb;
    void *user_data;
    char *last_status;
    struct actuator_node *next;
} actuator_node_t;

/**
 * @brief MQTT command message
 */
typedef struct {
    char actuator_id[32];
    char action[16];
    char value[64];
    uint32_t timestamp;
} mcp_command_t;

/**
 * @brief Bridge context structure
 */
typedef struct {
    // Configuration
    mcp_bridge_config_t config;
    char device_id[MCP_BRIDGE_DEVICE_ID_LEN];
    const char *device_name;
    const char *device_location;

    // State
    bool initialized;
    bool running;
    bool wifi_connected;
    bool mqtt_connected;
    uint32_t wifi_retry_count;
    uint32_t mqtt_retry_count;
    
    // Component lists
    sensor_node_t *sensors;
    actuator_node_t *actuators;
    uint8_t sensor_count;
    uint8_t actuator_count;
    
    // Event handling
    mcp_event_handler_t event_handler;
    void *event_handler_user_data;
    
    // FreeRTOS objects
    TaskHandle_t sensor_task_handle;
    TaskHandle_t actuator_task_handle;
    TaskHandle_t watchdog_task_handle;
    SemaphoreHandle_t mutex;
    QueueHandle_t command_queue;
    EventGroupHandle_t wifi_event_group;
    EventGroupHandle_t mqtt_event_group;
    
    // MQTT client
    esp_mqtt_client_handle_t mqtt_client;
    
    // Statistics
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t connection_failures;
    uint32_t sensor_read_errors;
    uint32_t boot_time;
    uint32_t wifi_reconnections;
    uint32_t mqtt_reconnections;

} mcp_bridge_context_t;

static mcp_bridge_context_t *g_bridge_ctx = NULL;

/* ==================== UTILITY FUNCTIONS ==================== */

/**
 * @brief Generate device ID based on MAC address
 */
static void generate_device_id(char *device_id, size_t max_len) {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(device_id, max_len, "esp32_%02x%02x%02x", mac[3], mac[4], mac[5]);
}

/**
 * @brief Get current timestamp (Unix timestamp in seconds)
 */
static uint32_t get_timestamp(void) {
    time_t now;
    time(&now);
    return (uint32_t)now;
}

/**
 * @brief Send event to application
 */
static void send_event(mcp_event_type_t type, void *data) {
    if (!g_bridge_ctx || !g_bridge_ctx->event_handler) {
        return;
    }
    
    mcp_event_t event = {
        .type = type
    };
    
    // Copy event-specific data
    switch (type) {
        case MCP_EVENT_COMMAND_RECEIVED:
            if (data) {
                mcp_command_t *cmd = (mcp_command_t *)data;
                event.data.command.actuator_id = cmd->actuator_id;
                event.data.command.action = cmd->action;
                event.data.command.value = cmd->value;
            }
            break;
        case MCP_EVENT_ERROR:
            // Error data would be passed in specific format
            break;
        default:
            break;
    }
    
    g_bridge_ctx->event_handler(&event, g_bridge_ctx->event_handler_user_data);
}

/**
 * @brief Find sensor by ID
 */
static sensor_node_t* find_sensor(const char *sensor_id) {
    if (!g_bridge_ctx || !sensor_id) return NULL;
    
    sensor_node_t *node = g_bridge_ctx->sensors;
    while (node) {
        if (strcmp(node->sensor_id, sensor_id) == 0) {
            return node;
        }
        node = node->next;
    }
    return NULL;
}

/**
 * @brief Find actuator by ID
 */
static actuator_node_t* find_actuator(const char *actuator_id) {
    if (!g_bridge_ctx || !actuator_id) return NULL;
    
    actuator_node_t *node = g_bridge_ctx->actuators;
    while (node) {
        if (strcmp(node->actuator_id, actuator_id) == 0) {
            return node;
        }
        node = node->next;
    }
    return NULL;
}

/* ==================== JSON MESSAGE FORMATTING ==================== */

/**
 * @brief Create sensor data JSON message
 */
static char* create_sensor_message(const char *sensor_id, const char *sensor_type,
                                  float value, const char *unit) {
    cJSON *json = cJSON_CreateObject();
    cJSON *value_obj = cJSON_CreateObject();

    cJSON_AddStringToObject(json, "device_id", g_bridge_ctx->device_id);
    if (g_bridge_ctx->device_name) {
        cJSON_AddStringToObject(json, "device_name", g_bridge_ctx->device_name);
    }
    cJSON_AddStringToObject(json, "sensor_id", sensor_id);
    cJSON_AddNumberToObject(json, "timestamp", get_timestamp());
    cJSON_AddStringToObject(json, "type", "sensor");
    cJSON_AddStringToObject(json, "component", sensor_type);
    cJSON_AddStringToObject(json, "action", "read");

    cJSON_AddNumberToObject(value_obj, "reading", value);
    if (unit) {
        cJSON_AddStringToObject(value_obj, "unit", unit);
    }
    cJSON_AddNumberToObject(value_obj, "quality", 100); // Default quality

    cJSON_AddItemToObject(json, "value", value_obj);

    // Add system metrics
    cJSON *metrics = cJSON_CreateObject();
    cJSON_AddNumberToObject(metrics, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(metrics, "uptime", get_timestamp() - g_bridge_ctx->boot_time);
    cJSON_AddItemToObject(json, "metrics", metrics);

    char *json_string = cJSON_Print(json);
    cJSON_Delete(json);

    return json_string;
}

/**
 * @brief Create multi-value sensor data JSON message
 */
static char* create_multi_sensor_message(const char *sensor_id, const mcp_sensor_multi_value_t *values) {
    cJSON *json = cJSON_CreateObject();
    cJSON *value_obj = cJSON_CreateObject();

    cJSON_AddStringToObject(json, "device_id", g_bridge_ctx->device_id);
    if (g_bridge_ctx->device_name) {
        cJSON_AddStringToObject(json, "device_name", g_bridge_ctx->device_name);
    }
    cJSON_AddStringToObject(json, "sensor_id", sensor_id);
    cJSON_AddNumberToObject(json, "timestamp", get_timestamp());
    cJSON_AddStringToObject(json, "type", "sensor");
    cJSON_AddStringToObject(json, "action", "read");

    // Add each field as a nested object in value
    for (size_t i = 0; i < values->field_count; i++) {
        const mcp_sensor_field_t *field = &values->fields[i];
        cJSON *field_obj = cJSON_CreateObject();

        cJSON_AddNumberToObject(field_obj, "reading", field->value);
        if (field->unit) {
            cJSON_AddStringToObject(field_obj, "unit", field->unit);
        }
        cJSON_AddNumberToObject(field_obj, "quality", field->quality > 0 ? field->quality : 100);

        cJSON_AddItemToObject(value_obj, field->name, field_obj);
    }

    cJSON_AddItemToObject(json, "value", value_obj);

    // Add system metrics
    cJSON *metrics = cJSON_CreateObject();
    cJSON_AddNumberToObject(metrics, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(metrics, "uptime", get_timestamp() - g_bridge_ctx->boot_time);
    cJSON_AddItemToObject(json, "metrics", metrics);

    char *json_string = cJSON_Print(json);
    cJSON_Delete(json);

    return json_string;
}

/**
 * @brief Create capabilities JSON message
 */
static char* create_capabilities_message(void) {
    cJSON *json = cJSON_CreateObject();
    cJSON *sensors_array = cJSON_CreateArray();
    cJSON *actuators_array = cJSON_CreateArray();
    cJSON *metadata_obj = cJSON_CreateObject();

    cJSON_AddStringToObject(json, "device_id", g_bridge_ctx->device_id);
    cJSON_AddStringToObject(json, "firmware_version", "1.0.0"); // Could be made configurable

    // Add device name if configured
    if (g_bridge_ctx->device_name) {
        cJSON_AddStringToObject(json, "device_name", g_bridge_ctx->device_name);
    }

    // Add device location if configured
    if (g_bridge_ctx->device_location) {
        cJSON_AddStringToObject(json, "device_location", g_bridge_ctx->device_location);
    }

    // Add sensors
    sensor_node_t *sensor = g_bridge_ctx->sensors;
    while (sensor) {
        // Add sensor ID to array
        cJSON_AddItemToArray(sensors_array, cJSON_CreateString(sensor->sensor_id));

        // Add sensor metadata
        cJSON *sensor_meta = cJSON_CreateObject();

        if (sensor->is_multi_value) {
            cJSON_AddStringToObject(sensor_meta, "type", "multi_value");
            if (sensor->metadata.description) {
                cJSON_AddStringToObject(sensor_meta, "description", sensor->metadata.description);
            }
            if (sensor->metadata.location) {
                cJSON_AddStringToObject(sensor_meta, "location", sensor->metadata.location);
            }

            // Add field metadata
            if (sensor->field_metadata && sensor->field_count > 0) {
                cJSON *fields_obj = cJSON_CreateObject();
                for (size_t i = 0; i < sensor->field_count; i++) {
                    cJSON *field_meta = cJSON_CreateObject();
                    const mcp_sensor_field_metadata_t *field = &sensor->field_metadata[i];

                    if (field->unit) {
                        cJSON_AddStringToObject(field_meta, "unit", field->unit);
                    }
                    if (field->metric_type) {
                        cJSON_AddStringToObject(field_meta, "metric_type", field->metric_type);
                    }
                    cJSON_AddNumberToObject(field_meta, "min_range", field->min_range);
                    cJSON_AddNumberToObject(field_meta, "max_range", field->max_range);
                    cJSON_AddNumberToObject(field_meta, "accuracy", field->accuracy);

                    cJSON_AddItemToObject(fields_obj, field->name, field_meta);
                }
                cJSON_AddItemToObject(sensor_meta, "fields", fields_obj);
            }
        } else {
            cJSON_AddStringToObject(sensor_meta, "type", "single_value");
            cJSON_AddStringToObject(sensor_meta, "component", sensor->type);
            if (sensor->unit) {
                cJSON_AddStringToObject(sensor_meta, "unit", sensor->unit);
            }
            cJSON_AddNumberToObject(sensor_meta, "min_range", sensor->metadata.min_range);
            cJSON_AddNumberToObject(sensor_meta, "max_range", sensor->metadata.max_range);
            cJSON_AddNumberToObject(sensor_meta, "accuracy", sensor->metadata.accuracy);
            if (sensor->metadata.description) {
                cJSON_AddStringToObject(sensor_meta, "description", sensor->metadata.description);
            }
            if (sensor->metadata.location) {
                cJSON_AddStringToObject(sensor_meta, "location", sensor->metadata.location);
            }
        }

        cJSON_AddItemToObject(metadata_obj, sensor->sensor_id, sensor_meta);
        sensor = sensor->next;
    }

    // Add actuators
    actuator_node_t *actuator = g_bridge_ctx->actuators;
    while (actuator) {
        cJSON_AddItemToArray(actuators_array, cJSON_CreateString(actuator->type));

        // Add actuator metadata
        cJSON *actuator_meta = cJSON_CreateObject();
        if (actuator->metadata.value_type) {
            cJSON_AddStringToObject(actuator_meta, "value_type", actuator->metadata.value_type);
        }
        if (actuator->metadata.description) {
            cJSON_AddStringToObject(actuator_meta, "description", actuator->metadata.description);
        }

        // Add supported actions
        if (actuator->metadata.supported_actions) {
            cJSON *actions_array = cJSON_CreateArray();
            for (int i = 0; actuator->metadata.supported_actions[i] != NULL; i++) {
                cJSON_AddItemToArray(actions_array, cJSON_CreateString(actuator->metadata.supported_actions[i]));
            }
            cJSON_AddItemToObject(actuator_meta, "supported_actions", actions_array);
        }

        cJSON_AddItemToObject(metadata_obj, actuator->type, actuator_meta);
        actuator = actuator->next;
    }

    cJSON_AddItemToObject(json, "sensors", sensors_array);
    cJSON_AddItemToObject(json, "actuators", actuators_array);
    cJSON_AddItemToObject(json, "metadata", metadata_obj);

    char *json_string = cJSON_Print(json);
    cJSON_Delete(json);

    return json_string;
}

/* ==================== WIFI MANAGEMENT ==================== */

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Infinite retry with exponential backoff (max 60 seconds between retries)
        g_bridge_ctx->wifi_retry_count++;
        g_bridge_ctx->wifi_connected = false;

        // Calculate backoff delay: min(2^retry_count * 1000ms, 60000ms)
        uint32_t delay_ms = (1 << (g_bridge_ctx->wifi_retry_count - 1)) * 1000;
        if (delay_ms > 60000) delay_ms = 60000;

        ESP_LOGW(TAG, "WiFi disconnected. Retry #%d in %d seconds",
                 g_bridge_ctx->wifi_retry_count, delay_ms / 1000);

        // Delay then reconnect (non-blocking)
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        esp_wifi_connect();

        send_event(MCP_EVENT_WIFI_DISCONNECTED, NULL);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        g_bridge_ctx->wifi_retry_count = 0;
        g_bridge_ctx->wifi_connected = true;
        xEventGroupSetBits(g_bridge_ctx->wifi_event_group, WIFI_CONNECTED_BIT);
        send_event(MCP_EVENT_WIFI_CONNECTED, NULL);
    }
}

/**
 * @brief Initialize WiFi
 */
static esp_err_t wifi_init_internal(const mcp_bridge_config_t *config) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    // Copy SSID and password
    strncpy((char*)wifi_config.sta.ssid, config->wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, config->wifi_password, sizeof(wifi_config.sta.password) - 1);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization finished");
    
    return ESP_OK;
}

/* ==================== MQTT MANAGEMENT ==================== */

/**
 * @brief MQTT event handler
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            g_bridge_ctx->mqtt_connected = true;
            g_bridge_ctx->mqtt_retry_count = 0;
            xEventGroupSetBits(g_bridge_ctx->mqtt_event_group, MQTT_CONNECTED_BIT);
            send_event(MCP_EVENT_MQTT_CONNECTED, NULL);
            
            // Subscribe to actuator command topics
            actuator_node_t *actuator = g_bridge_ctx->actuators;
            while (actuator) {
                char topic[MCP_BRIDGE_MAX_TOPIC_LEN];
                snprintf(topic, sizeof(topic), "devices/%s/actuators/%s/cmd", 
                        g_bridge_ctx->device_id, actuator->type);
                esp_mqtt_client_subscribe(g_bridge_ctx->mqtt_client, topic, 1);
                ESP_LOGI(TAG, "Subscribed to %s", topic);
                actuator = actuator->next;
            }
            
            // Publish capabilities
            char *capabilities = create_capabilities_message();
            if (capabilities) {
                char topic[MCP_BRIDGE_MAX_TOPIC_LEN];
                snprintf(topic, sizeof(topic), "devices/%s/capabilities", g_bridge_ctx->device_id);
                esp_mqtt_client_publish(g_bridge_ctx->mqtt_client, topic, capabilities, 0, 1, true);
                free(capabilities);
            }
            
            // Publish online status
            mcp_bridge_publish_device_status("online");
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            g_bridge_ctx->mqtt_connected = false;
            g_bridge_ctx->mqtt_reconnections++;

            // MQTT client has auto-reconnect, but we track disconnection events
            // The library will automatically attempt to reconnect
            ESP_LOGI(TAG, "MQTT will auto-reconnect (total reconnections: %d)",
                     g_bridge_ctx->mqtt_reconnections);

            xEventGroupSetBits(g_bridge_ctx->mqtt_event_group, MQTT_FAIL_BIT);
            send_event(MCP_EVENT_MQTT_DISCONNECTED, NULL);
            break;
            
        case MQTT_EVENT_DATA: {
            ESP_LOGI(TAG, "MQTT message received: %.*s", event->topic_len, event->topic);
            g_bridge_ctx->messages_received++;
            
            // Parse topic to extract actuator type
            char topic[MCP_BRIDGE_MAX_TOPIC_LEN];
            strncpy(topic, event->topic, event->topic_len);
            topic[event->topic_len] = '\0';
            
            // Topic format: devices/{device_id}/actuators/{actuator_type}/cmd
            char *token = strtok(topic, "/");
            if (token && strcmp(token, "devices") == 0) {
                token = strtok(NULL, "/"); // device_id
                token = strtok(NULL, "/"); // "actuators"
                if (token && strcmp(token, "actuators") == 0) {
                    char *actuator_type = strtok(NULL, "/");
                    token = strtok(NULL, "/"); // "cmd"
                    
                    if (actuator_type && token && strcmp(token, "cmd") == 0) {
                        // Parse JSON payload
                        char payload[MCP_BRIDGE_MAX_MESSAGE_LEN];
                        strncpy(payload, event->data, event->data_len);
                        payload[event->data_len] = '\0';
                        
                        cJSON *json = cJSON_Parse(payload);
                        if (json) {
                            cJSON *action_json = cJSON_GetObjectItem(json, "action");
                            cJSON *value_json = cJSON_GetObjectItem(json, "value");
                            
                            if (action_json && cJSON_IsString(action_json)) {
                                mcp_command_t cmd = {0};
                                strncpy(cmd.actuator_id, actuator_type, sizeof(cmd.actuator_id) - 1);
                                strncpy(cmd.action, action_json->valuestring, sizeof(cmd.action) - 1);
                                
                                if (value_json) {
                                    if (cJSON_IsString(value_json)) {
                                        strncpy(cmd.value, value_json->valuestring, sizeof(cmd.value) - 1);
                                    } else if (cJSON_IsNumber(value_json)) {
                                        snprintf(cmd.value, sizeof(cmd.value), "%.2f", value_json->valuedouble);
                                    } else if (cJSON_IsBool(value_json)) {
                                        strncpy(cmd.value, cJSON_IsTrue(value_json) ? "true" : "false", sizeof(cmd.value) - 1);
                                    }
                                }
                                cmd.timestamp = get_timestamp();
                                
                                // Queue command for processing
                                if (xQueueSend(g_bridge_ctx->command_queue, &cmd, 0) != pdTRUE) {
                                    ESP_LOGW(TAG, "Command queue full, dropping command");
                                }
                                
                                send_event(MCP_EVENT_COMMAND_RECEIVED, &cmd);
                            }
                            cJSON_Delete(json);
                        }
                    }
                }
            }
            break;
        }
        
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            g_bridge_ctx->connection_failures++;
            break;
            
        default:
            break;
    }
}

/**
 * @brief Initialize MQTT client
 */
static esp_err_t mqtt_init_internal(const mcp_bridge_config_t *config) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = config->mqtt_broker_uri,
        .credentials.client_id = g_bridge_ctx->device_id,
        .session.last_will = {
            .topic = NULL, // Will be set below
            .msg = "{\"value\":\"offline\"}",
            .qos = 1,
            .retain = true
        },
        .network.disable_auto_reconnect = false,  // Enable auto-reconnect (default, but explicit)
        .network.reconnect_timeout_ms = 10000,    // Retry every 10 seconds
        .network.timeout_ms = 10000               // Connection timeout
    };
    
    // Set last will topic
    static char will_topic[MCP_BRIDGE_MAX_TOPIC_LEN];
    snprintf(will_topic, sizeof(will_topic), "devices/%s/status", g_bridge_ctx->device_id);
    mqtt_cfg.session.last_will.topic = will_topic;
    
    g_bridge_ctx->mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!g_bridge_ctx->mqtt_client) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(g_bridge_ctx->mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(g_bridge_ctx->mqtt_client));
    
    return ESP_OK;
}

/* ==================== TASK IMPLEMENTATIONS ==================== */

/**
 * @brief Sensor polling task
 */
static void sensor_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Sensor polling task started");
    
    while (g_bridge_ctx->running) {
        // Wait for the configured interval
        vTaskDelayUntil(&xLastWakeTime, 
                       pdMS_TO_TICKS(g_bridge_ctx->config.sensor_publish_interval_ms));
        
        // Skip if not connected
        if (!g_bridge_ctx->mqtt_connected) {
            continue;
        }
        
        // Poll all registered sensors
        xSemaphoreTake(g_bridge_ctx->mutex, portMAX_DELAY);

        sensor_node_t *sensor = g_bridge_ctx->sensors;
        while (sensor) {
            esp_err_t ret;
            char *message = NULL;
            char topic[MCP_BRIDGE_MAX_TOPIC_LEN];

            if (sensor->is_multi_value) {
                // Multi-value sensor
                mcp_sensor_multi_value_t values = {0};
                ret = sensor->read_multi_cb(sensor->sensor_id, &values, sensor->user_data);

                if (ret == ESP_OK && values.field_count > 0) {
                    sensor->last_read_time = get_timestamp();

                    // Create and publish multi-value sensor data
                    message = create_multi_sensor_message(sensor->sensor_id, &values);
                    if (message) {
                        snprintf(topic, sizeof(topic), "devices/%s/sensors/%s/data",
                                g_bridge_ctx->device_id, sensor->sensor_id);
                    }
                }
            } else {
                // Single-value sensor
                float value;
                ret = sensor->read_cb(sensor->sensor_id, &value, sensor->user_data);

                if (ret == ESP_OK) {
                    sensor->last_value = value;
                    sensor->last_read_time = get_timestamp();

                    // Create and publish sensor data
                    message = create_sensor_message(sensor->sensor_id, sensor->type, value, sensor->unit);
                    if (message) {
                        snprintf(topic, sizeof(topic), "devices/%s/sensors/%s/data",
                                g_bridge_ctx->device_id, sensor->type);
                    }
                }
            }

            if (ret == ESP_OK && message) {
                int msg_id = esp_mqtt_client_publish(g_bridge_ctx->mqtt_client, topic, message, 0, 0, false);
                if (msg_id >= 0) {
                    g_bridge_ctx->messages_sent++;
                    ESP_LOGD(TAG, "Published sensor %s data", sensor->sensor_id);
                } else {
                    ESP_LOGE(TAG, "Failed to publish sensor data for %s", sensor->sensor_id);
                }

                free(message);
            } else if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read sensor %s: %s",
                        sensor->sensor_id, esp_err_to_name(ret));
                g_bridge_ctx->sensor_read_errors++;
            }

            sensor = sensor->next;
        }

        xSemaphoreGive(g_bridge_ctx->mutex);
    }
    
    vTaskDelete(NULL);
}

/**
 * @brief Actuator command processing task
 */
static void actuator_task(void *pvParameters) {
    mcp_command_t cmd;
    
    ESP_LOGI(TAG, "Actuator command task started");
    
    while (g_bridge_ctx->running) {
        if (xQueueReceive(g_bridge_ctx->command_queue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGI(TAG, "Processing command for %s: %s = %s", cmd.actuator_id, cmd.action, cmd.value);
            
            // Find the actuator
            actuator_node_t *actuator = find_actuator(cmd.actuator_id);
            if (actuator) {
                esp_err_t ret = actuator->control_cb(cmd.actuator_id, cmd.action, cmd.value, actuator->user_data);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Actuator control failed for %s: %s", cmd.actuator_id, esp_err_to_name(ret));
                    
                    // Publish error
                    char error_msg[128];
                    snprintf(error_msg, sizeof(error_msg), "Actuator control failed: %s", esp_err_to_name(ret));
                    mcp_bridge_publish_error("actuator_error", error_msg, 2);
                }
            } else {
                ESP_LOGE(TAG, "Unknown actuator: %s", cmd.actuator_id);
            }
        }
    }
    
    vTaskDelete(NULL);
}

/**
 * @brief Watchdog task
 */
static void watchdog_task(void *pvParameters) {
    ESP_LOGI(TAG, "Watchdog task started");
    
    while (g_bridge_ctx->running) {
        // Check system health
        if (esp_get_free_heap_size() < 10000) {
            ESP_LOGW(TAG, "Low memory warning: %d bytes free", esp_get_free_heap_size());
            mcp_bridge_publish_error("low_memory", "Free heap below 10KB", 1);
        }
        
        // Check connectivity
        if (!g_bridge_ctx->wifi_connected || !g_bridge_ctx->mqtt_connected) {
            ESP_LOGW(TAG, "Connectivity issues - WiFi: %s, MQTT: %s", 
                    g_bridge_ctx->wifi_connected ? "OK" : "FAIL",
                    g_bridge_ctx->mqtt_connected ? "OK" : "FAIL");
        }
        
        vTaskDelay(pdMS_TO_TICKS(30000)); // Check every 30 seconds
    }
    
    vTaskDelete(NULL);
}

/* ==================== PUBLIC API IMPLEMENTATION ==================== */

esp_err_t mcp_bridge_init(const mcp_bridge_config_t *config) {
    if (g_bridge_ctx != NULL) {
        ESP_LOGE(TAG, "Bridge already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Allocate context
    g_bridge_ctx = calloc(1, sizeof(mcp_bridge_context_t));
    if (!g_bridge_ctx) {
        ESP_LOGE(TAG, "Failed to allocate bridge context");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy or set default configuration
    if (config) {
        memcpy(&g_bridge_ctx->config, config, sizeof(mcp_bridge_config_t));
    } else {
        // Use default values
        g_bridge_ctx->config.wifi_ssid = CONFIG_MCP_BRIDGE_WIFI_SSID;
        g_bridge_ctx->config.wifi_password = CONFIG_MCP_BRIDGE_WIFI_PASSWORD;
        g_bridge_ctx->config.mqtt_broker_uri = CONFIG_MCP_BRIDGE_MQTT_BROKER_URL;
        g_bridge_ctx->config.sensor_publish_interval_ms = CONFIG_MCP_BRIDGE_SENSOR_PUBLISH_INTERVAL;
        g_bridge_ctx->config.enable_watchdog = CONFIG_MCP_BRIDGE_ENABLE_WATCHDOG;
    }
    
    // Validate configuration
    if (!g_bridge_ctx->config.wifi_ssid || !g_bridge_ctx->config.wifi_password || !g_bridge_ctx->config.mqtt_broker_uri) {
        ESP_LOGE(TAG, "Invalid configuration: missing required parameters");
        free(g_bridge_ctx);
        g_bridge_ctx = NULL;
        return ESP_ERR_INVALID_ARG;
    }
    
    // Generate or copy device ID
    if (config && config->device_id) {
        strncpy(g_bridge_ctx->device_id, config->device_id, sizeof(g_bridge_ctx->device_id) - 1);
    } else {
        generate_device_id(g_bridge_ctx->device_id, sizeof(g_bridge_ctx->device_id));
    }

    // Store device name
    if (config && config->device_name) {
        g_bridge_ctx->device_name = config->device_name;
    }

    // Store device location
    if (config && config->device_location) {
        g_bridge_ctx->device_location = config->device_location;
    }
    
    // Create synchronization primitives
    g_bridge_ctx->mutex = xSemaphoreCreateMutex();
    if (!g_bridge_ctx->mutex) {
        free(g_bridge_ctx);
        g_bridge_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    g_bridge_ctx->command_queue = xQueueCreate(MCP_BRIDGE_COMMAND_QUEUE_SIZE, sizeof(mcp_command_t));
    if (!g_bridge_ctx->command_queue) {
        vSemaphoreDelete(g_bridge_ctx->mutex);
        free(g_bridge_ctx);
        g_bridge_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    g_bridge_ctx->wifi_event_group = xEventGroupCreate();
    g_bridge_ctx->mqtt_event_group = xEventGroupCreate();
    if (!g_bridge_ctx->wifi_event_group || !g_bridge_ctx->mqtt_event_group) {
        if (g_bridge_ctx->wifi_event_group) vEventGroupDelete(g_bridge_ctx->wifi_event_group);
        if (g_bridge_ctx->mqtt_event_group) vEventGroupDelete(g_bridge_ctx->mqtt_event_group);
        vQueueDelete(g_bridge_ctx->command_queue);
        vSemaphoreDelete(g_bridge_ctx->mutex);
        free(g_bridge_ctx);
        g_bridge_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    g_bridge_ctx->boot_time = get_timestamp();
    g_bridge_ctx->initialized = true;
    
    ESP_LOGI(TAG, "MCP Bridge initialized with device ID: %s", g_bridge_ctx->device_id);
    
    return ESP_OK;
}

esp_err_t mcp_bridge_init_default(void) {
    return mcp_bridge_init(NULL);
}

esp_err_t mcp_bridge_start(void) {
    if (!g_bridge_ctx || !g_bridge_ctx->initialized) {
        ESP_LOGE(TAG, "Bridge not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_bridge_ctx->running) {
        ESP_LOGW(TAG, "Bridge already running");
        return ESP_OK;
    }
    
    g_bridge_ctx->running = true;
    
    // Initialize WiFi
    esp_err_t ret = wifi_init_internal(&g_bridge_ctx->config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        g_bridge_ctx->running = false;
        return ret;
    }
    
    // Wait for WiFi connection
    EventBits_t bits = xEventGroupWaitBits(g_bridge_ctx->wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          pdMS_TO_TICKS(30000));
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        g_bridge_ctx->running = false;
        return ESP_ERR_WIFI_CONN;
    } else {
        ESP_LOGE(TAG, "WiFi connection timeout");
        g_bridge_ctx->running = false;
        return ESP_ERR_TIMEOUT;
    }
    
    // Initialize MQTT
    ret = mqtt_init_internal(&g_bridge_ctx->config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT: %s", esp_err_to_name(ret));
        g_bridge_ctx->running = false;
        return ret;
    }
    
    // Create tasks
    xTaskCreate(sensor_task, "mcp_sensor", 4096, NULL, 5, &g_bridge_ctx->sensor_task_handle);
    xTaskCreate(actuator_task, "mcp_actuator", 3072, NULL, 6, &g_bridge_ctx->actuator_task_handle);
    
    if (g_bridge_ctx->config.enable_watchdog) {
        xTaskCreate(watchdog_task, "mcp_watchdog", 2048, NULL, 4, &g_bridge_ctx->watchdog_task_handle);
    }
    
    ESP_LOGI(TAG, "MCP Bridge started successfully");
    return ESP_OK;
}

esp_err_t mcp_bridge_stop(void) {
    if (!g_bridge_ctx || !g_bridge_ctx->running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_bridge_ctx->running = false;
    
    // Publish offline status
    if (g_bridge_ctx->mqtt_connected) {
        mcp_bridge_publish_device_status("offline");
    }
    
    // Stop tasks
    if (g_bridge_ctx->sensor_task_handle) {
        vTaskDelete(g_bridge_ctx->sensor_task_handle);
        g_bridge_ctx->sensor_task_handle = NULL;
    }
    if (g_bridge_ctx->actuator_task_handle) {
        vTaskDelete(g_bridge_ctx->actuator_task_handle);
        g_bridge_ctx->actuator_task_handle = NULL;
    }
    if (g_bridge_ctx->watchdog_task_handle) {
        vTaskDelete(g_bridge_ctx->watchdog_task_handle);
        g_bridge_ctx->watchdog_task_handle = NULL;
    }
    
    // Stop MQTT client
    if (g_bridge_ctx->mqtt_client) {
        esp_mqtt_client_stop(g_bridge_ctx->mqtt_client);
        esp_mqtt_client_destroy(g_bridge_ctx->mqtt_client);
        g_bridge_ctx->mqtt_client = NULL;
    }
    
    ESP_LOGI(TAG, "MCP Bridge stopped");
    return ESP_OK;
}

esp_err_t mcp_bridge_deinit(void) {
    if (!g_bridge_ctx) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_bridge_ctx->running) {
        ESP_LOGE(TAG, "Bridge must be stopped before deinitializing");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Free sensor list
    sensor_node_t *sensor = g_bridge_ctx->sensors;
    while (sensor) {
        sensor_node_t *next = sensor->next;
        free(sensor->sensor_id);
        free(sensor->type);
        free(sensor->unit);
        if (sensor->field_metadata) {
            free(sensor->field_metadata);
        }
        free(sensor);
        sensor = next;
    }
    
    // Free actuator list
    actuator_node_t *actuator = g_bridge_ctx->actuators;
    while (actuator) {
        actuator_node_t *next = actuator->next;
        free(actuator->actuator_id);
        free(actuator->type);
        free(actuator->last_status);
        free(actuator);
        actuator = next;
    }
    
    // Clean up synchronization objects
    if (g_bridge_ctx->mutex) vSemaphoreDelete(g_bridge_ctx->mutex);
    if (g_bridge_ctx->command_queue) vQueueDelete(g_bridge_ctx->command_queue);
    if (g_bridge_ctx->wifi_event_group) vEventGroupDelete(g_bridge_ctx->wifi_event_group);
    if (g_bridge_ctx->mqtt_event_group) vEventGroupDelete(g_bridge_ctx->mqtt_event_group);
    
    free(g_bridge_ctx);
    g_bridge_ctx = NULL;
    
    ESP_LOGI(TAG, "MCP Bridge deinitialized");
    return ESP_OK;
}

esp_err_t mcp_bridge_register_event_handler(mcp_event_handler_t handler, void *user_data) {
    if (!g_bridge_ctx || !handler) {
        return ESP_ERR_INVALID_ARG;
    }
    
    g_bridge_ctx->event_handler = handler;
    g_bridge_ctx->event_handler_user_data = user_data;
    
    return ESP_OK;
}

esp_err_t mcp_bridge_register_sensor(const char *sensor_id,
                                    const char *type,
                                    const char *unit,
                                    const mcp_sensor_metadata_t *metadata,
                                    mcp_sensor_read_cb_t read_cb,
                                    void *user_data) {
    if (!g_bridge_ctx || !sensor_id || !type || !read_cb) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_bridge_ctx->sensor_count >= MCP_BRIDGE_MAX_SENSORS) {
        ESP_LOGE(TAG, "Maximum number of sensors reached");
        return ESP_ERR_NO_MEM;
    }
    
    // Check for duplicate
    if (find_sensor(sensor_id)) {
        ESP_LOGE(TAG, "Sensor %s already registered", sensor_id);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Allocate new sensor node
    sensor_node_t *node = calloc(1, sizeof(sensor_node_t));
    if (!node) {
        return ESP_ERR_NO_MEM;
    }
    
    // Copy sensor information
    node->sensor_id = strdup(sensor_id);
    node->type = strdup(type);
    if (unit) {
        node->unit = strdup(unit);
    }
    if (metadata) {
        memcpy(&node->metadata, metadata, sizeof(mcp_sensor_metadata_t));
    }
    node->read_cb = read_cb;
    node->user_data = user_data;
    
    // Add to list
    xSemaphoreTake(g_bridge_ctx->mutex, portMAX_DELAY);
    node->next = g_bridge_ctx->sensors;
    g_bridge_ctx->sensors = node;
    g_bridge_ctx->sensor_count++;
    xSemaphoreGive(g_bridge_ctx->mutex);
    
    ESP_LOGI(TAG, "Registered sensor: %s (type: %s)", sensor_id, type);

    return ESP_OK;
}

esp_err_t mcp_bridge_register_multi_sensor(const char *sensor_id,
                                           const mcp_sensor_field_metadata_t *field_metadata,
                                           size_t field_count,
                                           const mcp_sensor_metadata_t *metadata,
                                           mcp_sensor_read_multi_cb_t read_cb,
                                           void *user_data) {
    if (!g_bridge_ctx || !sensor_id || !read_cb) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_bridge_ctx->sensor_count >= MCP_BRIDGE_MAX_SENSORS) {
        ESP_LOGE(TAG, "Maximum number of sensors reached");
        return ESP_ERR_NO_MEM;
    }

    // Check for duplicate
    if (find_sensor(sensor_id)) {
        ESP_LOGE(TAG, "Sensor %s already registered", sensor_id);
        return ESP_ERR_INVALID_STATE;
    }

    // Allocate new sensor node
    sensor_node_t *node = calloc(1, sizeof(sensor_node_t));
    if (!node) {
        return ESP_ERR_NO_MEM;
    }

    // Copy sensor information
    node->sensor_id = strdup(sensor_id);
    node->is_multi_value = true;
    if (metadata) {
        memcpy(&node->metadata, metadata, sizeof(mcp_sensor_metadata_t));
    }
    node->read_multi_cb = read_cb;
    node->user_data = user_data;

    // Copy field metadata if provided
    if (field_metadata && field_count > 0) {
        node->field_metadata = calloc(field_count, sizeof(mcp_sensor_field_metadata_t));
        if (!node->field_metadata) {
            free(node->sensor_id);
            free(node);
            return ESP_ERR_NO_MEM;
        }
        memcpy(node->field_metadata, field_metadata, field_count * sizeof(mcp_sensor_field_metadata_t));
        node->field_count = field_count;
    }

    // Add to list
    xSemaphoreTake(g_bridge_ctx->mutex, portMAX_DELAY);
    node->next = g_bridge_ctx->sensors;
    g_bridge_ctx->sensors = node;
    g_bridge_ctx->sensor_count++;
    xSemaphoreGive(g_bridge_ctx->mutex);

    ESP_LOGI(TAG, "Registered multi-value sensor: %s with %d fields", sensor_id, field_count);

    return ESP_OK;
}

esp_err_t mcp_bridge_register_actuator(const char *actuator_id,
                                      const char *type,
                                      const mcp_actuator_metadata_t *metadata,
                                      mcp_actuator_control_cb_t control_cb,
                                      void *user_data) {
    if (!g_bridge_ctx || !actuator_id || !type || !control_cb) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_bridge_ctx->actuator_count >= MCP_BRIDGE_MAX_ACTUATORS) {
        ESP_LOGE(TAG, "Maximum number of actuators reached");
        return ESP_ERR_NO_MEM;
    }
    
    // Check for duplicate
    if (find_actuator(actuator_id)) {
        ESP_LOGE(TAG, "Actuator %s already registered", actuator_id);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Allocate new actuator node
    actuator_node_t *node = calloc(1, sizeof(actuator_node_t));
    if (!node) {
        return ESP_ERR_NO_MEM;
    }
    
    // Copy actuator information
    node->actuator_id = strdup(actuator_id);
    node->type = strdup(type);
    if (metadata) {
        memcpy(&node->metadata, metadata, sizeof(mcp_actuator_metadata_t));
    }
    node->control_cb = control_cb;
    node->user_data = user_data;
    
    // Add to list
    xSemaphoreTake(g_bridge_ctx->mutex, portMAX_DELAY);
    node->next = g_bridge_ctx->actuators;
    g_bridge_ctx->actuators = node;
    g_bridge_ctx->actuator_count++;
    xSemaphoreGive(g_bridge_ctx->mutex);
    
    ESP_LOGI(TAG, "Registered actuator: %s (type: %s)", actuator_id, type);
    
    return ESP_OK;
}

esp_err_t mcp_bridge_publish_sensor_data(const char *sensor_id, float value) {
    if (!g_bridge_ctx || !sensor_id) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_bridge_ctx->mqtt_connected) {
        ESP_LOGW(TAG, "Cannot publish - MQTT not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    sensor_node_t *sensor = find_sensor(sensor_id);
    if (!sensor) {
        ESP_LOGE(TAG, "Unknown sensor: %s", sensor_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Create and publish sensor data
    char *message = create_sensor_message(sensor_id, sensor->type, value, sensor->unit);
    if (!message) {
        return ESP_ERR_NO_MEM;
    }
    
    char topic[MCP_BRIDGE_MAX_TOPIC_LEN];
    snprintf(topic, sizeof(topic), "devices/%s/sensors/%s/data", 
            g_bridge_ctx->device_id, sensor->type);
    
    int msg_id = esp_mqtt_client_publish(g_bridge_ctx->mqtt_client, topic, message, 0, 0, false);
    free(message);
    
    if (msg_id >= 0) {
        g_bridge_ctx->messages_sent++;
        sensor->last_value = value;
        sensor->last_read_time = get_timestamp();
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t mcp_bridge_publish_actuator_status(const char *actuator_id, const char *status) {
    if (!g_bridge_ctx || !actuator_id || !status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_bridge_ctx->mqtt_connected) {
        ESP_LOGW(TAG, "Cannot publish - MQTT not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    actuator_node_t *actuator = find_actuator(actuator_id);
    if (!actuator) {
        ESP_LOGE(TAG, "Unknown actuator: %s", actuator_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Create status message
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "device_id", g_bridge_ctx->device_id);
    cJSON_AddNumberToObject(json, "timestamp", get_timestamp());
    cJSON_AddStringToObject(json, "value", status);
    
    char *message = cJSON_Print(json);
    cJSON_Delete(json);
    
    if (!message) {
        return ESP_ERR_NO_MEM;
    }
    
    char topic[MCP_BRIDGE_MAX_TOPIC_LEN];
    snprintf(topic, sizeof(topic), "devices/%s/actuators/%s/status", 
            g_bridge_ctx->device_id, actuator->type);
    
    int msg_id = esp_mqtt_client_publish(g_bridge_ctx->mqtt_client, topic, message, 0, 1, false);
    free(message);
    
    if (msg_id >= 0) {
        g_bridge_ctx->messages_sent++;
        
        // Update last status
        free(actuator->last_status);
        actuator->last_status = strdup(status);
        
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t mcp_bridge_publish_device_status(const char *status) {
    if (!g_bridge_ctx || !status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_bridge_ctx->mqtt_client) {
        ESP_LOGW(TAG, "Cannot publish - MQTT client not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create status message
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "value", status);
    cJSON_AddNumberToObject(json, "timestamp", get_timestamp());
    
    char *message = cJSON_Print(json);
    cJSON_Delete(json);
    
    if (!message) {
        return ESP_ERR_NO_MEM;
    }
    
    char topic[MCP_BRIDGE_MAX_TOPIC_LEN];
    snprintf(topic, sizeof(topic), "devices/%s/status", g_bridge_ctx->device_id);
    
    int msg_id = esp_mqtt_client_publish(g_bridge_ctx->mqtt_client, topic, message, 0, 1, true);
    free(message);
    
    if (msg_id >= 0) {
        g_bridge_ctx->messages_sent++;
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t mcp_bridge_publish_error(const char *error_type, 
                                  const char *message, 
                                  uint8_t severity) {
    if (!g_bridge_ctx || !error_type || !message) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_bridge_ctx->mqtt_connected) {
        ESP_LOGW(TAG, "Cannot publish error - MQTT not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create error message
    cJSON *json = cJSON_CreateObject();
    cJSON *value_obj = cJSON_CreateObject();
    
    cJSON_AddStringToObject(json, "device_id", g_bridge_ctx->device_id);
    cJSON_AddNumberToObject(json, "timestamp", get_timestamp());
    
    cJSON_AddStringToObject(value_obj, "error_type", error_type);
    cJSON_AddStringToObject(value_obj, "message", message);
    cJSON_AddNumberToObject(value_obj, "severity", severity);
    
    cJSON_AddItemToObject(json, "value", value_obj);
    
    char *json_message = cJSON_Print(json);
    cJSON_Delete(json);
    
    if (!json_message) {
        return ESP_ERR_NO_MEM;
    }
    
    char topic[MCP_BRIDGE_MAX_TOPIC_LEN];
    snprintf(topic, sizeof(topic), "devices/%s/error", g_bridge_ctx->device_id);
    
    int msg_id = esp_mqtt_client_publish(g_bridge_ctx->mqtt_client, topic, json_message, 0, 1, false);
    free(json_message);
    
    if (msg_id >= 0) {
        g_bridge_ctx->messages_sent++;
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t mcp_bridge_get_status(bool *wifi_connected, bool *mqtt_connected) {
    if (!g_bridge_ctx) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (wifi_connected) {
        *wifi_connected = g_bridge_ctx->wifi_connected;
    }
    if (mqtt_connected) {
        *mqtt_connected = g_bridge_ctx->mqtt_connected;
    }
    
    return ESP_OK;
}

const char* mcp_bridge_get_device_id(void) {
    return g_bridge_ctx ? g_bridge_ctx->device_id : NULL;
}

esp_err_t mcp_bridge_reconnect(void) {
    if (!g_bridge_ctx) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Forcing reconnection...");
    
    // Reset retry counters
    g_bridge_ctx->wifi_retry_count = 0;
    g_bridge_ctx->mqtt_retry_count = 0;
    
    // Disconnect and reconnect WiFi
    if (g_bridge_ctx->wifi_connected) {
        esp_wifi_disconnect();
    }
    
    // Stop and restart MQTT client
    if (g_bridge_ctx->mqtt_client) {
        esp_mqtt_client_stop(g_bridge_ctx->mqtt_client);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_mqtt_client_start(g_bridge_ctx->mqtt_client);
    }
    
    return ESP_OK;
}
