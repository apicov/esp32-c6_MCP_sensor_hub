/**
 * @file esp_mcp_bridge.h
 * @brief ESP32 MQTT-MCP Bridge Library Public API
 * 
 * This library provides a bridge between ESP32 IoT devices and MCP (Model Context Protocol)
 * servers using MQTT as the transport layer. It handles device management, sensor/actuator
 * abstraction, and standardized message formatting.
 */

#ifndef ESP_MCP_BRIDGE_H
#define ESP_MCP_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "mcp_device.h"

/**
 * @brief MCP Bridge specific error codes
 */
typedef enum {
    MCP_BRIDGE_ERR_BASE = ESP_ERR_INVALID_STATE,
    MCP_BRIDGE_ERR_WIFI_FAILED = 0x7001,        /**< WiFi connection failed */
    MCP_BRIDGE_ERR_MQTT_FAILED,                 /**< MQTT connection failed */
    MCP_BRIDGE_ERR_SENSOR_FAILED,               /**< Sensor read failed */
    MCP_BRIDGE_ERR_ACTUATOR_FAILED,             /**< Actuator control failed */
    MCP_BRIDGE_ERR_MEMORY_FULL,                 /**< Memory allocation failed */
    MCP_BRIDGE_ERR_CONFIG_INVALID,              /**< Invalid configuration */
    MCP_BRIDGE_ERR_NOT_INITIALIZED,             /**< Bridge not initialized */
    MCP_BRIDGE_ERR_ALREADY_RUNNING,             /**< Bridge already running */
    MCP_BRIDGE_ERR_TLS_FAILED,                  /**< TLS/SSL connection failed */
    MCP_BRIDGE_ERR_AUTH_FAILED,                 /**< Authentication failed */
} mcp_bridge_err_t;

/**
 * @brief MQTT QoS configuration
 */
typedef struct {
    uint8_t sensor_qos;                         /**< QoS for sensor data (0-2) */
    uint8_t actuator_qos;                       /**< QoS for actuator commands (0-2) */
    uint8_t status_qos;                         /**< QoS for status messages (0-2) */
    uint8_t error_qos;                          /**< QoS for error messages (0-2) */
} mcp_mqtt_qos_config_t;

/**
 * @brief TLS/SSL configuration
 */
typedef struct {
    bool enable_tls;                            /**< Enable TLS/SSL */
    const char *ca_cert_pem;                    /**< CA certificate PEM */
    const char *client_cert_pem;                /**< Client certificate PEM */
    const char *client_key_pem;                 /**< Client private key PEM */
    bool skip_cert_verification;                /**< Skip certificate verification (insecure) */
    const char *alpn_protocols[4];              /**< ALPN protocol list */
} mcp_tls_config_t;

/**
 * @brief MCP Bridge configuration structure
 */
typedef struct {
    const char *wifi_ssid;                      /**< WiFi SSID (NULL to use Kconfig) */
    const char *wifi_password;                  /**< WiFi password (NULL to use Kconfig) */
    const char *mqtt_broker_uri;                /**< MQTT broker URI (NULL to use Kconfig) */
    const char *mqtt_username;                  /**< MQTT username (NULL for no auth) */
    const char *mqtt_password;                  /**< MQTT password (NULL for no auth) */
    const char *device_id;                      /**< Device ID (NULL to auto-generate) */
    uint32_t sensor_publish_interval_ms;        /**< Sensor publish interval (0 for default) */
    uint32_t command_timeout_ms;                /**< Command timeout in milliseconds */
    bool enable_watchdog;                       /**< Enable watchdog timer */
    bool enable_device_auth;                    /**< Enable device authentication */
    uint8_t log_level;                         /**< Log level (0-5) */
    mcp_mqtt_qos_config_t qos_config;          /**< MQTT QoS configuration */
    mcp_tls_config_t tls_config;               /**< TLS/SSL configuration */
} mcp_bridge_config_t;

/**
 * @brief MCP Bridge metrics structure
 */
typedef struct {
    uint32_t messages_sent;                     /**< Total messages sent */
    uint32_t messages_received;                 /**< Total messages received */
    uint32_t connection_failures;               /**< Number of connection failures */
    uint32_t sensor_read_errors;                /**< Number of sensor read errors */
    uint32_t actuator_errors;                   /**< Number of actuator control errors */
    uint32_t uptime_seconds;                    /**< Device uptime in seconds */
    uint32_t wifi_reconnections;                /**< Number of WiFi reconnections */
    uint32_t mqtt_reconnections;                /**< Number of MQTT reconnections */
    uint32_t free_heap_size;                    /**< Current free heap size */
    uint32_t min_free_heap_size;                /**< Minimum free heap size since boot */
} mcp_bridge_metrics_t;

/**
 * @brief MCP Bridge event types
 */
typedef enum {
    MCP_EVENT_WIFI_CONNECTED,                   /**< WiFi connected */
    MCP_EVENT_WIFI_DISCONNECTED,                /**< WiFi disconnected */
    MCP_EVENT_MQTT_CONNECTED,                   /**< MQTT connected */
    MCP_EVENT_MQTT_DISCONNECTED,                /**< MQTT disconnected */
    MCP_EVENT_COMMAND_RECEIVED,                 /**< Actuator command received */
    MCP_EVENT_SENSOR_READ_ERROR,                /**< Sensor read error */
    MCP_EVENT_ACTUATOR_ERROR,                   /**< Actuator control error */
    MCP_EVENT_LOW_MEMORY,                       /**< Low memory warning */
    MCP_EVENT_TLS_ERROR,                        /**< TLS/SSL error */
    MCP_EVENT_AUTH_ERROR,                       /**< Authentication error */
    MCP_EVENT_ERROR,                           /**< General error occurred */
} mcp_event_type_t;

/**
 * @brief MCP Bridge event data
 */
typedef struct {
    mcp_event_type_t type;                      /**< Event type */
    union {
        struct {
            const char *actuator_id;
            const char *action;
            void *value;
            uint32_t timestamp;
        } command;                              /**< Command event data */
        struct {
            const char *sensor_id;
            esp_err_t error_code;
            const char *error_message;
        } sensor_error;                         /**< Sensor error data */
        struct {
            const char *actuator_id;
            esp_err_t error_code;
            const char *error_message;
        } actuator_error;                       /**< Actuator error data */
        struct {
            const char *error_type;
            const char *message;
            uint8_t severity;
        } error;                                /**< General error event data */
        struct {
            uint32_t free_heap;
            uint32_t threshold;
        } low_memory;                           /**< Low memory event data */
    } data;
} mcp_event_t;

/**
 * @brief Event handler callback type
 */
typedef void (*mcp_event_handler_t)(const mcp_event_t *event, void *user_data);

/**
 * @brief Sensor read callback type
 * @param sensor_id Sensor identifier
 * @param value Output parameter for sensor value
 * @param user_data User data passed during registration
 * @return ESP_OK on success, error code on failure
 */
typedef esp_err_t (*mcp_sensor_read_cb_t)(const char *sensor_id, float *value, void *user_data);

/**
 * @brief Multi-value sensor read callback type
 * @param sensor_id Sensor identifier
 * @param values Output parameter for multi-value sensor readings
 * @param user_data User data passed during registration
 * @return ESP_OK on success, error code on failure
 */
typedef esp_err_t (*mcp_sensor_read_multi_cb_t)(const char *sensor_id, mcp_sensor_multi_value_t *values, void *user_data);

/**
 * @brief Actuator control callback type
 * @param actuator_id Actuator identifier
 * @param action Action to perform (read/write/toggle)
 * @param value Value to set (NULL for read/toggle)
 * @param user_data User data passed during registration
 * @return ESP_OK on success, error code on failure
 */
typedef esp_err_t (*mcp_actuator_control_cb_t)(const char *actuator_id, 
                                               const char *action, 
                                               const void *value, 
                                               void *user_data);

/**
 * @brief Initialize the MCP Bridge with default configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_init_default(void);

/**
 * @brief Initialize the MCP Bridge with custom configuration
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_init(const mcp_bridge_config_t *config);

/**
 * @brief Start the MCP Bridge
 * 
 * This function starts all bridge tasks and begins connection attempts.
 * Must be called after mcp_bridge_init().
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_start(void);

/**
 * @brief Stop the MCP Bridge
 * 
 * Gracefully shuts down all connections and stops all tasks.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_stop(void);

/**
 * @brief Deinitialize the MCP Bridge
 * 
 * Frees all resources. Bridge must be stopped before calling this.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_deinit(void);

/**
 * @brief Register an event handler
 * @param handler Event handler function
 * @param user_data User data to pass to handler
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_register_event_handler(mcp_event_handler_t handler, void *user_data);

/**
 * @brief Register a sensor
 * @param sensor_id Unique sensor identifier
 * @param type Sensor type (temperature, humidity, etc.)
 * @param unit Unit of measurement
 * @param metadata Additional sensor metadata (can be NULL)
 * @param read_cb Callback function to read sensor value
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_register_sensor(const char *sensor_id,
                                    const char *type,
                                    const char *unit,
                                    const mcp_sensor_metadata_t *metadata,
                                    mcp_sensor_read_cb_t read_cb,
                                    void *user_data);

/**
 * @brief Register a multi-value sensor
 *
 * Use this for sensors that provide multiple readings (e.g., a sensor chip
 * that measures temperature, humidity, and pressure).
 *
 * @param sensor_id Unique sensor identifier
 * @param field_metadata Array of field metadata (NULL-terminated or use field_count)
 * @param field_count Number of fields (if 0, field_metadata must be NULL-terminated)
 * @param metadata General sensor metadata (can be NULL)
 * @param read_cb Callback function to read all sensor values
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_register_multi_sensor(const char *sensor_id,
                                           const mcp_sensor_field_metadata_t *field_metadata,
                                           size_t field_count,
                                           const mcp_sensor_metadata_t *metadata,
                                           mcp_sensor_read_multi_cb_t read_cb,
                                           void *user_data);

/**
 * @brief Register an actuator
 * @param actuator_id Unique actuator identifier
 * @param type Actuator type (led, relay, motor, etc.)
 * @param metadata Additional actuator metadata (can be NULL)
 * @param control_cb Callback function to control actuator
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_register_actuator(const char *actuator_id,
                                      const char *type,
                                      const mcp_actuator_metadata_t *metadata,
                                      mcp_actuator_control_cb_t control_cb,
                                      void *user_data);

/**
 * @brief Manually publish sensor data
 * 
 * Use this for event-driven sensors that don't follow the regular polling interval.
 * 
 * @param sensor_id Sensor identifier
 * @param value Sensor value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_publish_sensor_data(const char *sensor_id, float value);

/**
 * @brief Publish multiple sensor readings in batch
 * 
 * More efficient than individual calls for multiple sensors.
 * 
 * @param readings Array of sensor readings
 * @param count Number of readings in array
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_publish_sensor_batch(const mcp_sensor_reading_t *readings, size_t count);

/**
 * @brief Publish actuator status
 * @param actuator_id Actuator identifier
 * @param status Status string (e.g., "on", "off", "50%")
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_publish_actuator_status(const char *actuator_id, const char *status);

/**
 * @brief Publish device status
 * @param status Status string (e.g., "online", "offline", "error")
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_publish_device_status(const char *status);

/**
 * @brief Publish error message
 * @param error_type Error type/category
 * @param message Error message
 * @param severity Severity level (0=info, 1=warning, 2=error, 3=critical)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_publish_error(const char *error_type, 
                                  const char *message, 
                                  uint8_t severity);

/**
 * @brief Get current connection status
 * @param wifi_connected Output: WiFi connection status
 * @param mqtt_connected Output: MQTT connection status
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_get_status(bool *wifi_connected, bool *mqtt_connected);

/**
 * @brief Get bridge metrics
 * @param metrics Output: Bridge metrics structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_get_metrics(mcp_bridge_metrics_t *metrics);

/**
 * @brief Get device ID
 * @return Device ID string (do not free)
 */
const char* mcp_bridge_get_device_id(void);

/**
 * @brief Force reconnection
 * 
 * Forces the bridge to reconnect WiFi and MQTT connections.
 * Useful for handling configuration changes.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_reconnect(void);

/**
 * @brief Update configuration at runtime
 * 
 * Updates bridge configuration without full restart.
 * Note: Some changes may require reconnection.
 * 
 * @param config New configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_update_config(const mcp_bridge_config_t *config);

/**
 * @brief Enable/disable streaming mode for a sensor
 * 
 * In streaming mode, sensor data is published at high frequency.
 * 
 * @param sensor_id Sensor identifier
 * @param enable Enable or disable streaming
 * @param interval_ms Streaming interval in milliseconds (if enabled)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_set_sensor_streaming(const char *sensor_id, bool enable, uint32_t interval_ms);

/**
 * @brief Reset bridge statistics
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_bridge_reset_metrics(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_MCP_BRIDGE_H */ 