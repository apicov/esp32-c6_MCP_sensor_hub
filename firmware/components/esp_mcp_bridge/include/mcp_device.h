/**
 * @file mcp_device.h
 * @brief Device abstraction layer for MCP Bridge
 */

#ifndef MCP_DEVICE_H
#define MCP_DEVICE_H
#include "esp_chip_info.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Sensor reading structure for batch operations
 */
typedef struct {
    const char *sensor_id;              /**< Sensor identifier */
    const char *sensor_type;            /**< Sensor type */
    float value;                        /**< Sensor value */
    const char *unit;                   /**< Unit of measurement */
    uint32_t timestamp;                 /**< Reading timestamp */
    float quality;                      /**< Data quality (0-100) */
} mcp_sensor_reading_t;

/**
 * @brief Sensor metadata structure
 */
typedef struct {
    float min_range;                    /**< Minimum measurable value */
    float max_range;                    /**< Maximum measurable value */
    float accuracy;                     /**< Accuracy/precision */
    uint32_t update_interval_ms;        /**< Update interval in milliseconds */
    const char *description;            /**< Human-readable description */
    bool calibration_required;          /**< Whether calibration is required */
    uint32_t calibration_interval_s;    /**< Calibration interval in seconds */
} mcp_sensor_metadata_t;

/**
 * @brief Actuator metadata structure
 */
typedef struct {
    const char *value_type;             /**< Value type (boolean, integer, float, string) */
    const char *description;            /**< Human-readable description */
    const char **supported_actions;     /**< NULL-terminated array of supported actions */
    const void *min_value;              /**< Minimum value (optional) */
    const void *max_value;              /**< Maximum value (optional) */
    uint32_t response_time_ms;          /**< Expected response time in milliseconds */
    bool requires_confirmation;         /**< Whether command requires confirmation */
} mcp_actuator_metadata_t;

/**
 * @brief Device capabilities structure
 */
typedef struct {
    const char *device_id;              /**< Unique device identifier */
    const char *firmware_version;       /**< Firmware version string */
    const char *hardware_version;       /**< Hardware version string */
    const char *manufacturer;           /**< Manufacturer name */
    const char *model;                  /**< Device model */
    const char *serial_number;          /**< Device serial number */
    uint32_t max_sensors;               /**< Maximum number of sensors */
    uint32_t max_actuators;             /**< Maximum number of actuators */
    bool supports_ota_update;           /**< Whether OTA updates are supported */
    bool supports_remote_config;        /**< Whether remote configuration is supported */
} mcp_device_info_t;

/**
 * @brief Sensor calibration data
 */
typedef struct {
    float offset;                       /**< Calibration offset */
    float scale;                        /**< Calibration scale factor */
    uint32_t last_calibration;          /**< Last calibration timestamp */
    bool is_valid;                      /**< Whether calibration is valid */
} mcp_sensor_calibration_t;

/**
 * @brief Actuator state information
 */
typedef struct {
    const char *current_state;          /**< Current actuator state */
    uint32_t last_command_time;         /**< Last command timestamp */
    uint32_t total_operations;          /**< Total number of operations */
    bool is_operational;                /**< Whether actuator is operational */
    const char *last_error;             /**< Last error message */
} mcp_actuator_state_t;

/* ==================== DEVICE UTILITY FUNCTIONS ==================== */

/**
 * @brief Generate a unique device ID based on MAC address
 * @param device_id Output buffer for device ID
 * @param max_len Maximum length of device ID buffer
 * @param prefix Optional prefix for device ID (can be NULL)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_device_generate_id(char *device_id, size_t max_len, const char *prefix);

/**
 * @brief Validate sensor metadata structure
 * @param metadata Sensor metadata to validate
 * @return ESP_OK if valid, error code if invalid
 */
esp_err_t mcp_device_validate_sensor_metadata(const mcp_sensor_metadata_t *metadata);

/**
 * @brief Validate actuator metadata structure
 * @param metadata Actuator metadata to validate
 * @return ESP_OK if valid, error code if invalid
 */
esp_err_t mcp_device_validate_actuator_metadata(const mcp_actuator_metadata_t *metadata);

/**
 * @brief Apply sensor calibration to raw value
 * @param raw_value Raw sensor reading
 * @param calibration Calibration data
 * @return Calibrated sensor value
 */
float mcp_device_apply_sensor_calibration(float raw_value, const mcp_sensor_calibration_t *calibration);

/**
 * @brief Check if sensor calibration is expired
 * @param calibration Calibration data
 * @param interval_seconds Calibration interval in seconds
 * @return true if calibration is expired, false otherwise
 */
bool mcp_device_is_calibration_expired(const mcp_sensor_calibration_t *calibration, uint32_t interval_seconds);

/**
 * @brief Create default sensor calibration
 * @return Default calibration structure
 */
mcp_sensor_calibration_t mcp_device_create_default_calibration(void);

/**
 * @brief Validate device info structure
 * @param info Device info to validate
 * @return ESP_OK if valid, error code if invalid
 */
esp_err_t mcp_device_validate_info(const mcp_device_info_t *info);

/**
 * @brief Get system device information
 * @param info Output structure for device info
 * @param device_id Device ID string
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mcp_device_get_system_info(mcp_device_info_t *info, const char *device_id);

#ifdef __cplusplus
}
#endif

#endif /* MCP_DEVICE_H */ 