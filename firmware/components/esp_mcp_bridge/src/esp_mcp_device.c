/**
 * @file esp_mcp_device.c
 * @brief Device abstraction layer implementation for MCP Bridge
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "mcp_device.h"

static const char *TAG = "MCP_DEVICE";

/**
 * @brief Generate a unique device ID based on MAC address
 */
esp_err_t mcp_device_generate_id(char *device_id, size_t max_len, const char *prefix) {
    if (!device_id || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t mac[6];
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MAC address");
        return ret;
    }
    
    if (prefix) {
        snprintf(device_id, max_len, "%s_%02x%02x%02x", 
                prefix, mac[3], mac[4], mac[5]);
    } else {
        snprintf(device_id, max_len, "esp32_%02x%02x%02x", 
                mac[3], mac[4], mac[5]);
    }
    
    ESP_LOGI(TAG, "Generated device ID: %s", device_id);
    return ESP_OK;
}

/**
 * @brief Validate sensor metadata
 */
esp_err_t mcp_device_validate_sensor_metadata(const mcp_sensor_metadata_t *metadata) {
    if (!metadata) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (metadata->min_range >= metadata->max_range) {
        ESP_LOGE(TAG, "Invalid sensor range: min >= max");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (metadata->update_interval_ms == 0) {
        ESP_LOGW(TAG, "Sensor update interval is 0 - sensor will not auto-publish");
    }
    
    return ESP_OK;
}

/**
 * @brief Validate actuator metadata  
 */
esp_err_t mcp_device_validate_actuator_metadata(const mcp_actuator_metadata_t *metadata) {
    if (!metadata) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!metadata->value_type) {
        ESP_LOGE(TAG, "Actuator metadata missing value_type");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!metadata->supported_actions || !metadata->supported_actions[0]) {
        ESP_LOGE(TAG, "Actuator metadata missing supported_actions");
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

/**
 * @brief Apply sensor calibration to raw value
 */
float mcp_device_apply_sensor_calibration(float raw_value, const mcp_sensor_calibration_t *calibration) {
    if (!calibration || !calibration->is_valid) {
        return raw_value;
    }
    
    return (raw_value * calibration->scale) + calibration->offset;
}

/**
 * @brief Check if sensor calibration is expired
 */
bool mcp_device_is_calibration_expired(const mcp_sensor_calibration_t *calibration, uint32_t interval_seconds) {
    if (!calibration || !calibration->is_valid || interval_seconds == 0) {
        return false;
    }
    
    uint32_t current_time = esp_log_timestamp() / 1000; // Convert to seconds
    return (current_time - calibration->last_calibration) > interval_seconds;
}

/**
 * @brief Create default sensor calibration
 */
mcp_sensor_calibration_t mcp_device_create_default_calibration(void) {
    mcp_sensor_calibration_t calibration = {
        .offset = 0.0,
        .scale = 1.0,
        .last_calibration = esp_log_timestamp() / 1000,
        .is_valid = true
    };
    return calibration;
}

/**
 * @brief Validate device info structure
 */
esp_err_t mcp_device_validate_info(const mcp_device_info_t *info) {
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!info->device_id || strlen(info->device_id) == 0) {
        ESP_LOGE(TAG, "Device info missing device_id");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!info->firmware_version || strlen(info->firmware_version) == 0) {
        ESP_LOGW(TAG, "Device info missing firmware_version");
    }
    
    return ESP_OK;
}

/**
 * @brief Get system device information
 */
esp_err_t mcp_device_get_system_info(mcp_device_info_t *info, const char *device_id) {
    if (!info || !device_id) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Fill in system information
    info->device_id = device_id;
    info->firmware_version = "1.0.0";
    info->hardware_version = "ESP32";
    info->manufacturer = "Espressif";
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    switch(chip_info.model) {
        case CHIP_ESP32:
            info->model = "ESP32";
            break;
        case CHIP_ESP32S2:
            info->model = "ESP32-S2";
            break;
        case CHIP_ESP32S3:
            info->model = "ESP32-S3";
            break;
        case CHIP_ESP32C3:
            info->model = "ESP32-C3";
            break;
        default:
            info->model = "ESP32-Unknown";
            break;
    }
    
    info->serial_number = device_id; // Use device ID as serial for now
    info->max_sensors = 16;
    info->max_actuators = 16;
    info->supports_ota_update = true;
    info->supports_remote_config = true;
    
    return ESP_OK;
}
