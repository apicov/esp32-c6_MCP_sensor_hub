#include <cstdint>
#include <cstdio>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ESP-IDF
#include "esp_system.h"
#include <esp_log.h>

// components
#include "bmp280.h"
#include "scd30.h"
#include "esp_mcp_bridge.h"
#include <string.h>

#define LCD_ADDR 0x27
#define SCD30_ADDR 0x61

#define SDA_PIN  (gpio_num_t) 6
#define SCL_PIN  (gpio_num_t) 7
#define I2C_PORT (gpio_num_t) 0

static const char *TAG = "ESP32_C6_SENSOR_HUB"; // Tag for logging

// Global SCD30 device handle
static i2c_dev_t scd30_dev;
static bool scd30_initialized = false;

// Shared measurement cache to avoid multiple reads
static struct {
    float co2;
    float temperature;
    float humidity;
    uint32_t timestamp;
    bool valid;
    SemaphoreHandle_t mutex;
} scd30_cache;

// Global BME280/BMP280 device handle
static bmp280_t bmp_dev;
static bool bmp_initialized = false;
static bool is_bme280 = false; // true if BME280 (with humidity), false if BMP280

// Shared measurement cache for BMP280/BME280
static struct {
    float pressure;
    float temperature;
    float humidity; // only valid if is_bme280 is true
    uint32_t timestamp;
    bool valid;
    SemaphoreHandle_t mutex;
} bmp_cache;

// Function declarations
static esp_err_t scd30_read_cached_measurement(void);
static esp_err_t bmp_read_cached_measurement(void);
static esp_err_t bmp_sensor_read_pressure(const char *sensor_id, float *value, void *user_data);
static esp_err_t bmp_sensor_read_temperature_bmp(const char *sensor_id, float *value, void *user_data);
static esp_err_t bme_sensor_read_humidity(const char *sensor_id, float *value, void *user_data);
static esp_err_t init_bmp_sensor(void);
static esp_err_t scd30_sensor_read_co2(const char *sensor_id, float *value, void *user_data);
static esp_err_t scd30_sensor_read_temperature(const char *sensor_id, float *value, void *user_data);
static esp_err_t scd30_sensor_read_humidity(const char *sensor_id, float *value, void *user_data);
static void mcp_event_handler(const mcp_event_t *event, void *user_data);
static esp_err_t init_scd30_sensor(void);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-C6 MCP Sensor Hub Application");
    
    // Initialize I2C bus
    ESP_ERROR_CHECK(i2cdev_init());
    
    // Initialize SCD30 sensor
    ESP_ERROR_CHECK(init_scd30_sensor());
    
    // Initialize BMP280/BME280 sensor
    ESP_ERROR_CHECK(init_bmp_sensor());
    
    // Configure MCP Bridge with project-specific settings
    mcp_bridge_config_t bridge_config = {
        .wifi_ssid = CONFIG_ESP32_C6_WIFI_SSID,
        .wifi_password = CONFIG_ESP32_C6_WIFI_PASSWORD,
        .mqtt_broker_uri = CONFIG_ESP32_C6_MQTT_BROKER_URI,
        .mqtt_username = NULL,
        .mqtt_password = NULL,
        .device_id = CONFIG_ESP32_C6_DEVICE_ID,
        .sensor_publish_interval_ms = CONFIG_ESP32_C6_SENSOR_INTERVAL,
        .command_timeout_ms = 5000,
        .enable_watchdog = true,
        .enable_device_auth = false,
        .log_level = 3, // INFO level
        .qos_config = {
            .sensor_qos = 0,
            .actuator_qos = 1,
            .status_qos = 1,
            .error_qos = 1
        },
        .tls_config = {
            .enable_tls = false,
            .ca_cert_pem = NULL,
            .client_cert_pem = NULL,
            .client_key_pem = NULL,
            .skip_cert_verification = false,
            .alpn_protocols = {NULL}
        }
    };
    
    // Initialize MCP Bridge with custom configuration
    ESP_ERROR_CHECK(mcp_bridge_init(&bridge_config));
    
    // Register event handler
    ESP_ERROR_CHECK(mcp_bridge_register_event_handler(mcp_event_handler, NULL));
    
    // Register SCD30 sensors with metadata
    mcp_sensor_metadata_t co2_metadata = {
        .min_range = 400.0f,
        .max_range = 10000.0f,
        .accuracy = 30.0f,  // ±30 ppm
        .update_interval_ms = 2000,
        .description = "SCD30 CO2 concentration sensor",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    
    mcp_sensor_metadata_t temp_metadata = {
        .min_range = -40.0f,
        .max_range = 70.0f,
        .accuracy = 0.4f,   // ±0.4°C
        .update_interval_ms = 2000,
        .description = "SCD30 temperature sensor",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    
    mcp_sensor_metadata_t humidity_metadata = {
        .min_range = 0.0f,
        .max_range = 100.0f,
        .accuracy = 3.0f,   // ±3% RH
        .update_interval_ms = 2000,
        .description = "SCD30 relative humidity sensor",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    
    // Register SCD30 sensors
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("scd30_co2", "co2", "ppm", &co2_metadata, scd30_sensor_read_co2, NULL));
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("scd30_temperature", "temperature", "°C", &temp_metadata, scd30_sensor_read_temperature, NULL));
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("scd30_humidity", "humidity", "%RH", &humidity_metadata, scd30_sensor_read_humidity, NULL));
    
    // Register BMP280/BME280 sensors
    mcp_sensor_metadata_t pressure_metadata = {
        .min_range = 30000.0f,   // 300 hPa
        .max_range = 110000.0f,  // 1100 hPa
        .accuracy = 12.0f,       // ±12 Pa (±0.12 hPa)
        .update_interval_ms = 2000,
        .description = "BMP280/BME280 atmospheric pressure sensor",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    
    mcp_sensor_metadata_t temp_bmp_metadata = {
        .min_range = -40.0f,
        .max_range = 85.0f,
        .accuracy = 1.0f,        // ±1°C
        .update_interval_ms = 2000,
        .description = "BMP280/BME280 temperature sensor",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("bmp_pressure", "pressure", "Pa", &pressure_metadata, bmp_sensor_read_pressure, NULL));
    ESP_ERROR_CHECK(mcp_bridge_register_sensor("bmp_temperature", "temperature_bmp", "°C", &temp_bmp_metadata, bmp_sensor_read_temperature_bmp, NULL));
    
    // Register BME280 humidity sensor if available
    if (is_bme280) {
        mcp_sensor_metadata_t humidity_bme_metadata = {
            .min_range = 0.0f,
            .max_range = 100.0f,
            .accuracy = 3.0f,    // ±3% RH
            .update_interval_ms = 2000,
            .description = "BME280 relative humidity sensor",
            .calibration_required = false,
            .calibration_interval_s = 0
        };
        ESP_ERROR_CHECK(mcp_bridge_register_sensor("bme_humidity", "humidity_bme", "%RH", &humidity_bme_metadata, bme_sensor_read_humidity, NULL));
    }
    
    // Start the MCP Bridge
    ESP_ERROR_CHECK(mcp_bridge_start());
    
    ESP_LOGI(TAG, "MCP Sensor Hub started successfully with all sensors");
}

// Initialize SCD30 sensor
static esp_err_t init_scd30_sensor(void)
{
    ESP_LOGI(TAG, "Initializing SCD30 sensor");
    
    // Initialize SCD30 device descriptor
    ESP_ERROR_CHECK(scd30_init_desc(&scd30_dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7));
    
    // Read firmware version
    uint16_t version, major_ver, minor_ver;
    ESP_ERROR_CHECK(scd30_read_firmware_version(&scd30_dev, &version));
    
    major_ver = (version >> 8) & 0xf;
    minor_ver = version & 0xf;
    ESP_LOGI(TAG, "SCD30 Firmware Version: %d.%d", major_ver, minor_ver);
    
    // Start continuous measurement
    ESP_LOGI(TAG, "Starting SCD30 continuous measurement");
    ESP_ERROR_CHECK(scd30_trigger_continuous_measurement(&scd30_dev, 0));
    
    // Initialize measurement cache
    scd30_cache.mutex = xSemaphoreCreateMutex();
    if (!scd30_cache.mutex) {
        ESP_LOGE(TAG, "Failed to create SCD30 cache mutex");
        return ESP_ERR_NO_MEM;
    }
    scd30_cache.valid = false;
    scd30_cache.timestamp = 0;
    
    // Wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    scd30_initialized = true;
    ESP_LOGI(TAG, "SCD30 sensor initialized successfully");
    
    return ESP_OK;
}

// Initialize BMP280/BME280 sensor
static esp_err_t init_bmp_sensor(void)
{
    ESP_LOGI(TAG, "Initializing BMP280/BME280 sensor");
    
    // Initialize device descriptor
    memset(&bmp_dev, 0, sizeof(bmp280_t));
    ESP_ERROR_CHECK(bmp280_init_desc(&bmp_dev, BMP280_I2C_ADDRESS_0, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7));
    
    // Initialize with default parameters
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    ESP_ERROR_CHECK(bmp280_init(&bmp_dev, &params));
    
    // Detect if it's BME280 (with humidity) or BMP280 (pressure + temperature only)
    is_bme280 = (bmp_dev.id == BME280_CHIP_ID);
    ESP_LOGI(TAG, "Detected sensor: %s", is_bme280 ? "BME280" : "BMP280");
    
    // Initialize measurement cache
    bmp_cache.mutex = xSemaphoreCreateMutex();
    if (!bmp_cache.mutex) {
        ESP_LOGE(TAG, "Failed to create BMP cache mutex");
        return ESP_ERR_NO_MEM;
    }
    bmp_cache.valid = false;
    bmp_cache.timestamp = 0;
    
    // Wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    bmp_initialized = true;
    ESP_LOGI(TAG, "BMP280/BME280 sensor initialized successfully");
    
    return ESP_OK;
}

// Read BMP280/BME280 measurement and update cache
static esp_err_t bmp_read_cached_measurement(void)
{
    if (!bmp_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if we have a recent valid measurement (within 1 second)
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    xSemaphoreTake(bmp_cache.mutex, portMAX_DELAY);
    
    if (bmp_cache.valid && (current_time - bmp_cache.timestamp) < 1000) {
        // Use cached data if it's less than 1 second old
        xSemaphoreGive(bmp_cache.mutex);
        return ESP_OK;
    }
    
    xSemaphoreGive(bmp_cache.mutex);
    
    // Read new measurement
    float pressure, temperature, humidity = 0.0f;
    esp_err_t ret = bmp280_read_float(&bmp_dev, &temperature, &pressure, &humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280/BME280 measurement: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Update cache with all values
    xSemaphoreTake(bmp_cache.mutex, portMAX_DELAY);
    bmp_cache.pressure = pressure;
    bmp_cache.temperature = temperature;
    if (is_bme280) {
        bmp_cache.humidity = humidity;
    }
    bmp_cache.timestamp = current_time;
    bmp_cache.valid = true;
    xSemaphoreGive(bmp_cache.mutex);
    
    ESP_LOGD(TAG, "BMP280/BME280 measurement updated - Pressure: %.2f Pa, Temp: %.2f °C%s", 
             pressure, temperature, is_bme280 ? ", Humidity: %.2f %%RH" : "");
    
    return ESP_OK;
}

// BMP280/BME280 pressure sensor read callback
static esp_err_t bmp_sensor_read_pressure(const char *sensor_id, float *value, void *user_data)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = bmp_read_cached_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    
    xSemaphoreTake(bmp_cache.mutex, portMAX_DELAY);
    *value = bmp_cache.pressure;
    xSemaphoreGive(bmp_cache.mutex);
    
    return ESP_OK;
}

// BMP280/BME280 temperature sensor read callback
static esp_err_t bmp_sensor_read_temperature_bmp(const char *sensor_id, float *value, void *user_data)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = bmp_read_cached_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    
    xSemaphoreTake(bmp_cache.mutex, portMAX_DELAY);
    *value = bmp_cache.temperature;
    xSemaphoreGive(bmp_cache.mutex);
    
    return ESP_OK;
}

// BME280 humidity sensor read callback
static esp_err_t bme_sensor_read_humidity(const char *sensor_id, float *value, void *user_data)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!is_bme280) {
        ESP_LOGE(TAG, "Humidity not available on BMP280");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    esp_err_t ret = bmp_read_cached_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    
    xSemaphoreTake(bmp_cache.mutex, portMAX_DELAY);
    *value = bmp_cache.humidity;
    xSemaphoreGive(bmp_cache.mutex);
    
    return ESP_OK;
}

// Read SCD30 measurement and update cache
static esp_err_t scd30_read_cached_measurement(void)
{
    if (!scd30_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if we have a recent valid measurement (within 1 second)
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    xSemaphoreTake(scd30_cache.mutex, portMAX_DELAY);
    
    if (scd30_cache.valid && (current_time - scd30_cache.timestamp) < 1000) {
        // Use cached data if it's less than 1 second old
        xSemaphoreGive(scd30_cache.mutex);
        return ESP_OK;
    }
    
    xSemaphoreGive(scd30_cache.mutex);
    
    // Check if new data is ready
    bool data_ready;
    esp_err_t ret = scd30_get_data_ready_status(&scd30_dev, &data_ready);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to check SCD30 data ready status");
        return ret;
    }
    
    if (!data_ready) {
        // Return cached data if available, otherwise error
        xSemaphoreTake(scd30_cache.mutex, portMAX_DELAY);
        bool has_cached = scd30_cache.valid;
        xSemaphoreGive(scd30_cache.mutex);
        
        if (has_cached) {
            ESP_LOGD(TAG, "SCD30 data not ready, using cached values");
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "SCD30 data not ready and no cached data available");
            return ESP_ERR_NOT_FINISHED;
        }
    }
    
    // Read new measurement (all three values at once)
    float co2, temperature, humidity;
    ret = scd30_read_measurement(&scd30_dev, &co2, &temperature, &humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SCD30 measurement: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Validate CO2 reading
    if (co2 <= 0) {
        ESP_LOGW(TAG, "Invalid CO2 reading: %.0f ppm", co2);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Update cache with all three values
    xSemaphoreTake(scd30_cache.mutex, portMAX_DELAY);
    scd30_cache.co2 = co2;
    scd30_cache.temperature = temperature;
    scd30_cache.humidity = humidity;
    scd30_cache.timestamp = current_time;
    scd30_cache.valid = true;
    xSemaphoreGive(scd30_cache.mutex);
    
    ESP_LOGD(TAG, "SCD30 measurement updated - CO2: %.0f ppm, Temp: %.2f °C, Humidity: %.2f %%RH", 
             co2, temperature, humidity);
    
    return ESP_OK;
}

// SCD30 CO2 sensor read callback
static esp_err_t scd30_sensor_read_co2(const char *sensor_id, float *value, void *user_data)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = scd30_read_cached_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    
    xSemaphoreTake(scd30_cache.mutex, portMAX_DELAY);
    *value = scd30_cache.co2;
    xSemaphoreGive(scd30_cache.mutex);
    
    return ESP_OK;
}

// SCD30 temperature sensor read callback
static esp_err_t scd30_sensor_read_temperature(const char *sensor_id, float *value, void *user_data)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = scd30_read_cached_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    
    xSemaphoreTake(scd30_cache.mutex, portMAX_DELAY);
    *value = scd30_cache.temperature;
    xSemaphoreGive(scd30_cache.mutex);
    
    return ESP_OK;
}

// SCD30 humidity sensor read callback
static esp_err_t scd30_sensor_read_humidity(const char *sensor_id, float *value, void *user_data)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = scd30_read_cached_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    
    xSemaphoreTake(scd30_cache.mutex, portMAX_DELAY);
    *value = scd30_cache.humidity;
    xSemaphoreGive(scd30_cache.mutex);
    
    return ESP_OK;
}

// MCP Bridge event handler
static void mcp_event_handler(const mcp_event_t *event, void *user_data)
{
    switch (event->type) {
        case MCP_EVENT_WIFI_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected");
            break;
            
        case MCP_EVENT_WIFI_DISCONNECTED:
            ESP_LOGW(TAG, "WiFi disconnected");
            break;
            
        case MCP_EVENT_MQTT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            break;
            
        case MCP_EVENT_MQTT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
            
        case MCP_EVENT_SENSOR_READ_ERROR:
            ESP_LOGE(TAG, "Sensor read error: %s", event->data.sensor_error.error_message);
            break;
            
        case MCP_EVENT_ERROR:
            ESP_LOGE(TAG, "MCP Bridge error: %s", event->data.error.message);
            break;
            
        default:
            ESP_LOGD(TAG, "MCP Bridge event: %d", event->type);
            break;
    }
}