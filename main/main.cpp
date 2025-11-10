#include <cstdint>
#include <cstdio>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ESP-IDF
#include "esp_system.h"
#include <esp_log.h>
#include "esp_sntp.h"

// components
#include "bmp280.h"
#include "scd30.h"
#include "hm3301.h"
#include "ds3231.h"
#include "esp_mcp_bridge.h"
#include <string.h>
#include <sys/time.h>

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

// Global DS3231 RTC device handle
static i2c_dev_t ds3231_dev;
static bool ds3231_initialized = false;

// Global HM3301 device handle
static i2c_dev_t hm3301_dev;
static bool hm3301_initialized = false;

// Shared measurement cache for HM3301
static struct {
    hm3301_data_t data;
    uint32_t timestamp;
    bool valid;
    SemaphoreHandle_t mutex;
} hm3301_cache;

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
static esp_err_t hm3301_read_cached_measurement(void);
static esp_err_t scd30_sensor_read_multi(const char *sensor_id, mcp_sensor_multi_value_t *values, void *user_data);
static esp_err_t bmp_sensor_read_multi(const char *sensor_id, mcp_sensor_multi_value_t *values, void *user_data);
static esp_err_t hm3301_sensor_read_multi(const char *sensor_id, mcp_sensor_multi_value_t *values, void *user_data);
static void mcp_event_handler(const mcp_event_t *event, void *user_data);
static esp_err_t init_scd30_sensor(void);
static esp_err_t init_bmp_sensor(void);
static esp_err_t init_hm3301_sensor(void);
static esp_err_t init_ds3231_rtc(void);
static time_t get_rtc_timestamp(void);
static void sync_time_from_ntp(void);
static void time_sync_notification_cb(struct timeval *tv);

void update_sensors_task(void *p);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-C6 MCP Sensor Hub Application");
    
    // Initialize I2C bus
    esp_err_t ret = i2cdev_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Cannot continue without I2C - aborting");
        return;
    }

    // Initialize DS3231 RTC (optional - continue if it fails)
    ret = init_ds3231_rtc();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DS3231 RTC initialization failed (%s) - continuing without RTC", esp_err_to_name(ret));
        ds3231_initialized = false; // Ensure flag is explicitly set to false
    } else {
        ESP_LOGI(TAG, "DS3231 RTC initialization successful");
    }

    // Initialize SCD30 sensor BEFORE HM3301 (optional - continue if it fails)
    ret = init_scd30_sensor();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SCD30 sensor initialization failed (%s) - continuing without SCD30", esp_err_to_name(ret));
        scd30_initialized = false; // Ensure flag is explicitly set to false
    } else {
        ESP_LOGI(TAG, "SCD30 sensor initialization successful");
    }

    // Initialize BMP280/BME280 sensor (optional - continue if it fails)
    ret = init_bmp_sensor();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BMP280/BME280 sensor initialization failed (%s) - continuing without BMP280/BME280", esp_err_to_name(ret));
        bmp_initialized = false; // Ensure flag is explicitly set to false
    } else {
        ESP_LOGI(TAG, "BMP280/BME280 sensor initialization successful");
    }

    // Initialize HM3301 sensor LAST (optional - continue if it fails)
    // Note: HM3301 is initialized last because if it's faulty, it can interfere with I2C bus
    ESP_LOGI(TAG, "Initializing HM3301 sensor (attempting last to avoid I2C bus issues)...");
    ret = init_hm3301_sensor();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "HM3301 sensor initialization failed (%s) - continuing without HM3301", esp_err_to_name(ret));
        ESP_LOGW(TAG, "If HM3301 is connected but failing, check wiring and power supply");
        hm3301_initialized = false; // Ensure flag is explicitly set to false
    } else {
        ESP_LOGI(TAG, "HM3301 sensor initialization successful");
    }

    // Configure MCP Bridge with project-specific settings
    mcp_bridge_config_t bridge_config = {
        .wifi_ssid = CONFIG_ESP32_C6_WIFI_SSID,
        .wifi_password = CONFIG_ESP32_C6_WIFI_PASSWORD,
        .mqtt_broker_uri = CONFIG_ESP32_C6_MQTT_BROKER_URI,
        .mqtt_username = NULL,
        .mqtt_password = NULL,
        .device_id = CONFIG_ESP32_C6_DEVICE_ID,
        .device_name = "esp32_home",
        .device_location = "home",
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

    // Register SCD30 as multi-value sensor (only if initialized)
    if (scd30_initialized) {
        mcp_sensor_field_metadata_t scd30_fields[] = {
            {.name = "co2", .unit = "ppm", .metric_type = "co2", .min_range = 400.0f, .max_range = 10000.0f, .accuracy = 30.0f},
            {.name = "temperature", .unit = "°C", .metric_type = "temperature", .min_range = -40.0f, .max_range = 70.0f, .accuracy = 0.4f},
            {.name = "humidity", .unit = "%RH", .metric_type = "humidity", .min_range = 0.0f, .max_range = 100.0f, .accuracy = 3.0f}
        };
        mcp_sensor_metadata_t scd30_metadata = {
            .min_range = 0.0f,
            .max_range = 0.0f,
            .accuracy = 0.0f,
            .update_interval_ms = 10000,
            .description = "SCD30 environmental sensor (CO2, temperature, humidity)",
            .location = "home",
            .calibration_required = false,
            .calibration_interval_s = 0
        };
        ret = mcp_bridge_register_multi_sensor("scd30", scd30_fields, 3, &scd30_metadata, scd30_sensor_read_multi, NULL);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "SCD30 sensor registered successfully");
        } else {
            ESP_LOGE(TAG, "Failed to register SCD30 sensor: %s", esp_err_to_name(ret));
        }
    }

    // Register BMP280/BME280 as multi-value sensor (only if initialized)
    if (bmp_initialized) {
        mcp_sensor_field_metadata_t bmp_fields[] = {
            {.name = "pressure", .unit = "Pa", .metric_type = "pressure", .min_range = 30000.0f, .max_range = 110000.0f, .accuracy = 12.0f},
            {.name = "temperature", .unit = "°C", .metric_type = "temperature", .min_range = -40.0f, .max_range = 85.0f, .accuracy = 1.0f},
            {.name = "humidity", .unit = "%RH", .metric_type = "humidity", .min_range = 0.0f, .max_range = 100.0f, .accuracy = 3.0f}
        };
        size_t bmp_field_count = is_bme280 ? 3 : 2;  // BME280 has humidity, BMP280 doesn't
        mcp_sensor_metadata_t bmp_metadata = {
            .min_range = 0.0f,
            .max_range = 0.0f,
            .accuracy = 0.0f,
            .update_interval_ms = 2000,
            .description = is_bme280 ? "BME280 environmental sensor (pressure, temperature, humidity)" : "BMP280 sensor (pressure, temperature)",
            .location = "home",
            .calibration_required = false,
            .calibration_interval_s = 0
        };
        ret = mcp_bridge_register_multi_sensor("bmp280", bmp_fields, bmp_field_count, &bmp_metadata, bmp_sensor_read_multi, NULL);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "BMP280/BME280 sensor registered successfully");
        } else {
            ESP_LOGE(TAG, "Failed to register BMP280/BME280 sensor: %s", esp_err_to_name(ret));
        }
    }

    // Register HM3301 as multi-value sensor (only if initialized)
    if (hm3301_initialized) {
        // NOTE: Only registering PM concentration fields for HM-3301 (weighing mode only)
        // If using HM-3302 or HM-3602, uncomment particle count fields below
        mcp_sensor_field_metadata_t hm3301_fields[] = {
            {.name = "pm1.0_std", .unit = "ug/m3", .metric_type = "pm1.0", .min_range = 0.0f, .max_range = 1000.0f, .accuracy = 10.0f},
            {.name = "pm2.5_std", .unit = "ug/m3", .metric_type = "pm2.5", .min_range = 0.0f, .max_range = 1000.0f, .accuracy = 10.0f},
            {.name = "pm10_std", .unit = "ug/m3", .metric_type = "pm10", .min_range = 0.0f, .max_range = 1000.0f, .accuracy = 10.0f},
            {.name = "pm1.0_atm", .unit = "ug/m3", .metric_type = "pm1.0_atm", .min_range = 0.0f, .max_range = 1000.0f, .accuracy = 10.0f},
            {.name = "pm2.5_atm", .unit = "ug/m3", .metric_type = "pm2.5_atm", .min_range = 0.0f, .max_range = 1000.0f, .accuracy = 10.0f},
            {.name = "pm10_atm", .unit = "ug/m3", .metric_type = "pm10_atm", .min_range = 0.0f, .max_range = 1000.0f, .accuracy = 10.0f}
            // Uncomment for HM-3302/HM-3602 (counting + weighing mode):
            // {.name = "particles_0.3um", .unit = "count/0.1L", .metric_type = "particle_count", .min_range = 0.0f, .max_range = 65535.0f, .accuracy = 1.0f},
            // {.name = "particles_0.5um", .unit = "count/0.1L", .metric_type = "particle_count", .min_range = 0.0f, .max_range = 65535.0f, .accuracy = 1.0f},
            // {.name = "particles_1.0um", .unit = "count/0.1L", .metric_type = "particle_count", .min_range = 0.0f, .max_range = 65535.0f, .accuracy = 1.0f},
            // {.name = "particles_2.5um", .unit = "count/0.1L", .metric_type = "particle_count", .min_range = 0.0f, .max_range = 65535.0f, .accuracy = 1.0f},
            // {.name = "particles_5.0um", .unit = "count/0.1L", .metric_type = "particle_count", .min_range = 0.0f, .max_range = 65535.0f, .accuracy = 1.0f},
            // {.name = "particles_10um", .unit = "count/0.1L", .metric_type = "particle_count", .min_range = 0.0f, .max_range = 65535.0f, .accuracy = 1.0f}
        };
        mcp_sensor_metadata_t hm3301_metadata = {
            .min_range = 0.0f,
            .max_range = 0.0f,
            .accuracy = 0.0f,
            .update_interval_ms = 2000,
            .description = "HM3301 Particulate Matter Sensor (PM1.0, PM2.5, PM10)",
            .location = "home",
            .calibration_required = false,
            .calibration_interval_s = 0
        };
        ret = mcp_bridge_register_multi_sensor("hm3301", hm3301_fields, 6, &hm3301_metadata, hm3301_sensor_read_multi, NULL);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "HM3301 sensor registered successfully");
        } else {
            ESP_LOGE(TAG, "Failed to register HM3301 sensor: %s", esp_err_to_name(ret));
        }
    }

    // Start the MCP Bridge
    ESP_ERROR_CHECK(mcp_bridge_start());

    // Create task to periodically update sensor measurements
    ESP_LOGI(TAG, "Creating sensor update task");
    // check if task creation is successful
    if ( xTaskCreate(update_sensors_task, "update_sensors_task", 4096, NULL, 5, NULL)
            != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor update task");
        return;
    }

    // Report which sensors were successfully initialized
    int sensor_count = 0;
    if (scd30_initialized) sensor_count++;
    if (bmp_initialized) sensor_count++;
    if (hm3301_initialized) sensor_count++;

    ESP_LOGI(TAG, "MCP Sensor Hub started successfully with %d sensor(s):", sensor_count);
    if (scd30_initialized) ESP_LOGI(TAG, "  - SCD30 (CO2, temperature, humidity)");
    if (bmp_initialized) ESP_LOGI(TAG, "  - %s (pressure, temperature%s)",
                                  is_bme280 ? "BME280" : "BMP280",
                                  is_bme280 ? ", humidity" : "");
    if (hm3301_initialized) ESP_LOGI(TAG, "  - HM3301 (particulate matter)");
    if (ds3231_initialized) ESP_LOGI(TAG, "  - DS3231 (RTC)");

    if (sensor_count == 0) {
        ESP_LOGW(TAG, "Warning: No sensors initialized - MCP Bridge running without sensors");
    }
}



void update_sensors_task(void *p)
{
    ESP_LOGI(TAG, "Starting sensor update task");

    const TickType_t interval = pdMS_TO_TICKS(30000);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        ESP_LOGI(TAG, "Updating sensor measurements");
        // Read and update BMP280/BME280 measurement
        if (bmp_initialized) {
            esp_err_t ret = bmp_read_cached_measurement();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read BMP280/BME280 measurement: %s", esp_err_to_name(ret));
            }
        }

        // Read and update SCD30 measurement
        if (scd30_initialized) {
            esp_err_t ret = scd30_read_cached_measurement();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read SCD30 measurement: %s", esp_err_to_name(ret));
            }
        }

        // Read and update HM3301 measurement
        if (hm3301_initialized) {
            esp_err_t ret = hm3301_read_cached_measurement();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read HM3301 measurement: %s", esp_err_to_name(ret));
            }
        }

        // Sleep for the configured sensor update interval
        //vTaskDelay(pdMS_TO_TICKS(CONFIG_ESP32_C6_SENSOR_INTERVAL));

        // Delay until the next cycle, keeping the interval constant
        vTaskDelayUntil(&last_wake_time, interval);
    }
}



// Initialize SCD30 sensor
static esp_err_t init_scd30_sensor(void)
{
    ESP_LOGI(TAG, "Initializing SCD30 sensor");

    // Initialize SCD30 device descriptor
    esp_err_t ret = scd30_init_desc(&scd30_dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SCD30 device descriptor: %s", esp_err_to_name(ret));
        return ret;
    }

    // wait for the sensor to be ready
    vTaskDelay(pdMS_TO_TICKS(500));

    // Read firmware version
    uint16_t version, major_ver, minor_ver;
    ret = scd30_read_firmware_version(&scd30_dev, &version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SCD30 firmware version: %s", esp_err_to_name(ret));
        scd30_free_desc(&scd30_dev);
        return ret;
    }

    major_ver = (version >> 8) & 0xf;
    minor_ver = version & 0xf;
    ESP_LOGI(TAG, "SCD30 Firmware Version: %d.%d", major_ver, minor_ver);

    // Set measurement interval to 10 seconds (must be done before starting measurement)
    ESP_LOGI(TAG, "Setting SCD30 measurement interval to 10 seconds");
    ret = scd30_set_measurement_interval(&scd30_dev, 10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SCD30 measurement interval: %s", esp_err_to_name(ret));
        scd30_free_desc(&scd30_dev);
        return ret;
    }

    // Small delay after setting interval (stored in non-volatile memory)
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify the interval was set
    uint16_t interval;
    ret = scd30_get_measurement_interval(&scd30_dev, &interval);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SCD30 measurement interval: %s", esp_err_to_name(ret));
        scd30_free_desc(&scd30_dev);
        return ret;
    }
    ESP_LOGI(TAG, "SCD30 measurement interval confirmed: %d seconds", interval);

    // Start continuous measurement
    ESP_LOGI(TAG, "Starting SCD30 continuous measurement");
    ret = scd30_trigger_continuous_measurement(&scd30_dev, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start SCD30 continuous measurement: %s", esp_err_to_name(ret));
        scd30_free_desc(&scd30_dev);
        return ret;
    }

    // Small delay after starting measurement
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize measurement cache
    scd30_cache.mutex = xSemaphoreCreateMutex();
    if (!scd30_cache.mutex) {
        ESP_LOGE(TAG, "Failed to create SCD30 cache mutex");
        return ESP_ERR_NO_MEM;
    }
    scd30_cache.valid = false;
    scd30_cache.timestamp = 0;
    
    // Wait for first measurement to be ready
    // With 2-second interval, first data should be ready within ~2 seconds
    ESP_LOGI(TAG, "Waiting for first SCD30 measurement to be ready...");
    bool data_ready = false;
    int retry_count = 0;
    const int max_retries = 20; // Try for up to 10 seconds (20 * 500ms)

    while (!data_ready && retry_count < max_retries) {
        vTaskDelay(pdMS_TO_TICKS(500));
        ret = scd30_get_data_ready_status(&scd30_dev, &data_ready);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to check SCD30 data ready status: %s", esp_err_to_name(ret));
            // Clean up before returning
            vSemaphoreDelete(scd30_cache.mutex);
            scd30_free_desc(&scd30_dev);
            return ret;
        }
        retry_count++;
        if (!data_ready) {
            ESP_LOGD(TAG, "SCD30 data not ready yet (attempt %d/%d)", retry_count, max_retries);
        }
    }

    if (data_ready) {
        ESP_LOGI(TAG, "SCD30 first measurement ready after %.1f seconds", (retry_count * 0.5));
    } else {
        ESP_LOGW(TAG, "SCD30 data still not ready after %d seconds - continuing anyway", max_retries / 2);
    }

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
    esp_err_t ret = bmp280_init_desc(&bmp_dev, BMP280_I2C_ADDRESS_0, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMP280/BME280 device descriptor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Give sensor time to stabilize before communicating
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize with default parameters
    ESP_LOGI(TAG, "Attempting to communicate with BMP280/BME280...");
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    ret = bmp280_init(&bmp_dev, &params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMP280/BME280: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "BMP280/BME280 may not be connected or may be faulty");
        bmp280_free_desc(&bmp_dev);
        return ret;
    }

    // Detect if it's BME280 (with humidity) or BMP280 (pressure + temperature only)
    is_bme280 = (bmp_dev.id == BME280_CHIP_ID);
    ESP_LOGI(TAG, "Detected sensor: %s", is_bme280 ? "BME280" : "BMP280");
    
    // Initialize measurement cache
    bmp_cache.mutex = xSemaphoreCreateMutex();
    if (!bmp_cache.mutex) {
        ESP_LOGE(TAG, "Failed to create BMP cache mutex");
        bmp280_free_desc(&bmp_dev);
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

    // Take mutex for entire operation to prevent concurrent I2C access
    xSemaphoreTake(bmp_cache.mutex, portMAX_DELAY);

    // Check if we have a recent valid measurement (within 1 second)
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (bmp_cache.valid && (current_time - bmp_cache.timestamp) < 1000) {
        // Use cached data if it's less than 1 second old
        xSemaphoreGive(bmp_cache.mutex);
        return ESP_OK;
    }

    // Read new measurement (while holding mutex to prevent concurrent I2C access)
    float pressure, temperature, humidity = 0.0f;
    esp_err_t ret = bmp280_read_float(&bmp_dev, &temperature, &pressure, &humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280/BME280 measurement: %s", esp_err_to_name(ret));
        xSemaphoreGive(bmp_cache.mutex);
        return ret;
    }

    // Update cache with all values
    bmp_cache.pressure = pressure;
    bmp_cache.temperature = temperature;
    if (is_bme280) {
        bmp_cache.humidity = humidity;
    }
    bmp_cache.timestamp = current_time;
    bmp_cache.valid = true;

    xSemaphoreGive(bmp_cache.mutex);

    if (is_bme280) {
        ESP_LOGI(TAG, "BME280 measurement updated - Pressure: %.2f Pa, Temp: %.2f °C, Humidity: %.2f %%RH",
                 pressure, temperature, humidity);
    } else {
        ESP_LOGI(TAG, "BMP280 measurement updated - Pressure: %.2f Pa, Temp: %.2f °C",
                 pressure, temperature);
    }

    return ESP_OK;
}

// BMP280/BME280 multi-value sensor read callback
static esp_err_t bmp_sensor_read_multi(const char *sensor_id, mcp_sensor_multi_value_t *values, void *user_data)
{
    if (!values) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = bmp_read_cached_measurement();
    if (ret != ESP_OK) {
        return ret;
    }

    // Allocate fields array (2 or 3 values depending on BME280/BMP280)
    static mcp_sensor_field_t fields[3];
    size_t field_count = 0;

    xSemaphoreTake(bmp_cache.mutex, portMAX_DELAY);

    fields[field_count].name = "pressure";
    fields[field_count].value = bmp_cache.pressure;
    fields[field_count].unit = "Pa";
    fields[field_count].quality = 100;
    field_count++;

    fields[field_count].name = "temperature";
    fields[field_count].value = bmp_cache.temperature;
    fields[field_count].unit = "°C";
    fields[field_count].quality = 100;
    field_count++;

    if (is_bme280) {
        fields[field_count].name = "humidity";
        fields[field_count].value = bmp_cache.humidity;
        fields[field_count].unit = "%RH";
        fields[field_count].quality = 100;
        field_count++;
    }

    xSemaphoreGive(bmp_cache.mutex);

    values->fields = fields;
    values->field_count = field_count;

    return ESP_OK;
}

// Initialize HM3301 sensor
static esp_err_t init_hm3301_sensor(void)
{
    ESP_LOGI(TAG, "Initializing HM3301 sensor");

    // Initialize HM3301 device descriptor
    esp_err_t ret = hm3301_init_desc(&hm3301_dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HM3301 device descriptor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for sensor to be ready
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize sensor (select I2C communication mode)
    // Note: This will fail if sensor is not connected (I2C NACK)
    ESP_LOGI(TAG, "Attempting to communicate with HM3301...");
    ret = hm3301_init(&hm3301_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HM3301: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "HM3301 may not be connected or may be faulty");
        hm3301_free_desc(&hm3301_dev);
        return ret;
    }

    // Initialize measurement cache
    hm3301_cache.mutex = xSemaphoreCreateMutex();
    if (!hm3301_cache.mutex) {
        ESP_LOGE(TAG, "Failed to create HM3301 cache mutex");
        hm3301_free_desc(&hm3301_dev);
        return ESP_ERR_NO_MEM;
    }
    hm3301_cache.valid = false;
    hm3301_cache.timestamp = 0;

    // Wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    hm3301_initialized = true;
    ESP_LOGI(TAG, "HM3301 sensor initialized successfully");

    return ESP_OK;
}

// Read HM3301 measurement and update cache
static esp_err_t hm3301_read_cached_measurement(void)
{
    if (!hm3301_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Take mutex for entire operation to prevent concurrent I2C access
    xSemaphoreTake(hm3301_cache.mutex, portMAX_DELAY);

    // Check if we have a recent valid measurement (within 1 second)
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (hm3301_cache.valid && (current_time - hm3301_cache.timestamp) < 1000) {
        // Use cached data if it's less than 1 second old
        xSemaphoreGive(hm3301_cache.mutex);
        return ESP_OK;
    }

    // Read new measurement (while holding mutex to prevent concurrent I2C access)
    hm3301_data_t data;
    esp_err_t ret = hm3301_read_data(&hm3301_dev, &data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read HM3301 measurement: %s", esp_err_to_name(ret));
        xSemaphoreGive(hm3301_cache.mutex);
        return ret;
    }

    // Update cache
    hm3301_cache.data = data;
    hm3301_cache.timestamp = current_time;
    hm3301_cache.valid = true;

    xSemaphoreGive(hm3301_cache.mutex);

    ESP_LOGI(TAG, "HM3301 measurement updated - PM1.0: %d/%d, PM2.5: %d/%d, PM10: %d/%d ug/m3",
             data.pm1_0_std, data.pm1_0_atm,
             data.pm2_5_std, data.pm2_5_atm,
             data.pm10_std, data.pm10_atm);
    // Uncomment for HM-3302/HM-3602 to log particle counts:
    // ESP_LOGI(TAG, "HM3301 particle counts - 0.3um: %d, 0.5um: %d, 1.0um: %d, 2.5um: %d, 5.0um: %d, 10um: %d",
    //          data.particles_03um, data.particles_05um, data.particles_10um,
    //          data.particles_25um, data.particles_50um, data.particles_100um);

    return ESP_OK;
}

// HM3301 multi-value sensor read callback
static esp_err_t hm3301_sensor_read_multi(const char *sensor_id, mcp_sensor_multi_value_t *values, void *user_data)
{
    if (!values) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = hm3301_read_cached_measurement();
    if (ret != ESP_OK) {
        return ret;
    }

    // Allocate fields array (6 PM concentrations for HM-3301)
    // Change to 12 and uncomment particle fields below for HM-3302/HM-3602
    static mcp_sensor_field_t fields[6];

    xSemaphoreTake(hm3301_cache.mutex, portMAX_DELAY);

    fields[0].name = "pm1.0_std";
    fields[0].value = (float)hm3301_cache.data.pm1_0_std;
    fields[0].unit = "ug/m3";
    fields[0].quality = 100;

    fields[1].name = "pm2.5_std";
    fields[1].value = (float)hm3301_cache.data.pm2_5_std;
    fields[1].unit = "ug/m3";
    fields[1].quality = 100;

    fields[2].name = "pm10_std";
    fields[2].value = (float)hm3301_cache.data.pm10_std;
    fields[2].unit = "ug/m3";
    fields[2].quality = 100;

    fields[3].name = "pm1.0_atm";
    fields[3].value = (float)hm3301_cache.data.pm1_0_atm;
    fields[3].unit = "ug/m3";
    fields[3].quality = 100;

    fields[4].name = "pm2.5_atm";
    fields[4].value = (float)hm3301_cache.data.pm2_5_atm;
    fields[4].unit = "ug/m3";
    fields[4].quality = 100;

    fields[5].name = "pm10_atm";
    fields[5].value = (float)hm3301_cache.data.pm10_atm;
    fields[5].unit = "ug/m3";
    fields[5].quality = 100;

    // Uncomment for HM-3302/HM-3602 (counting + weighing mode):
    // fields[6].name = "particles_0.3um";
    // fields[6].value = (float)hm3301_cache.data.particles_03um;
    // fields[6].unit = "count/0.1L";
    // fields[6].quality = 100;

    // fields[7].name = "particles_0.5um";
    // fields[7].value = (float)hm3301_cache.data.particles_05um;
    // fields[7].unit = "count/0.1L";
    // fields[7].quality = 100;

    // fields[8].name = "particles_1.0um";
    // fields[8].value = (float)hm3301_cache.data.particles_10um;
    // fields[8].unit = "count/0.1L";
    // fields[8].quality = 100;

    // fields[9].name = "particles_2.5um";
    // fields[9].value = (float)hm3301_cache.data.particles_25um;
    // fields[9].unit = "count/0.1L";
    // fields[9].quality = 100;

    // fields[10].name = "particles_5.0um";
    // fields[10].value = (float)hm3301_cache.data.particles_50um;
    // fields[10].unit = "count/0.1L";
    // fields[10].quality = 100;

    // fields[11].name = "particles_10um";
    // fields[11].value = (float)hm3301_cache.data.particles_100um;
    // fields[11].unit = "count/0.1L";
    // fields[11].quality = 100;

    xSemaphoreGive(hm3301_cache.mutex);

    values->fields = fields;
    values->field_count = 6;

    return ESP_OK;
}

// Read SCD30 measurement and update cache
static esp_err_t scd30_read_cached_measurement(void)
{
    if (!scd30_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Take mutex for entire operation to prevent concurrent I2C access
    xSemaphoreTake(scd30_cache.mutex, portMAX_DELAY);

    // Check if we have a recent valid measurement (within 1 second)
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (scd30_cache.valid && (current_time - scd30_cache.timestamp) < 1000) {
        // Use cached data if it's less than 1 second old
        xSemaphoreGive(scd30_cache.mutex);
        return ESP_OK;
    }

    // Check if new data is ready (while holding mutex)
    bool data_ready;
    esp_err_t ret = scd30_get_data_ready_status(&scd30_dev, &data_ready);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to check SCD30 data ready status: %s", esp_err_to_name(ret));
        xSemaphoreGive(scd30_cache.mutex);
        return ret;
    }

    ESP_LOGD(TAG, "SCD30 data ready status: %s", data_ready ? "YES" : "NO");

    if (!data_ready) {
        // Return cached data if available, otherwise error
        bool has_cached = scd30_cache.valid;

        if (has_cached) {
            ESP_LOGD(TAG, "SCD30 data not ready, using cached values");
            xSemaphoreGive(scd30_cache.mutex);
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "SCD30 data not ready and no cached data available");
            xSemaphoreGive(scd30_cache.mutex);
            return ESP_ERR_NOT_FINISHED;
        }
    }

    // Read new measurement (while holding mutex to prevent concurrent I2C access)
    float co2, temperature, humidity;
    ret = scd30_read_measurement(&scd30_dev, &co2, &temperature, &humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SCD30 measurement: %s", esp_err_to_name(ret));
        xSemaphoreGive(scd30_cache.mutex);
        return ret;
    }

    // Validate CO2 reading
    if (co2 <= 0) {
        ESP_LOGW(TAG, "Invalid CO2 reading: %.0f ppm", co2);
        xSemaphoreGive(scd30_cache.mutex);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Update cache with all three values
    scd30_cache.co2 = co2;
    scd30_cache.temperature = temperature;
    scd30_cache.humidity = humidity;
    scd30_cache.timestamp = current_time;
    scd30_cache.valid = true;

    xSemaphoreGive(scd30_cache.mutex);

    ESP_LOGI(TAG, "SCD30 measurement updated - CO2: %.0f ppm, Temp: %.2f °C, Humidity: %.2f %%RH",
             co2, temperature, humidity);

    return ESP_OK;
}

// SCD30 multi-value sensor read callback
static esp_err_t scd30_sensor_read_multi(const char *sensor_id, mcp_sensor_multi_value_t *values, void *user_data)
{
    if (!values) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = scd30_read_cached_measurement();
    if (ret != ESP_OK) {
        return ret;
    }

    // Allocate fields array (3 values: CO2, temperature, humidity)
    static mcp_sensor_field_t fields[3];

    xSemaphoreTake(scd30_cache.mutex, portMAX_DELAY);

    fields[0].name = "co2";
    fields[0].value = scd30_cache.co2;
    fields[0].unit = "ppm";
    fields[0].quality = 100;

    fields[1].name = "temperature";
    fields[1].value = scd30_cache.temperature;
    fields[1].unit = "°C";
    fields[1].quality = 100;

    fields[2].name = "humidity";
    fields[2].value = scd30_cache.humidity;
    fields[2].unit = "%RH";
    fields[2].quality = 100;

    xSemaphoreGive(scd30_cache.mutex);

    values->fields = fields;
    values->field_count = 3;

    return ESP_OK;
}

// Initialize DS3231 RTC
static esp_err_t init_ds3231_rtc(void)
{
    ESP_LOGI(TAG, "Initializing DS3231 RTC");

    // Initialize DS3231 device descriptor
    esp_err_t ret = ds3231_init_desc(&ds3231_dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DS3231 device descriptor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check oscillator stop flag (indicates lost power/time)
    bool stopped;
    ret = ds3231_get_oscillator_stop_flag(&ds3231_dev, &stopped);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get DS3231 oscillator stop flag: %s", esp_err_to_name(ret));
        ds3231_free_desc(&ds3231_dev);
        return ret;
    }

    if (stopped) {
        ESP_LOGW(TAG, "RTC oscillator was stopped - time may be invalid");
        ESP_LOGW(TAG, "Set RTC time manually using ds3231_set_time()");
        // Clear the flag
        ret = ds3231_clear_oscillator_stop_flag(&ds3231_dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear DS3231 oscillator stop flag: %s", esp_err_to_name(ret));
            ds3231_free_desc(&ds3231_dev);
            return ret;
        }
    }

    // Read current time from RTC
    struct tm rtc_time;
    ret = ds3231_get_time(&ds3231_dev, &rtc_time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read DS3231 time: %s", esp_err_to_name(ret));
        ds3231_free_desc(&ds3231_dev);
        return ret;
    }

    // Set ESP32 system time from RTC
    time_t now = mktime(&rtc_time);
    struct timeval tv = { .tv_sec = now, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &rtc_time);
    ESP_LOGI(TAG, "RTC time: %s (Unix: %ld)", time_str, (long)now);

    ds3231_initialized = true;
    ESP_LOGI(TAG, "DS3231 RTC initialized successfully");

    return ESP_OK;
}

// Get Unix timestamp from RTC
static time_t get_rtc_timestamp(void)
{
    if (!ds3231_initialized) {
        return 0;
    }

    struct tm rtc_time;
    esp_err_t ret = ds3231_get_time(&ds3231_dev, &rtc_time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read RTC time: %s", esp_err_to_name(ret));
        return 0;
    }

    return mktime(&rtc_time);
}

// SNTP time synchronization notification callback
static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized from NTP server");

    // Update RTC with the new time
    if (ds3231_initialized) {
        time_t now = tv->tv_sec;
        struct tm timeinfo;
        gmtime_r(&now, &timeinfo);

        esp_err_t ret = ds3231_set_time(&ds3231_dev, &timeinfo);
        if (ret == ESP_OK) {
            char time_str[64];
            strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S UTC", &timeinfo);
            ESP_LOGI(TAG, "RTC updated to: %s", time_str);
        } else {
            ESP_LOGE(TAG, "Failed to update RTC: %s", esp_err_to_name(ret));
        }
    }
}

// Initialize and start SNTP time synchronization
static void sync_time_from_ntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP time synchronization");

    // Set timezone to UTC (adjust if needed)
    setenv("TZ", "UTC", 1);
    tzset();

    // Set callback for time sync notification
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    // Configure SNTP
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_setservername(2, "time.cloudflare.com");

    // Start SNTP
    esp_sntp_init();

    ESP_LOGI(TAG, "SNTP initialized, waiting for time sync...");
}

// MCP Bridge event handler
static void mcp_event_handler(const mcp_event_t *event, void *user_data)
{
    switch (event->type) {
        case MCP_EVENT_WIFI_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected");
            // Synchronize time from NTP server
            sync_time_from_ntp();
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