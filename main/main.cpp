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
static esp_err_t scd30_sensor_read_multi(const char *sensor_id, mcp_sensor_multi_value_t *values, void *user_data);
static esp_err_t bmp_sensor_read_multi(const char *sensor_id, mcp_sensor_multi_value_t *values, void *user_data);
static void mcp_event_handler(const mcp_event_t *event, void *user_data);
static esp_err_t init_scd30_sensor(void);
static esp_err_t init_bmp_sensor(void);
static esp_err_t init_ds3231_rtc(void);
static time_t get_rtc_timestamp(void);
static void sync_time_from_ntp(void);
static void time_sync_notification_cb(struct timeval *tv);

void update_sensors_task(void *p);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-C6 MCP Sensor Hub Application");
    
    // Initialize I2C bus
    ESP_ERROR_CHECK(i2cdev_init());

    // Initialize DS3231 RTC
    ESP_ERROR_CHECK(init_ds3231_rtc());

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

    // Register SCD30 as multi-value sensor
    mcp_sensor_field_metadata_t scd30_fields[] = {
        {.name = "co2", .unit = "ppm", .metric_type = "co2", .min_range = 400.0f, .max_range = 10000.0f, .accuracy = 30.0f},
        {.name = "temperature", .unit = "°C", .metric_type = "temperature", .min_range = -40.0f, .max_range = 70.0f, .accuracy = 0.4f},
        {.name = "humidity", .unit = "%RH", .metric_type = "humidity", .min_range = 0.0f, .max_range = 100.0f, .accuracy = 3.0f}
    };
    mcp_sensor_metadata_t scd30_metadata = {
        .min_range = 0.0f,
        .max_range = 0.0f,
        .accuracy = 0.0f,
        .update_interval_ms = 2000,
        .description = "SCD30 environmental sensor (CO2, temperature, humidity)",
        .location = "home",
        .calibration_required = false,
        .calibration_interval_s = 0
    };
    ESP_ERROR_CHECK(mcp_bridge_register_multi_sensor("scd30", scd30_fields, 3, &scd30_metadata, scd30_sensor_read_multi, NULL));

    // Register BMP280/BME280 as multi-value sensor
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
    ESP_ERROR_CHECK(mcp_bridge_register_multi_sensor("bmp280", bmp_fields, bmp_field_count, &bmp_metadata, bmp_sensor_read_multi, NULL));
    
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

    ESP_LOGI(TAG, "MCP Sensor Hub started successfully with all sensors");
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
    ESP_ERROR_CHECK(scd30_init_desc(&scd30_dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7));
    // wait for the sensor to be ready
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Read firmware version
    uint16_t version, major_ver, minor_ver;
    ESP_ERROR_CHECK(scd30_read_firmware_version(&scd30_dev, &version));
    
    major_ver = (version >> 8) & 0xf;
    minor_ver = version & 0xf;
    ESP_LOGI(TAG, "SCD30 Firmware Version: %d.%d", major_ver, minor_ver);
    
    // Set measurement interval to 10 seconds (must be done before starting measurement)
    ESP_LOGI(TAG, "Setting SCD30 measurement interval to 10 seconds");
    ESP_ERROR_CHECK(scd30_set_measurement_interval(&scd30_dev, 10));

    // Small delay after setting interval (stored in non-volatile memory)
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify the interval was set
    uint16_t interval;
    ESP_ERROR_CHECK(scd30_get_measurement_interval(&scd30_dev, &interval));
    ESP_LOGI(TAG, "SCD30 measurement interval confirmed: %d seconds", interval);

    // Start continuous measurement
    //ESP_LOGI(TAG, "Starting SCD30 continuous measurement");
    //ESP_ERROR_CHECK(scd30_trigger_continuous_measurement(&scd30_dev, 0));

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
        esp_err_t ret = scd30_get_data_ready_status(&scd30_dev, &data_ready);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to check SCD30 data ready status: %s", esp_err_to_name(ret));
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
        ESP_LOGE(TAG, "Failed to check SCD30 data ready status: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "SCD30 data ready status: %s", data_ready ? "YES" : "NO");

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
    ESP_ERROR_CHECK(ds3231_init_desc(&ds3231_dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7));

    // Check oscillator stop flag (indicates lost power/time)
    bool stopped;
    ESP_ERROR_CHECK(ds3231_get_oscillator_stop_flag(&ds3231_dev, &stopped));

    if (stopped) {
        ESP_LOGW(TAG, "RTC oscillator was stopped - time may be invalid");
        ESP_LOGW(TAG, "Set RTC time manually using ds3231_set_time()");
        // Clear the flag
        ESP_ERROR_CHECK(ds3231_clear_oscillator_stop_flag(&ds3231_dev));
    }

    // Read current time from RTC
    struct tm rtc_time;
    ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &rtc_time));

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