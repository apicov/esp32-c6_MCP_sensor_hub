#include <cstdint>
#include <cstdio>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ESP-IDF
#include "esp_system.h"
#include <esp_log.h>

// components
#include "scd30.h"

#define LCD_ADDR 0x27
#define SCD30_ADDR 0x61

#define SDA_PIN  (gpio_num_t) 6
#define SCL_PIN  (gpio_num_t) 7
#define I2C_PORT (gpio_num_t) 0

static const char *TAG = "WIFI_STA"; // Tag for logging

void scd30_task(void *p);

extern "C" void app_main(void)
{
	ESP_LOGI(__func__,"starting app_main");
    // configure I2C bus
    ESP_ERROR_CHECK(i2cdev_init());
   

    xTaskCreate(&scd30_task, "Test Task", 2048, NULL, 5, NULL);

}

void scd30_task(void *p)
    {
        i2c_dev_t dev;
        ESP_ERROR_CHECK(scd30_init_desc(&dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7));
        

        uint16_t version, major_ver, minor_ver;

        ESP_ERROR_CHECK(scd30_read_firmware_version(&dev, &version));

        major_ver = (version >> 8) & 0xf;
        minor_ver = version & 0xf;

        ESP_LOGI(TAG, "SCD30 Firmware Version: %d.%d", major_ver, minor_ver);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        ESP_LOGI(TAG, "Starting continuous measurement");
        ESP_ERROR_CHECK(scd30_trigger_continuous_measurement(&dev, 0));

        float co2, temperature, humidity;
        bool data_ready;
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(2000));

            scd30_get_data_ready_status(&dev, &data_ready);

            if (data_ready)
            {
                esp_err_t res = scd30_read_measurement(&dev, &co2, &temperature, &humidity);
                if (res != ESP_OK)
                {
                    ESP_LOGE(TAG, "Error reading results %d (%s)", res, esp_err_to_name(res));
                    continue;
                }

                if (co2 == 0)
                {
                    ESP_LOGW(TAG, "Invalid sample detected, skipping");
                    continue;
                }

                ESP_LOGI(TAG, "CO2: %.0f ppm", co2);
                ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
                ESP_LOGI(TAG, "Humidity: %.2f %%", humidity);
            }
        }
    } // end of producer