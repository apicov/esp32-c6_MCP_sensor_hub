#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h"

#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "nvs_flash.h"

// 
#include "scd30.h"

void scd30_task(void *p);

extern "C" void  app_main() {
    ESP_ERROR_CHECK(i2cdev_init()); // Initialize I2C bus
    xTaskCreate(scd30_task, "scd30_task", 4096, NULL, 5, NULL);
}


void scd30_task(void *p)
    {
        i2c_dev_t dev;
        ESP_ERROR_CHECK(scd30_init_desc(&dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7));
        

        uint16_t version, major_ver, minor_ver;

        ESP_ERROR_CHECK(scd30_read_firmware_version(&dev, &version));

        major_ver = (version >> 8) & 0xf;
        minor_ver = version & 0xf;

        ESP_LOGI(__func__, "SCD30 Firmware Version: %d.%d", major_ver, minor_ver);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        ESP_LOGI(__func__, "Starting continuous measurement");
        ESP_ERROR_CHECK(scd30_trigger_continuous_measurement(&dev, 0));

        float co2, temperature, humidity;
        bool data_ready;
        while (1)
        {
                    ESP_LOGI(__func__, ".../n");

            vTaskDelay(pdMS_TO_TICKS(2000));

            scd30_get_data_ready_status(&dev, &data_ready);

            if (data_ready)
            {
                esp_err_t res = scd30_read_measurement(&dev, &co2, &temperature, &humidity);
                if (res != ESP_OK)
                {
                    ESP_LOGE(__func__, "Error reading results %d (%s)", res, esp_err_to_name(res));
                    continue;
                }

                if (co2 == 0)
                {
                    ESP_LOGW(__func__, "Invalid sample detected, skipping");
                    continue;
                }

                ESP_LOGI(__func__, "CO2: %.0f ppm", co2);
                ESP_LOGI(__func__, "Temperature: %.2f °C", temperature);
                ESP_LOGI(__func__, "Humidity: %.2f %%", humidity);
            }
        }
    }