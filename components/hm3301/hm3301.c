/*
 * Copyright (c) 2018 Seeed Technology Co., Ltd.
 * Copyright (c) 2025 ESP-IDF port
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file hm3301.c
 *
 * ESP-IDF driver for Seeed HM3301 PM2.5 Particulate Matter Sensor
 *
 * Ported from Arduino library: https://github.com/Seeed-Studio/Seeed_PM2_5_sensor_HM3301
 *
 * Copyright (c) 2018 Seeed Technology Co., Ltd.
 * Copyright (c) 2025 ESP-IDF port
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "hm3301.h"

#define I2C_FREQ_HZ 100000 // 100kHz

static const char *TAG = "hm3301";

#define SELECT_COMM_CMD 0x88

#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)

#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

esp_err_t hm3301_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = HM3301_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t hm3301_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t hm3301_init(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    ESP_LOGI(TAG, "Initializing HM3301 sensor");

    // Send select I2C communication command
    uint8_t cmd = SELECT_COMM_CMD;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &cmd, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    ESP_LOGI(TAG, "HM3301 initialized successfully");

    return ESP_OK;
}

esp_err_t hm3301_read_raw(i2c_dev_t *dev, uint8_t *data, uint32_t data_len)
{
    CHECK_ARG(dev);
    CHECK_ARG(data);
    CHECK_ARG(data_len == 29);

    I2C_DEV_TAKE_MUTEX(dev);

    // Request 29 bytes from sensor
    esp_err_t ret = i2c_dev_read(dev, NULL, 0, data, data_len);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read from HM3301: %s", esp_err_to_name(ret));
        I2C_DEV_GIVE_MUTEX(dev);
        return ret;
    }

    I2C_DEV_GIVE_MUTEX(dev);

    ESP_LOGI(TAG, "Raw data read successfully");
    ESP_LOG_BUFFER_HEX(TAG, data, data_len);

    return ESP_OK;
}

esp_err_t hm3301_read_data(i2c_dev_t *dev, hm3301_data_t *data)
{
    CHECK_ARG(dev);
    CHECK_ARG(data);

    uint8_t buf[29];
    CHECK(hm3301_read_raw(dev, buf, 29));

    // Verify checksum (sum of bytes 0-27 should equal byte 28)
    uint8_t checksum = 0;
    for (int i = 0; i < 28; i++)
    {
        checksum += buf[i];
    }

    if (checksum != buf[28])
    {
        ESP_LOGE(TAG, "Checksum mismatch: calculated 0x%02x, expected 0x%02x", checksum, buf[28]);
        return ESP_ERR_INVALID_CRC;
    }

    // Parse data according to HM3301 data format (1-indexed in datasheet, 0-indexed in code)
    // Data format: each value is 2 bytes (big-endian)
    // Data1-2 (buf[0-1]): Reserved
    // Data3-4 (buf[2-3]): Sensor number
    // Data5-6 (buf[4-5]): PM1.0 (CF=1, Standard)
    // Data7-8 (buf[6-7]): PM2.5 (CF=1, Standard)
    // Data9-10 (buf[8-9]): PM10 (CF=1, Standard)
    // Data11-12 (buf[10-11]): PM1.0 (Atmospheric)
    // Data13-14 (buf[12-13]): PM2.5 (Atmospheric)
    // Data15-16 (buf[14-15]): PM10 (Atmospheric)

    data->pm1_0_std = ((uint16_t)buf[4] << 8) | buf[5];
    data->pm2_5_std = ((uint16_t)buf[6] << 8) | buf[7];
    data->pm10_std = ((uint16_t)buf[8] << 8) | buf[9];
    data->pm1_0_atm = ((uint16_t)buf[10] << 8) | buf[11];
    data->pm2_5_atm = ((uint16_t)buf[12] << 8) | buf[13];
    data->pm10_atm = ((uint16_t)buf[14] << 8) | buf[15];

    // Data17-28 (buf[16-27]): Particle counts in 0.1L of air
    data->particles_03um = ((uint16_t)buf[16] << 8) | buf[17];
    data->particles_05um = ((uint16_t)buf[18] << 8) | buf[19];
    data->particles_10um = ((uint16_t)buf[20] << 8) | buf[21];
    data->particles_25um = ((uint16_t)buf[22] << 8) | buf[23];
    data->particles_50um = ((uint16_t)buf[24] << 8) | buf[25];
    data->particles_100um = ((uint16_t)buf[26] << 8) | buf[27];

    ESP_LOGD(TAG, "PM1.0: std=%d atm=%d, PM2.5: std=%d atm=%d, PM10: std=%d atm=%d",
             data->pm1_0_std, data->pm1_0_atm,
             data->pm2_5_std, data->pm2_5_atm,
             data->pm10_std, data->pm10_atm);
    ESP_LOGD(TAG, "Particles: 0.3um=%d, 0.5um=%d, 1.0um=%d, 2.5um=%d, 5.0um=%d, 10um=%d",
             data->particles_03um, data->particles_05um, data->particles_10um,
             data->particles_25um, data->particles_50um, data->particles_100um);

    return ESP_OK;
}
