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
 * @file hm3301.h
 * @defgroup hm3301 hm3301
 * @{
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
#ifndef __HM3301_H__
#define __HM3301_H__

#include <i2cdev.h>
#include <esp_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HM3301_I2C_ADDR 0x40

/**
 * @brief Particulate matter measurement data
 */
typedef struct {
    uint16_t pm1_0_std;      /**< PM1.0 concentration (CF=1, Standard), ug/m3 */
    uint16_t pm2_5_std;      /**< PM2.5 concentration (CF=1, Standard), ug/m3 */
    uint16_t pm10_std;       /**< PM10 concentration (CF=1, Standard), ug/m3 */
    uint16_t pm1_0_atm;      /**< PM1.0 concentration (Atmospheric environment), ug/m3 */
    uint16_t pm2_5_atm;      /**< PM2.5 concentration (Atmospheric environment), ug/m3 */
    uint16_t pm10_atm;       /**< PM10 concentration (Atmospheric environment), ug/m3 */
} hm3301_data_t;

/**
 * @brief Initialize device descriptor.
 *
 * @param dev      Device descriptor
 * @param port     I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return         `ESP_OK` on success
 */
esp_err_t hm3301_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t hm3301_free_desc(i2c_dev_t *dev);

/**
 * @brief Initialize sensor and select I2C communication mode.
 *
 * Must be called after hm3301_init_desc() and before reading data.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t hm3301_init(i2c_dev_t *dev);

/**
 * @brief Read raw sensor data (29 bytes).
 *
 * Reads the raw 29-byte buffer from the sensor. The buffer includes:
 * - Bytes 0-1: Sensor number
 * - Bytes 2-27: PM data (see hm3301_read_data for parsed values)
 * - Byte 28: Checksum (sum of bytes 0-27)
 *
 * @param dev      Device descriptor
 * @param data     Buffer to store raw data (must be at least 29 bytes)
 * @param data_len Length of buffer (should be 29)
 * @return         `ESP_OK` on success
 */
esp_err_t hm3301_read_raw(i2c_dev_t *dev, uint8_t *data, uint32_t data_len);

/**
 * @brief Read and parse particulate matter measurements.
 *
 * Reads sensor data and parses it into a structured format.
 * The checksum is validated automatically.
 *
 * @param dev   Device descriptor
 * @param data  Pointer to structure to store parsed data
 * @return      `ESP_OK` on success, `ESP_ERR_INVALID_CRC` on checksum failure
 */
esp_err_t hm3301_read_data(i2c_dev_t *dev, hm3301_data_t *data);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __HM3301_H__ */
