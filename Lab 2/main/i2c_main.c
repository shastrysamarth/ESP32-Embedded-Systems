
/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a SHTC3 inertial measurement unit.
*/
#include <stdio.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO           8       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           10      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define SHTC3_SENSOR_ADDR         0x70        /*!< Address of the SHTC3 sensor */
#define SHTC3_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
#define SHTC3_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
#define SHTC3_RESET_BIT           7

/**
 * @brief Read a sequence of bytes from a SHTC3 sensor registers
 */
static esp_err_t shtc3_register_read(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t len)
{
    return i2c_master_receive(dev_handle, data, len, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Write a byte to a SHTC3 sensor register
 */
static esp_err_t shtc3_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHTC3_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

static bool sht_crc_ok(uint8_t msb, uint8_t lsb, uint8_t crc_rx) {
    uint8_t crc = 0xFF;
    uint8_t data[2] = { msb, lsb };
    for (int i = 0; i < 2; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return (crc == crc_rx);
}

void app_main(void)
{
    uint8_t rx[6];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully (SHTC3 @ 0x%02X)", SHTC3_SENSOR_ADDR);

    // SHTC3 commands (send as two separate bytes using the existing write helper):
    // Wake-up  : 0x3517  -> write 0x35, 0x17
    // Measure T: 0x7866  -> write 0x7C, 0xA2  (T-first, clock stretching enabled)
    // Sleep    : 0xB098  -> write 0xB0, 0x98

    while (1) {
        // Wake the sensor
        ESP_ERROR_CHECK(shtc3_register_write_byte(dev_handle, 0x35, 0x17));
        
        vTaskDelay(pdMS_TO_TICKS(200));

        // Start measurement (temperature first)
        ESP_ERROR_CHECK(shtc3_register_write_byte(dev_handle, 0x7C, 0xA2));

        // Wait for conversion (normal mode typ ~10.8 ms, use 12 ms)
        vTaskDelay(pdMS_TO_TICKS(200));

        // Read first 3 bytes: temp MSB, temp LSB, CRC (read straight from device)
        // NOTE: We call the lower-level receive API directly to avoid sending a register byte.
        ESP_ERROR_CHECK(shtc3_register_read(dev_handle, rx, sizeof(rx)));

        bool t_ok  = sht_crc_ok(rx[0], rx[1], rx[2]);
        bool rh_ok = sht_crc_ok(rx[3], rx[4], rx[5]);
        
        if (!t_ok) {
            ESP_LOGE(TAG, "CRC mismatch on temperature: got 0x%02X", rx[2]);
        }
        if (!rh_ok) {
            ESP_LOGE(TAG, "CRC mismatch on humidity: got 0x%02X", rx[5]);
        }
        
        
        if (t_ok && rh_ok) {
            uint16_t rawT  = ((uint16_t)rx[0] << 8) | rx[1];
            uint16_t rawRH = ((uint16_t)rx[3] << 8) | rx[4];

            // Convert raw values according to SHTC3 datasheet
            float tC  = -45.0f + 175.0f * ((float)rawT  / 65535.0f);
            float rh  = 100.0f * ((float)rawRH / 65535.0f);
            if (rh < 0.0f)   rh = 0.0f;
            if (rh > 100.0f) rh = 100.0f;

            int tCi = (int)lrintf(tC);
            int tFi = (int)lrintf(tC * 9.0f / 5.0f + 32.0f);
            int rhi = (int)lrintf(rh);

            printf("Temperature is %dC (or %dF) with a %d%% humidity\n", tCi, tFi, rhi);
        }

        // Put the sensor back to sleep
        esp_err_t es = shtc3_register_write_byte(dev_handle, 0xB0, 0x98);
        if (es != ESP_OK) ESP_LOGW(TAG, "sleep failed: %s", esp_err_to_name(es));

        // Every 2 seconds per your lab requirement
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // (Unreachable in this loop, but keeping your original cleanup style:)
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}