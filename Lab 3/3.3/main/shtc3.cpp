#include <stdio.h>
#include <string.h>
#include <math.h>
#include "DFRobot_RGBLCD1602.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"

static DFRobot_RGBLCD1602 lcd(
		0x2D, 16, 2, 0x3E, 
		(gpio_num_t)10, (gpio_num_t)8, 
		100000);

static const uint8_t SHTC3_ADDR = 0x70;          // 7-bit
static const uint16_t SHTC3_WAKEUP   = 0x3517;   // wake
static const uint16_t SHTC3_SLEEP    = 0xB098;   // sleep
static const uint16_t SHTC3_MEAS_TF  = 0x7CA2;

// CRC-8 params: poly 0x31, init 0xFF
static uint8_t shtc3_crc8(const uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i=0; i<len; ++i) {
    crc ^= data[i];
    for (int b=0; b<8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static esp_err_t shtc3_cmd(i2c_master_dev_handle_t dev, uint16_t cmd) {
  uint8_t buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
  return i2c_master_transmit(dev, buf, 2, 1000);
}

static esp_err_t shtc3_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *out_dev) {
  i2c_device_config_t devcfg = {};
  devcfg.dev_addr_length = (i2c_addr_bit_len_t)0;
  devcfg.device_address = SHTC3_ADDR;
  devcfg.scl_speed_hz   = 100000;
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &devcfg, out_dev), TAG, "add_device failed");

  // Wake once
  ESP_RETURN_ON_ERROR(shtc3_cmd(*out_dev, SHTC3_WAKEUP), TAG, "wakeup failed");
  vTaskDelay(pdMS_TO_TICKS(1));
  return ESP_OK;
}

static esp_err_t shtc3_read_trh(i2c_master_dev_handle_t dev, float *out_T_C, float *out_RH) {
  // Trigger single measurement (T first)
  esp_err_t err = shtc3_cmd(dev, SHTC3_WAKEUP);
  if (err != ESP_OK) return err;
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_RETURN_ON_ERROR(shtc3_cmd(dev, SHTC3_MEAS_TF), TAG, "meas cmd failed");

  // Typical meas time ~10–12ms in normal mode, add margin
  vTaskDelay(pdMS_TO_TICKS(15));

  // Read 6 bytes: T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC
  uint8_t rx[6] = {0};
  ESP_RETURN_ON_ERROR(i2c_master_receive(dev, rx, 6, 1000), TAG, "read failed");

  // CRC check each 2-byte word
  if (shtc3_crc8(rx, 2) != rx[2])   return ESP_FAIL;
  if (shtc3_crc8(rx+3, 2) != rx[5]) return ESP_FAIL;

  uint16_t rawT  = ((uint16_t)rx[0] << 8) | rx[1];
  uint16_t rawRH = ((uint16_t)rx[3] << 8) | rx[4];

  // Convert (datasheet):
  // T(°C)  = -45 + 175 * rawT / 65535
  // RH(%)  = 100 * rawRH / 65535
  *out_T_C = -45.0f + 175.0f * ((float)rawT  / 65535.0f);
  *out_RH  = 100.0f * ((float)rawRH / 65535.0f);

  // Optional: go back to sleep to save power
  (void)shtc3_cmd(dev, SHTC3_SLEEP);

  return ESP_OK;
}

// Scan I2C bus and print all 7-bit addresses that ACK.
static void i2c_scan(i2c_master_bus_handle_t bus) {
  printf("I2C scan: ");
  for (uint8_t addr = 0x03; addr < 0x78; ++addr) {
    i2c_master_dev_handle_t dev;
    i2c_device_config_t c = {};
  #ifdef I2C_ADDR_BIT_LEN_7
    c.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  #else
    c.dev_addr_length = (i2c_addr_bit_len_t)0;
  #endif
    c.device_address = addr;
    c.scl_speed_hz = 100000;
    if (i2c_master_bus_add_device(bus, &c, &dev) == ESP_OK) {
      // zero-length transmit works as a probe on IDF 5.x
      uint8_t dummy;
      esp_err_t err = i2c_master_transmit(dev, &dummy, 1, 200);
      i2c_master_bus_rm_device(dev);
      if (err == ESP_OK || err == ESP_ERR_TIMEOUT) printf("0x%02X ", addr);
    }
  }
  printf("\n");
}

extern "C" void app_main(void) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lcd.init();
        lcd.home();
        lcd.clear();
        i2c_scan(lcd.bus());
        vTaskDelay(pdMS_TO_TICKS(500));
        lcd.setRGB(255, 255, 255);
        vTaskDelay(pdMS_TO_TICKS(500));
        //lcd.setBacklight(1);
          
        i2c_master_dev_handle_t shtc3 = nullptr;
        if (shtc3_init(lcd.bus(), &shtc3) != ESP_OK) {
          lcd.setCursor(0,0); lcd.printstr("SHTC3 init ERR");
          while (true) vTaskDelay(pdMS_TO_TICKS(1000));
        }

        char line[17];
        
        while (true) {
          float tC=0, rh=0;
          esp_err_t ok = shtc3_read_trh(shtc3, &tC, &rh);

          // Row 0: Temperature
          lcd.setCursor(0,0);
          if (ok == ESP_OK) {
            // Example: "T: 23.4 C"
            snprintf(line, sizeof(line), "Temp: %5.1f C", tC);
          } else {
            snprintf(line, sizeof(line), "Temp: ---   ");
          }
          // ensure exactly 16 chars printed (pad with spaces)
          char pad1[17]; snprintf(pad1, sizeof(pad1), "%-16s", line);
          lcd.printstr(pad1);

          // Row 1: Humidity
          lcd.setCursor(0,1);
          if (ok == ESP_OK) {
            // Example: "RH: 45.2 %%"
            snprintf(line, sizeof(line), "Hum : %5.1f %%", rh);
          } else {
            snprintf(line, sizeof(line), "Hum : ---  ");
          }
          char pad2[17]; snprintf(pad2, sizeof(pad2), "%-16s", line);
          lcd.printstr(pad2);

          vTaskDelay(pdMS_TO_TICKS(2000));
        }

}