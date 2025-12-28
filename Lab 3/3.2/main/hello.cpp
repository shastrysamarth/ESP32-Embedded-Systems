#include <stdio.h>
#include <string.h>
#include <math.h>
#include "DFRobot_RGBLCD1602.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static DFRobot_RGBLCD1602 lcd(
		0x2D, 16, 2, 0x3E, 
		(gpio_num_t)10, (gpio_num_t)8, 
		100000);

extern "C" void app_main(void) {
        lcd.init();
	while (true) {
		lcd.home();
		lcd.clear();
		lcd.setRGB(255, 255, 255);
		lcd.setBacklight(1);
		lcd.setCursor(0, 0);
		lcd.printstr("Hello CSE121!");
		lcd.setCursor(0, 1);
		lcd.printstr("Samarth Shastry");

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}