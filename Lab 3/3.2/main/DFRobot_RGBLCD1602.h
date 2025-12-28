#pragma once
#include <stdint.h>
#include <stddef.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

// Keep constants and API surface compatible with the original header
#define LCD_ADDRESS     (0x7c>>1)  // 0x3E
#define RGB_ADDRESS     (0xc0>>1)
#define WHITE           0
#define RED             1
#define GREEN           2
#define BLUE            3

#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08

#define LCD_CLEARDISPLAY   0x01
#define LCD_RETURNHOME     0x02
#define LCD_ENTRYMODESET   0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT    0x10
#define LCD_FUNCTIONSET    0x20
#define LCD_SETCGRAMADDR   0x40
#define LCD_SETDDRAMADDR   0x80

#define LCD_ENTRYRIGHT           0x00
#define LCD_ENTRYLEFT            0x02
#define LCD_ENTRYSHIFTINCREMENT  0x01
#define LCD_ENTRYSHIFTDECREMENT  0x00

#define LCD_DISPLAYON   0x04
#define LCD_DISPLAYOFF  0x00
#define LCD_CURSORON    0x02
#define LCD_CURSOROFF   0x00
#define LCD_BLINKON     0x01
#define LCD_BLINKOFF    0x00

#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE  0x00
#define LCD_MOVERIGHT   0x04
#define LCD_MOVELEFT    0x00

#define LCD_8BITMODE  0x10
#define LCD_4BITMODE  0x00
#define LCD_2LINE     0x08
#define LCD_1LINE     0x00
#define LCD_5x10DOTS  0x04
#define LCD_5x8DOTS   0x00

static const char* TAG = "RGBLCD1602";

class DFRobot_RGBLCD1602 {
public:
  // Constructor mirrors the Arduino one, but without Wire/Print.
  // RGBAddr: backlight driver addr (e.g., 0x62 or 0x2D/0x6B),
  // lcdCols/lcdRows: geometry (16x2),
  // lcdAddr: LCD controller addr (usually 0x3E).
  DFRobot_RGBLCD1602(uint8_t RGBAddr, uint8_t lcdCols=16, uint8_t lcdRows=2, uint8_t lcdAddr=LCD_ADDRESS,
                     gpio_num_t sda=(gpio_num_t)10, gpio_num_t scl=(gpio_num_t)8, uint32_t i2c_hz=100000);

  // Public (Arduino-like) API
  void init();
  void clear();
  void home();
  i2c_master_bus_handle_t bus() const { return _bus; }

  void noDisplay();
  void display();
  void stopBlink();
  void blink();
  void noCursor();
  void cursor();

  void scrollDisplayLeft();
  void scrollDisplayRight();
  void leftToRight();
  void rightToLeft();
  void noAutoscroll();
  void autoscroll();

  void customSymbol(uint8_t location, uint8_t charmap[]);
  void setCursor(uint8_t col, uint8_t row);

  void setRGB(uint8_t r, uint8_t g, uint8_t b);
  void setPWM(uint8_t colorReg, uint8_t pwm) { setReg(colorReg, pwm); if(_RGBAddr==0x6B){ setReg(0x07, pwm);} }
  void setColor(uint8_t color);
  void closeBacklight(){ setRGB(0,0,0); }
  void setColorWhite(){ setRGB(255,255,255); }
  void setBacklight(bool mode){ if(mode) setColorWhite(); else closeBacklight(); }

  // Write a single character (compat with original’s write signature)
  size_t write(uint8_t value);
  // Simple string print (since you referenced lcd.printstr(...))
  size_t printstr(const char* s);

  // Registers depend on RGBAddr variant (same as original)
  uint8_t REG_RED   = 0x01;
  uint8_t REG_GREEN = 0x02;
  uint8_t REG_BLUE  = 0x03;
  uint8_t REG_ONLY  = 0x01;

private:
  void begin(uint8_t rows, uint8_t charSize = LCD_5x8DOTS);
  void send(uint8_t *data, uint8_t len);
  void command(uint8_t data);
  void setReg(uint8_t addr, uint8_t data);

  // ESP-IDF I²C
  esp_err_t i2c_bus_init(gpio_num_t sda, gpio_num_t scl, uint32_t hz);
  esp_err_t add_device(uint8_t addr, i2c_master_dev_handle_t* out);
  esp_err_t tx(i2c_master_dev_handle_t dev, const uint8_t* bytes, size_t n);

  // Backlight chip init variants
  void rgb_init_sequence();

private:
  uint8_t _showFunction = 0;
  uint8_t _showControl  = 0;
  uint8_t _showMode     = 0;
  uint8_t _numLines     = 0;
  uint8_t _currLine     = 0;

  uint8_t _lcdAddr = LCD_ADDRESS;
  uint8_t _RGBAddr = 0x62; // default common PCA9633
  uint8_t _cols = 16;
  uint8_t _rows = 2;

  i2c_master_bus_handle_t _bus = nullptr;
  i2c_master_dev_handle_t _lcd = nullptr;
  i2c_master_dev_handle_t _rgb = nullptr;
};