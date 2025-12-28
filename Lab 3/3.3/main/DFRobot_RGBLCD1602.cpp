#include <stdio.h>
#include <string.h>
#include "DFRobot_RGBLCD1602.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Control prefixes (AiP31068/HD44780 over I2C expander)
static constexpr uint8_t CTRL_CMD  = 0x00;
static constexpr uint8_t CTRL_DATA = 0x40;

static const uint8_t COLOR_DEFINE[4][3] = {
  {255,255,255}, // WHITE
  {255,0,0},     // RED
  {0,255,0},     // GREEN
  {0,0,255}      // BLUE
};

DFRobot_RGBLCD1602::DFRobot_RGBLCD1602(uint8_t RGBAddr, uint8_t lcdCols, uint8_t lcdRows, uint8_t lcdAddr,
                                       gpio_num_t sda, gpio_num_t scl, uint32_t i2c_hz)
: _lcdAddr(lcdAddr), _RGBAddr(RGBAddr), _cols(lcdCols), _rows(lcdRows) {
  ESP_ERROR_CHECK(i2c_bus_init(sda, scl, i2c_hz));
  ESP_ERROR_CHECK(add_device(_lcdAddr, &_lcd));
  ESP_ERROR_CHECK(add_device(_RGBAddr, &_rgb));
}

esp_err_t DFRobot_RGBLCD1602::i2c_bus_init(gpio_num_t sda, gpio_num_t scl, uint32_t hz) {
  i2c_master_bus_config_t buscfg = {};
  buscfg.i2c_port = I2C_NUM_0;
  buscfg.sda_io_num = sda;
  buscfg.scl_io_num = scl;
  buscfg.clk_source = I2C_CLK_SRC_DEFAULT;
//buscfg.glitch_ignore_cnt = 7;
//buscfg.flags.enable_internal_pullup = true;
  ESP_ERROR_CHECK(i2c_new_master_bus(&buscfg, &_bus));
  (void)hz; // each device sets its own scl_speed_hz
  return ESP_OK;
}

esp_err_t DFRobot_RGBLCD1602::add_device(uint8_t addr, i2c_master_dev_handle_t* out) {
  i2c_device_config_t devcfg = {};
  devcfg.dev_addr_length = (i2c_addr_bit_len_t)0;
  devcfg.device_address = addr;
  devcfg.scl_speed_hz = 100000;
  return i2c_master_bus_add_device(_bus, &devcfg, out);
}

esp_err_t DFRobot_RGBLCD1602::tx(i2c_master_dev_handle_t dev, const uint8_t* bytes, size_t n) {
  return i2c_master_transmit(dev, bytes, n, 1000);
}

void DFRobot_RGBLCD1602::init() {
  _showFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  begin(_cols, _rows);
}

void DFRobot_RGBLCD1602::begin(uint8_t rows, uint8_t charSize) {
  if (rows > 1) _showFunction |= LCD_2LINE;
  _numLines = rows;
  _currLine = 0;
  if ((charSize != 0) && (rows == 1)) _showFunction |= LCD_5x10DOTS;

  vTaskDelay(pdMS_TO_TICKS(50));
  command(LCD_FUNCTIONSET | _showFunction); vTaskDelay(pdMS_TO_TICKS(5));
  command(LCD_FUNCTIONSET | _showFunction); vTaskDelay(pdMS_TO_TICKS(5));
  command(LCD_FUNCTIONSET | _showFunction);

  _showControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  display();
  clear();

  _showMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  command(LCD_ENTRYMODESET | _showMode);

  rgb_init_sequence();
  setColorWhite();
}

void DFRobot_RGBLCD1602::rgb_init_sequence() {
  // Try to roughly mirror original init behavior across variants
  if (_RGBAddr == (0xc0>>1)) {
    // legacy branch (kept for completeness)
    setReg(REG_MODE1, 0x00);
    setReg(REG_OUTPUT, 0xFF);
    setReg(REG_MODE2, 0x20);
  } else if (_RGBAddr == (0x60>>1)) {
    setReg(0x01, 0x00);
    setReg(0x02, 0xFF);
    setReg(0x04, 0x15);
  } else if (_RGBAddr == 0x6B) {
    setReg(0x2F, 0x00);
    setReg(0x00, 0x20);
    setReg(0x01, 0x00);
    setReg(0x02, 0x01);
    setReg(0x03, 0x04);
  } else {
    // Common PCA9633 @ 0x62 or similar
    setReg(REG_MODE1, 0x00);
    setReg(REG_MODE2, 0x00);
    setReg(REG_OUTPUT, 0xAA); // individual PWM control
  }
}

void DFRobot_RGBLCD1602::clear() {
  command(LCD_CLEARDISPLAY);
  vTaskDelay(pdMS_TO_TICKS(2));
}
void DFRobot_RGBLCD1602::home()  {
  command(LCD_RETURNHOME);
  vTaskDelay(pdMS_TO_TICKS(2));
}

void DFRobot_RGBLCD1602::noDisplay(){ _showControl &= ~LCD_DISPLAYON; command(LCD_DISPLAYCONTROL | _showControl); }
void DFRobot_RGBLCD1602::display(){   _showControl |=  LCD_DISPLAYON; command(LCD_DISPLAYCONTROL | _showControl); }
void DFRobot_RGBLCD1602::stopBlink(){ _showControl &= ~LCD_BLINKON;   command(LCD_DISPLAYCONTROL | _showControl); }
void DFRobot_RGBLCD1602::blink(){     _showControl |=  LCD_BLINKON;   command(LCD_DISPLAYCONTROL | _showControl); }
void DFRobot_RGBLCD1602::noCursor(){  _showControl &= ~LCD_CURSORON;  command(LCD_DISPLAYCONTROL | _showControl); }
void DFRobot_RGBLCD1602::cursor(){    _showControl |=  LCD_CURSORON;  command(LCD_DISPLAYCONTROL | _showControl); }

void DFRobot_RGBLCD1602::scrollDisplayLeft()  { command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT); }
void DFRobot_RGBLCD1602::scrollDisplayRight() { command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT); }

void DFRobot_RGBLCD1602::leftToRight(){
  _showMode |= LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _showMode);
}
void DFRobot_RGBLCD1602::rightToLeft(){
  _showMode &= ~LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _showMode);
}
void DFRobot_RGBLCD1602::noAutoscroll(){
  _showMode &= ~LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _showMode);
}
void DFRobot_RGBLCD1602::autoscroll(){
  _showMode |= LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _showMode);
}

void DFRobot_RGBLCD1602::customSymbol(uint8_t location, uint8_t charmap[]) {
  location &= 0x7;
  command(LCD_SETCGRAMADDR | (location << 3));
  uint8_t buf[9]; buf[0]=CTRL_DATA;
  for(int i=0;i<8;i++) buf[1+i]=charmap[i];
  tx(_lcd, buf, 9);
}


void DFRobot_RGBLCD1602::setCursor(uint8_t col, uint8_t row) {
  uint8_t base = row ? 0x40 : 0x00;
  uint8_t ddram = base + col;
  uint8_t buf[2] = { CTRL_CMD, (uint8_t)(LCD_SETDDRAMADDR | ddram) };
  tx(_lcd, buf, 2);
  vTaskDelay(pdMS_TO_TICKS(1));
}

void DFRobot_RGBLCD1602::setRGB(uint8_t r, uint8_t g, uint8_t b) {
  // Match Arduino lib’s scaling branches
  if (_RGBAddr == (0x60>>1)) {
    uint16_t tr = (uint16_t)r*192/255;
    uint16_t tg = (uint16_t)g*192/255;
    uint16_t tb = (uint16_t)b*192/255;
    setReg(REG_RED, tr);
    setReg(REG_GREEN, tg);
    setReg(REG_BLUE, tb);
  } else {
    setReg(REG_RED,   r);
    setReg(REG_GREEN, g);
    setReg(REG_BLUE,  b);
    if (_RGBAddr == 0x6B) setReg(0x07, 0xFF);
  }
}

void DFRobot_RGBLCD1602::setColor(uint8_t color) {
  if (color > 3) return;
  setRGB(COLOR_DEFINE[color][0], COLOR_DEFINE[color][1], COLOR_DEFINE[color][2]);
}

size_t DFRobot_RGBLCD1602::write(uint8_t value) {
  uint8_t buf[2] = { CTRL_DATA, value };
  tx(_lcd, buf, 2);
  return 1;
}

size_t DFRobot_RGBLCD1602::printstr(const char* s) {
  size_t n=0; 
  if (!s) return 0;
  while (*s) { n += write((uint8_t)*s++); }
  return n;
}

void DFRobot_RGBLCD1602::command(uint8_t value) {
  uint8_t buf[2] = { CTRL_CMD, value };
  tx(_lcd, buf, 2);
}

void DFRobot_RGBLCD1602::send(uint8_t *data, uint8_t len) {
  // For compatibility with original code paths that prepended CTRL byte already
  tx(_lcd, data, len);
}

void DFRobot_RGBLCD1602::setReg(uint8_t addr, uint8_t data) {
  uint8_t buf[2] = { addr, data };
  tx(_rgb, buf, 2);
}