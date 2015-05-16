#pragma once

#define OLED1_address         0x3C  // OLED at address 0x3C in 7bit
#define OLED2_address         0x3D  // OLED at address 0x3D in 7bit

extern uint8_t OLED_Type;           // OLED Type 1, 2

void i2c_clear_OLED(void);
void i2c_clr_row(uint8_t row);
bool i2c_OLED_init(void);
bool initI2cLCD(bool cli);
void i2cOSD(void);
void i2c_OLED_send_char(unsigned char ascii);
void OLED_Status(void);

#define SSD1306_LCDWIDTH    128
#define SSD1306_LCDHEIGHT   64

#define SSD1306_SETCONTRAST                             0x81
#define SSD1306_DISPLAYALLON_RESUME                     0xA4
#define SSD1306_DISPLAYALLON                            0xA5
#define SSD1306_NORMALDISPLAY                           0xA6
#define SSD1306_INVERTDISPLAY                           0xA7
#define SSD1306_DISPLAYOFF                              0xAE
#define SSD1306_DISPLAYON                               0xAF

#define SSD1306_SETDISPLAYOFFSET                        0xD3
#define SSD1306_SETCOMPINS                              0xDA

#define SSD1306_SETVCOMDETECT                           0xDB

#define SSD1306_SETDISPLAYCLOCKDIV                      0xD5
#define SSD1306_SETPRECHARGE                            0xD9

#define SSD1306_SETMULTIPLEX                            0xA8

#define SSD1306_SETLOWCOLUMN                            0x00
#define SSD1306_SETHIGHCOLUMN                           0x10

#define SSD1306_SETSTARTLINE                            0x40

#define SSD1306_MEMORYMODE                              0x20

#define SSD1306_COMSCANINC                              0xC0
#define SSD1306_COMSCANDEC                              0xC8

#define SSD1306_SEGREMAP                                0xA0

#define SSD1306_CHARGEPUMP                              0x8D

#define SSD1306_EXTERNALVCC                             0x01
#define SSD1306_SWITCHCAPVCC                            0x02

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL                         0x2F
#define SSD1306_DEACTIVATE_SCROLL                       0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA                0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL                 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL                  0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL    0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL     0x2A
