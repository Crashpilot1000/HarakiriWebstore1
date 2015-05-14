// ************************************************************************************************************
// LCD & display & monitoring
// Code for an OLED-Display 128x64 from Wide HK
// ************************************************************************************************************
// based of MultiWii, June 2013 Johannes

#include "board.h"
#include "mw.h"

extern char * const mixerNames[];    // to display the current mixer
static uint8_t curOLED_address;      // OLED address of the currentOLED
uint8_t OLED_Type;                   // 1, 2

void i2c_OLED_set_line(uint8_t row);

bool i2cLCD;                         // true, if an OLED-Display is connected

const uint8_t myFont[][5] = {
// Refer to "Times New Roman" Font Database... 5 x 7 font
// ASCII control characters (character code 0-31 = 0x00-0x1F) OBVIOUSLY NOT IN FONT
// ASCII printable characters (character code 32-127 = 0x20-0x7F)   
        { 0x00,0x00,0x00,0x00,0x00}, //   (  0)    - 0x20 <SPACE>
        { 0x00,0x00,0x4F,0x00,0x00}, //   (  1)  ! - 0x21 Exclamation Mark
        { 0x00,0x07,0x00,0x07,0x00}, //   (  2)  " - 0x22 Quotation Mark
        { 0x14,0x7F,0x14,0x7F,0x14}, //   (  3)  # - 0x23 Number Sign
        { 0x24,0x2A,0x7F,0x2A,0x12}, //   (  4)  $ - 0x24 Dollar Sign
        { 0x23,0x13,0x08,0x64,0x62}, //   (  5)  % - 0x25 Percent Sign
        { 0x36,0x49,0x55,0x22,0x50}, //   (  6)  & - 0x26 Ampersand
        { 0x00,0x05,0x03,0x00,0x00}, //   (  7)  ' - 0x27 Apostrophe
        { 0x00,0x1C,0x22,0x41,0x00}, //   (  8)  ( - 0x28 Left Parenthesis
        { 0x00,0x41,0x22,0x1C,0x00}, //   (  9)  ) - 0x29 Right Parenthesis
        { 0x14,0x08,0x3E,0x08,0x14}, //   ( 10)  * - 0x2A Asterisk
        { 0x08,0x08,0x3E,0x08,0x08}, //   ( 11)  + - 0x2B Plus Sign
        { 0x00,0x50,0x30,0x00,0x00}, //   ( 12)  , - 0x2C Comma
        { 0x08,0x08,0x08,0x08,0x08}, //   ( 13)  - - 0x2D Hyphen-Minus
        { 0x00,0x60,0x60,0x00,0x00}, //   ( 14)  . - 0x2E Full Stop
        { 0x20,0x10,0x08,0x04,0x02}, //   ( 15)  / - 0x2F Solidus
        { 0x3E,0x51,0x49,0x45,0x3E}, //   ( 16)  0 - 0x30 Digit Zero
        { 0x00,0x42,0x7F,0x40,0x00}, //   ( 17)  1 - 0x31 Digit One
        { 0x42,0x61,0x51,0x49,0x46}, //   ( 18)  2 - 0x32 Digit Two
        { 0x21,0x41,0x45,0x4B,0x31}, //   ( 19)  3 - 0x33 Digit Three
        { 0x18,0x14,0x12,0x7F,0x10}, //   ( 20)  4 - 0x34 Digit Four
        { 0x27,0x45,0x45,0x45,0x39}, //   ( 21)  5 - 0x35 Digit Five
        { 0x3C,0x4A,0x49,0x49,0x30}, //   ( 22)  6 - 0x36 Digit Six
        { 0x01,0x71,0x09,0x05,0x03}, //   ( 23)  7 - 0x37 Digit Seven
        { 0x36,0x49,0x49,0x49,0x36}, //   ( 24)  8 - 0x38 Digit Eight
        { 0x06,0x49,0x49,0x29,0x1E}, //   ( 25)  9 - 0x39 Dight Nine
        { 0x00,0x36,0x36,0x00,0x00}, //   ( 26)  : - 0x3A Colon
        { 0x00,0x56,0x36,0x00,0x00}, //   ( 27)  ; - 0x3B Semicolon
        { 0x08,0x14,0x22,0x41,0x00}, //   ( 28)  < - 0x3C Less-Than Sign
        { 0x14,0x14,0x14,0x14,0x14}, //   ( 29)  = - 0x3D Equals Sign
        { 0x00,0x41,0x22,0x14,0x08}, //   ( 30)  > - 0x3E Greater-Than Sign
        { 0x02,0x01,0x51,0x09,0x06}, //   ( 31)  ? - 0x3F Question Mark
        { 0x32,0x49,0x79,0x41,0x3E}, //   ( 32)  @ - 0x40 Commercial At
        { 0x7E,0x11,0x11,0x11,0x7E}, //   ( 33)  A - 0x41 Latin Capital Letter A
        { 0x7F,0x49,0x49,0x49,0x36}, //   ( 34)  B - 0x42 Latin Capital Letter B
        { 0x3E,0x41,0x41,0x41,0x22}, //   ( 35)  C - 0x43 Latin Capital Letter C
        { 0x7F,0x41,0x41,0x22,0x1C}, //   ( 36)  D - 0x44 Latin Capital Letter D
        { 0x7F,0x49,0x49,0x49,0x41}, //   ( 37)  E - 0x45 Latin Capital Letter E
        { 0x7F,0x09,0x09,0x09,0x01}, //   ( 38)  F - 0x46 Latin Capital Letter F
        { 0x3E,0x41,0x49,0x49,0x7A}, //   ( 39)  G - 0x47 Latin Capital Letter G
        { 0x7F,0x08,0x08,0x08,0x7F}, //   ( 40)  H - 0x48 Latin Capital Letter H
        { 0x00,0x41,0x7F,0x41,0x00}, //   ( 41)  I - 0x49 Latin Capital Letter I
        { 0x20,0x40,0x41,0x3F,0x01}, //   ( 42)  J - 0x4A Latin Capital Letter J
        { 0x7F,0x08,0x14,0x22,0x41}, //   ( 43)  K - 0x4B Latin Capital Letter K
        { 0x7F,0x40,0x40,0x40,0x40}, //   ( 44)  L - 0x4C Latin Capital Letter L
        { 0x7F,0x02,0x0C,0x02,0x7F}, //   ( 45)  M - 0x4D Latin Capital Letter M
        { 0x7F,0x04,0x08,0x10,0x7F}, //   ( 46)  N - 0x4E Latin Capital Letter N
        { 0x3E,0x41,0x41,0x41,0x3E}, //   ( 47)  O - 0x4F Latin Capital Letter O
        { 0x7F,0x09,0x09,0x09,0x06}, //   ( 48)  P - 0x50 Latin Capital Letter P
        { 0x3E,0x41,0x51,0x21,0x5E}, //   ( 49)  Q - 0x51 Latin Capital Letter Q
        { 0x7F,0x09,0x19,0x29,0x46}, //   ( 50)  R - 0x52 Latin Capital Letter R
        { 0x46,0x49,0x49,0x49,0x31}, //   ( 51)  S - 0x53 Latin Capital Letter S
        { 0x01,0x01,0x7F,0x01,0x01}, //   ( 52)  T - 0x54 Latin Capital Letter T
        { 0x3F,0x40,0x40,0x40,0x3F}, //   ( 53)  U - 0x55 Latin Capital Letter U
        { 0x1F,0x20,0x40,0x20,0x1F}, //   ( 54)  V - 0x56 Latin Capital Letter V
        { 0x3F,0x40,0x38,0x40,0x3F}, //   ( 55)  W - 0x57 Latin Capital Letter W
        { 0x63,0x14,0x08,0x14,0x63}, //   ( 56)  X - 0x58 Latin Capital Letter X
        { 0x07,0x08,0x70,0x08,0x07}, //   ( 57)  Y - 0x59 Latin Capital Letter Y
        { 0x61,0x51,0x49,0x45,0x43}, //   ( 58)  Z - 0x5A Latin Capital Letter Z
        { 0x00,0x7F,0x41,0x41,0x00}, //   ( 59)  [ - 0x5B Left Square Bracket
        { 0x02,0x04,0x08,0x10,0x20}, //   ( 60)  \ - 0x5C Reverse Solidus
        { 0x00,0x41,0x41,0x7F,0x00}, //   ( 61)  ] - 0x5D Right Square Bracket
        { 0x04,0x02,0x01,0x02,0x04}, //   ( 62)  ^ - 0x5E Circumflex Accent
        { 0x40,0x40,0x40,0x40,0x40}, //   ( 63)  _ - 0x5F Low Line
        { 0x01,0x02,0x04,0x00,0x00}, //   ( 64)  ` - 0x60 Grave Accent
        { 0x20,0x54,0x54,0x54,0x78}, //   ( 65)  a - 0x61 Latin Small Letter A
        { 0x7F,0x48,0x44,0x44,0x38}, //   ( 66)  b - 0x62 Latin Small Letter B
        { 0x38,0x44,0x44,0x44,0x20}, //   ( 67)  c - 0x63 Latin Small Letter C
        { 0x38,0x44,0x44,0x48,0x7F}, //   ( 68)  d - 0x64 Latin Small Letter D
        { 0x38,0x54,0x54,0x54,0x18}, //   ( 69)  e - 0x65 Latin Small Letter E
        { 0x08,0x7E,0x09,0x01,0x02}, //   ( 70)  f - 0x66 Latin Small Letter F
        { 0x06,0x49,0x49,0x49,0x3F}, //   ( 71)  g - 0x67 Latin Small Letter G
        { 0x7F,0x08,0x04,0x04,0x78}, //   ( 72)  h - 0x68 Latin Small Letter H
        { 0x00,0x44,0x7D,0x40,0x00}, //   ( 73)  i - 0x69 Latin Small Letter I
        { 0x20,0x40,0x44,0x3D,0x00}, //   ( 74)  j - 0x6A Latin Small Letter J
        { 0x7F,0x10,0x28,0x44,0x00}, //   ( 75)  k - 0x6B Latin Small Letter K
        { 0x00,0x41,0x7F,0x40,0x00}, //   ( 76)  l - 0x6C Latin Small Letter L
        { 0x7C,0x04,0x18,0x04,0x7C}, //   ( 77)  m - 0x6D Latin Small Letter M
        { 0x7C,0x08,0x04,0x04,0x78}, //   ( 78)  n - 0x6E Latin Small Letter N
        { 0x38,0x44,0x44,0x44,0x38}, //   ( 79)  o - 0x6F Latin Small Letter O
        { 0x7C,0x14,0x14,0x14,0x08}, //   ( 80)  p - 0x70 Latin Small Letter P
        { 0x08,0x14,0x14,0x18,0x7C}, //   ( 81)  q - 0x71 Latin Small Letter Q
        { 0x7C,0x08,0x04,0x04,0x08}, //   ( 82)  r - 0x72 Latin Small Letter R
        { 0x48,0x54,0x54,0x54,0x20}, //   ( 83)  s - 0x73 Latin Small Letter S
        { 0x04,0x3F,0x44,0x40,0x20}, //   ( 84)  t - 0x74 Latin Small Letter T
        { 0x3C,0x40,0x40,0x20,0x7C}, //   ( 85)  u - 0x75 Latin Small Letter U
        { 0x1C,0x20,0x40,0x20,0x1C}, //   ( 86)  v - 0x76 Latin Small Letter V
        { 0x3C,0x40,0x30,0x40,0x3C}, //   ( 87)  w - 0x77 Latin Small Letter W
        { 0x44,0x28,0x10,0x28,0x44}, //   ( 88)  x - 0x78 Latin Small Letter X
        { 0x0C,0x50,0x50,0x50,0x3C}, //   ( 89)  y - 0x79 Latin Small Letter Y
        { 0x44,0x64,0x54,0x4C,0x44}, //   ( 90)  z - 0x7A Latin Small Letter Z
        { 0x00,0x08,0x36,0x41,0x00}, //   ( 91)  { - 0x7B Left Curly Bracket
        { 0x00,0x00,0x7F,0x00,0x00}, //   ( 92)  | - 0x7C Vertical Line
        { 0x00,0x41,0x36,0x08,0x00}, //   ( 93)  } - 0x7D Right Curly Bracket
        { 0x02,0x01,0x02,0x04,0x02}, //   ( 94)  ~ - 0x7E Tilde
        
// The extended ASCII codes (character code 128-255 = 0x80-0xFF) EXCLUDED FOR NOW
/*
        { 0x3E,0x55,0x55,0x41,0x22}, //   ( 95)  C - 0x80 <Control>
        { 0x00,0x00,0x00,0x00,0x00}, //   ( 96)    - 0xA0 No-Break Space
        { 0x00,0x00,0x79,0x00,0x00}, //   ( 97)  ! - 0xA1 Inverted Exclamation Mark
        { 0x18,0x24,0x74,0x2E,0x24}, //   ( 98)  c - 0xA2 Cent Sign
        { 0x48,0x7E,0x49,0x42,0x40}, //   ( 99)  L - 0xA3 Pound Sign
        { 0x5D,0x22,0x22,0x22,0x5D}, //   (100)  o - 0xA4 Currency Sign
        { 0x15,0x16,0x7C,0x16,0x15}, //   (101)  Y - 0xA5 Yen Sign
        { 0x00,0x00,0x77,0x00,0x00}, //   (102)  | - 0xA6 Broken Bar
        { 0x0A,0x55,0x55,0x55,0x28}, //   (103)    - 0xA7 Section Sign
        { 0x00,0x01,0x00,0x01,0x00}, //   (104)  " - 0xA8 Diaeresis
        { 0x00,0x0A,0x0D,0x0A,0x04}, //   (105)    - 0xAA Feminine Ordinal Indicator
        { 0x08,0x14,0x2A,0x14,0x22}, //   (106) << - 0xAB Left-Pointing Double Angle Quotation Mark
        { 0x04,0x04,0x04,0x04,0x1C}, //   (107)    - 0xAC Not Sign
        { 0x00,0x08,0x08,0x08,0x00}, //   (108)  - - 0xAD Soft Hyphen
        { 0x01,0x01,0x01,0x01,0x01}, //   (109)    - 0xAF Macron
        { 0x00,0x02,0x05,0x02,0x00}, //   (110)    - 0xB0 Degree Sign
        { 0x44,0x44,0x5F,0x44,0x44}, //   (111) +- - 0xB1 Plus-Minus Sign
        { 0x00,0x00,0x04,0x02,0x01}, //   (112)  ` - 0xB4 Acute Accent
        { 0x7E,0x20,0x20,0x10,0x3E}, //   (113)  u - 0xB5 Micro Sign
        { 0x06,0x0F,0x7F,0x00,0x7F}, //   (114)    - 0xB6 Pilcrow Sign
        { 0x00,0x18,0x18,0x00,0x00}, //   (115)  . - 0xB7 Middle Dot
        { 0x00,0x40,0x50,0x20,0x00}, //   (116)    - 0xB8 Cedilla
        { 0x00,0x0A,0x0D,0x0A,0x00}, //   (117)    - 0xBA Masculine Ordinal Indicator
        { 0x22,0x14,0x2A,0x14,0x08}, //   (118) >> - 0xBB Right-Pointing Double Angle Quotation Mark
        { 0x17,0x08,0x34,0x2A,0x7D}, //   (119) /4 - 0xBC Vulgar Fraction One Quarter
        { 0x17,0x08,0x04,0x6A,0x59}, //   (120) /2 - 0xBD Vulgar Fraction One Half
        { 0x30,0x48,0x45,0x40,0x20}, //   (121)  ? - 0xBE Inverted Question Mark
        { 0x08,0x08,0x08,0x08,0x08}, //   (122)    - 0xBF Bargraph - 0
        { 0x7E,0x08,0x08,0x08,0x08}, //   (123)    - 0xBF Bargraph - 1
        { 0x7E,0x7E,0x08,0x08,0x08}, //   (124)    - 0xBF Bargraph - 2
        { 0x7E,0x7E,0x7E,0x08,0x08}, //   (125)    - 0xBF Bargraph - 3
        { 0x7E,0x7E,0x7E,0x7E,0x08}, //   (126)    - 0xBF Bargraph - 4
        { 0x7E,0x7E,0x7E,0x7E,0x7E}, //   (127)    - 0xBF Bargraph - 5
*/        
};
#define FONTDATASETCOUNT (sizeof(myFont) / sizeof(myFont[0]))

unsigned char digit10000(uint16_t v)
{
    return '0' + v / 10000;
}

unsigned char digit1000(uint16_t v)
{
    return '0' + v / 1000 - (v / 10000) * 10;
}

unsigned char digit100(uint16_t v)
{
    return '0' + v / 100 - (v / 1000) * 10;
}

unsigned char digit10(uint16_t v)
{
    return '0' + v / 10 - (v / 100) * 10;
}

unsigned char digit1(uint16_t v)
{
    return '0' + v - (v / 10) * 10;
}

void i2c_OLED_send_cmd(uint8_t command)
{
    uint8_t  hexval = 0;
    if(!OLED_Type) return;
    if(OLED_Type == 1) hexval = 0x80;
    i2cWrite(curOLED_address, hexval, (uint8_t)command);
}

void i2c_OLED_send_byte(uint8_t val)
{
    if(!OLED_Type) return;
    i2cWrite(curOLED_address, 0x40, (uint8_t)val);
}

void i2c_OLED_send_char(unsigned char ascii)
{
    uint16_t i, offset = constrain_int(ascii - 32, 0, FONTDATASETCOUNT - 1); // Ensure Range to not exceed fontarray
    for( i = 0; i < 5; i++) i2c_OLED_send_byte(myFont[offset][i]);
    i2c_OLED_send_byte(0);                                      // Don't know why this is here ask the OLED-guys.
}

void i2c_OLED_LCDprint(uint8_t i)
{
    i2c_OLED_send_char(i);
}

void i2c_OLED_LCDprintChar(const char *s)
{
    while (*s) {i2c_OLED_LCDprint(*s++);}
}

void i2c_clear_OLED(void)
{
    uint16_t i;
    if(!OLED_Type) return;
    i2c_OLED_send_cmd(0xa6);                                    // Set Normal Display
    i2c_OLED_send_cmd(0xae);                                    // Display OFF
    i2c_OLED_send_cmd(0x20);                                    // Set Memory Addressing Mode
    i2c_OLED_send_cmd(0x00);                                    // Set Memory Addressing Mode to Horizontal addressing mode
    i2c_OLED_send_cmd(0xb0);                                    // set page address to 0
    i2c_OLED_send_cmd(0X40);                                    // Display start line register to 0
    i2c_OLED_send_cmd(0);                                       // Set low col address to 0
    i2c_OLED_send_cmd(0x10);                                    // Set high col address to 0
    for(i = 0; i < 1024; i++) i2c_OLED_send_byte(0);            // fill the display's RAM with 0 128 * 64pixl
    i2c_OLED_send_cmd(0x81);                                    // Setup CONTRAST CONTROL, following byte is the contrast Value... always a 2 byte instruction
    i2c_OLED_send_cmd(200);                                     // Here you can set the brightness 1 = dull, 255 is very bright
    i2c_OLED_send_cmd(0xaf);                                    // display on
}

bool i2c_OLED_init(void)
{
    uint8_t dummy;
    if (i2cRead(OLED1_address, 0xA0, 1, &dummy))
    {
        curOLED_address = OLED1_address;
        OLED_Type = 1;
    }
    else
    {
        if (i2cRead(OLED2_address, 0xA0, 1, &dummy))
        {
            curOLED_address = OLED2_address;
            OLED_Type = 2;
        }
        else
        {
            OLED_Type = 0;
            return false;
        }
    }
    // Init sequence for 128x64 OLED module
    i2c_OLED_send_cmd(SSD1306_DISPLAYOFF);                      // 0xAE
    i2c_OLED_send_cmd(SSD1306_SETDISPLAYCLOCKDIV);              // 0xD5
    i2c_OLED_send_cmd(0x80);                                    // the suggested ratio 0x80
    i2c_OLED_send_cmd(SSD1306_SETMULTIPLEX);                    // 0xA8
    i2c_OLED_send_cmd(0x3F);
    i2c_OLED_send_cmd(SSD1306_SETDISPLAYOFFSET);                // 0xD3
    i2c_OLED_send_cmd(0x0);                                     // no offset
    i2c_OLED_send_cmd(SSD1306_SETSTARTLINE | 0x0);              // line #0
    i2c_OLED_send_cmd(SSD1306_CHARGEPUMP);                      // 0x8D
    i2c_OLED_send_cmd(0x14);                                    // vccstate != SSD1306_EXTERNALVCC)
    i2c_OLED_send_cmd(SSD1306_MEMORYMODE);                      // 0x20
    i2c_OLED_send_cmd(0x00);                                    // 0x0 act like ks0108
    i2c_OLED_send_cmd(SSD1306_SEGREMAP | 0x1);
    i2c_OLED_send_cmd(SSD1306_COMSCANDEC);
    i2c_OLED_send_cmd(SSD1306_SETCOMPINS);                      // 0xDA
    i2c_OLED_send_cmd(0x12);
    i2c_OLED_send_cmd(SSD1306_SETCONTRAST);                     // 0x81
    i2c_OLED_send_cmd(0xCF);                                    //vccstate != SSD1306_EXTERNALVCC)
    i2c_OLED_send_cmd(SSD1306_SETPRECHARGE);                    // 0xd9
    i2c_OLED_send_cmd(0xF1);                                    //vccstate != SSD1306_EXTERNALVCC)
    i2c_OLED_send_cmd(SSD1306_SETVCOMDETECT);                   // 0xDB
    i2c_OLED_send_cmd(0x40);
    i2c_OLED_send_cmd(SSD1306_DISPLAYALLON_RESUME);             // 0xA4
    i2c_OLED_send_cmd(SSD1306_NORMALDISPLAY);                   // 0xA6
    delay(20);
    i2c_OLED_set_line(1);
    return true;
}

void i2c_OLED_set_XY(uint8_t col, uint8_t row)                  //  Not used in MW V2.0 but its here anyway!
{
    if(!OLED_Type) return;
    i2c_OLED_send_cmd(0xb0 + row);                              // set page address
    i2c_OLED_send_cmd(0x00 + (8 * col & 0x0f));                 // set low col address
    i2c_OLED_send_cmd(0x10 + ((8 * col >> 4) & 0x0f));          // set high col address
}

void i2c_OLED_set_line(uint8_t row)                             // goto the beginning of a single row, compattible with LCD_CONFIG
{
    if(!OLED_Type) return;
    i2c_OLED_send_cmd(0xb0 + row);                              // set page address
    i2c_OLED_send_cmd(0);                                       // set low col address
    i2c_OLED_send_cmd(0x10);                                    // set high col address
}

void i2c_OLED_LCDsetLine(uint8_t line)                          // Line = 1 to 8
{
    if(!OLED_Type) return;
    i2c_OLED_set_line(line - 1);
}

bool initI2cLCD(bool cli)
{
    i2cLCD = false;
    if (i2c_OLED_init())
    {
        if (cli) i2cLCD = true;                                 // all printf output now to the OLED-Display
        i2c_clear_OLED();
        i2c_OLED_LCDsetLine(1); i2c_OLED_LCDprintChar(FIRMWAREFORLCD);
        i2c_OLED_LCDsetLine(3); i2c_OLED_LCDprintChar("To ENTER CONFIG      ");
        i2c_OLED_LCDsetLine(4); i2c_OLED_LCDprintChar("PITCH FWD & YAW RIGHT");
        return true;
    }
    return false;
}

void i2c_clr_line(uint8_t line)
{
    if(!OLED_Type) return;
    i2c_OLED_LCDsetLine(line);
    printf("                      ");
    //      1234567890123456789012
    i2c_OLED_LCDsetLine(line);
}

void OLED_Status(void)
{
    static uint8_t OLEDDelay = 0;
    char line[22], smag[5];
    uint16_t tmp0;

    OLEDDelay++;
    if (OLEDDelay >= 30)
    {
        OLEDDelay = 0;
        sprintf(line, "MAG : WARN    ", (int16_t)heading);
        if (cfg.mag_calibrated)
        {
            sprintf(smag, "%4d", (int16_t)heading);
            line[6] = smag[0];
            line[7] = smag[1];
            line[8] = smag[2];
            line[9] = smag[3];
        }
        i2c_OLED_LCDsetLine(1); i2c_OLED_LCDprintChar(line);
        sprintf(line, "VBAT: --,-V AGL: ----");
        if (FEATURE_VBAT)
        {
            line[6] = digit100(vbat);
            line[7] = digit10(vbat);
            line[9] = digit1(vbat);
        }
        if (EstAlt < 0) line[16] = '-';
        tmp0     = (int16_t)abs_int((int32_t)EstAlt / 100);
        line[17] = digit10000(tmp0);
        line[18] = digit1000(tmp0);
        line[19] = digit100(tmp0);
        line[20] = digit10(tmp0);
        i2c_OLED_LCDsetLine(2);
        i2c_OLED_LCDprintChar(line);
        
        sprintf(line, "LAT :  .-+-.-------  ");
        if (FEATURE_GPS)
        {
            line[6]  = Real_GPS_coord[LAT] < 0 ? 'S' : 'N';
            line[8]  = '0' + Real_GPS_coord[LAT]  / 1000000000;
            line[9]  = '0' + Real_GPS_coord[LAT]  / 100000000 - (Real_GPS_coord[LAT] / 1000000000) * 10;
            line[10] = '0' + Real_GPS_coord[LAT]  / 10000000  - (Real_GPS_coord[LAT] / 100000000)  * 10;
            line[12] = '0' + Real_GPS_coord[LAT]  / 1000000   - (Real_GPS_coord[LAT] / 10000000)   * 10;
            line[13] = '0' + Real_GPS_coord[LAT]  / 100000    - (Real_GPS_coord[LAT] / 1000000)    * 10;
            line[14] = '0' + Real_GPS_coord[LAT]  / 10000     - (Real_GPS_coord[LAT] / 100000)     * 10;
            line[15] = '0' + Real_GPS_coord[LAT]  / 1000      - (Real_GPS_coord[LAT] / 10000)      * 10;
            line[16] = '0' + Real_GPS_coord[LAT]  / 100       - (Real_GPS_coord[LAT] / 1000)       * 10;
            line[17] = '0' + Real_GPS_coord[LAT]  / 10        - (Real_GPS_coord[LAT] / 100)        * 10;
            line[18] = '0' + Real_GPS_coord[LAT]              - (Real_GPS_coord[LAT] / 10)         * 10;
        }          
        i2c_OLED_LCDsetLine(3);
        i2c_OLED_LCDprintChar(line);
          
        sprintf(line, "LON :  .-+-.-------  ");
        if (FEATURE_GPS)
        {
            line[6]  = Real_GPS_coord[LON] < 0 ? 'W' : 'E';
            line[8]  = '0' + Real_GPS_coord[LON]  / 1000000000;
            line[9]  = '0' + Real_GPS_coord[LON]  / 100000000 - (Real_GPS_coord[LON] / 1000000000) * 10;
            line[10] = '0' + Real_GPS_coord[LON]  / 10000000  - (Real_GPS_coord[LON] / 100000000)  * 10;
            line[12] = '0' + Real_GPS_coord[LON]  / 1000000   - (Real_GPS_coord[LON] / 10000000)   * 10;
            line[13] = '0' + Real_GPS_coord[LON]  / 100000    - (Real_GPS_coord[LON] / 1000000)    * 10;
            line[14] = '0' + Real_GPS_coord[LON]  / 10000     - (Real_GPS_coord[LON] / 100000)     * 10;
            line[15] = '0' + Real_GPS_coord[LON]  / 1000      - (Real_GPS_coord[LON] / 10000)      * 10;
            line[16] = '0' + Real_GPS_coord[LON]  / 100       - (Real_GPS_coord[LON] / 1000)       * 10;
            line[17] = '0' + Real_GPS_coord[LON]  / 10        - (Real_GPS_coord[LON] / 100)        * 10;
            line[18] = '0' + Real_GPS_coord[LON]              - (Real_GPS_coord[LON] / 10)         * 10;
        }
        i2c_OLED_LCDsetLine(4);
        i2c_OLED_LCDprintChar(line);
        
        if (FEATURE_GPS) sprintf(line, "SAT : %d   FIX : %d  ", GPS_numSat, GPS_FIX);
        else             sprintf(line, "SAT : -    FIX : -   ");
        i2c_OLED_LCDsetLine(5);
        i2c_OLED_LCDprintChar(line);
    }
}
