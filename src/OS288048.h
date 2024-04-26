/*!
  @file OS288048.h

OS288048 for Adafruit_GFX

This is a library for OSRAM Pictiva OS288048 Grayscale OLEDs based on SSD0332
drivers

Code based on:
 * venice1200/SSD1322_for_Adafruit_GFX
 * adafruit/Adafruit_SSD1327
 * adafruit/Adafruit-GFX-Library/Adafruit_GrayOLED
modfied for an SSD0332 OLED with 288x64 Pixel.

OLED Interfaces: I2C, 3SPI

These displays use I2C or SPI to communicate.
I2C requires 2 pins (SCL+SDA) and optionally a RESET pin.
SPI requires 4 pins (MOSI, SCK, select, data/command) and optionally a reset
pin. Hardware SPI or 'bitbang' software SPI are both supported.

************************************************************

Osram Pictiva OLED 288x48
OS288048PQ33MY0C11
OS288048PQ33MG1C11
OS288048PQ33MO2C11

The Solomon Systech SSD0332 rgb OLED driver is limited to 96 pixel wide but
 using each R,G,B of the 96 pixels to get 288 pixel

-----------------------------------------------------------
Original Adafruit Header for the SSD1327 OLED (BSD License)
-----------------------------------------------------------

This is a library for our Grayscale OLEDs based on SSD1327 drivers
  Pick one up today in the adafruit shop!
  ------> https://www.adafruit.com/products/4741
These displays use I2C or SPI to communicate
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!
Written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution

*********************************************************************/

#ifndef OS288048_H
#define OS288048_H

#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>

#define MONOOLED_BLACK 0   ///< Default black 'color' for monochrome OLEDS
#define MONOOLED_WHITE 1   ///< Default white 'color' for monochrome OLEDS
#define MONOOLED_INVERSE 2 ///< Default inversion command for monochrome OLEDS

#define PICTIVA_BLACK MONOOLED_BLACK
#define PICTIVA_WHITE MONOOLED_WHITE

#define SSD0332_I2C_ADDRESS 0x3C
#define SSD0332_I2C_ADDRESS1 0x3D

// Fundamental Commands
#define SSD0332_SETCOLUMN 0x15         // Set Column Address
#define SSD0332_SETROW 0x75            // Set Row Address
#define SSD0332_CMD_CONTRASTA 0x81     // Set contrast for color A
#define SSD0332_CMD_CONTRASTB 0x82     // Set contrast for color B
#define SSD0332_CMD_CONTRASTC 0x83     // Set contrast for color C
#define SSD0332_CMD_MASTERCURRENT 0x87 // Master current control
#define SSD0332_CMD_SETREMAP 0xA0      // Set re-map & data format
#define SSD0332_CMD_STARTLINE 0xA1     // Set display start line
#define SSD0332_CMD_DISPLAYOFFSET 0xA2 // Set display offset
#define SSD0332_CMD_NORMALDISPLAY 0xA4 // Set display to normal mode
#define SSD0332_CMD_DISPLAYALLON 0xA5  // Set entire display ON
#define SSD0332_CMD_DISPLAYALLOFF 0xA6 // Set entire display OFF
#define SSD0332_CMD_INVERTDISPLAY 0xA7 // Invert display
#define SSD0332_CMD_SETMULTIPLEX 0xA8  // Set multiplex ratio
#define SSD0332_CMD_SETMASTER 0xAD     // Set master configuration
#define SSD0332_CMD_DISPLAYOFF 0xAE    // Display OFF (sleep mode)
#define SSD0332_CMD_DISPLAYON 0xAF     // Normal Brightness Display ON
#define SSD0332_CMD_POWERMODE 0xB0     // Power save mode
#define SSD0332_CMD_PRECHARGE 0xB1     // Phase 1 and 2 period adjustment
#define SSD0332_CMD_CLOCKDIV                                                   \
  0xB3 // Set display clock divider/oscillator frequency
#define SSD0332_CMD_SETGRAYSCALE 0xB8    // Set gray scale table
#define SSD0332_CMD_RESETGRAYSCALE 0xB9  // Enable Linear Gray Scale Table
#define SSD0332_CMD_PRECHARGELEVELA 0xBB // Set pre-charge voltage for Color A
#define SSD0332_CMD_PRECHARGELEVELB 0xBC // Set pre-charge voltage for Color B
#define SSD0332_CMD_PRECHARGELEVELC 0xBD // Set pre-charge voltage for Color C
#define SSD0332_CMD_VCOMH 0xBE           // Set Vcomh voltage
#define SSD0332_CMD_NOP 0xE3             // Command for No Operation

// Graphic Commands
#define SSD0332_CMD_DRAWLINE 0x21 // Draw line
#define SSD0332_CMD_DRAWRECT 0x22 // Draw rectangle
#define SSD0332_CMD_COPY 0x23     // Copy
#define SSD0332_CMD_DIM 0x24      // Dim Window
#define SSD0332_CMD_CLEAR 0x25    // Clear Window
#define SSD0332_CMD_FILL 0x26     // Fill enable/disable

/*!
    @brief  The controller object for OS288048 grayscale OLED displays
*/
class OS288048 : public Adafruit_GFX {
public:
  OS288048(uint16_t w, uint16_t h, TwoWire *twi = &Wire, int8_t rst_pin = -1,
           uint32_t preclk = 400000, uint32_t postclk = 100000);
  OS288048(uint16_t w, uint16_t h, int8_t mosi_pin, int8_t sclk_pin,
           int8_t dc_pin, int8_t rst_pin, int8_t cs_pin);
  OS288048(uint16_t w, uint16_t h, SPIClass *spi, int8_t dc_pin, int8_t rst_pin,
           int8_t cs_pin, uint32_t bitrate = 16000000UL);
  OS288048(uint8_t bpp, uint16_t w, uint16_t h, TwoWire *twi = &Wire,
           int8_t rst_pin = -1, uint32_t preclk = 400000,
           uint32_t postclk = 100000);
  OS288048(uint8_t bpp, uint16_t w, uint16_t h, int8_t mosi_pin,
           int8_t sclk_pin, int8_t dc_pin, int8_t rst_pin, int8_t cs_pin);
  OS288048(uint8_t bpp, uint16_t w, uint16_t h, SPIClass *spi, int8_t dc_pin,
           int8_t rst_pin, int8_t cs_pin, uint32_t bitrate = 16000000UL);

  ~OS288048(void);

  bool begin(uint8_t i2caddr = SSD0332_I2C_ADDRESS, bool reset = true);
  GFXfont *getFont();
  //  void draw4bppBitmap(const uint8_t bitmap[]);
  //  void draw4bppBitmap(uint8_t *bitmap);
  void display();
  void clearDisplay(void);
  void invertDisplay(bool i);
  void displayOff();
  void displayOn();
  void allPixelOff();
  void allPixelOn();
  void setContrast(uint8_t level);
  void verticalScroll(uint8_t val);
  void setBrightness(uint8_t val);
  void powerSave(bool mode);
  void clearArea(int x0, int y0, int x1, int y1);
  void dimWindow(int x0, int y0, int x1, int y1);
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  bool getPixel(int16_t x, int16_t y);
  uint8_t *getBuffer(void);

  void oled_command(uint8_t c);
  bool oled_commandList(const uint8_t *c, uint8_t n);

protected:
  bool _init(uint8_t i2caddr = 0x3C, bool reset = true);

  Adafruit_SPIDevice *spi_dev = NULL; ///< The SPI interface BusIO device
  Adafruit_I2CDevice *i2c_dev = NULL; ///< The I2C interface BusIO device
  int32_t i2c_preclk = 400000,        ///< Configurable 'high speed' I2C rate
      i2c_postclk = 100000;           ///< Configurable 'low speed' I2C rate
  uint8_t *buffer = NULL; ///< Internal 1:1 framebuffer of display mem

  int16_t window_x1, ///< Dirty tracking window minimum x
      window_y1,     ///< Dirty tracking window minimum y
      window_x2,     ///< Dirty tracking window maximum x
      window_y2;     ///< Dirty tracking window maximum y

  int dcPin,  ///< The Arduino pin connected to D/C (for SPI)
      csPin,  ///< The Arduino pin connected to CS (for SPI)
      rstPin; ///< The Arduino pin connected to reset (-1 if unused)

  uint8_t _bpp = 1; ///< Bits per pixel color for this display

private:
  TwoWire *_theWire = NULL; ///< The underlying hardware I2C
  int8_t page_offset = 0;
  int8_t column_offset = 0;
  inline void oled_i2c_data_command(uint8_t c);
  void oled_data(uint8_t c);
  void checkBounds(int &x0, int &y0, int &x1, int &y1, uint8_t &c1, uint8_t &c2,
                   uint8_t &sx0, uint8_t &sx1);
  void clearScreen(uint8_t r1 = 0, uint8_t c1 = 0, uint8_t r2 = 47,
                   uint8_t c2 = 95);
  void dimScreen(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2);
};

#endif // OS288048_H