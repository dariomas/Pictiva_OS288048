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

#include "OS288048.h"

// SOME DEFINES AND STATIC VARIABLES USED INTERNALLY -----------------------

#define grayoled_swap(a, b)                                                    \
  (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation

// CONSTRUCTORS, DESTRUCTOR ------------------------------------------------

/*!
    @brief  Constructor for I2C-interfaced OS288048 displays.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  twi
            Pointer to an existing TwoWire instance (e.g. &Wire, the
            microcontroller's primary I2C bus).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  clkDuring
            Speed (in Hz) for Wire transmissions in OS288048 library calls.
            Defaults to 400000 (400 KHz), a known 'safe' value for most
            microcontrollers, and meets the OS288048 datasheet spec.
            Some systems can operate I2C faster (800 KHz for ESP32, 1 MHz
            for many other 32-bit MCUs), and some (perhaps not all)
            OS288048's can work with this -- so it's optionally be specified
            here and is not a default behavior. (Ignored if using pre-1.5.7
            Arduino software, which operates I2C at a fixed 100 KHz.)
    @param  clkAfter
            Speed (in Hz) for Wire transmissions following OS288048 library
            calls. Defaults to 100000 (100 KHz), the default Arduino Wire
            speed. This is done rather than leaving it at the 'during' speed
            because other devices on the I2C bus might not be compatible
            with the faster rate. (Ignored if using pre-1.5.7 Arduino
            software, which operates I2C at a fixed 100 KHz.)
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/

OS288048::OS288048(uint16_t w, uint16_t h, TwoWire *twi, int8_t rst_pin,
                   uint32_t clkDuring, uint32_t clkAfter)
    : Adafruit_GFX(w, h), i2c_preclk(clkDuring), i2c_postclk(clkAfter),
      buffer(NULL), dcPin(-1), csPin(-1), rstPin(rst_pin), _bpp(1) {
  i2c_dev = NULL;
  _theWire = twi;
}

/*!
    @brief  Constructor for SPI OS288048 displays, using software (bitbang)
            SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  mosi_pin
            MOSI (master out, slave in) pin (using Arduino pin numbering).
            This transfers serial data from microcontroller to display.
    @param  sclk_pin
            SCLK (serial clock) pin (using Arduino pin numbering).
            This clocks each bit from MOSI.
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
OS288048::OS288048(uint16_t w, uint16_t h, int8_t mosi_pin, int8_t sclk_pin,
                   int8_t dc_pin, int8_t rst_pin, int8_t cs_pin)
    : Adafruit_GFX(w, h), dcPin(dc_pin), csPin(cs_pin), rstPin(rst_pin),
      _bpp(1) {

  spi_dev = new Adafruit_SPIDevice(cs_pin, sclk_pin, -1, mosi_pin, 1000000);
}

/*!
    @brief  Constructor for SPI OS288048 displays, using native hardware SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  spi
            Pointer to an existing SPIClass instance (e.g. &SPI, the
            microcontroller's primary SPI bus).
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @param  bitrate
            SPI clock rate for transfers to this display. Default if
            unspecified is 8000000UL (8 MHz).
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
OS288048::OS288048(uint16_t w, uint16_t h, SPIClass *spi, int8_t dc_pin,
                   int8_t rst_pin, int8_t cs_pin, uint32_t bitrate)
    : Adafruit_GFX(w, h), dcPin(dc_pin), csPin(cs_pin), rstPin(rst_pin),
      _bpp(1) {

  spi_dev = new Adafruit_SPIDevice(cs_pin, bitrate, SPI_BITORDER_MSBFIRST,
                                   SPI_MODE0, spi);
}

/*!
    @brief  Constructor for I2C-interfaced OS288048 OLED displays.
    @param  bpp Bits per pixel, 1 for monochrome, 2 for 4-gray, 4 for 16-gray
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  twi
            Pointer to an existing TwoWire instance (e.g. &Wire, the
            microcontroller's primary I2C bus).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  clkDuring
            Speed (in Hz) for Wire transmissions in library calls.
            Defaults to 400000 (400 KHz), a known 'safe' value for most
            microcontrollers, and meets the OLED datasheet spec.
            Some systems can operate I2C faster (800 KHz for ESP32, 1 MHz
            for many other 32-bit MCUs), and some (perhaps not all)
            Many OLED's can work with this -- so it's optionally be specified
            here and is not a default behavior. (Ignored if using pre-1.5.7
            Arduino software, which operates I2C at a fixed 100 KHz.)
    @param  clkAfter
            Speed (in Hz) for Wire transmissions following library
            calls. Defaults to 100000 (100 KHz), the default Arduino Wire
            speed. This is done rather than leaving it at the 'during' speed
            because other devices on the I2C bus might not be compatible
            with the faster rate. (Ignored if using pre-1.5.7 Arduino
            software, which operates I2C at a fixed 100 KHz.)
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
OS288048::OS288048(uint8_t bpp, uint16_t w, uint16_t h, TwoWire *twi,
                   int8_t rst_pin, uint32_t clkDuring, uint32_t clkAfter)
    : Adafruit_GFX(w, h), i2c_preclk(clkDuring), i2c_postclk(clkAfter),
      buffer(NULL), dcPin(-1), csPin(-1), rstPin(rst_pin), _bpp(bpp) {
  i2c_dev = NULL;
  _theWire = twi;
}

/*!
    @brief  Constructor for SPI OS288048 OLED displays, using software (bitbang)
            SPI.
    @param  bpp Bits per pixel, 1 for monochrome, 2 for 4-gray, 4 for 16-gray
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  mosi_pin
            MOSI (master out, slave in) pin (using Arduino pin numbering).
            This transfers serial data from microcontroller to display.
    @param  sclk_pin
            SCLK (serial clock) pin (using Arduino pin numbering).
            This clocks each bit from MOSI.
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
OS288048::OS288048(uint8_t bpp, uint16_t w, uint16_t h, int8_t mosi_pin,
                   int8_t sclk_pin, int8_t dc_pin, int8_t rst_pin,
                   int8_t cs_pin)
    : Adafruit_GFX(w, h), dcPin(dc_pin), csPin(cs_pin), rstPin(rst_pin),
      _bpp(bpp) {

  spi_dev = new Adafruit_SPIDevice(cs_pin, sclk_pin, -1, mosi_pin, 1000000);
}

/*!
    @brief  Constructor for SPI OS288048 OLED displays, using native hardware
   SPI.
    @param  bpp Bits per pixel, 1 for monochrome, 2 for 4-gray, 4 for 16-gray
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  spi
            Pointer to an existing SPIClass instance (e.g. &SPI, the
            microcontroller's primary SPI bus).
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @param  bitrate
            SPI clock rate for transfers to this display. Default if
            unspecified is 8000000UL (8 MHz).
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
OS288048::OS288048(uint8_t bpp, uint16_t w, uint16_t h, SPIClass *spi,
                   int8_t dc_pin, int8_t rst_pin, int8_t cs_pin,
                   uint32_t bitrate)
    : Adafruit_GFX(w, h), dcPin(dc_pin), csPin(cs_pin), rstPin(rst_pin),
      _bpp(bpp) {

  spi_dev = new Adafruit_SPIDevice(cs_pin, bitrate, SPI_BITORDER_MSBFIRST,
                                   SPI_MODE0, spi);
}

/*!
    @brief  Destructor for OS288048 object.
*/
OS288048::~OS288048(void) {
  if (buffer) {
    free(buffer);
    buffer = NULL;
  }
  if (spi_dev)
    delete spi_dev;
  if (i2c_dev)
    delete i2c_dev;
}

// LOW-LEVEL UTILS ---------------------------------------------------------

/*!
    @brief Issue single data byte to OLED, using I2C or hard/soft SPI as needed.
    @param c The single byte data
*/
void OS288048::oled_i2c_data_command(uint8_t c) {
  // I2C
  uint8_t buf[2] = {0x00, c}; // Co = 0, D/C = 0
  i2c_dev->write(buf, 2);
}

/*!
    @brief Issue single data byte to OLED, using I2C or hard/soft SPI as needed.
    @param c The single byte data
*/
void OS288048::oled_data(uint8_t c) {
  if (i2c_dev) { // I2C
    oled_i2c_data_command(c);
  } else { // SPI (hw or soft) -- transaction started in calling function
    digitalWrite(dcPin, HIGH);
    spi_dev->write(&c, 1);
  }
}

/*!
    @brief Issue single command byte to OLED, using I2C or hard/soft SPI as
   needed.
    @param c The single byte command
*/
void OS288048::oled_command(uint8_t c) {
  if (i2c_dev) { // I2C
    oled_i2c_data_command(c);
  } else { // SPI (hw or soft) -- transaction started in calling function
    digitalWrite(dcPin, LOW);
    spi_dev->write(&c, 1);
  }
}

// Issue list of commands to GrayOLED
/*!
    @brief Issue multiple bytes of commands OLED, using I2C or hard/soft SPI as
   needed.
    @param c Pointer to the command array
    @param n The number of bytes in the command array
    @returns True for success on ability to write the data in I2C.
*/

bool OS288048::oled_commandList(const uint8_t *c, uint8_t n) {
  if (i2c_dev) {            // I2C
    uint8_t dc_byte = 0x00; // Co = 0, D/C = 0
    if (!i2c_dev->write((uint8_t *)c, n, true, &dc_byte, 1)) {
      return false;
    }
  } else { // SPI -- transaction started in calling function
    digitalWrite(dcPin, LOW);
    if (!spi_dev->write((uint8_t *)c, n)) {
      return false;
    }
  }
  return true;
}

// ALLOCATE & INIT DISPLAY -------------------------------------------------

/*!
    @brief  Allocate RAM for image buffer, initialize peripherals and pins.
            Note that subclasses must call this before other begin() init
    @param  addr
            I2C address of corresponding oled display.
            SPI displays (hardware or software) do not use addresses, but
            this argument is still required. Default if unspecified is 0x3C.
    @param  reset
            If true, and if the reset pin passed to the constructor is
            valid, a hard reset will be performed before initializing the
            display. If using multiple oled displays on the same bus, and
            if they all share the same reset pin, you should only pass true
            on the first display being initialized, false on all others,
            else the already-initialized displays would be reset. Default if
            unspecified is true.
    @return true on successful allocation/init, false otherwise.
            Well-behaved code should check the return value before
            proceeding.
    @note   MUST call this function before any drawing or updates!
*/
bool OS288048::_init(uint8_t addr, bool reset) {

  // attempt to malloc the bitmap framebuffer
  if ((!buffer) &&
      !(buffer = (uint8_t *)malloc(_bpp * HEIGHT * ((WIDTH + 7) >> 3)))) {
    return false;
  }

  // Reset OLED if requested and reset pin specified in constructor
  if (reset && (rstPin >= 0)) {
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, HIGH);
    delay(10);                  // VDD goes high at start, pause
    digitalWrite(rstPin, LOW);  // Bring reset low
    delay(10);                  // Wait 10 ms
    digitalWrite(rstPin, HIGH); // Bring out of reset
    delay(10);
  }

  // Setup pin directions
  if (_theWire) { // using I2C
    i2c_dev = new Adafruit_I2CDevice(addr, _theWire);
    // look for i2c address:
    if (!i2c_dev || !i2c_dev->begin()) {
      return false;
    }
  } else { // Using one of the SPI modes, either soft or hardware
    if (!spi_dev || !spi_dev->begin()) {
      return false;
    }
    pinMode(dcPin, OUTPUT); // Set data/command pin as output
  }

  clearDisplay();

  // set max dirty window
  window_x1 = 0;
  window_y1 = 0;
  window_x2 = WIDTH - 1;
  window_y2 = HEIGHT - 1;

  return true; // Success
}

/*!
    @brief  Allocate RAM for image buffer, initialize peripherals and pins.
    @param  addr
            I2C address of corresponding OS288048 display.
            SPI displays (hardware or software) do not use addresses, but
            this argument is still required (pass 0 or any value really,
            it will simply be ignored). Default if unspecified is 0.
    @param  reset
            If true, and if the reset pin passed to the constructor is
            valid, a hard reset will be performed before initializing the
            display. If using multiple OS288048 displays on the same bus, and
            if they all share the same reset pin, you should only pass true
            on the first display being initialized, false on all others,
            else the already-initialized displays would be reset. Default if
            unspecified is true.
    @return true on successful allocation/init, false otherwise.
            Well-behaved code should check the return value before
            proceeding.
    @note   MUST call this function before any drawing or updates!
*/
bool OS288048::begin(uint8_t addr, bool reset) {

  if (!_init(addr, reset)) {
    Serial.println("!Adafruit_GrayOLED: Unable to initialize OLED");
    return false;
  }
  // Init Sequence
  oled_command(SSD0332_CMD_DISPLAYOFF); // 0xAE
  oled_command(SSD0332_CMD_SETREMAP);   // 0xA0
                                        /*
                                A[0]=0, Horizontal address increment (POR)
                                A[0]=1, Vertical address increment
                                A[1]=0, Column address 0 is mapped to SEG0 (POR)
                                A[1]=1, Column address 95 is mapped to SEG0
                                A[4]=0, Scan from COM 0 to COM [N –1]
                                A[4]=1, Scan from COM [N-1] to COM0. Where N is the Multiplex ratio.
                                A[5]=0, Disable COM Split Odd Even (POR)
                                A[5]=1, Enable COM Split Odd Even
                                A[7:6]=00, 256 color format
                                A[7:6]=01, 65k color format(POR)
                                         0: Horizontal address increment
                                         1: Column address 95 is mapped to SEG0
                                         2: nc
                                         3: nc
                                         4: Scan from COM[N-1] to COM0 . Where N is the Multiplex ratio.
                                         5: Enable COM Split Odd Even
                                         6.7: 256 color format
                                        */
  oled_command(0b00110010);             // 0x32
  oled_command(SSD0332_CMD_SETMASTER);  // 0xAD
  /*
A[0]=0, Select external VCC supply at Display ON
A[0]=1, Select internal booster at Display ON (POR)
A[1]=0, Select external VCOMH voltage supply at Display ON
A[1]=1, Select internal VCOMH regulator at Display ON (POR)
A[2]=0, Select External VP voltage supply
A[2]=1, Select Internal VP (POR)
        0: Select external VCC supply
        1: Select internal VCOMH regulator
        2: Select Internal VP
  */
  oled_command(0b10001110);            // 0x8E
  oled_command(SSD0332_CMD_STARTLINE); // 0xA1
  // Set out of display 16 screen lines at bottom (64 - 16).
  oled_command(48);                    // 0x0F
  setContrast(0x80);                   // 0x81, 0x80, 0x82, 0x80, 0x83, 0x80
  setBrightness(12);                   // 0x87, 0x0C
  clearScreen();                       // 0x25, 0x00, 0x00, 0x3F, 0x2F
  oled_command(SSD0332_CMD_DISPLAYON); // 0xAF
  page_offset = 0;

  delay(100); // 100ms delay recommended

  return true; // Success
}

/*!
    @brief  Do the actual writing of the internal frame buffer to display RAM
*/
void OS288048::display(void) {
  // ESP needs a periodic yield() call to avoid watchdog reset.
  // With the limited size of SSD0332 displays, and the fast bitrate
  // being used (1 MHz or more), I think one delay() immediately before
  // a screen write and one immediately after should cover it.  But if
  // not, if this becomes a problem, delay() might be added in the
  // 32-byte transfer condition below.
  delay(0);
  if ((window_x1 >= WIDTH) || (window_y1 >= HEIGHT) || (window_x2 < 0) ||
      (window_y2 < 0))
    return; // Nothing to do

  uint8_t *ptr = buffer;
  uint8_t maxbuff = 128; // max buffer size for I2C/SPI data transfer.
  static const uint8_t dc_byte = 0x40; // I2C Data command byte. Maybe #define
                                       /*
                                       A[8]=0, Co – Continuation bit, the transmission of the following information
                                       will contain data bytes only.                                      A[8]=1, Co – Continuation bit                                      A[7]=0, D/C# –
                                       Data / Command Selection bit, it defines the following data byte as a command.
                                       A[7]=1, D/C# – Data / Command Selection bit”, it defines the following data
                                       byte as a data which will be stored at the GDDRAM
                                       */
  uint8_t rows = HEIGHT - 1;           // display memory last row address
  uint8_t cols =
      (int16_t)((WIDTH + 2) / 3) - 1; // display memory last column address

  uint16_t bytes_per_row =
      (uint16_t)_bpp *
      (uint16_t)((WIDTH + 7) >>
                 3); // frame-buffer (288 pixels) 36 bytes
                     // *1 for 1-bit B/W in 8-bit (3-3-2 subpixels)
                     // *2 for 2-bit grayscale in 8-bit (3-3-2 subpixels)
                     // *4 for 4-bit grayscale in 16-bit (5-6-5 subpixesl)
  // dirty window display memory addresses:
  // colummn address
  uint16_t row_start =
      min(((uint16_t)cols), (uint16_t)(window_x1 / 3)); // integer division
  uint16_t row_end =
      max((uint16_t)0, (uint16_t)(window_x2 / 3)); // integer division
  // dirty row start/end pixels
  window_y1 = min(((uint8_t)rows), (uint8_t)window_y1);
  window_y2 = max((uint8_t)0, (uint8_t)window_y2);

  // dirty column start/end pixels aligned to 3-subpixel boundary
  window_x1 = row_start * 3;
  window_x2 = (row_end + 1) * 3 - 1;

  if (i2c_dev) { // I2C
    // Set high speed clk
    i2c_dev->setSpeed(i2c_preclk);
    maxbuff = i2c_dev->maxBufferSize() - 1;
  }
  oled_command(SSD0332_SETROW);    // 0x75, Oled from Row 00h(0) to 3Fh(63)
  oled_command(window_y1);         // 00
  oled_command(window_y2);         // 63
  oled_command(SSD0332_SETCOLUMN); // 0x15, Oled from Column 00h(0) to 5F(95)
  oled_command(row_start);         // 00
  oled_command(row_end);           // 95

  uint8_t dirty_bytes =
      row_end - row_start + 1; // Number of bytes to send per row
  uint8_t *dirty_buffer =
      (uint8_t *)malloc(dirty_bytes); // Allocate memory for dirty row buffer
                                      // *2 for 16-bit color, *1 for 8-bit color
  uint8_t pixel_index = window_x1 & 7; // pixel in frame-buffer byte
  uint8_t sub_index = window_x1 % 3;   // subpixel in display byte

  for (uint8_t row = window_y1; row <= window_y2; row++) // for every rows ...
  {
    ptr = buffer +
          ((uint16_t)row * (((uint16_t)WIDTH + 7) >> 3)); //  * (uint16_t)_bpp;
    uint8_t dirty_index = 0; // Dirty row buffer index
    uint8_t i = pixel_index; // pixel in frame-buffer row
    uint8_t j = sub_index;   // subpixel in display row
    uint8_t data = 0;        // pixel value
    for (uint16_t col = window_x1; col <= window_x2; col++) {
      uint8_t level = 0; // gray level
      uint8_t black = 0; // black value
      if (_bpp == 1) {
        black = 0b11100000 >> (3 * j);
        if ((*(ptr + (col >> 3)) & (0x80 >> (col & 7))) != 0) {
          data |= black; // set black subpixel
        } else {
          data &= ~black; // reset white subpixel
        }
      }
      if (j++ == 2) // next subpixel
      {
        j = 0;
        dirty_buffer[dirty_index++] = data;
        data = 0;
      }
    }
    while (dirty_index) {
      uint8_t to_write = min(dirty_index, maxbuff);

      if (i2c_dev) {
        i2c_dev->write(dirty_buffer, to_write, true, &dc_byte, 1);
      } else {
        digitalWrite(dcPin, HIGH); // TODO: last byte not committed?
        // TODO: SPI Transfer
        spi_dev->write(dirty_buffer, to_write);
      }
      dirty_index -= to_write;
    }
  }
  oled_command(
      SSD0332_CMD_NOP); // FIX: SPI write not completed on last data byte

  if (i2c_dev) { // I2C
    // Set low speed clk
    i2c_dev->setSpeed(i2c_postclk);
  }
  // TODO: SPI End-transaction

  if (dirty_buffer) {
    free(dirty_buffer);
    dirty_buffer = NULL;
  }
  // reset dirty window
  window_x1 = WIDTH;
  window_y1 = HEIGHT;
  window_x2 = -1; // -1
  window_y2 = -1; // -1
}

// DRAWING FUNCTIONS -------------------------------------------------------

/*!
    @brief  Clear contents of display buffer (set all pixels to off).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void OS288048::clearDisplay(void) {
  memset(buffer, MONOOLED_BLACK, (_bpp * HEIGHT * ((WIDTH + 7) >> 3)));
  // set max dirty window
  window_x1 = 0;
  window_y1 = 0;
  window_x2 = WIDTH - 1;
  window_y2 = HEIGHT - 1;
}

/*!
    @brief  Set/clear/invert a single pixel. This is also invoked by the
            Adafruit_GFX library in generating many higher-level graphics
            primitives.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  color
            Pixel color, one of: MONOOLED_BLACK, MONOOLED_WHITE or
   MONOOLED_INVERT.
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void OS288048::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      grayoled_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      grayoled_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }

    // adjust dirty window
    window_x1 = min(window_x1, x);
    window_y1 = min(window_y1, y);
    window_x2 = max(window_x2, x);
    window_y2 = max(window_y2, y);

    if (_bpp == 1) {
      uint8_t *pixelptr = &buffer[(x >> 3) + y * ((WIDTH + 7) >> 3)]; // >> 3
      switch (color) {
      case MONOOLED_WHITE:
        *pixelptr |= 0x80 >> (x & 7);
        break;
      case MONOOLED_BLACK:
        *pixelptr &= ~(0x80 >> (x & 7));
        break;
      case MONOOLED_INVERSE:
        *pixelptr ^= 0x80 >> (x & 7);
        break;
      }
    }
  }
}

/*!
    @brief  Return color of a single pixel in display buffer.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @return true if pixel is set (usually MONOOLED_WHITE, unless display invert
   mode is enabled), false if clear (MONOOLED_BLACK).
    @note   Reads from buffer contents; may not reflect current contents of
            screen if display() has not been called.
*/
bool OS288048::getPixel(int16_t x, int16_t y) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      grayoled_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      grayoled_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }
    uint8_t ptr = buffer[(x >> 3) + y * ((WIDTH + 7) >> 3)];
    return ((ptr) & (0x80 >> (x & 7))) != 0;
  }
  return false; // Pixel out of bounds
}

/*!
    @brief  Get base address of display buffer for direct reading or writing.
    @return Pointer to an unsigned 8-bit array, column-major, columns padded
            to full byte boundary if needed.
*/
uint8_t *OS288048::getBuffer(void) { return buffer; }

/*!
    @brief  Returns the (Pointer to the) actual Font
    @param  None
*/
GFXfont *OS288048::getFont(void) { return gfxFont; }

// OTHER HARDWARE SETTINGS -------------------------------------------------

/*!
    @brief  Enable or disable display invert mode (white-on-black vs
            black-on-white).
    @param  i
            If true, switch to invert mode (black-on-white), else normal
            mode (white-on-black).
    @note   The gray level of display data are swapped such that
                         “GS0” <-> “GS63”, “GS1” <-> “GS62”
*/
void OS288048::invertDisplay(bool i) {
  oled_command(i ? SSD0332_CMD_INVERTDISPLAY : SSD0332_CMD_NORMALDISPLAY);
}

/*!
    @brief  Power Display off
*/
void OS288048::displayOff(void) { oled_command(SSD0332_CMD_DISPLAYOFF); }

/*!
    @brief  Power Display on
*/
void OS288048::displayOn(void) { oled_command(SSD0332_CMD_DISPLAYON); }

/*!
    @brief  Set all Pixel full off (GS=0)
    @note   regardless of the contents of the display data RAM
*/
void OS288048::allPixelOff(void) { oled_command(SSD0332_CMD_DISPLAYALLOFF); }

/*!
    @brief  Set all Pixel full on (GS=63)
    @note   regardless of the contents of the display data RAM
*/
void OS288048::allPixelOn(void) { oled_command(SSD0332_CMD_DISPLAYALLON); }

/*!
    @brief  Adjust the display contrast.
    @param  level The contrast level from 0 to 0xFF
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed.
*/
void OS288048::setContrast(uint8_t level) {
  uint8_t array[] = {SSD0332_CMD_CONTRASTA, level, SSD0332_CMD_CONTRASTB, level,
                     SSD0332_CMD_CONTRASTC, level};
  oled_commandList(array, sizeof(array));
}

/*!
    @brief Set vertical scroll by COM from 0-63. Display on top the 16 rows from
   Row48 to Row63.
    @param val The Display Offset
*/
void OS288048::verticalScroll(uint8_t val) {
  if (val > 63)
    val = 63;
  uint8_t array[] = {SSD0332_CMD_DISPLAYOFFSET, val};
  oled_commandList(array, sizeof(array));
}

/*!
    @brief Adjust the master current attenuation factor from 1/16, 2/16… to
   16/16.
    @param val The Current Control. From 0 to 15 for no attenuation.
    @note  Original segment output current of a color is 160uA at scale factor =
   16, setting scale factor to 8 to reduce the current to 80uA. Change master
   current selects different contrast slope
*/
void OS288048::setBrightness(uint8_t val) {
  if (val > 15)
    val = 15;
  uint8_t array[] = {SSD0332_CMD_MASTERCURRENT, val};
  oled_commandList(array, sizeof(array));
}

/*!
    @brief This command is used in enabling or disabling the power save mode.
    @param mode Enable Power save mode
*/
void OS288048::powerSave(bool mode) {
  uint8_t array[] = {SSD0332_CMD_POWERMODE, 0x00};
  if (mode) {
    array[1] = 0b0001001;
  }
  oled_commandList(array, sizeof(array));
}

/*!
    @brief The graphic display data RAM content of the specified window area
   will be set to zero.
    @param r1 Starting point (Row 1)
    @param c1 Starting point (Column 1)
    @param r2 Ending point (Row 2)
    @param c2 Eending point (Column 2)
*/
void OS288048::clearScreen(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2) {
  uint8_t array[] = {SSD0332_CMD_CLEAR, c1, r1, c2, r2};
  oled_commandList(array, sizeof(array));
  delayMicroseconds(400); // increase if there's image flickering
  // undocumented bug, after this command the SSD0332 needs some time to
  // stabilize
}

/*!
    @brief  This command will dim the window area.
    @param r1 Starting point (Row 1)
    @param c1 Starting point (Column 1)
    @param r2 Ending point (Row 2)
    @param c2 Eending point (Column 2)
    @note   After the execution of this command, the selected window area will
   become darker as follow: Original gray scale  -	New gray scale after dim
   window command GS0 ~ GS15 		 - 	No change GS16 ~ GS19 -
   GS4 GS20 ~ GS23 	 - 	GS5 : 		 - 	: GS60 ~ GS63 	 -
   GS15 Additional execution of this command over the same window area will not
   change the data content
*/
void OS288048::dimScreen(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2) {
  uint8_t array[] = {SSD0332_CMD_DIM, c1, r1, c2, r2};
  oled_commandList(array, sizeof(array));
}

/*!
    @brief This command sets the window area to clear the window display.
    @param x0 Starting point column in pixel coordinates
    @param y0 Starting point row in pixel coordinates
    @param x1 Ending point column in pixel coordinates
    @param y1 Ending point row in pixel coordinates
*/
void OS288048::clearArea(int x0, int y0, int x1, int y1) {
  uint8_t c1, c2, sx0, sx1;
  checkBounds(x0, y0, x1, y1, c1, c2, sx0, sx1);
  fillRect(x0, y0, (x1 - x0), (y1 - y0), MONOOLED_BLACK);
  return;

  if (sx0 > 0) {
    // clear from sx0 subpixel in column c1 from y0 to y1
    // TODO
    ++c1;
  }
  if (sx1 > 0) {
    // clear to sx1 subpixel in column c2 from y0 to y1
    // TODO
    ++c2;
  }
  clearScreen(c1, y0, c2, y1);
  // TODO: clear framebuffer area (x0, y0) - (x1, y1)
}

/*!
    @brief This command change the brightness in the window area.
    @param x0 Starting point column in pixel coordinates
    @param y0 Starting point row in pixel coordinates
    @param x1 Ending point column in pixel coordinates
    @param y1 Ending point row in pixel coordinates
*/
void OS288048::dimWindow(int x0, int y0, int x1, int y1) {
  uint8_t c1, c2, sx0, sx1;
  checkBounds(x0, y0, x1, y1, c1, c2, sx0, sx1);
  if (sx0 > 0) {
    // dim from sx0 subpixel in column c1 from y0 to y1
    // TODO
    ++c1;
  }
  if (sx1 > 0) {
    // dim to sx1 subpixel in column c2 from y0 to y1
    // TODO
    ++c2;
  }
  dimScreen(c1, y0, c2, y1);
  // TODO: set framebuffer area (x0, y0) - (x1, y1)
}

/*!
    @brief This function check pixel bounds coordinates and calculates display
   columns.
    @param x0  Starting point column in pixel coordinates
    @param y0  Starting point row in pixel coordinates
    @param x1  Ending point column in pixel coordinates
    @param y1  Ending point row in pixel coordinates
    @param c1  Starting point column in display coordinates
    @param c2  Ending point column in display coordinates
    @param sx0 Starting point column sub-pixel order
    @param sx1 Ending point column sub-pixel order
*/
void OS288048::checkBounds(int &x0, int &y0, int &x1, int &y1, uint8_t &c1,
                           uint8_t &c2, uint8_t &sx0, uint8_t &sx1) {
  // check bounds a < 0, y1 < 0, , c > 95, y2 > 63
  if (x0 < 0)
    x0 = 0;
  if (y0 < 0)
    y0 = 0;
  if (x1 > 287)
    x1 = 287;
  if (y1 > 63)
    y1 = 63;
  c1 = x0 / 3;
  c2 = x1 / 3;
  sx0 = x0 % 3;
  sx1 = x1 % 3;
}
