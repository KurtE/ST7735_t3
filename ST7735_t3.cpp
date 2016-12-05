/*************************************************** 
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "ST7735_t3.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>


// Constructor when using software SPI.  All output pins are configurable.
ST7735_t3::ST7735_t3(uint8_t cs, uint8_t rs, uint8_t sid, uint8_t sclk, uint8_t rst, SPINClass *pspin) :
 Adafruit_GFX(ST7735_TFTWIDTH, ST7735_TFTHEIGHT_18 )
{
	_cs   = cs;
	_rs   = rs;
	_sid  = sid;
	_sclk = sclk;
	_rst  = rst;
	hwSPI = false;

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
	// Note, I will cheat and if pins are HWSPI, I will use...
	_pspin	  = pspin;
	_pkinetisk_spi = _pspin->kinetisk_spi();
#endif
#if defined(__MKL26Z64__)
	_pspin	  = pspin;
#endif
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ST7735_t3::ST7735_t3(uint8_t cs, uint8_t rs, uint8_t rst, SPINClass *pspin) :
  Adafruit_GFX(ST7735_TFTWIDTH, ST7735_TFTHEIGHT_18 ) {
	_cs   = cs;
	_rs   = rs;
	_rst  = rst;
	hwSPI = true;
	_sid  = _sclk = (uint8_t)-1;

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
	_pspin	  = pspin;
	_pkinetisk_spi = _pspin->kinetisk_spi();
#endif
#if defined(__MKL26Z64__)
	_pspin	  = pspin;
#endif
}



/***************************************************************/
/*     Teensy 3.0, 3.1, 3.2, 3.5, 3.6                          */
/***************************************************************/
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

inline void ST7735_t3::beginSPITransaction()
{
	if (hwSPI) {
		_pspin->beginTransaction(SPISettings(ST7735_SPICLOCK, MSBFIRST, SPI_MODE0));
	}
	if (cspin) {
		*cspin = 0;
	}
}

inline void ST7735_t3::endSPITransaction()
{
	if (cspin)
		*cspin = 1;
	if (hwSPI) {
		_pspin->endTransaction();	
	}
}


inline void ST7735_t3::spiwrite(uint8_t c)
{
	for (uint8_t bit = 0x80; bit; bit >>= 1) {
		*datapin = ((c & bit) ? 1 : 0);
		*clkpin = 1;
		*clkpin = 0;
	}
}

void ST7735_t3::writecommand(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
		_pspin->waitFifoNotFull();
	} else {
		*rspin = 0;
		spiwrite(c);
	}
}

void ST7735_t3::writedata(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		_pspin->waitFifoNotFull();
	} else {
		*rspin = 1;
		spiwrite(c);
	}
}

void ST7735_t3::writedata16(uint16_t d)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1);
		_pspin->waitFifoNotFull();
	} else {
		*rspin = 1;
		spiwrite(d >> 8);
		spiwrite(d);
	}
}

void ST7735_t3::writedata_last(uint8_t c) 
{
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
		_pspin->waitTransmitComplete(mcr);
	} else {
		*rspin = 1;
		spiwrite(c);
	}
}

void ST7735_t3::writedata16_last(uint16_t d)
{
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_EOQ;
		_pspin->waitTransmitComplete(mcr);
	} else {
		*rspin = 1;
		spiwrite(d >> 8);
		spiwrite(d);
	}
}


/***************************************************************/
/*     Teensy LC                                               */
/***************************************************************/
#elif defined(__MKL26Z64__)
inline void ST7735_t3::beginSPITransaction()
{
	if (hwSPI) {
		_pspin->beginTransaction(SPISettings(ST7735_SPICLOCK, MSBFIRST, SPI_MODE0));
	}
	*csport &= ~cspinmask;
}

inline void ST7735_t3::endSPITransaction()
{
	*csport |= cspinmask;
	if (hwSPI) {
		_pspin->endTransaction();	
	}
}


inline void ST7735_t3::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (hwSPI) {
		_pspin->transfer(c);
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) *dataport |=  datapinmask;
			else        *dataport &= ~datapinmask;
			*clkport |=  clkpinmask;
			*clkport &= ~clkpinmask;
		}
	}
}

void ST7735_t3::writecommand(uint8_t c)
{
	*rsport &= ~rspinmask;
	spiwrite(c);
}

void ST7735_t3::writedata(uint8_t c)
{
	*rsport |=  rspinmask;
	spiwrite(c);
} 

void ST7735_t3::writedata16(uint16_t d)
{
	*rsport |=  rspinmask;
	spiwrite(d >> 8);
	spiwrite(d);
} 

void ST7735_t3::writedata16_last(uint16_t d)
{
	*rsport |=  rspinmask;
	spiwrite(d >> 8);
	spiwrite(d);
} 

#endif //


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd2green144[] = {         // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void ST7735_t3::commandList(const uint8_t *addr)
{
	uint8_t  numCommands, numArgs;
	uint16_t ms;

	beginSPITransaction();
	numCommands = pgm_read_byte(addr++);		// Number of commands to follow
	while(numCommands--) {				// For each command...
		writecommand(pgm_read_byte(addr++));	//   Read, issue command
		numArgs  = pgm_read_byte(addr++);	//   Number of args to follow
		ms       = numArgs & DELAY;		//   If hibit set, delay follows args
		numArgs &= ~DELAY;			//   Mask out delay bit
		while(numArgs--) {			//   For each argument...
			writedata(pgm_read_byte(addr++)); //   Read, issue argument
		}

		if(ms) {
			ms = pgm_read_byte(addr++);	// Read post-command delay time (ms)
			if(ms == 255) ms = 500;		// If 255, delay for 500 ms
			delay(ms);
		}
	}
}


// Initialization code common to both 'B' and 'R' type displays
void ST7735_t3::commonInit(const uint8_t *cmdList)
{
	colstart  = rowstart = 0; // May be overridden in init func

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
//////////////////////////
   // verify SPI pins are valid;
	// allow user to say use current ones...
	hwSPI = true;	// lets assume 
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	if (!(_pspin->pinIsMOSI(_sid)) || !(_pspin->pinIsSCK(_sclk)) || !_pspin->pinIsChipSelect(_rs)) {
		#ifdef SPIN1_OBJECT_CREATED			
		if (SPIN1.pinIsMOSI(_sid) && SPIN1.pinIsSCK(_sclk) && SPIN1.pinIsChipSelect(_rs)) {
			_pspin = &SPIN1;
			_pkinetisk_spi = _pspin->kinetisk_spi();
			Serial.println("SST7735_t3: SPIN1 automatically selected");
		} else {
			#ifdef SPIN2_OBJECT_CREATED			
			if (SPIN2.pinIsMOSI(_sid) && SPIN2.pinIsSCK(_sclk) && SPIN2.pinIsChipSelect(_rs)) {
				_pspin = &SPIN2;
				_pkinetisk_spi = _pspin->kinetisk_spi();
				Serial.println("SST7735_t3: SPIN2 automatically selected");
			} else {
			#endif
		#endif
				hwSPI = false;
			#ifdef SPIN2_OBJECT_CREATED			
    		}
    		#endif
		#ifdef SPIN1_OBJECT_CREATED			
		}
		#endif

	}
	if (hwSPI) {
        _pspin->setMOSI(_sid);
		_pspin->setSCK(_sclk);

		_pspin->begin();
		if (_pspin->pinIsChipSelect(_cs, _rs)) {
			pcs_data = _pspin->setCS(_cs);
			pcs_command = pcs_data | _pspin->setCS(_rs);
			cspin = 0; // Let know that we are not setting manual
		} else {
			// We already verified that _rs was valid CS pin above.
			pcs_data = 0;
			pcs_command = pcs_data | _pspin->setCS(_rs);
			pinMode(_cs, OUTPUT);
			cspin = portOutputRegister(digitalPinToPort(_cs));
			*cspin = 1;
			Serial.println("SST7735_t3: Manual CS Pin");
		}
		Serial.println("SST7735_t3: Using Hardware SPI");
	} else {
		hwSPI = false;
		cspin = portOutputRegister(digitalPinToPort(_cs));
		rspin = portOutputRegister(digitalPinToPort(_rs));
		clkpin = portOutputRegister(digitalPinToPort(_sclk));
		datapin = portOutputRegister(digitalPinToPort(_sid));
		*cspin = 1;
		*rspin = 0;
		*clkpin = 0;
		*datapin = 0;
		pinMode(_cs, OUTPUT);
		pinMode(_rs, OUTPUT);
		pinMode(_sclk, OUTPUT);
			pinMode(_sid, OUTPUT);
	}
    // Teensy LC
#elif defined(__MKL26Z64__)
	hwSPI1 = false;
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	
	if (!(_pspin->pinIsMOSI(_sid)) || !(_pspin->pinIsSCK(_sclk))) {
		#ifdef SPIN1_OBJECT_CREATED			
		if (SPIN1.pinIsMOSI(_sid) && SPIN1.pinIsSCK(_sclk)) {
			_pspin = &SPIN1;
			Serial.println("SST7735_t3: SPIN1 automatically selected");
		} else {
		#endif
			hwSPI = false;
		#ifdef SPIN1_OBJECT_CREATED			
		}
		#endif

	}
	if (hwSPI) {
        _pspin->setMOSI(_sid);
		_pspin->setSCK(_sclk);

		_pspin->begin();
	} else {
		pinMode(_sclk, OUTPUT);
		pinMode(_sid , OUTPUT);
		clkport     = portOutputRegister(digitalPinToPort(_sclk));
		dataport    = portOutputRegister(digitalPinToPort(_sid));
		clkpinmask  = digitalPinToBitMask(_sclk);
		datapinmask = digitalPinToBitMask(_sid);
		*clkport   &= ~clkpinmask;
		*dataport  &= ~datapinmask;
	}
	// toggle RST low to reset; CS low so it'll listen to us
	*csport &= ~cspinmask;

#endif

	if (_rst) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(5);
		digitalWrite(_rst, LOW);
		delay(100);
		digitalWrite(_rst, HIGH);
		delay(5);
	}

	if(cmdList) commandList(cmdList);
}


// Initialization for ST7735B screens
void ST7735_t3::initB(void)
{
	commonInit(Bcmd);
}


// Initialization for ST7735R screens (green or red tabs)
void ST7735_t3::initR(uint8_t options)
{
	commonInit(Rcmd1);
	if (options == INITR_GREENTAB) {
		commandList(Rcmd2green);
		colstart = 2;
		rowstart = 1;
	} else if(options == INITR_144GREENTAB) {
		_height = ST7735_TFTHEIGHT_144;
		commandList(Rcmd2green144);
		colstart = 2;
		rowstart = 3;
	} else {
		// colstart, rowstart left at default '0' values
		commandList(Rcmd2red);
	}
	commandList(Rcmd3);

	// if black, change MADCTL color filter
	if (options == INITR_BLACKTAB) {
		writecommand(ST7735_MADCTL);
		writedata(0xC0);
	}

	tabcolor = options;
}


void ST7735_t3::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	beginSPITransaction();
	setAddr(x0, y0, x1, y1);
	writecommand(ST7735_RAMWR); // write to RAM
	endSPITransaction();
}


void ST7735_t3::pushColor(uint16_t color)
{
	beginSPITransaction();
	writedata16_last(color);
	endSPITransaction();
}

void ST7735_t3::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
	beginSPITransaction();
	setAddr(x,y,x+1,y+1);
	writecommand(ST7735_RAMWR);
	writedata16_last(color);
	endSPITransaction();
}


void ST7735_t3::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((y+h-1) >= _height) h = _height-y;
	beginSPITransaction();
	setAddr(x, y, x, y+h-1);
	writecommand(ST7735_RAMWR);
	while (h-- > 1) {
		writedata16(color);
	}
	writedata16_last(color);
	endSPITransaction();
}


void ST7735_t3::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((x+w-1) >= _width)  w = _width-x;
	beginSPITransaction();
	setAddr(x, y, x+w-1, y);
	writecommand(ST7735_RAMWR);
	while (w-- > 1) {
		writedata16(color);
	}
	writedata16_last(color);
	endSPITransaction();
}



void ST7735_t3::fillScreen(uint16_t color)
{
	fillRect(0, 0,  _width, _height, color);
}



// fill a rectangle
void ST7735_t3::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if ((x >= _width) || (y >= _height)) return;
	if ((x + w - 1) >= _width)  w = _width  - x;
	if ((y + h - 1) >= _height) h = _height - y;
	beginSPITransaction();
	setAddr(x, y, x+w-1, y+h-1);
	writecommand(ST7735_RAMWR);
	for (y=h; y>0; y--) {
		for(x=w; x>1; x--) {
			writedata16(color);
		}
		writedata16_last(color);
		if (y > 1 && (y & 1)) {
			endSPITransaction();
			beginSPITransaction();
		}
	}
	endSPITransaction();
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ST7735_t3::setRotation(uint8_t m)
{
	beginSPITransaction();
	writecommand(ST7735_MADCTL);
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
		} else {
			writedata(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
		}
		_width  = ST7735_TFTWIDTH;
		if (tabcolor == INITR_144GREENTAB) {
			_height = ST7735_TFTHEIGHT_144;
		} else {
			_height = ST7735_TFTHEIGHT_18;
		}
		break;
	case 1:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
		} else {
			writedata(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		}
		if (tabcolor == INITR_144GREENTAB) {
			_width = ST7735_TFTHEIGHT_144;
		} else {
			_width = ST7735_TFTHEIGHT_18;
		}
		_height = ST7735_TFTWIDTH;
		break;
	case 2:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_RGB);
		} else {
			writedata(MADCTL_BGR);
		}
		_width  = ST7735_TFTWIDTH;
		if (tabcolor == INITR_144GREENTAB) {
			_height = ST7735_TFTHEIGHT_144;
		} else {
			_height = ST7735_TFTHEIGHT_18;
		}
		break;
	case 3:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
		} else {
			writedata(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
		}
		if (tabcolor == INITR_144GREENTAB) {
			_width = ST7735_TFTHEIGHT_144;
		} else {
			_width = ST7735_TFTHEIGHT_18;
		}
		_height = ST7735_TFTWIDTH;
		break;
	}
	endSPITransaction();
}


void ST7735_t3::invertDisplay(boolean i)
{
	beginSPITransaction();
	writecommand(i ? ST7735_INVON : ST7735_INVOFF);
	endSPITransaction();

}

