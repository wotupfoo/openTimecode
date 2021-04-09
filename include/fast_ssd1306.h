#pragma once
#ifndef _FAST_SSD1306_H_
#define _FAST_SSD1306_H_

#include <Adafruit_SSD1306.h>

#if ARDUINO >= 100
#define WIRE_WRITE wire->write ///< Wire write function in recent Arduino lib
#else
#define WIRE_WRITE wire->send ///< Wire write function in older Arduino lib
#endif

#ifdef HAVE_PORTREG
#define SSD1306_SELECT *csPort &= ~csPinMask;       ///< Device select
#define SSD1306_DESELECT *csPort |= csPinMask;      ///< Device deselect
#define SSD1306_MODE_COMMAND *dcPort &= ~dcPinMask; ///< Command mode
#define SSD1306_MODE_DATA *dcPort |= dcPinMask;     ///< Data mode
#else
#define SSD1306_SELECT digitalWrite(csPin, LOW);       ///< Device select
#define SSD1306_DESELECT digitalWrite(csPin, HIGH);    ///< Device deselect
#define SSD1306_MODE_COMMAND digitalWrite(dcPin, LOW); ///< Command mode
#define SSD1306_MODE_DATA digitalWrite(dcPin, HIGH);   ///< Data mode
#endif

#if (ARDUINO >= 157) && !defined(ARDUINO_STM32_FEATHER)
#define SETWIRECLOCK wire->setClock(wireClk)    ///< Set before I2C transfer
#define RESWIRECLOCK wire->setClock(restoreClk) ///< Restore after I2C xfer
#else // setClock() is not present in older Arduino Wire lib (or WICED)
#define SETWIRECLOCK ///< Dummy stand-in define
#define RESWIRECLOCK ///< keeps compiler happy
#endif

#if defined(SPI_HAS_TRANSACTION)
#define SPI_TRANSACTION_START spi->beginTransaction(spiSettings) ///< Pre-SPI
#define SPI_TRANSACTION_END spi->endTransaction()                ///< Post-SPI
#else // SPI transactions likewise not present in older Arduino SPI lib
#define SPI_TRANSACTION_START ///< Dummy stand-in define
#define SPI_TRANSACTION_END   ///< keeps compiler happy
#endif

// The definition of 'transaction' is broadened a bit in the context of
// this library -- referring not just to SPI transactions (if supported
// in the version of the SPI library being used), but also chip select
// (if SPI is being used, whether hardware or soft), and also to the
// beginning and end of I2C transfers (the Wire clock may be sped up before
// issuing data to the display, then restored to the default rate afterward
// so other I2C device types still work).  All of these are encapsulated
// in the TRANSACTION_* macros.

// Check first if Wire, then hardware SPI, then soft SPI:
#define TRANSACTION_START                                                      \
  if (wire) {                                                                  \
    SETWIRECLOCK;                                                              \
  } else {                                                                     \
    if (spi) {                                                                 \
      SPI_TRANSACTION_START;                                                   \
    }                                                                          \
    SSD1306_SELECT;                                                            \
  } ///< Wire, SPI or bitbang transfer setup
#define TRANSACTION_END                                                        \
  if (wire) {                                                                  \
    RESWIRECLOCK;                                                              \
  } else {                                                                     \
    SSD1306_DESELECT;                                                          \
    if (spi) {                                                                 \
      SPI_TRANSACTION_END;                                                     \
    }                                                                          \
  } ///< Wire, SPI or bitbang transfer end

class fast_ssd1306: public Adafruit_SSD1306
{
  	public:
		fast_ssd1306(uint8_t w, uint8_t h, TwoWire *twi, uint8_t* bufptr = NULL);
		void fastdisplay(void);
		bool begin(uint8_t vcs, uint8_t addr, bool reset, bool periphBegin);
		uint8_t* getOnscreen(void);
		uint8_t* getOffscreen(void);

	private:
		uint8_t *onscreen = NULL;	// This is what's on screen now
		TwoWire* wire = NULL;
		int8_t i2caddr;
		uint8_t *offscreen;	// This is the newly drawn buffer from Adafruit_SSD1306::buffer
		void setPageAddress(uint8_t page);
		void setColumnAddress(uint8_t col);
		void senddata(uint8_t* ptr, uint8_t num);
		void senddata1(uint8_t a);
		void senddata2(uint8_t a, uint8_t b);
		void senddata4(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
};

// CONSTRUCTOR
fast_ssd1306::fast_ssd1306(uint8_t w, uint8_t h, TwoWire *twi, uint8_t* bufptr) 
	:Adafruit_SSD1306(w, h, twi), onscreen(bufptr), wire(twi)
{
}

// PUBLIC FUNCTIONS
bool fast_ssd1306::begin(uint8_t vcs, uint8_t addr, bool reset,
							 bool periphBegin)
{
	if ( (Adafruit_SSD1306::begin(vcs, addr, reset, periphBegin)) == false ) {
		return false;
	}
	offscreen = Adafruit_SSD1306::getBuffer();
	if(onscreen == NULL) {
		if( (onscreen = (uint8_t*) calloc( (width()*(height()+7))/8, sizeof(uint8_t) )) == false ) {
			return false;
		}
	}
	if (wire) { // Using I2C
		// If I2C address is unspecified, use default
		// (0x3C for 32-pixel-tall displays, 0x3D for all others).
		i2caddr = addr ? addr : ((HEIGHT == 32) ? 0x3C : 0x3D);
		return true;
	}
}

void fast_ssd1306::fastdisplay(void)
{
	uint8_t col, page;
	bool pageSet = false;
	for (page=0; page<6; page++) {
		bool columnSet = false;
		for (col=0 ; col<width(); col+=4) {
			int addr = col * height() + page;
			// Update in groups of four to balance #transmissions vs wasted refresh
			if( offscreen[addr    ] != onscreen[addr    ] || 
				offscreen[addr + 1] != onscreen[addr + 1] || 
				offscreen[addr + 2] != onscreen[addr + 2] || 
				offscreen[addr + 3] != onscreen[addr + 3] ) 
			{
				onscreen[addr    ] = offscreen[addr    ];
				onscreen[addr + 1] = offscreen[addr + 1];
				onscreen[addr + 2] = offscreen[addr + 2];
				onscreen[addr + 3] = offscreen[addr + 3];
				if(!pageSet)
				{
					setPageAddress(page);
					pageSet = true;
				} 
				if(!columnSet)
				{
					setColumnAddress(col);
					columnSet = true;
				} 
				//senddata(&onscreen[addr],4);
				//senddata1(onscreen[addr]);
				//senddata2(onscreen[addr], onscreen[addr + 1]);
				senddata4(	onscreen[addr], 	onscreen[addr + 1],
							onscreen[addr + 2], onscreen[addr + 3] );
			} else {
				columnSet = false;
			}
		}
		pageSet = false;
	}  
}

uint8_t* fast_ssd1306::getOnscreen() 
{
	return onscreen;
}

uint8_t* fast_ssd1306::getOffscreen() 
{
	return offscreen;
}

// PRIVATE FUNCTIONS
void fast_ssd1306::setPageAddress(uint8_t page)
{
//    Adafruit_SSD1306::ssd1306_command1(c);
}

void fast_ssd1306::setColumnAddress(uint8_t col)
{
//    Adafruit_SSD1306::ssd1306_command1(c);
}

void fast_ssd1306::senddata(uint8_t* ptr, uint8_t num)
{
#warning Implement fast_ssd1306::senddata
}

void fast_ssd1306::senddata1(uint8_t a)
{
#warning Implement fast_ssd1306::senddata1
}

void fast_ssd1306::senddata2(uint8_t a, uint8_t b)
{
#warning Implement fast_ssd1306::senddata2
}

void fast_ssd1306::senddata4(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	if (wire) { // I2C
		wire->beginTransmission(i2caddr);
		WIRE_WRITE(a);
		WIRE_WRITE(b);
		WIRE_WRITE(c);
		WIRE_WRITE(d);
		wire->endTransmission();
	} else { // SPI
		SSD1306_MODE_DATA
		Adafruit_SSD1306::SPIwrite(a);
		SPIwrite(b);
		SPIwrite(c);
		SPIwrite(d);
	}
	TRANSACTION_END
}

#endif // _FAST_SSD1306_H_
