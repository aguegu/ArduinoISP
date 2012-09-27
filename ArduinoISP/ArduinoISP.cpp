// ArduinoISP version 04m3
// Copyright (c) 2008-2011 Randall Bohn
// If you require a license, see 
//     http://www.opensource.org/licenses/bsd-license.php
//
// This sketch turns the Arduino into a AVRISP
// using the following arduino pins:
//
// pin name:    not-mega:         mega(1280 and 2560)
// slave reset: 10:               53 
// MOSI:        11:               51 
// MISO:        12:               50 
// SCK:         13:               52 
//
// Put an LED (with resistor) on the following pins:
// 9: Heartbeat   - shows the programmer is running
// 8: Error       - Lights up if something goes wrong (use red if that makes sense)
// 7: Programming - In communication with the slave
//
// 23 July 2011 Randall Bohn
// -Address Arduino issue 509 :: Portability of ArduinoISP
// http://code.google.com/p/arduino/issues/detail?id=509
//
// October 2010 by Randall Bohn
// - Write to EEPROM > 256 bytes
// - Better use of LEDs:
// -- Flash LED_PMODE on each flash commit
// -- Flash LED_PMODE while writing EEPROM (both give visual feedback of writing progress)
// - Light LED_ERR whenever we hit a STK_NOSYNC. Turn it off when back in sync.
// - Use pins_arduino.h (should also work on Arduino Mega)
//
// October 2009 by David A. Mellis
// - Added support for the read signature command
// 
// February 2009 by Randall Bohn
// - Added support for writing to EEPROM (what took so long?)
// Windows users should consider WinAVR's avrdude instead of the
// avrdude included with Arduino software.
//
// January 2008 by Randall Bohn
// - Thanks to Amplificar for helping me with the STK500 protocol
// - The AVRISP/STK500 (mk I) protocol is used in the arduino bootloader
// - The SPI functions herein were developed for the AVR910_ARD programmer 
// - More information at http://code.google.com/p/mega-isp
// 
// September 2012 by Weihong Guan (aGuegu)
// - modify to self use

#include "pins_arduino.h"
#include "SPI.h"

#define RESET     SS

#define LED_HEATBEAT    9
#define LED_ERROR   	8
#define LED_IN_PROGRAME 7

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...
#define PTIME 30

#define EECHUNK (32)

void pulse(uint8_t pin, uint8_t times);

void avrisp();

void reply(bool has_byte = false, byte val = 0x00, bool send_ok = true);
uint8_t getch();

uint8_t writeFlashPage(uint16_t address, uint16_t length);

uint8_t error = false;
uint8_t inProgramming = false;
// address for reading and writing, set by 'U' command
uint8_t buff[256]; // global block storage

void setup()
{
	Serial.begin(115200);
	SPI.setDataMode(0);
	SPI.setBitOrder(MSBFIRST);
	// Clock Div can be 2,4,8,16,32,64, or 128
	SPI.setClockDivider(SPI_CLOCK_DIV64);

	pinMode(LED_IN_PROGRAME, OUTPUT);
	pulse(LED_IN_PROGRAME, 2);
	pinMode(LED_ERROR, OUTPUT);
	pulse(LED_ERROR, 2);
	pinMode(LED_HEATBEAT, OUTPUT);
	pulse(LED_HEATBEAT, 2);
}

typedef struct param
{
	uint8_t device_signature;
	uint8_t revision;
	uint8_t progtype;
	uint8_t parmode;
	uint8_t polling;
	uint8_t selftimed;
	uint8_t lockbytes;
	uint8_t fusebytes;
	uint8_t flash_poll;
	uint16_t eeprom_poll;
	uint16_t page_size;
	uint16_t eeprom_size;
	uint32_t flash_size;
} parameter;

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.

void reply(bool has_byte, byte val, bool send_ok)
{
	if (getch() == CRC_EOP)
	{
		Serial.write(STK_INSYNC);

		if (has_byte)
			Serial.write(val);

		if (send_ok)
			Serial.write(STK_OK);
	}
	else
	{
		error = true;
		Serial.write(STK_NOSYNC);
	}
}

void heartbeat()
{
	static bool state;
	static uint16_t timer = 0;
	if (timer == 0x4000)
	{
		digitalWrite(LED_HEATBEAT, state);
		state = !state;
		timer = 0;
	}
	timer++;
}

void loop(void)
{
	digitalWrite(LED_IN_PROGRAME, inProgramming);
	digitalWrite(LED_ERROR, error);

	heartbeat();

	if (Serial.available())
		avrisp();
}

uint8_t getch()
{
	while (!Serial.available())
		;
	return Serial.read();
}

void fill(uint8_t n)
{
	for (uint8_t x = 0; x < n; x++)
	{
		buff[x] = getch();
	}
}

void pulse(uint8_t pin, uint8_t times)
{
	while (times--)
	{
		digitalWrite(pin, HIGH);
		delay(PTIME);
		digitalWrite(pin, LOW);
		delay(PTIME);
	}
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	SPI.transfer(a);
	SPI.transfer(b);
	SPI.transfer(c);
	return SPI.transfer(d);
}

uint8_t spi_transaction(uint8_t a, uint16_t b, uint8_t c)
{
	return spi_transaction(a, highByte(b), lowByte(b), c);
}

void replyVersion(uint8_t c)
{
	switch (c)
	{
	case 0x80:
		reply(true, HWVER);
		break;
	case 0x81:
		reply(true, SWMAJ);
		break;
	case 0x82:
		reply(true, SWMIN);
		break;
	case 0x93:
		reply(true, 'S'); // serial programmer
		break;
	default:
		reply(true, 0);
		break;
	}
}

void setParameters()
{
	// call this after reading paramter packet into buff[]
	param.device_signature = buff[0];
	param.revision = buff[1];
	param.progtype = buff[2];
	param.parmode = buff[3];
	param.polling = buff[4];
	param.selftimed = buff[5];
	param.lockbytes = buff[6];
	param.fusebytes = buff[7];
	param.flash_poll = buff[8];
	// ignore buff[9] (= buff[8])
	// following are 16 bits (big endian)
	param.eeprom_poll = makeWord(buff[10], buff[11]);
	param.page_size = makeWord(buff[12], buff[13]);
	param.eeprom_size = makeWord(buff[14], buff[15]);

	// 32 bits flashsize (big endian)
	param.flash_size = buff[16] * 0x01000000UL + buff[17] * 0x00010000UL
			+ buff[18] * 0x00000100UL + buff[19];
}

void beginProgram()
{
	SPI.begin();
	digitalWrite(RESET, HIGH);
	pinMode(RESET, OUTPUT);
	digitalWrite(SCK, LOW);
	delay(20);
	digitalWrite(RESET, LOW);
	spi_transaction(0xAC, 0x53, 0x00, 0x00);
	inProgramming = true;
}

void endProgram()
{
	SPI.end();
	digitalWrite(RESET, HIGH);
	pinMode(RESET, INPUT);
	inProgramming = false;
}

void universal()
{
	uint8_t ch;

	fill(4);
	ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
	reply(true, ch);
}

void flash(uint8_t hilo, uint16_t addr, uint8_t data)
{
	spi_transaction(0x40 + 0x08 * hilo, addr, data);
}

void writeFlashPageID(uint16_t addr)
{
	digitalWrite(LED_IN_PROGRAME, LOW);

	spi_transaction(0x4C, addr, 0);

	digitalWrite(LED_IN_PROGRAME, HIGH);
}

//#define _current_page(x) (here & 0xFFFFE0)
uint16_t getPage(uint16_t addr)
{
	uint16_t page = addr;

	switch (param.page_size)
	{
	case 32:
		page &= 0xFFFFFFF0;
		break;
	case 64:
		page &= 0xFFFFFFE0;
		break;
	case 128:
		page &= 0xFFFFFFC0;
		break;
	case 256:
		page &= 0xFFFFFF80;
		break;
	}

	return page;
}

uint8_t writeFlashPage(uint16_t address, uint16_t length)
{
	uint16_t x = 0;
	uint16_t page = getPage(address);

	while (x < length)
	{
		if (page != getPage(address))
		{
			writeFlashPageID(page);
			page = getPage(address);
		}
		flash(LOW, address, buff[x++]);
		flash(HIGH, address, buff[x++]);
		address++;
	}

	writeFlashPageID(page);

	return STK_OK;
}

uint8_t writeEeprom(uint16_t address, uint16_t length)
{
	// here is a word address, get the byte address

	if (length > param.eeprom_size)
	{
		error = true;
		return STK_FAILED;
	}

	fill(length);

	digitalWrite(LED_IN_PROGRAME, LOW);
	for (uint16_t x = 0, addr = address << 1; x < length; x++)
	{
		spi_transaction(0xC0, addr + x, buff[x]);
		delay(8);
	}
	digitalWrite(LED_IN_PROGRAME, HIGH);

	return STK_OK;
}

void programPage(uint16_t address)
{
	uint16_t length = makeWord(getch(), getch());
	uint8_t mem_type = getch();

	if (mem_type == 'F')
	{
		fill(length);
		reply(true, writeFlashPage(address, length), false);
		return;
	}
	if (mem_type == 'E')
	{
		reply(true, writeEeprom(address, length), false);
		return;
	}

	Serial.write(STK_FAILED);
}

uint8_t readFlash(uint8_t hilo, uint16_t addr)
{
	return spi_transaction(0x20 + hilo * 0x08, addr, 0);
}

char readFlashPage(uint16_t address, uint16_t length)
{
	for (uint16_t x = 0; x < length; x += 2)
	{
		uint8_t low = readFlash(LOW, address);
		Serial.write(low);
		uint8_t high = readFlash(HIGH, address);
		Serial.write(high);
		address++;
	}
	return STK_OK;
}

char readEepromPage(uint16_t address, uint16_t length)
{
	// here again we have a word address
	uint16_t start = address * 2;
	for (uint16_t x = 0; x < length; x++)
	{
		uint16_t addr = start + x;
		uint8_t ee = spi_transaction(0xA0, addr, 0x00);
		Serial.write(ee);
	}
	return STK_OK;
}

void readPage(uint16_t address)
{
	uint8_t result = STK_FAILED;
	uint16_t length = makeWord(getch(), getch());
	uint8_t memtype = getch();

	if (getch() != CRC_EOP)
	{
		error = true;
		Serial.write(STK_NOSYNC);
		return;
	}

	Serial.write(STK_INSYNC);

	if (memtype == 'F')
		result = readFlashPage(address, length);
	if (memtype == 'E')
		result = readEepromPage(address, length);

	Serial.write(result);
}

void readSignature()
{
	if (getch() != CRC_EOP)
	{
		error = true;
		Serial.write(STK_NOSYNC);
		return;
	}

	Serial.write(STK_INSYNC);
	uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
	Serial.write(high);
	uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
	Serial.write(middle);
	uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
	Serial.write(low);
	Serial.write(STK_OK);
}
//////////////////////////////////////////
//////////////////////////////////////////

////////////////////////////////////
////////////////////////////////////
void avrisp()
{
	uint8_t ch = getch();
	static uint16_t address = 0;
	switch (ch)
	{
	case '0': // signon
		error = false;
		reply();
		break;
	case '1':
		if (getch() == CRC_EOP)
		{
			Serial.write(STK_INSYNC);
			Serial.print("AVR ISP");
			Serial.write(STK_OK);
		}
		else
		{
			error = true;
			Serial.write(STK_NOSYNC);
		}
		break;
	case 'A':
		replyVersion(getch());
		break;
	case 'B':
		fill(20);
		setParameters();
		reply();
		break;
	case 'E': // extended parameters - ignore for now
		fill(5);
		reply();
		break;
	case 'P':
		inProgramming ? pulse(LED_ERROR, 3) : beginProgram();
		reply();
		break;
	case 'U': // set address (word)
		address = getch();
		address += getch() << 8;
		reply();
		break;
	case 0x60: //STK_PROG_FLASH
		getch();
		getch();
		reply();
		break;
	case 0x61: //STK_PROG_DATA
		getch();
		reply();
		break;
	case 0x64: //STK_PROG_PAGE
		programPage(address);
		break;
	case 0x74: //STK_READ_PAGE 't'
		readPage(address);
		break;
	case 'V': //0x56
		universal();
		break;
	case 'Q': //0x51
		error = false;
		endProgram();
		reply();
		break;
	case 0x75: //STK_READ_SIGN 'u'
		readSignature();
		break;
		// expecting a command, not CRC_EOP
		// this is how we can get back in sync
	case CRC_EOP:
		error = true;
		Serial.write(STK_NOSYNC);
		break;
		// anything else we will return STK_UNKNOWN
	default:
		error = true;
		Serial.write(getch() == CRC_EOP ? STK_UNKNOWN : STK_NOSYNC);
		break;
	}
}

