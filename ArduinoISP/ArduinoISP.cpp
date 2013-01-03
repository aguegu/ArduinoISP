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

#include "pins_arduino.h"
#include "SPI.h"

#define RESET     SS

#define LED_HEARTBEAT	9
#define LED_ERROR		8
#define LED_PROGRAMMING	7

#define HARDWARE_VERSION	2
#define FIRMWARE_MAJOR_VERSION	1
#define FIRMWARE_MINOR_VERSION	18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20

static uint8_t _error = 0;
static bool _programming = false;
static uint16_t _address; // address for reading and writing, set by 'U' command
uint8_t _buff[256]; // global block storage

typedef struct param
{
	uint8_t signature;
	uint8_t revision;
	uint8_t progtype;
	uint8_t parmode;
	uint8_t polling;
	uint8_t selftimed;
	uint8_t lock_bytes;
	uint8_t fuse_bytes;
	uint8_t flash_poll;
	uint16_t eeprom_poll;
	uint16_t flash_pagesize;
	uint16_t eeprom_size;
	uint32_t flash_size;
} parameter;

parameter _param;

void reply(bool has_byte = false, byte val = 0x00, bool send_ok = true);
void pulse(uint8_t pin, uint8_t times);
void avrisp();

void heartbeat()
{
	static bool state;
	static uint16_t timer;
	if (timer > 0x4000)
	{
		digitalWrite(LED_HEARTBEAT, state);
		state = !state;
		timer = 0;
	}
	timer++;

	digitalWrite(LED_PROGRAMMING, _programming);
	digitalWrite(LED_ERROR, _error);
}

void setup()
{
	Serial.begin(115200);

	SPI.setDataMode(0);
	SPI.setBitOrder(MSBFIRST);
	// Clock Div can be 2,4,8,16,32,64, or 128
	SPI.setClockDivider(SPI_CLOCK_DIV32);

	pinMode(LED_PROGRAMMING, OUTPUT);
	pulse(LED_PROGRAMMING, 2);

	pinMode(LED_ERROR, OUTPUT);
	pulse(LED_ERROR, 2);

	pinMode(LED_HEARTBEAT, OUTPUT);
	pulse(LED_HEARTBEAT, 2);

}

void loop(void)
{
	heartbeat();

	if (Serial.available())
	{
		avrisp();
	}
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
		_buff[x] = getch();
	}
}

void pulse(uint8_t pin, uint8_t times)
{
	while (times--)
	{
		digitalWrite(pin, HIGH);
		delay(30);
		digitalWrite(pin, LOW);
		delay(30);
	}
}

uint8_t spiTransfer(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	SPI.transfer(a);
	SPI.transfer(b);
	SPI.transfer(c);
	return SPI.transfer(d);
}

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
		_error++;
		Serial.write(STK_NOSYNC);
	}
}

void replyVersion(uint8_t c)
{
	switch (c)
	{
	case 0x80:
		reply(true, HARDWARE_VERSION);
		break;
	case 0x81:
		reply(true, FIRMWARE_MAJOR_VERSION);
		break;
	case 0x82:
		reply(true, FIRMWARE_MINOR_VERSION);
		break;
	case 0x93:
		reply(true, 'S'); // serial programmer
		break;
	default:
		reply(true);
		break;
	}
}

void setParameters()
{
	// call this after reading paramter packet into buff[]
	_param.signature = _buff[0];
	_param.revision = _buff[1];
	_param.progtype = _buff[2];
	_param.parmode = _buff[3];
	_param.polling = _buff[4];
	_param.selftimed = _buff[5];
	_param.lock_bytes = _buff[6];
	_param.fuse_bytes = _buff[7];
	_param.flash_poll = _buff[8];

	_param.eeprom_poll = makeWord(_buff[10], _buff[11]);
	_param.flash_pagesize = makeWord(_buff[12], _buff[13]);
	_param.eeprom_size = makeWord(_buff[14], _buff[15]);

	// 32 bits flashsize (big endian)
	_param.flash_size = _buff[16] * 0x01000000 + _buff[17] * 0x00010000
			+ _buff[18] * 0x00000100 + _buff[19];
}

void beginProgramming()
{
	SPI.begin();

	digitalWrite(RESET, HIGH);
	pinMode(RESET, OUTPUT);

	digitalWrite(SCK, LOW);
	digitalWrite(RESET, LOW);
	digitalWrite(RESET, HIGH);
	digitalWrite(RESET, LOW);

	delay(20);

	spiTransfer(0xAC, 0x53, 0x00, 0x00);
	_programming = true;
}

void endProgramming()
{
	SPI.end();
	digitalWrite(RESET, HIGH);
	pinMode(RESET, INPUT);
	_programming = false;
}

void universal()
{
	uint8_t ch;

	fill(4);
	ch = spiTransfer(_buff[0], _buff[1], _buff[2], _buff[3]);
	reply(true, ch);
}

void flash(uint8_t hilo, uint16_t addr, uint8_t data)
{
	spiTransfer(0x40 + 8 * hilo, (addr >> 8) & 0xFF, addr & 0xFF, data);
}

void commit(uint16_t addr)
{
	spiTransfer(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

uint16_t getPage(uint16_t addr)
{
	uint16_t page = addr;

	switch (_param.flash_pagesize)
	{
	case 32:
		page &= 0xFFF0;
		break;
	case 64:
		page &= 0xFFE0;
		break;
	case 128:
		page &= 0xFFC0;
		break;
	case 256:
		page &= 0xFF80;
		break;
	}

	return page;
}

uint8_t writeFlashPages(uint16_t length)
{
	uint16_t x = 0;
	word page = getPage(_address);
	while (x < length)
	{
		if (page != getPage(_address))
		{
			commit(page);
			page = getPage(_address);
		}
		flash(LOW, _address, _buff[x++]);
		flash(HIGH, _address, _buff[x++]);
		_address++;
	}

	commit(page);

	return STK_OK;
}

void writeFlash(uint16_t length)
{
	fill(length);
	if (CRC_EOP == getch())
	{
		Serial.write(STK_INSYNC);
		Serial.write(writeFlashPages(length));
	}
	else
	{
		_error++;
		Serial.write(STK_NOSYNC);
	}
}

// write (length) bytes, (start) is a byte address
uint8_t writeEepromChunk(uint16_t start, uint16_t length)
{
	// this writes byte-by-byte,
	// page writing may be faster (4 bytes at a time)
	fill(length);

	for (uint16_t x = 0; x < length; x++)
	{
		uint16_t addr = start + x;
		spiTransfer(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, _buff[x]);
		delay(9);
	}

	return STK_OK;
}

#define EECHUNK (32)
uint8_t writeEeprom(uint16_t length)
{
	// here is a word address, get the byte address
	uint16_t start = _address * 2;
	uint16_t remaining = length;
	if (length > _param.eeprom_size)
	{
		_error++;
		return STK_FAILED;
	}
	while (remaining > EECHUNK)
	{
		writeEepromChunk(start, EECHUNK);
		start += EECHUNK;
		remaining -= EECHUNK;
	}
	writeEepromChunk(start, remaining);
	return STK_OK;
}

void programPage()
{
	uint8_t result = STK_FAILED;
	uint16_t length = 256 * getch();
	length += getch();
	uint8_t memtype = getch();
	// flash memory @here, (length) bytes
	if (memtype == 'F')
	{
		writeFlash(length);
		return;
	}
	if (memtype == 'E')
	{
		result = writeEeprom(length);
		if (CRC_EOP == getch())
		{
			Serial.write(STK_INSYNC);
			Serial.write(result);
		}
		else
		{
			_error++;
			Serial.write(STK_NOSYNC);
		}
		return;
	}
	Serial.write(STK_FAILED);
	return;
}

uint8_t readFlash(uint8_t hilo, uint16_t addr)
{
	return spiTransfer(0x20 + hilo * 8, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

uint8_t readFlashPage(uint16_t length)
{
	for (uint16_t x = 0; x < length; x += 2)
	{
		uint8_t low = readFlash(LOW, _address);
		Serial.write(low);
		uint8_t high = readFlash(HIGH, _address);
		Serial.write(high);
		_address++;
	}
	return STK_OK;
}

uint8_t readEepromPage(uint16_t length)
{
	// here again we have a word address
	uint16_t start = _address * 2;
	for (uint16_t x = 0; x < length; x++)
	{
		uint16_t addr = start + x;
		uint8_t ee = spiTransfer(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
		Serial.write(ee);
	}
	return STK_OK;
}

void readPage()
{
	uint8_t result = STK_FAILED;
	uint16_t length = 256 * getch();
	length += getch();
	uint8_t memtype = getch();
	if (CRC_EOP != getch())
	{
		_error++;
		Serial.write(STK_NOSYNC);
		return;
	}
	Serial.write(STK_INSYNC);
	if (memtype == 'F')
		result = readFlashPage(length);
	if (memtype == 'E')
		result = readEepromPage(length);
	Serial.write(result);
	return;
}

void readSignature()
{
	if (CRC_EOP != getch())
	{
		_error++;
		Serial.write(STK_NOSYNC);
		return;
	}
	Serial.write(STK_INSYNC);
	uint8_t high = spiTransfer(0x30, 0x00, 0x00, 0x00);
	Serial.write(high);
	uint8_t middle = spiTransfer(0x30, 0x00, 0x01, 0x00);
	Serial.write(middle);
	uint8_t low = spiTransfer(0x30, 0x00, 0x02, 0x00);
	Serial.write(low);
	Serial.write(STK_OK);
}

void avrisp()
{
	uint8_t ch = getch();
	switch (ch)
	{
	case '0': // signon
		_error = 0;
		reply(false);
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
			_error++;
			Serial.write(STK_NOSYNC);
		}
		break;
	case 'A':
		replyVersion(getch());
		break;
	case 'B':
		fill(20);
		setParameters();
		reply(false);
		break;
	case 'E': // extended parameters - ignore for now
		fill(5);
		reply(false);
		break;

	case 'P':
		if (_programming)
		{
			pulse(LED_ERROR, 3);
		}
		else
		{
			beginProgramming();
		}
		reply(false);
		break;
	case 'U': // set address (word)
		_address = getch();
		_address += 256 * getch();
		reply(false);
		break;

	case 0x60: //STK_PROG_FLASH
		getch();
		getch();
		reply(false);
		break;
	case 0x61: //STK_PROG_DATA
		getch();
		reply(false);
		break;
	case 0x64: //STK_PROG_PAGE
		programPage();
		break;

	case 0x74: //STK_READ_PAGE 't'
		readPage();
		break;

	case 'V': //0x56
		universal();
		break;
	case 'Q': //0x51
		_error = 0;
		endProgramming();
		reply(false);
		break;

	case 0x75: //STK_READ_SIGN 'u'
		readSignature();
		break;
		// expecting a command, not CRC_EOP
		// this is how we can get back in sync
	case CRC_EOP:
		_error++;
		Serial.write(STK_NOSYNC);
		break;
		// anything else we will return STK_UNKNOWN
	default:
		_error++;
		if (CRC_EOP == getch())
			Serial.write(STK_UNKNOWN);
		else
			Serial.write(STK_NOSYNC);
		break;
	}
}

