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

#define LED_HB    9
#define LED_ERR   8
#define LED_PMODE 7
#define PROG_FLICKER true

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

void reply(void);
void reply(uint8_t b);

uint8_t write_eeprom_chunk(uint16_t start, uint16_t length);
uint8_t write_flash_pages(uint16_t length);

void setup()
{
	Serial.begin(19200);
	SPI.setDataMode(0);
	SPI.setBitOrder(MSBFIRST);
	// Clock Div can be 2,4,8,16,32,64, or 128
	SPI.setClockDivider(SPI_CLOCK_DIV128);
	pinMode(LED_PMODE, OUTPUT);
	pulse(LED_PMODE, 2);
	pinMode(LED_ERR, OUTPUT);
	pulse(LED_ERR, 2);
	pinMode(LED_HB, OUTPUT);
	pulse(LED_HB, 2);
}

uint8_t error = 0;
uint8_t pmode = 0;
// address for reading and writing, set by 'U' command
int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (makeWord(*addr, *(addr+1)))

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
uint8_t hbval = 128;
int8_t hbdelta = 8;

void heartbeat()
{
	if (hbval > 192 || hbval < 32)
		hbdelta = -hbdelta;
	hbval += hbdelta;
	analogWrite(LED_HB, hbval);
	delay(20);
}

void loop(void)
{
	// is pmode active?
	digitalWrite(LED_PMODE, pmode);
	// is there an error?
	digitalWrite(LED_ERR, error);

	// light the heartbeat LED
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

void fill(int n)
{
	for (int x = 0; x < n; x++)
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

void prog_lamp(int state)
{
	if (PROG_FLICKER)
		digitalWrite(LED_PMODE, state);
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	SPI.transfer(a);
	SPI.transfer(b);
	SPI.transfer(c);
	return SPI.transfer(d);
}

void reply(void)
{
	if (getch() == CRC_EOP)
	{
		Serial.write(STK_INSYNC);
		Serial.write(STK_OK);
	}
	else
	{
		error++;
		Serial.write(STK_NOSYNC);
	}
}

void reply(uint8_t b)
{
	if (getch() == CRC_EOP)
	{
		Serial.write(STK_INSYNC);
		Serial.write(b);
		Serial.write(STK_OK);
	}
	else
	{
		error++;
		Serial.write(STK_NOSYNC);
	}
}

void get_version(uint8_t c)
{
	switch (c)
	{
	case 0x80:
		reply(HWVER);
		break;
	case 0x81:
		reply(SWMAJ);
		break;
	case 0x82:
		reply(SWMIN);
		break;
	case 0x93:
		reply('S'); // serial programmer
		break;
	default:
		reply(0);
		break;
	}
}

void set_parameters()
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
	param.eeprom_poll = beget16(&buff[10]);
	param.page_size = beget16(&buff[12]);
	param.eeprom_size = beget16(&buff[14]);

	// 32 bits flashsize (big endian)
	param.flash_size = buff[16] * 0x01000000UL + buff[17] * 0x00010000UL
			+ buff[18] * 0x00000100UL + buff[19];
}

void start_pmode()
{
	SPI.begin();
	digitalWrite(RESET, HIGH);
	pinMode(RESET, OUTPUT);
	digitalWrite(SCK, LOW);
	delay(20);
	digitalWrite(RESET, LOW);
	spi_transaction(0xAC, 0x53, 0x00, 0x00);
	pmode = true;
}

void end_pmode()
{
	SPI.end();
	digitalWrite(RESET, HIGH);
	pinMode(RESET, INPUT);
	pmode = false;
}

void universal()
{
	uint8_t ch;

	fill(4);
	ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
	reply(ch);
}

void flash(uint8_t hilo, uint16_t addr, uint8_t data)
{
	spi_transaction(0x40 + 0x08 * hilo, highByte(addr), lowByte(addr), data);
}

void commit(int addr)
{
	if (PROG_FLICKER)
		prog_lamp (LOW);

	spi_transaction(0x4C, highByte(addr), lowByte(addr), 0);

	if (PROG_FLICKER)
	{
		delay(PTIME);
		prog_lamp (HIGH);
	}
}

//#define _current_page(x) (here & 0xFFFFE0)
uint16_t current_page(uint16_t addr)
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

void write_flash(int length)
{
	fill(length);
	if (getch() == CRC_EOP)
	{
		Serial.write(STK_INSYNC);
		Serial.write(write_flash_pages(length));
	}
	else
	{
		error++;
		Serial.write(STK_NOSYNC);
	}
}

uint8_t write_flash_pages(uint16_t length)
{
	uint16_t x = 0;
	uint16_t page = current_page(here);

	while (x < length)
	{
		if (page != current_page(here))
		{
			commit(page);
			page = current_page(here);
		}
		flash(LOW, here, buff[x++]);
		flash(HIGH, here, buff[x++]);
		here++;
	}

	commit(page);

	return STK_OK;
}

uint8_t write_eeprom(uint16_t length)
{
	// here is a word address, get the byte address
	int start = here * 2;
	int remaining = length;
	if (length > param.eeprom_size)
	{
		error++;
		return STK_FAILED;
	}
	while (remaining > EECHUNK)
	{
		write_eeprom_chunk(start, EECHUNK);
		start += EECHUNK;
		remaining -= EECHUNK;
	}
	write_eeprom_chunk(start, remaining);
	return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(uint16_t start, uint16_t length)
{
	// this writes byte-by-byte,
	// page writing may be faster (4 bytes at a time)
	fill(length);
	prog_lamp (LOW);
	for (uint16_t x = 0; x < length; x++)
	{
		uint16_t addr = start + x;
		spi_transaction(0xC0, highByte(addr), lowByte(addr), buff[x]);
		delay(45);
	}
	prog_lamp (HIGH);
	return STK_OK;
}

void program_page()
{
	char result = (char) STK_FAILED;
	int length = 256 * getch();
	length += getch();
	char memtype = getch();
	// flash memory @here, (length) bytes
	if (memtype == 'F')
	{
		write_flash(length);
		return;
	}
	if (memtype == 'E')
	{
		result = (char) write_eeprom(length);
		if (getch() == CRC_EOP)
		{
			Serial.write(STK_INSYNC);
			Serial.write(result);
		}
		else
		{
			error++;
			Serial.write(STK_NOSYNC);
		}
		return;
	}

	Serial.write(STK_FAILED);
}

uint8_t flash_read(uint8_t hilo, uint16_t addr)
{
	return spi_transaction(0x20 + hilo * 0x08, highByte(addr), lowByte(addr), 0);
}

char flash_read_page(uint16_t length)
{
	for (uint16_t x = 0; x < length; x += 2)
	{
		uint8_t low = flash_read(LOW, here);
		Serial.write(low);
		uint8_t high = flash_read(HIGH, here);
		Serial.write(high);
		here++;
	}
	return STK_OK;
}

char eeprom_read_page(uint16_t length)
{
	// here again we have a word address
	uint16_t start = here * 2;
	for (uint16_t x = 0; x < length; x++)
	{
		uint16_t addr = start + x;
		uint8_t ee = spi_transaction(0xA0, highByte(addr), lowByte(addr), 0x00);
		Serial.write(ee);
	}
	return STK_OK;
}

void read_page()
{
	uint8_t result = STK_FAILED;
//	int length = 256 * getch();
//	length += getch();
	uint16_t length = makeWord(getch(), getch());
	uint8_t memtype = getch();

	if (getch() != CRC_EOP)
	{
		error++;
		Serial.write(STK_NOSYNC);
		return;
	}

	Serial.write(STK_INSYNC);

	if (memtype == 'F')
		result = flash_read_page(length);
	if (memtype == 'E')
		result = eeprom_read_page(length);

	Serial.write(result);
}

void read_signature()
{
	if (getch() != CRC_EOP)
	{
		error++;
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
	uint8_t data, low, high;
	uint8_t ch = getch();
	switch (ch)
	{
	case '0': // signon
		error = 0;
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
			error++;
			Serial.write(STK_NOSYNC);
		}
		break;
	case 'A':
		get_version(getch());
		break;
	case 'B':
		fill(20);
		set_parameters();
		reply();
		break;
	case 'E': // extended parameters - ignore for now
		fill(5);
		reply();
		break;
	case 'P':
		pmode ? pulse(LED_ERR, 3) : start_pmode();
		reply();
		break;
	case 'U': // set address (word)
		here = getch();
		here += 256 * getch();
		reply();
		break;
	case 0x60: //STK_PROG_FLASH
		low = getch();
		high = getch();
		reply();
		break;
	case 0x61: //STK_PROG_DATA
		data = getch();
		reply();
		break;
	case 0x64: //STK_PROG_PAGE
		program_page();
		break;
	case 0x74: //STK_READ_PAGE 't'
		read_page();
		break;
	case 'V': //0x56
		universal();
		break;
	case 'Q': //0x51
		error = 0;
		end_pmode();
		reply();
		break;
	case 0x75: //STK_READ_SIGN 'u'
		read_signature();
		break;
		// expecting a command, not CRC_EOP
		// this is how we can get back in sync
	case CRC_EOP:
		error++;
		Serial.write(STK_NOSYNC);
		break;
		// anything else we will return STK_UNKNOWN
	default:
		error++;
		Serial.write(getch() == CRC_EOP ? STK_UNKNOWN : STK_NOSYNC);
		break;
	}
}

