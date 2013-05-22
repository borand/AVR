/*
 * main.c
 *
 *  Created on: Sep 11, 2009
 *      Author: Andrzej
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include "global.h"
#include "uart.h"
#include "rprintf.h"
#include "vt100.h"
#include "cmdline.h"
#include "a2d.h"
#include "timer.h"

#define DEBUG 0

void CmdLineLoop(void);
void HelpFunction(void);
void GetVersion(void);
void test(void);

void Poke(void);
void Peek(void);
void Dump(void);

uint8_t Run;

int main(void)
{
	uartInit(); //Initialize UART
	uartSetBaudRate(115200);//Default Baudrate
	rprintfInit(uartSendByte);
	cmdlineInit();
	cmdlineSetOutputFunc(uartSendByte);
	vt100Init();
	vt100ClearScreen();

	cmdlineAddCommand("help",  HelpFunction);
	cmdlineAddCommand("poke", Poke);
	cmdlineAddCommand("peek", Peek);
	cmdlineAddCommand("dump", Dump);
	cmdlineAddCommand("test", test);

	GetVersion();

	cmdlineInputFunc('\r');
	CmdLineLoop();
	return 0;
}
void CmdLineLoop(void)
{
	u08 c;
	// set state to run
	Run = TRUE;
	// main loop
	while (Run)
	{
		// pass characters received on the uart (serial port)
		// into the cmdline processor
		while (uartReceiveByte(&c))
		{
			switch(c)
			{
				case 'H':
					HelpFunction();
					cmdlineInputFunc('\r');
					break;
				case 'I':
					GetVersion();
					cmdlineInputFunc('\r');
					break;
				default:
					cmdlineInputFunc(c);
			}
		}
		// run the cmdline execution functions
		cmdlineMainLoop();
	}
}

void HelpFunction(void)
{
	rprintfProgStrM("Commands:\n");
	rprintfProgStrM("H                : print help\n");
	rprintfProgStrM("I                : prints device ID and version info\n");
	rprintfProgStrM("test             : test function\n");
	rprintfProgStrM("peek [reg]       : returns dec, hex and bin value of register\n");
	rprintfProgStrM("poke [reg] [val] : sets register value to [val] \n");
	rprintfProgStrM("test             : test function\n");
}
void GetVersion(void)
{
	rprintfProgStrM("Avr328pConsole 328P DAQ V13.05.10, By Andrzej Borowiec");
}

////////////////////////////////////////////////////////////////
//Testing and utility functions
void test(void)

{
	uint8_t arg1 = (uint8_t) cmdlineGetArgInt(1);
	uint8_t arg2 = (uint8_t) cmdlineGetArgInt(2);
	uint8_t value;

	//InitA2D();
	rprintfProgStrM("<t>");
	rprintf("%d", timer0GetOverflowCount());
	rprintf(" %d",a2dConvert10bit(0));
	rprintf(" %d",a2dConvert10bit(0));
	rprintfProgStrM("</t>\n");

	value = arg1 & arg2;
	rprintfNum(10,4,FALSE,' ',value);rprintf("\t");
	rprintfNum(16,4,FALSE,' ',value);rprintf("\t");
	rprintfNum(2,8,FALSE,'0',value);rprintf("\t");
}

void Poke(void) {

	uint16_t address = 0;
	uint8_t value = 0;

	address = cmdlineGetArgHex(1);
	value = cmdlineGetArgHex(2);
	_SFR_MEM8(address) = value;
}
void Peek(void) {

	uint16_t address = 0;
	uint8_t value = 0;

	address = cmdlineGetArgHex(1);
	value = _SFR_MEM8(address);
	//rprintf("%d\t0x%x\t0b %b\r\n", value,value,value);
	rprintfNum(10,4,FALSE,' ',value);rprintf("\t");
	rprintfNum(16,4,FALSE,' ',value);rprintf("\t");
	rprintfNum(2,8,FALSE,'0',value);rprintf("\t");
	rprintf("\n");
}
void Dump(void) {

	uint16_t address, start, stop;
	uint8_t value, add_h, add_l;
	uint8_t col = 0;

	start = cmdlineGetArgHex(1);
	stop = cmdlineGetArgHex(2);

	for (address = start; address <= stop; address++) {
		value = _SFR_MEM8(address);
		if (col == 0) {
			add_h = (address >> 8);
			add_l = (address & 0xff);
			rprintf("0x%x%x 0x%x", add_h, add_l, value);
		} else {
			rprintf(" 0x%x", value);
		}
		col++;
		if (col == 8) {
			rprintf("\r\n");
			col = 0;
		}
	}

	rprintf("\r\n");

}
