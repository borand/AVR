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


#include "DS1820.h"
#include "simple_servo.h"

#define DEBUG 0
//#define F_CPU 16000000

void CmdLineLoop(void);
void HelpFunction(void);
void GetDevNumber(void);
void SetDevNumber(void);
void InitA2D(void);
void GetA2D(void);
void GetDIO(void);
void GetPortB(void);
void GetPortD(void);
void GetVersion(void);
void PortIO(void);
void test(void);
void GetTemperature(void);
void LoadRom(void);
void SaveThermometerIdToRom(void);
void ReadRom(void);
void GetDevIDs(void);
void SetDevIDs(void);
//void setTiming(void);
void ChangeTmermPin(void);
void setServoPosition(void);
void SpiTest(void);
//void SetSSRPower(void);
void Timer0Func(void);
void SetPrintOutputMode(void);
void PrintTagOpen(void);
void PrintTagClose(void);
void PrintTagSeparator(void);
void PrintAllData(void);

void Poke(void);
void Peek(void);
void Dump(void);

#define NUM_OF_ADCS 6
#define NUM_OF_DIOS 8

#define TERMINAL 0
#define JSON     1
#define HTML     2

typedef struct {
	uint8_t label[20];
} Label_t;
uint8_t EEMEM eep_devid = 1;
Label_t eep_dev_location[1] EEMEM;
Label_t eep_adc_id[NUM_OF_ADCS] EEMEM;
Label_t eep_dio_id[NUM_OF_DIOS] EEMEM;
uint8_t Run;
uint8_t print_mode;
uint8_t busy, streaming;
uint8_t last_port_d;

int main(void)
{
	uartInit(); //Initialize UART
	uartSetBaudRate(115200);//Default Baudrate
	rprintfInit(uartSendByte);
	cmdlineInit();
	cmdlineSetOutputFunc(uartSendByte);
	vt100Init();
	vt100ClearScreen();

	DDRB = _BV(PB2) | _BV(PB3) | _BV(PB5) ;

	PORTD = _BV(PB2) | _BV(PB3);
	last_port_d = PIND & (1 << 2);

	sbi(PORTB,PB2);

	timer0Init();
	timer0SetPrescaler(TIMER_CLK_DIV1024);
	timerAttach(0, Timer0Func);
	InitA2D();
	//SSRInit();

	print_mode = JSON;
	busy       = 0;
	streaming  = 0;

	therm_init();
	therm_start_measurement();



//	//**********************************************************
//	//External interrupts for the encoders
//	extintInit();
//	// configure external interrupts for rising-edge triggering.
//	// when a rising-edge pulse arrives on INT0 or INT1,
//	// the interrupt will be triggered
//	// rprintf("Configuring external interrupts\r\n");
//	extintConfigure(PD2, EXTINT_EDGE_RISING);
//	extintConfigure(PD3, EXTINT_EDGE_RISING);
//
//	// attach user interrupt routines.
//	// when the interrupt is triggered, the user routines will be executed
//	// rprintf("Attaching user interrupt routines\r\n");
//	extintAttach(PD2, LEncoderInterrupt);
//	extintAttach(PD3, REncoderInterrupt);
//
//	// enable the interrupts
//	// rprintf("Enabling external interrupts\r\n");
//	// (support for this has not yet been added to the library)
//	sbi(GIMSK, INT0);
//	sbi(GIMSK, INT1);

	//ServoInit();
	//SetServoPosition(SERVO_NEUTRAL_WIDTH, 0);

	// add commands to the command database
	//cmdlineAddCommand("exit",		exitFunction);
	//cmdlineAddCommand("a2d",   getA2D);
	cmdlineAddCommand("help",  HelpFunction);
	cmdlineAddCommand("cp",    ChangeTmermPin);
	cmdlineAddCommand("dev",   SetDevNumber);
	//cmdlineAddCommand("load",  loadRom);
	cmdlineAddCommand("pin",   PortIO);
	cmdlineAddCommand("save",  SaveThermometerIdToRom);
	cmdlineAddCommand("test",  test);
	cmdlineAddCommand("temp",  GetTemperature);
	cmdlineAddCommand("label", SetDevIDs);
	cmdlineAddCommand("servo", SetServoPosition);
	cmdlineAddCommand("mode",  SetPrintOutputMode);
	cmdlineAddCommand("poke", Poke);
	cmdlineAddCommand("peek", Peek);
	cmdlineAddCommand("dump", Dump);
	cmdlineAddCommand("test", test);
	//cmdlineAddCommand("settiming",setTiming);
	//cmdlineAddCommand("timing",therm_print_timing);
	//cmdlineAddCommand("reset", therm_reset);
	// print version info

	GetVersion();
	// send a CR to cmdline input to stimulate a prompt
	ServoInit();
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
				case 'A':
					PrintTagOpen();
					GetA2D();
					PrintTagClose();
					cmdlineInputFunc('\n');
					break;
				case 'B':
					GetPortB();
					cmdlineInputFunc('\r');
					break;
				case 'C':
					vt100ClearScreen();
					vt100SetCursorPos(1, 1);
					cmdlineInputFunc('\r');
					break;
				case 'D':
					GetPortD();
					GetDIO();
					cmdlineInputFunc('\n');
					break;
				case 'H':
					HelpFunction();
					cmdlineInputFunc('\r');
					break;
				case 'I':
					GetVersion();
					cmdlineInputFunc('\r');
					break;
				case 'L':
					LoadRom();
					cmdlineInputFunc('\r');
					break;
				case 'N':
					rprintfProgStrM("{\"DaqBoard\":");
					GetDevNumber();
					rprintfProgStrM("}\n");
					break;
				case 'M':
					streaming = ~streaming;
					rprintf("streaming = %d\n",streaming);
					break;
				case 'R':
					ReadRom();
					cmdlineInputFunc('\r');
					break;
				case 'S':
					therm_print_scratchpad();
					cmdlineInputFunc('\r');
					break;
				case 'T':
					PrintTagOpen();
					GetTemperature();
					PrintTagClose();
					cmdlineInputFunc('\r');
					break;
				case 'P':
					GetDevIDs();
					cmdlineInputFunc('\r');
					break;
				case 'Z':
					PrintAllData();
					break;
				case 'X':
					rprintfProgStrM("<TIMER0 OVF RESET>\n");
					timer0ClearOverflowCount();
					break;
				default:
					cmdlineInputFunc(c);
			}
		}
		// run the cmdline execution functions
		cmdlineMainLoop();
	}
}
void PrintAllData(void)
{
	PrintTagOpen();
	rprintfProgStrM("{\"DaqBoard\":");
	GetDevNumber();
	rprintfProgStrM(",");
	rprintfProgStrM("\"PortB\":");
	GetPortB();
	rprintf(",");
	rprintfProgStrM("\"PortD\":");
	GetPortD();
	rprintf(",");
	//GetDIO();
	//GetA2D();rprintf("\n");
	rprintfProgStrM("\"Temperature\":[");
	GetTemperature();
	rprintf("]}");
	PrintTagClose();
	rprintfCRLF();
}

void HelpFunction(void)
{
	rprintfProgStrM("Arduino DAQ commands:\n");
	//rprintfProgStrM("H                         : lists commands\n");
	rprintfProgStrM("A          : returns all a2d channels\n");
	rprintfProgStrM("B          : returns value of port B\n");
	rprintfProgStrM("C          : clear terminal screen\n");
	rprintfProgStrM("D          : returns value of port D\n");
	rprintfProgStrM("H          : print help\n");
	rprintfProgStrM("I          : prints device ID and version info\n");
	rprintfProgStrM("L          : display all DS1820 rom content\n");
	rprintfProgStrM("N          : print device number\n");
	rprintfProgStrM("M          : toggles streaming mode on and off\n");
	rprintfProgStrM("R          : reads rom of given device\n");
	rprintfProgStrM("S          : print last contents of last scratchpad\n");
	rprintfProgStrM("P          : print sensor description\n");
	rprintfProgStrM("Z          : print all sensor readings\n\n");

	rprintfProgStrM("cp [pin]   : change thermometer pin\n");
	rprintfProgStrM("dev [loc]  : save device location to rom\n");
	rprintfProgStrM("label      : assign sensor label\n");
	rprintfProgStrM("mode [0 1] : set print mode: 0 - terminal 1 - JSON\n");
	rprintfProgStrM("pin        : get/set state of pin on port B C or D\n");
	rprintfProgStrM("save       : save device ID to rom\n");
	rprintfProgStrM("temp [dev] : prints temperatures\n");
//	rprintfProgStrM("ssr  [pwm] : sets duty pwm on chan B0 [0 - 15625] 1Hz \n");
	rprintfProgStrM("test       : test function\n");
}

void GetVersion(void)
{
	rprintfProgStrM("Arduino 328P DAQ V13.05.10, By Andrzej Borowiec");
}
void GetDevNumber(void)
{
	Label_t Location;
	eeprom_read_block(&Location,&eep_dev_location[0],sizeof(Label_t));
	rprintf("{\"label\":\"");
	rprintfStr(Location.label);
	rprintf("\",\"ser_num\":%d,",eeprom_read_byte(&eep_devid));
	rprintf("\"firmware\":\"");
	GetVersion();
	rprintf("\"}");

}
void SetDevNumber(void)
{
	uint8_t devNum = (uint8_t) cmdlineGetArgInt(1);
	uint8_t *location = cmdlineGetArgStr(2);
	Label_t Location;
	if(devNum)
	{
		eeprom_write_byte(&eep_devid,devNum);
		strcpy(Location.label,location);
		eeprom_write_block(&Location,&eep_dev_location[0],sizeof(Label_t));
	}
}

void PortIO(void)
{
	uint8_t port = 0x0, portdir = 0x0, bit;
	uint8_t *portNum = cmdlineGetArgStr(1);
	uint8_t pinNum = (uint8_t) cmdlineGetArgInt(2);
	uint8_t *status = cmdlineGetArgStr(3);

	switch (portNum[0])
	{
	case 'd':
	case 'D':
		rprintfProgStrM("PORT D ");
		port = PIND;
		sbi(DDRD,pinNum);
		break;
	case 'b':
	case 'B':
		rprintfProgStrM("PORT B ");
		port = PINB;
		sbi(DDRB,pinNum);
		break;
	case 'c':
	case 'C':
		rprintfProgStrM("PORT C ");
		port    = PINC;
		portdir = DDRC;
		break;
	default:
		port = PINB;
	}

	switch (status[0])
	{
	case '1':
		rprintf("PIN %d = 1",pinNum);
		//sbi(portdir,pinNum);
		sbi(PORTD,pinNum);
		cbi(PIND,pinNum);
		break;
	case '0':
		rprintf("PIN %d = 0",pinNum);
		//sbi(portdir,pinNum);
		cbi(PORTD,pinNum);
		cbi(PIND,pinNum);
		break;
	case '?':
		cbi(portdir,pinNum);
		bit = port & (1 << pinNum);
		rprintf("PIN %d = 0",bit);
		break;
	case '!':
		rprintfProgStrM("toggle");
		toggle(PORTB,pinNum);
		break;
	default:
		rprintf("= %d",inb(port));
		;
	}
	rprintfCRLF();
}
void GetPortB(void)
{
	rprintf("<json>[%d]</json>",PINB);
}
void GetPortD(void)
{
	rprintf("<json>[%d]</json>",PIND);
}

void InitA2D(void)
{
	rprintfProgStrM("InitA2d()\n");

	DDRC  = 0x00;
	PORTC = 0x00;
	a2dInit();
	a2dSetPrescaler(ADC_PRESCALE_DIV128);
	a2dSetReference(ADC_REFERENCE_AVCC);
}
void GetA2D(void)
{
	uint8_t i;
	Label_t adc_label;

	// configure a2d port (PORTA) as input
	// so we can receive analog signals
	DDRC = 0x00;
	// make sure pull-up resistors are turned off
	PORTC = 0x00;

	// turn on and initialize A/D converter
	a2dInit();
	// set the a2d prescaler (clock division ratio)
	// - a lower prescale setting will make the a2d converter go faster
	// - a higher setting will make it go slower but the measurements
	//   will be more accurate
	// - other allowed prescale values can be found in a2d.h
	a2dSetPrescaler(ADC_PRESCALE_DIV128);

	// set the a2d reference
	// - the reference is the voltage against which a2d measurements are made
	// - other allowed reference values can be found in a2d.h
	a2dSetReference(ADC_REFERENCE_AVCC);

	// use a2dConvert8bit(channel#) to get an 8bit a2d reading
	// use a2dConvert10bit(channel#) to get a 10bit a2d reading
	for (i = 0; i <NUM_OF_ADCS; i++)
	{
		eeprom_read_block(&adc_label,&eep_adc_id[i],sizeof(Label_t));
		if(strlen(adc_label.label)>0)
		{
			if (print_mode==JSON)
				rprintfProgStrM("['");
			else
				;
			rprintfStr(adc_label.label);
			if (print_mode==JSON)
				rprintfProgStrM("',");
			else
				rprintfProgStrM(": \t");
			rprintf("%d", a2dConvert10bit(i));
			if (print_mode==JSON)
				rprintfProgStrM("],");
			else
				rprintfCRLF();
		}
	}
	a2dOff();
}

////////////////////////////////////////////////////////////////
//Thermometer functions
void Timer0Func(void)
{
	uint8_t new_port_d;
	new_port_d = PIND & (1 << 2);
	;
	// Function used to initial temperature conversion in all devices ever 1sec.
	// The DS1820s take 750ms to complete conversion in 12bit mode
	if (timer0GetOverflowCount() >= 61)
	{
		if (new_port_d != last_port_d)
		{
			//rprintfProgStrM("pin state change\n");
			rprintf("<json>[\"door-sw-01\", %d]</json>\r\n",new_port_d);
		}
		last_port_d = new_port_d;
		toggle(PINB,5);
		// Skip temperature measurement if we are in the process of reading out the temp
		if (!busy)
		{
			therm_reset();
			therm_start_measurement();
			if(streaming)
				PrintAllData();
		}
		timer0ClearOverflowCount();
	}
}
void ChangeTmermPin(void)
{
	therm_set_pin((uint8_t)cmdlineGetArgInt(1));
}
void GetTemperature(void)
{
	int16_t t[2];
	uint8_t i, device_count = 0, loop_count=0;
	int8_t devNum = (int8_t) cmdlineGetArgInt(1);

	busy = 1;
	if (devNum == 0)
	{
		for (i = 1; i <= 20; i++)
		{
			if (therm_load_devID(i))
				device_count++;
		}
		//rprintf("count: %d\n",device_count);
		for (i = 1; i <= 20; i++)
		{
			if (therm_load_devID(i))
			{
				loop_count++;
				if (print_mode == JSON)
					rprintfProgStrM("[\"");

				therm_print_devID();

				if (print_mode == JSON)
					rprintfProgStrM("\",");
				else
					rprintfProgStrM(" ");

				therm_read_result(t);

				if (print_mode == JSON)
				{
					rprintfProgStrM("]");
					if (loop_count < device_count)
						rprintfProgStrM(",");
				}
				else
					rprintfProgStrM(" C\n");
			}
		}
	}
	else if (devNum == -1)
	{
		therm_read_devID();
		therm_read_result(t);
	}
	else
	{
		if (therm_load_devID(devNum))
		{
			//therm_read_devID();
			//rprintf("<%d>",devNum);
			therm_read_result(t);
			//rprintf("</%d>",devNum);
		}
	}
	busy = 0;
}
void ReadRom(void)
{
	if(therm_read_devID())
		therm_print_devID();
	else
	{
		rprintf("CRC Error; ");
		therm_print_devID();
	}
}
void LoadRom(void)
{
	uint8_t crc_ok = 0, i;
	uint8_t devNum = (uint8_t) cmdlineGetArgInt(1);
	rprintfProgStrM("\n<ROM>\n");
	for (i = 1; i < 20; i++)
	{
		if (therm_load_devID(i))
		{
			rprintf("<R i=\"%d\">",i);
			therm_print_devID();
			rprintf("</R>\n");
		}
	}
	rprintfProgStrM("</ROM>\n");
}
void SaveThermometerIdToRom(void)
{
	uint8_t  devNum = (uint8_t) cmdlineGetArgInt(1);
	uint8_t  devID[8], i;
	for(i=0;i<8;i++)
		devID[i] = (uint8_t) cmdlineGetArgInt(i+2);

	if (devID[0] != 0)
		therm_set_devID(devID);
	therm_save_devID(devNum);
	rprintfProgStrM("Saving to ROM.\n");
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
void SetDevIDs(void)
{
	uint8_t *port  = cmdlineGetArgStr(1);
	uint8_t devNum = (uint8_t) cmdlineGetArgInt(2);
	uint8_t *label = cmdlineGetArgStr(3);
	Label_t Label;
	if (port[0] == 'a')
	{
		if((devNum >=0) && (devNum < NUM_OF_ADCS))
		{
			strcpy(Label.label,label);
			eeprom_write_block(&Label,&eep_adc_id[devNum],sizeof(Label_t));
		}
		else
			rprintfProgStrM("Invalid device number");

	}
	else if (port[0] == 'd')
	{
		if((devNum >=0) && (devNum < NUM_OF_DIOS))
		{
			strcpy(Label.label,label);
			eeprom_write_block(&Label,&eep_dio_id[devNum],sizeof(Label_t));
		}
		else
			rprintfProgStrM("Invalid device number");
	}
	else
	{
		rprintfProgStrM("Invalid syntax: label [adc|dio] devNum label_text\r\n");
	}

}
void GetDevIDs(void)
{
	uint8_t  i;
	Label_t Label;

	rprintfProgStrM("\n<ADC_label>\n");
	for(i=0;i<NUM_OF_ADCS;i++)
	{
		eeprom_read_block(&Label,&eep_adc_id[i],sizeof(Label_t));
		if (strlen(Label.label) > 0)
		{
			rprintf("<%d>",i);
			rprintfStr(Label.label);
			rprintf("</%d>\n",i);
		}
	}
	rprintfProgStrM("</ADC_label>\n");

	rprintfProgStrM("\n<DIO_label>\n");
	for(i=0;i<NUM_OF_DIOS;i++)
	{
		eeprom_read_block(&Label,&eep_dio_id[i],sizeof(Label_t));
		if (strlen(Label.label) > 0)
		{
			rprintf("<%d>",i);
			rprintfStr(Label.label);
			rprintf("</%d>\n",i);
		}
	}
	rprintfProgStrM("</DIO_label>\n");
}
void SetPrintOutputMode(void)
{
	int8_t mode = (int8_t) cmdlineGetArgInt(1);
	print_mode = mode;
	rprintfProgStrM("SetPrintOutputMode: ");
	if (mode)
		rprintfProgStrM("JSON (json parser friendly)\n");
	else
		rprintfProgStrM("terminal (human eye friendly)\n");

}
void PrintTagOpen(void)
{
	if (print_mode==JSON)
		rprintfProgStrM("<json>[");
	else
		rprintfProgStrM("\n");
}
void PrintTagClose(void)
{
	if (print_mode == JSON)
		rprintfProgStrM("]</json>");
	else
		rprintfProgStrM("\n");
}
void PrintTagSeparator(void);
////////////////////////////////////////////////////////////////
//SPI COM
void SpiTest(void)
{
	rprintfStr("\n<TEST>\n");
	uint8_t data = (uint8_t) cmdlineGetArgInt(1);
	uint8_t rep  = (uint8_t) cmdlineGetArgInt(2);
	uint8_t i;
//	/* Set MOSI and SCK output, all others input */

//	DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK);
//	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	SPSR = (1<<SPI2X);
//
	cbi(PORTB,PB2);
//	/* Start transmission */
	for(i=0;i<=rep;i++)
	{
	SPDR = data;
//	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)))
	;
	_delay_us(2);
	}
	sbi(PORTB,PB2);

	rprintfStr("</TEST>\n");
}
////////////////////////////////////////////////////////////////
//DIGITAL IO
void GetDIO(void)
{
	uint8_t i = (uint8_t)cmdlineGetArgInt(1);
	uint8_t bit;
	Label_t Label;
	rprintfStr("\n<DIO>\n");
	for(i=0;i<NUM_OF_DIOS;i++)
	{
		eeprom_read_block(&Label,&eep_dio_id[i],sizeof(Label_t));
		if (strlen(Label.label) > 0)
		{
			bit = (PIND & (1 << i)) == (1 << i);
			rprintfStr("<D sn=\"");
			rprintfStr(Label.label);
			rprintf("\">%d",bit);
			rprintfStr("</D>\n");
		}
	}
	rprintfStr("</DIO>\n");
}

void setServoPosition(void)
{
	uint16_t pulse_width = (uint16_t)cmdlineGetArgInt(1);
	uint16_t delay       = (uint8_t)cmdlineGetArgInt(2);
	SetServoPosition(pulse_width, delay);
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


//void SetSSRPower(void)
//{
//	uint16_t pulse_width = (uint16_t)cmdlineGetArgInt(1);
//	SetSSRPwm(pulse_width);
//}
