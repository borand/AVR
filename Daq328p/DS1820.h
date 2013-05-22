#include <avr/io.h>
#include <stdio.h>
#include "rprintf.h"

#ifndef F_CPU
#define F_CPU 16000000UL 		//Your clock speed in Hz (3Mhz here)
#endif

#define LOOP_CYCLES 16		//Number of cycles that the loop takes
#define us(num) (num/(LOOP_CYCLES*(1/(F_CPU/1000000.0))))

/* list of these commands translated into C defines:*/
#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xbe
#define THERM_CMD_WSCRATCHPAD 0x4e
#define THERM_CMD_CPYSCRATCHPAD 0x48
#define THERM_CMD_RECEEPROM 0xb8
#define THERM_CMD_RPWRSUPPLY 0xb4
#define THERM_CMD_SEARCHROM 0xf0
#define THERM_CMD_READROM 0x33
#define THERM_CMD_MATCHROM 0x55
#define THERM_CMD_SKIPROM 0xcc
#define THERM_CMD_ALARMSEARCH 0xec
/* constants */
#define THERM_DECIMAL_STEPS_12BIT 625 //.0625

//**************************************************************************
// GENERIC MACROS
#define PIN_LOW(reg, bit)  reg &=~(1<<bit)
#define PIN_HIGH(reg, bit) reg |= (1<<bit)
#define READ_PIN(reg,bit)  reg &= (1<<bit)
#define TOGGLE(reg,bit)    reg ^= (_BV(bit))

#define THERM_PORT PORTB
#define THERM_DDR  DDRB
#define THERM_PIN  PINB
#define THERM_DQ   PINB0
#define TRIG_PORT  PORTB
#define TRIG_DDR   DDRB
#define TRIG_PIN   PINB5

#define DS18B20    40
#define DS18S20    16

/* Utils */
#define THERM_INPUT_MODE(reg, bit)  reg &=~ (1<<bit)
#define THERM_OUTPUT_MODE(reg, bit) reg |=  (1<<bit)
#define THERM_LOW(reg, bit)  reg &=~(1<<bit)
#define THERM_HIGH(reg, bit) reg |=(1<<bit)

#define TRIG_LOW(bit)  TRIG_PORT&=~(1<<TRIG_PIN)
#define TRIG_HIGH(bit) TRIG_PORT|=(1<<TRIG_PIN)

//#define THERM_DEBUG 1

typedef struct //Hist_t EEPRAM
{
	uint16_t t_conv;
	uint16_t t_reset_tx;
	uint16_t t_reset_rx;
	uint8_t  t_write_low;
	uint8_t  t_write_slot;
	uint8_t  t_read_samp;
	uint8_t  t_read_slot;
	uint8_t  dev[20][8];
} EE_RAM_t;

typedef struct //DS1820 variables
{
	uint8_t  scratchpad[9];
	uint8_t  devID[8];
	int8_t   temp_digit;
	int16_t  temp_decimal;
	uint8_t  therm_pin;
	uint16_t t_conv;
	uint16_t t_reset_tx;
	uint16_t t_reset_rx;
	uint8_t  t_write_low;
	uint8_t  t_write_slot;
	uint8_t  t_read_samp;
	uint8_t  t_read_slot;
} DS1820_t;

void    therm_init(void);
void    therm_delay(uint16_t delay);
uint8_t therm_reset();
void    therm_write_bit(uint8_t bit);
uint8_t therm_read_bit(void);
uint8_t therm_read_byte(void);
void    therm_write_byte(uint8_t byte);
void    therm_print_scratchpad();
void    therm_print_devID();
void    therm_print_timing();
void    therm_set_timing(uint8_t time, uint16_t interval);
void    therm_set_pin(uint8_t newPin);
//

uint8_t therm_read_devID();
void    therm_send_devID();
uint8_t therm_load_devID(uint8_t devNum);
void    therm_set_devID(uint8_t *devID);
void    therm_save_devID(uint8_t devNum);
uint8_t therm_read_scratchpad(uint8_t numOfbytes);
void    therm_start_measurement();
uint8_t therm_read_result(int16_t *temperature);
uint8_t therm_read_temperature(uint8_t devNum, int16_t *temperature);

uint8_t therm_crc_is_OK(uint8_t *scratchpad, uint8_t *crc, uint8_t numOfBytes);
uint8_t therm_computeCRC8(uint8_t inData, uint8_t seed);
