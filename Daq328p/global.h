/*
 * global.h
 *
 *  Created on: Sep 11, 2009
 *      Author: Andrzej
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

// global AVRLIB defines
#include "avrlibdefs.h"
// global AVRLIB types definitions
#include "avrlibtypes.h"

// CYCLES_PER_US is used by some short delay loops
#define CYCLES_PER_US ((F_CPU+500000)/1000000) 	// cpu cycles per microsecond

// size of command database
// (maximum number of commands the cmdline system can handle)
#define CMDLINE_MAX_COMMANDS	30

// maximum length (number of characters) of each command string
// (quantity must include one additional byte for a null terminator)
#define CMDLINE_MAX_CMD_LENGTH	10

// allotted buffer size for command entry
// (must be enough chars for typed commands and the arguments that follow)
#define CMDLINE_BUFFERSIZE		80

// number of lines of command history to keep
// (each history buffer is CMDLINE_BUFFERSIZE in size)
// ***** ONLY ONE LINE OF COMMAND HISTORY IS CURRENTLY SUPPORTED
#define CMDLINE_HISTORYSIZE		1

//#define THERM_PORT PORTB
//#define THERM_DDR DDRB
//#define THERM_PIN PINB
//#define THERM_DQ  PINB0
///* Utils */
//#define THERM_INPUT_MODE() THERM_DDR&=~(1<<THERM_DQ)
//#define THERM_OUTPUT_MODE() THERM_DDR|=(1<<THERM_DQ)
//#define THERM_LOW() THERM_PORT&=~(1<<THERM_DQ)
//#define THERM_HIGH() THERM_PORT|=(1<<THERM_DQ)

#endif /* GLOBAL_H_ */
