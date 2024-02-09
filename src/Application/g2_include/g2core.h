/*
 * g2core.h - tinyg2 main header
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart, Jr.
 * Copyright (c) 2010 - 2014 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* Is this code over documented? Possibly.
 * We try to follow this (at least we are evolving to it). It's worth a read.
 * ftp://ftp.idsoftware.com/idstuff/doom3/source/CodeStyleConventions.doc
 */
#ifndef TINYG_H_ONCE
#define TINYG_H_ONCE

// common system includes
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
//#include "intrinsics.h"//sme: nop
#include "main.h"
   
/****** REVISIONS ******/   
#include "g2_error.h" 
  

#define value_flt value
   
/****** COMPILE-TIME SETTINGS ******/
#define __TEXT_MODE                 // enable text mode support (~14Kb) (also disables help screens)
#define __HELP_SCREENS              // enable help screens      (~3.5Kb)
#define __USER_DATA                 // enable user defined data groups
#define __STEP_CORRECTION           // enable virtual encoder step correction

/****** DEVELOPMENT SETTINGS ******/
#define __STEP_CORRECTION
#define __DIAGNOSTICS                           //Edge,  enables various debug functions
#define __DIAGNOSTIC_PARAMETERS			// enables system diagnostic parameters (_xx) in config_app

//#define __DEBUG_SETTINGS			// special settings. See settings.h
//#define __SUPPRESS_QUEUE_REPORTS //  sme: debug only

/* enumerate DDA stepper step pulse state */
typedef enum
{
  STEP_PULSE_STATE_OFF,
  STEP_PULSE_STATE_ON,
  STEP_PULSE_STATE_COUNT
}StepperStepPulseStateE;



/******************************************************************************
 ***** TINYG APPLICATION DEFINITIONS ******************************************
 ******************************************************************************/

typedef uint16_t magic_t;	// magic number size
#define MAGICNUM 0x12EF		// used for memory integrity assertions
#define BAD_MAGIC(a) (a != MAGICNUM)    // simple assertion test

// Note: If you change COORDS you must adjust the entries in cfgArray table in config.c
/***** Axes, motors & PWM channels used by the application *****/
// Axes, motors & PWM channels must be defines (not enums) so #ifdef <value> can be used

#define AXES		4//sme: just x,y,z,a    	// number of axes supported in this version
#define HOMING_AXES	3//sme: just x,y,z,a 4		// number of axes that can be homed (assumes Zxyabc sequence)
#define MOTORS		4//sme: just x,y,z,a 6		// number of motors on the board
#define COORDS		6// number of supported coordinate systems (1-6)

#ifndef TOOLS
#define TOOLS 5//8==ATC_TOOL_ID_COUNT must keep in sync //32        // number of entries in tool table (index starts at 1)
#endif

#define PWMS		2// number of supported PWM channels
#define NUM_MOTORS MOTORS //sme
#define NUM_AXES AXES     //sme


/**** Stepper DDA and dwell timer settings ****/
#define FREQUENCY_DDA		100000.0// Hz step frequency. Interrupts actually fire at 2x (200 KHz)
#define FREQUENCY_DWELL		1000UL//observe this is 1 millisecone per tick
#define MIN_SEGMENT_MS ((float)1.0)

//sme: debug planning queue full: 
#define PLANNER_QUEUE_SIZE (100)//11-2-2022 was 64
#define SECONDARY_QUEUE_SIZE (24)//11-2-2022 was 12

// Note: If you change COORDS you must adjust the entries in cfgArray table in config.c
typedef enum {
    AXIS_GET_NEXT_ERROR = -2,//sme
    AXIS_INIT_UNDEFINED = -1,//sme
    AXIS_HOMING_DONE = -1,//sme
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_A,
    LAST_LINEAR_AXIS = AXIS_Z,//sme               
    AXIS_COUNT //SME
} cmAxes;
 
typedef enum {  // external representation of axes (used in initialization)
    AXIS_X_EXTERNAL = 0,
    AXIS_Y_EXTERNAL,
    AXIS_Z_EXTERNAL,
    AXIS_Y2_EXTERNAL,
    AXIS_A_EXTERNAL=AXIS_Y2_EXTERNAL,
    AXIS_B_EXTERNAL,
    AXIS_C_EXTERNAL,

} cmAxesExternal;

/* Offsets: */
typedef enum 
{
    OFS_I = 0,
    OFS_J,
    OFS_K
} cmIJKOffsets;

/* SME: below states enums don't work. however, we must harmonize motor number with axis,
   and that will be
   MOTOR_1= 0 =x-axis, 
   MOTOR_2= 0 =y-axis,
   MOTOR_3= 0 =z-axis,
*/
typedef enum 
{
    MOTOR_1 = 0,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4 
} cmMotors;

typedef enum 
{
   PWM_ID1 = 0,
   PWM_SPINDLE=PWM_ID1,
   PWM_ID2
} cmPWMs;
#define PWM_1 PWM_ID1
/************************************************************************************
 ***** PLATFORM COMPATIBILITY *******************************************************
 ************************************************************************************/
					// Use macros to fake out AVR's and other AVRisms.
 
typedef uint8_t char_t;			// In the ARM/GCC++ version char_t is typedef'd to uint8_t
 					// because in C++ uint8_t and char are distinct types and
					// we want chars to behave as uint8's		// gets rely on nv->index having been set
#define GET_TABLE_WORD(a)  cfgArray[nv->index].a	// get word value from cfgArray
#define GET_TABLE_BYTE(a)  cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TABLE_FLOAT(a) cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TOKEN_BYTE(a)  (char )cfgArray[i].a	// get token byte value from cfgArray
#define GET_TOKEN_STRING(i,a) strcpy_P(a, (char *)&cfgArray[(index_t)i].token); // populate the token string given the index
#define GET_TEXT_ITEM(b,a) b[a]			       // get text from an array of strings in flash
#define GET_UNITS(a) msg_units[cm_get_units_mode(a)]

// IO settings
#define STD_IN 0				       // STDIO defaults (stdio is not yet used in the ARM version)
#define STD_OUT 0
#define STD_ERR 0

/* String compatibility
 *
 * The ARM stdio functions we are using still use char as input and output. The macros
 * below do the casts for most cases, but not all. Vararg functions like the printf()
 * family need special handling. These like char * as input and require casts as per:
 *
 *   printf((const char *)"Good Morning Hoboken!\n");
 *
 * The AVR also has "_P" variants that take strings as args.
 * On the ARM/GCC++ the _P functions are just aliases of the non-P variants.
 */
#define strncpy(d,s,l) (char *)strncpy((char *)d, (char *)s, l)
#define strncat(d,s,l) (char *)strncat((char *)d, (char *)s, l)//unavoidable compiler warning is using strncat; all commentators suggest use strncpy instead 
#define strpbrk(d,s) (char *)strpbrk((char *)d, (char *)s)
#define strcpy(d,s) (char *)strcpy((char *)d, (char *)s)
#define strcat(d,s) (char *)strcat((char *)d, (char *)s)
#define strstr(d,s) (char *)strstr((char *)d, (char *)s)
#define strchr(d,s) (char *)strchr((char *)d, (char)s)
#define strcmp(d,s) strcmp((char *)d, (char *)s)
#define strtod(d,p) strtod((char *)d, (char **)p)
#define strtof(d,p) strtof((char *)d, (char **)p)
#define strlen(s) strlen((char *)s)
 
/* sme: replacing fprinf_P with sprintf */
#define SPRINTF_DEST_BUF_LEN 1024 //sme debug, 10-7-2019 try doubling to  2048: did not help!!
extern char global_sprintf_dest_buf[SPRINTF_DEST_BUF_LEN];
extern int comms_mgr_write_msg( char * msg);
//#define SME_DISABLE_NV_WRITES   
#define SME_REPLACE_FPRINTF_WITH_SPRINTF
//#define SME_REPLACE_PRINTF_WITH_SPRINTF
#define ENABLE_NV_CALLS_CALLS

#ifdef SME_REPLACE_PRINTF_WITH_SPRINTF
#define PSTR global_sprintf_dest_buf, (const char *)
#define printf_P sprintf
#else
#define PSTR (const char *)
#define printf_P printf		// these functions want char * as inputs, not char *
#endif

#ifdef SME_REPLACE_FPRINTF_WITH_SPRINTF
#undef stderr
#define stderr global_sprintf_dest_buf
#define STDERR_SUBSTITUTE global_sprintf_dest_buf
#define fprintf_P sprintf
#define fprintf sprintf
#else
#define fprintf_P fprintf	// just sayin'
#endif

#define sprintf_P sprintf
#define strcpy_P strcpy
#define strncpy_P strncpy

extern stat_t status_code;	// allocated in main.c
#define MESSAGE_LEN 80		// global message string storage allocation

extern char  global_string_buf[];// allocated in main.c
extern  char *get_status_message(stat_t status);
extern  int g2_main(void); 

#endif // End of include guard: TINYG2_H_ONCE
