/*
 * g2_tinyg2.h - tinyg2 main header
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
#include "intrinsics.h"//sme: nop
/****** REVISIONS ******/   
#include "g2_error.h"   
#if 0//edge
#include "g2_g2core_info.h"            // see this file for build number and other identifying information
#else

#ifndef G2CORE_FIRMWARE_BUILD
#define G2CORE_FIRMWARE_BUILD   	101.06 
#endif
#define HW_PLATFORM_G2CORE_V9 1//sme
#define HW_VERSION_G2CORE_V9I  2//sme
#define G2CORE_FIRMWARE_VERSION		0.99			// firmware major version
#define G2CORE_CONFIG_VERSION		6			// add fxa slot for fixturing information
#define G2CORE_HARDWARE_PLATFORM        HW_PLATFORM_G2CORE_V9	// hardware platform indicator (2 = Native Arduino Due)
#define G2CORE_HARDWARE_VERSION		HW_VERSION_G2CORE_V9I	// hardware platform revision number
#define G2CORE_HARDWARE_VERSION_MAX    (G2CORE_HARDWARE_VERSION)

#endif
 
/****** COMPILE-TIME SETTINGS ******/
#if 1//edge
#define __TEXT_MODE                 // enable text mode support (~14Kb) (also disables help screens)
#define __HELP_SCREENS              // enable help screens      (~3.5Kb)
#define __USER_DATA                 // enable user defined data groups
#define __STEP_CORRECTION           // enable virtual encoder step correction
#else
#define __TEXT_MODE			// enables text mode support (~10Kb)
#define __HELP_SCREENS			// enables help screens (~3.5Kb)
#define __CANNED_TESTS 			// enables $tests 	(~12Kb)
#define __TEST_99 			// enables diagnostic test 99
#endif

/****** DEVELOPMENT SETTINGS ******/
#define __STEP_CORRECTION

#define __DIAGNOSTICS                           //Edge,  enables various debug functions
#define __DIAGNOSTIC_PARAMETERS			// enables system diagnostic parameters (_xx) in config_app

//#define __DEBUG_SETTINGS			// special settings. See settings.h
//#define __SUPPRESS_QUEUE_REPORTS //  sme: debug only




/******************************************************************************
 ***** TINYG APPLICATION DEFINITIONS ******************************************
 ******************************************************************************/

typedef uint16_t magic_t;	// magic number size
#define MAGICNUM 0x12EF		// used for memory integrity assertions

/***** Axes, motors & PWM channels used by the application *****/
// Axes, motors & PWM channels must be defines (not enums) so #ifdef <value> can be used

#define AXES		3//sme: just x,y,z for now6	// number of axes supported in this version
#define HOMING_AXES	3//sme: just x,y,z 4		// number of axes that can be homed (assumes Zxyabc sequence)
#define MOTORS		3//sme: just x,y,z 6		// number of motors on the board
#define COORDS		3//sme:  thought this would be okjust x,y,z 6	// number of supported coordinate systems (1-6)
#define PWMS		2// number of supported PWM channels
#define NUM_MOTORS MOTORS//sme
#define NUM_AXES AXES    //sme
#define SME_DEBUG_AXES		4//sme: just x,y,z for now 6	// number of axes supported in this version
#define SME_DEBUG_HOMING_AXES	4//sme: just x,y,z 4		// number of axes that can be homed (assumes Zxyabc sequence)
#define SME_DEBUG_MOTORS        4//sme: just x,y,z 6		// number of motors on the board
#define SME_DEBUG_COORDS	4//sme:  thought this would be ok just x,y,z 6 // number of supported coordinate systems (1-6)


/**** Stepper DDA and dwell timer settings ****/
#define FREQUENCY_DDA		200000.0	// Hz step frequency. Interrupts actually fire at 2x (400 KHz)
#define FREQUENCY_DWELL		1000UL


// Note: If you change COORDS you must adjust the entries in cfgArray table in config.c

typedef enum {
    AXIS_GET_NEXT_ERROR = -2,//sme
    AXIS_INIT_UNDEFINED = -1,//sme
    AXIS_HOMING_DONE = -1,//sme
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_A,
    //AXIS_B,
    //AXIS_C
    AXIS_COREXY_A = AXIS_X, // CoreXY uses A and B
    AXIS_COREXY_B = AXIS_Y,
    AXIS_4WIRE_A = AXIS_X, // 4Wire uses A, B, C, D
    AXIS_4WIRE_B = AXIS_Y, // 4Wire uses A, B, C, D
    AXIS_4WIRE_C = AXIS_Z, // 4Wire uses A, B, C, D
    AXIS_4WIRE_D = AXIS_A, // 4Wire uses A, B, C, D 
    //AXIS_4WIRE_Z = AXIS_B, // 4Wire uses A, B, C, D    
    AXIS_COUNT //SME
} cmAxes;

typedef enum 
{  // external representation of axes (used in initialization)
    AXIS_X_EXTERNAL = 0,
    AXIS_Y_EXTERNAL,
    AXIS_Z_EXTERNAL,
    AXIS_A_EXTERNAL,
   // AXIS_B_EXTERNAL,
    //AXIS_C_EXTERNAL, 
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
    MOTOR_4,
} cmMotors;
typedef enum 
{
    PWM_1 = 0,
    PWM_2
} cmPWMs;

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
#define TABLE_ELEMENT_IS_FLOAT() (( cfgArray[nv->index].flags & TYPE_FLOAT)==TYPE_FLOAT)
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
#define strncat(d,s,l) (char *)strncat((char *)d, (char *)s, l)
#define strpbrk(d,s) (char *)strpbrk((char *)d, (char *)s)
#define strcpy(d,s) (char *)strcpy((char *)d, (char *)s)
#define strcat(d,s) (char *)strcat((char *)d, (char *)s)
#define strstr(d,s) (char *)strstr((char *)d, (char *)s)
#define strchr(d,s) (char *)strchr((char *)d, (char)s)
#define strcmp(d,s) strcmp((char *)d, (char *)s)
#define strtod(d,p) strtod((char *)d, (char **)p)
#define strtof(d,p) strtof((char *)d, (char **)p)
#define strlen(s) strlen((char *)s)
#define isdigit(c) isdigit((char)c)
#define isalnum(c) isalnum((char)c)
#define tolower(c) (char )tolower((char)c)
#define toupper(c) (char )toupper((char)c)
   
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
char *get_status_message(stat_t status);

 
int g2_main(void); 


#endif // End of include guard: TINYG2_H_ONCE
