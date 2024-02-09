/*
 * settings.h - default runtime settings
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*	The values in this file are the default settings that are loaded into a virgin EEPROM,
 *	and can be changed using the config commands. After initial load the EEPROM values
 *	(or changed values) are used.
 *
 *	System and hardware settings that you shouldn't need to change are in hardware.h
 *	Application settings that also shouldn't need to be changed are in tinyg.h
 */

#ifndef SETTINGS_H_ONCE
#define SETTINGS_H_ONCE

/**** GENERAL SETTINGS ******************************************************/

// **** PLEASE NOTE **** Any of these may be overridden in machine profiles
// Do not assume these are the effective settings. Check the machine profile
#define RADIUS_MIN     (0.0001)   // minimum value for ABC radius settings
// Machine configuration settings
//#define CHORDAL_TOLERANCE 			0.01					// chordal accuracy for arc drawing (in mm)
#define SOFT_LIMIT_ENABLE			0						// 0 = off, 1 = on

//#define MOTOR_POWER_MODE			MOTOR_POWERED_IN_CYCLE	// default motor power mode (see cmMotorPowerMode in stepper.h)
#define MOTOR_POWER_TIMEOUT			2.00					// motor power timeout in seconds
#define MOTOR_POWER_LEVEL			0.375					// default motor power level 0.00 - 1.00 (ARM only)
//sme, use othermill settings #define PAUSE_DWELL_TIME			0.0

// Communications and reporting settings
#define COMM_MODE				AUTO_MODE//per edge,JSON_MODE				// one of: TEXT_MODE, JSON_MODE
#define TEXT_VERBOSITY				TV_VERBOSE				// one of: TV_SILENT, TV_VERBOSE
//sme not used #define NETWORK_MODE				NETWORK_STANDALONE

#define JSON_VERBOSITY				JV_LINENUM				// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
//sme; not used #define JSON_SYNTAX_MODE 			JSON_SYNTAX_STRICT		// one of JSON_SYNTAX_RELAXED, JSON_SYNTAX_STRICT
#define JSON_FOOTER_STYLE			1//sme: not in edge!						// 1 = footer w/checksum, 2 = footer w/window slots
#define JSON_FOOTER_DEPTH			0//sme: not in edge!						// 0 = footer is child of R, 1 = footer is child of response object (deprecated)

#define STATUS_REPORT_VERBOSITY		SR_FILTERED	// one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS		100		// milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS	250		// milliseconds - set $SV=0 to disable

#define STATUS_REPORT_DEFAULTS      "mpox", "mpoy", "mpoz", \
                                    "ofsx", "ofsy", "ofsz", \
                                    "g55x", "g55y", "g55z", \
                                    "unit", "stat", "coor", "momo", "dist", \
                                    "home", "mots", "plan", "line", "path", \
                                    "frmo", "prbe", "safe", "estp", "spc", \
                                    "hold", "macs", "cycs", "sps"   
#define QUEUE_REPORT_VERBOSITY		QR_OFF	        // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS		MILLIMETERS	// MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE		CANON_PLANE_XY	// CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM	G55		// per edge G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL 	PATH_CONTINUOUS

// Comm mode and echo levels
#define COM_EXPAND_CR			false
#define COM_ENABLE_ECHO			false
#define COM_ENABLE_FLOW_CONTROL		FLOW_CONTROL_XON // FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS

//**** DEBUG SETTINGS ****

#ifdef __DEBUG_SETTINGS

#undef JSON_FOOTER_STYLE
#define JSON_FOOTER_STYLE		2	// 1 = footer w/checksum, 2 = footer w/window slots

#undef JSON_VERBOSITY
//#define JSON_VERBOSITY				JV_SILENT// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define JSON_VERBOSITY			JV_LINENUM	// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE

#undef STATUS_REPORT_DEFAULTS
#define STATUS_REPORT_DEFAULTS "posx","posy","posz","posa","feed","vel","unit","coor","dist","frmo","stat"

#undef STATUS_REPORT_VERBOSITY
#define STATUS_REPORT_VERBOSITY		SR_FILTERED	// one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#endif

/**** MACHINE PROFILES ******************************************************/

// machine default profiles - choose only one:

#ifdef SETTINGS_FILE
#define SETTINGS_FILE_PATH <settings/SETTINGS_FILE>
#include SETTINGS_FILE_PATH
#else
#ifdef CHABO_DCNC
#include "g2_settings_bt_cncmm.h"
#elif defined CHABO_MINIMILL
#include "g2_settings_bt_minimill.h"
#elif defined CHABO_MINIMILL_ESC
#include "g2_settings_bt_minimill_esc.h"
#elif defined CHABO_LFP
#include "g2_settings_bt_lfp.h"
#elif defined CHABO_PLOTTER
#include "g2_settings_bt_plotter.h"
#endif
#endif


// If PWM_1 is not defined fill it with default values
#ifndef	P1_PWM_FREQUENCY
#define P1_PWM_FREQUENCY                100	// in Hz
#define P1_SPEED_LO                  1000	// in RPM (arbitrary units)
#define P1_SPEED_HI                  2000
#define P1_PHASE_LO                  0.125	// phase [0..1]
#define P1_PHASE_HI                  0.2
#define P1_PWM_K_VALUE               1.0
#endif //P1_PWM_FREQUENCY

/*** User-Defined Data Defaults ***/

#define USER_DATA_A0	0
#define USER_DATA_A1	0
#define USER_DATA_A2	0
#define USER_DATA_A3	0
#define USER_DATA_B0	0
#define USER_DATA_B1	0
#define USER_DATA_B2	0
#define USER_DATA_B3	0
#define USER_DATA_C0	0
#define USER_DATA_C1	0
#define USER_DATA_C2	0
#define USER_DATA_C3	0
#define USER_DATA_D0	0
#define USER_DATA_D1	0
#define USER_DATA_D2	0
#define USER_DATA_D3	0

#endif // End of include guard: SETTINGS_H_ONCE
