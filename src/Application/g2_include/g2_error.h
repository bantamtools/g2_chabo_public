/*
 * g2_error.h - g2core status codes
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2010 - 2018 Robert Giseburt
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
#ifndef ERROR_H_ONCE
#define ERROR_H_ONCE

/************************************************************************************
 * STATUS CODES
 *
 * Most of the status codes (except STAT_OK) are exceptions. These are typically
 * returned by the failed command and reported back via JSON or text.
 *
 * Status codes are divided into ranges for clarity and extensibility. At some point
 * this may break down and the whole thing will get messy(er), but it's advised not
 * to change the values of existing status codes once they are in distribution.
 *
 * Ranges are:
 *
 *   0 - 19     OS, communications and low-level status
 *              This range is aligned with the XIO codes and must be so (v8 only)
 *              Please don't change them without checking the corresponding values in xio.h
 *
 *  20 - 99     Generic internal and application errors.
 *              Internal errors start at 20 and work up,
 *              Assertion failures start at 99 and work down.
 *
 * 100 - 129    Generic data and input errors - not specific to Gcode or g2core
 *
 * 130 - 255    Gcode and g2core application errors and warnings
 *
 * See status_codes.cpp for associated message strings. Any changes to the codes may
 * also require changing the message strings and string array in status_codes.cpp
 */

// **** Declarations, functions and macros. See main.cpp for implementation ****
 
typedef uint8_t stat_t;
extern stat_t status_code;
extern  char *get_status_message(stat_t status);

// ritorno is a handy way to provide exception returns
// It returns only if an error occurred. (ritorno is Italian for return)
#define ritorno(a) if((status_code=a) != STAT_OK) { return(status_code); }

/**********************
 **** STATUS CODES ****
 **********************/

// OS, communications and low-level status (must align with XIO_xxxx codes in xio.h)
#define STAT_OK 0                       // function completed OK
#define STAT_ERROR 1                    // generic error return (EPERM)
#define STAT_EAGAIN 2                   // function would block here (call again)
#define STAT_NOOP 3                     // function had no-operation
#define STAT_COMPLETE 4                 // operation is complete
#define STAT_SHUTDOWN 5                 // operation was shutdown (terminated gracefully)
#define STAT_PANIC 6                    // system panic (not graceful)
#define STAT_EOL 7                      // function returned end-of-line
#define STAT_EOF 8                      // function returned end-of-file
#define STAT_FILE_NOT_OPEN 9
#define STAT_FILE_SIZE_EXCEEDED 10
#define STAT_NO_SUCH_DEVICE 11
#define STAT_BUFFER_EMPTY 12
#define STAT_BUFFER_FULL 13
#define STAT_BUFFER_FULL_FATAL 14
#define STAT_INITIALIZING 15            // initializing - not ready for use
#define STAT_ENTERING_BOOT_LOADER 16    // this code actually emitted from boot loader, not g2
#define STAT_FUNCTION_IS_STUBBED 17
#define STAT_ALARM 18                   // system alarm triggered
#define STAT_NO_DISPLAY 19              // suppress results display - presumably handled upstream
// NOTE: XIO codes align to here

// Internal errors and startup messages
#define STAT_INTERNAL_ERROR 20          // unrecoverable internal error
#define STAT_INTERNAL_RANGE_ERROR 21    // number range other than by user input
#define STAT_FLOATING_POINT_ERROR 22    // number conversion error
#define STAT_DIVIDE_BY_ZERO 23
#define STAT_INVALID_ADDRESS 24
#define STAT_READ_ONLY_ADDRESS 25
#define STAT_INIT_FAILURE 26
#define STAT_ERROR_27 27                // was ALARMED in 0.97
#define STAT_FAILED_TO_GET_PLANNER_BUFFER 28
#define STAT_GENERIC_EXCEPTION_REPORT 29 // used for test

#define STAT_PREP_LINE_MOVE_TIME_IS_INFINITE 30
#define STAT_PREP_LINE_MOVE_TIME_IS_NAN 31
#define STAT_FLOAT_IS_INFINITE 32
#define STAT_FLOAT_IS_NAN 33
#define STAT_PERSISTENCE_ERROR 34
#define STAT_BAD_STATUS_REPORT_SETTING 35
#define STAT_FAILED_GET_PLANNER_BUFFER 36
 
#define STAT_BACKPLAN_HIT_RUNNING_BUFFER 37//sme 3-23-2021: this was missing yet the error string was defined for this index
#define STAT_VFD_MODBUS_COMMS_FAULT 38  //sme 3-23-2021 Note: Spindle will continue running when comms are offline until f/w asserts the VFD_OnOff_RST gpio output

#define STAT_VFD_DRIVE_TRIPPED 39//sme optidrive tripped during operation an error code is provided

#define STAT_ATC_TOOL_SENSOR_FAULT_BOTH_SET 40 //One or both sensors are stuck high
#define STAT_ATC_TOOL_SENSOR_FAULT_BOTH_RESET 41 //One or both sensors are stuck low
#define STAT_ATC_UNIDENTIFIED_TOOL_ENGAGED 42 //Most likely a left-over installed tool discovered since a new powered of the mill
#define STAT_ATC_TOOL_NOT_ENGAGED 43 //Solenoid was commanded but Either failed air supply to solenoid or failed solenoid, but sensor reading is correct
#define STAT_ATC_TOOL_NOT_RELEASED 44 //Either failed air supply to solenoid or failed solenoid, but sensor reading is correct
#define STAT_ATC_TOOL_SENSORS_BOTH_FAULTY 45
#define STAT_ATC_RELEASED_SENSOR_STUCK_LOW 46
#define STAT_ATC_RELEASED_SENSOR_STUCK_HIGH 47
#define STAT_ATC_ENGAGED_SENSOR_STUCK_LOW 48
#define STAT_ATC_ENGAGED_SENSOR_STUCK_HIGH 49

#define STAT_ATC_INSTALLED_TOOL_NOT_SENSED 50//Was engaged. Possibly user released the tool manually or the engaged sensor is faulty
#define STAT_NO_MEDIUM_IN_DRIVE 51
#define STAT_MEDIUM_WRITE_PROTECTED 52
#define STAT_ERROR_DRIVE_NOT_INITIALIZED 53
#define STAT_ATC_TOOL_CARRIAGE_NOT_RELEASED 54
#define STAT_ATC_REQUEST_WHILE_SPINDLE_RUNNING 55
#define STAT_LIMIT_SWITCH_STUCK_LOW 56
#define STAT_LIMIT_SWITCH_STUCK_HIGH 57
#define STAT_LIMIT_SWITCH_REDUNDACY_FAULT 58
#define STAT_MANUAL_TOOL_ENGAGED_DETECTED 59//toggle switch used to install tool --outside of ATC control
#define STAT_MANUAL_TOOL_RELEASED_DETECTED 60//toggle switch used to release tool --outside of ATC control
#define STAT_ACTIVE_SWITCH_ANOMALY 61 //switch is active outside expected usage
#define STAT_ATC_SAFETY_RESPONSE 62
#define STAT_USB_COMMS_WDOG_TIMEOUT 63
#define STAT_USB_COMMS_DISCONNECTED 64
#define STAT_STEPPER_OT_WARNING 65
#define STAT_STEPPER_OT_SHUTDOWN 66
#define STAT_SPINDLE_OVER_TEMP 67
#define STAT_ERROR_68 68
#define STAT_ERROR_69 69

#define STAT_ERROR_70 70
#define STAT_ERROR_71 71
#define STAT_ERROR_72 72
#define STAT_ERROR_73 73
#define STAT_ERROR_74 74
#define STAT_ERROR_75 75
#define STAT_ERROR_76 76
#define STAT_ERROR_77 77
#define STAT_ERROR_78 78
#define STAT_ERROR_79 79

#define STAT_ERROR_80 80
#define STAT_ERROR_81 81
#define STAT_ERROR_82 82
#define STAT_ERROR_83 83
#define STAT_ERROR_84 84
#define STAT_ERROR_85 85
#define STAT_ERROR_86 86

// Assertion failures - build down from 99 until they meet the system internal errors

#define STAT_SPINDLE_ASSERTION_FAILURE 86
#define STAT_EXEC_ALINE_ASSERTION_FAILURE 87
#define STAT_BUFFER_FREE_ASSERTION_FAILURE 88
#define STAT_STATE_MANAGEMENT_ASSERTION_FAILURE 89
#define STAT_CONFIG_ASSERTION_FAILURE 90
#define STAT_XIO_ASSERTION_FAILURE 91
#define STAT_ENCODER_ASSERTION_FAILURE 92
#define STAT_STEPPER_ASSERTION_FAILURE 93
#define STAT_PLANNER_ASSERTION_FAILURE 94
#define STAT_CANONICAL_MACHINE_ASSERTION_FAILURE 95
#define STAT_CONTROLLER_ASSERTION_FAILURE 96
#define STAT_STACK_OVERFLOW 97
#define STAT_MEMORY_FAULT 98                    // generic memory corruption detected by magic numbers
#define STAT_GENERIC_ASSERTION_FAILURE 99       // generic assertion failure - unclassified

// Application and data input errors

// Generic data input errors

#define STAT_UNRECOGNIZED_NAME 100              // parser didn't recognize the name
#define STAT_INVALID_OR_MALFORMED_COMMAND 101   // malformed line to parser
#define	STAT_MALFORMED_COMMAND_INPUT 101// legacy malformed line to parser
#define STAT_BAD_NUMBER_FORMAT 102              // number format error
#define STAT_UNSUPPORTED_TYPE 103               // an otherwise valid JSON type is not supported
#define STAT_PARAMETER_IS_READ_ONLY 104         // input error: parameter cannot be set
#define STAT_PARAMETER_CANNOT_BE_READ 105       // input error: parameter cannot be returned
#define STAT_COMMAND_NOT_ACCEPTED 106           // input error: command cannot be accepted at this time
#define STAT_INPUT_EXCEEDS_MAX_LENGTH 107       // input error: input string is too long
#define STAT_INPUT_LESS_THAN_MIN_VALUE 108      // input error: value is under minimum
#define STAT_INPUT_EXCEEDS_MAX_VALUE 109        // input error: value is over maximum
#define STAT_INPUT_VALUE_RANGE_ERROR 110        // input error: value is out-of-range

#define STAT_JSON_SYNTAX_ERROR 111              // JSON input string is not well formed
#define STAT_JSON_TOO_MANY_PAIRS 112            // JSON input string has too many JSON pairs
#define STAT_JSON_OUTPUT_TOO_LONG 113           // JSON output exceeds buffer size
#define STAT_NESTED_TXT_CONTAINER 114           // JSON 'txt' fields cannot be nested
#define STAT_MAX_DEPTH_EXCEEDED 115             // JSON exceeded maximum nesting depth
#define STAT_VALUE_TYPE_ERROR 116               // JSON value does not agree with variable type

#define STAT_INPUT_FROM_MUTED_CHANNEL_ERROR 117 // input from a muted channel was ignored
#define STAT_CHECKSUM_MATCH_FAILED 118          // the provided checksum didn't match
#define STAT_LINE_NUMBER_OUT_OF_SEQUENCE 119    // the provided line number was out of sequence
#define STAT_MISSING_LINE_NUMBER_WITH_CHECKSUM 120 // if a checksum is provided, a line number should be present as well

#define STAT_COMMS_RX_QUEUE_OVERWRITTEN 121     //inadequate flow control from desktop app streaming messages
#define STAT_PROBE_FEEDHOLD_TIMEOUT   122       //12-20-2022 bandaid an unexplained chabo feedhold stuck state--force the feedhold to complete
#define STAT_PROBE_WAITING_TIMEOUT    123       //02-06-2023 probe waiting to contact, timed out
#define STAT_PERSISTENCE_FILE_TOO_LARGE 124     //02-20-2023  While loop vulnerable to infinite loop mitigated
#define STAT_ERROR_125 125
#define STAT_ERROR_126 126
#define STAT_ERROR_127 127
#define STAT_ERROR_128 128
#define STAT_ERROR_129 129

// Gcode errors and warnings (Most originate from NIST - by concept, not number)
// Fascinating: http://www.cncalarms.com/

#define STAT_GCODE_GENERIC_INPUT_ERROR 130      // generic error for gcode input
#define STAT_GCODE_COMMAND_UNSUPPORTED 131      // G command is not supported
#define STAT_MCODE_COMMAND_UNSUPPORTED 132      // M command is not supported
#define STAT_GCODE_MODAL_GROUP_VIOLATION 133    // gcode modal group error
#define STAT_AXIS_IS_MISSING 134                // command requires at least one axis present
#define	STAT_GCODE_AXIS_IS_MISSING 134		// legacy command requires at least one axis present
#define STAT_AXIS_CANNOT_BE_PRESENT 135         // error if G80 has axis words
#define STAT_AXIS_IS_INVALID 136                // an axis is specified that is illegal for the command
#define STAT_AXIS_IS_NOT_CONFIGURED 137         // WARNING: attempt to program an axis that is disabled
#define STAT_AXIS_NUMBER_IS_MISSING 138         // axis word is missing its value
#define STAT_AXIS_NUMBER_IS_INVALID 139         // axis word value is illegal
#define STAT_GCODE_ACTIVE_PLANE_IS_MISSING 140	// legacy  active plane is not programmed
#define STAT_ACTIVE_PLANE_IS_MISSING 140        // active plane is not programmed
#define STAT_ACTIVE_PLANE_IS_INVALID 141        // active plane selected is not valid for this command
#define STAT_FEEDRATE_NOT_SPECIFIED 142         // move has no feedrate


#define STAT_INVERSE_TIME_MODE_CANNOT_BE_USED 143  // G38.2 and some canned cycles cannot accept inverse time mode
#define STAT_ROTARY_AXIS_CANNOT_BE_USED 144     // G38.2 and some other commands cannot have rotary axes
#define STAT_GCODE_G53_WITHOUT_G0_OR_G1 145     // G0 or G1 must be active for G53
#define STAT_REQUESTED_VELOCITY_EXCEEDS_LIMITS 146
#define STAT_CUTTER_COMPENSATION_CANNOT_BE_ENABLED 147
#define STAT_PROGRAMMED_POINT_SAME_AS_CURRENT_POINT 148
#define STAT_SPINDLE_SPEED_BELOW_MINIMUM 149

#define STAT_SPINDLE_SPEED_MAX_EXCEEDED 150
#define STAT_SPINDLE_MUST_BE_OFF 151
#define STAT_SPINDLE_MUST_BE_TURNING 152            // some canned cycles require spindle to be turning when called
#define STAT_ARC_SPECIFICATION_ERROR 153            // generic arc specification error
#define STAT_ARC_HAS_IMPOSSIBLE_CENTER_POINT 154    // trap (.05 inch/.5 mm) OR ((.0005 inch/.005mm) AND .1% of radius condition
#define STAT_ARC_HAS_ROTARY_AXIS 155                // arc specification includes a rotary axis
#define STAT_ARC_AXIS_MISSING_FOR_SELECTED_PLANE 156  // arc is missing axis (axes) required by selected plane
#define STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE 157 // one or both offsets are not specified
#define STAT_ARC_RADIUS_OUT_OF_TOLERANCE 158        // WARNING - radius arc is too large - accuracy in question
#define STAT_ARC_ENDPOINT_IS_STARTING_POINT 159

#define STAT_P_WORD_IS_MISSING 160                  // P must be present for dwells and other functions
#define STAT_P_WORD_IS_INVALID 161                  // generic P value error
#define STAT_P_WORD_IS_ZERO 162
#define STAT_P_WORD_IS_NEGATIVE 163                 // dwells require positive P values
#define STAT_P_WORD_IS_NOT_AN_INTEGER 164           // G10s and other commands require integer P numbers
#define STAT_P_WORD_IS_NOT_VALID_TOOL_NUMBER 165    // invalid tool number
#define STAT_D_WORD_IS_MISSING 166
#define STAT_D_WORD_IS_INVALID 167
#define STAT_E_WORD_IS_MISSING 168
#define STAT_E_WORD_IS_INVALID 169

#define STAT_H_WORD_IS_MISSING 170
#define STAT_H_WORD_IS_INVALID 171
#define STAT_L_WORD_IS_MISSING 172
#define STAT_L_WORD_IS_INVALID 173
#define STAT_Q_WORD_IS_MISSING 174
#define STAT_Q_WORD_IS_INVALID 175
#define STAT_R_WORD_IS_MISSING 176
#define STAT_R_WORD_IS_INVALID 177
#define STAT_S_WORD_IS_MISSING 178
#define STAT_S_WORD_IS_INVALID 179

#define STAT_T_WORD_IS_MISSING 180
#define STAT_T_WORD_IS_INVALID 181

/* reserved for Gcode or other program errors */

#define STAT_ERROR_182 182
#define STAT_ERROR_183 183
#define STAT_ERROR_184 184
#define STAT_ERROR_185 185
#define STAT_ERROR_186 186
#define STAT_ERROR_187 187
#define STAT_ERROR_188 188
#define STAT_ERROR_189 189

#define STAT_ERROR_190 190
#define STAT_ERROR_191 191
#define STAT_ERROR_192 192
#define STAT_ERROR_193 193
#define STAT_ERROR_194 194
#define STAT_ERROR_195 195
#define STAT_ERROR_196 196
#define STAT_ERROR_197 197
#define STAT_ERROR_198 198
#define STAT_ERROR_199 199

// g2core errors and warnings

#define STAT_GENERIC_ERROR 200
#define STAT_MINIMUM_LENGTH_MOVE 201            // move is less than minimum length
#define STAT_MINIMUM_TIME_MOVE 202              // move is less than minimum time
#define STAT_LIMIT_SWITCH_HIT 203               // a limit switch was hit causing shutdown
#define STAT_COMMAND_REJECTED_BY_ALARM 204      // command was not processed because machine is alarmed
#define STAT_COMMAND_REJECTED_BY_SHUTDOWN 205   // command was not processed because machine is shutdown
#define STAT_COMMAND_REJECTED_BY_PANIC 206      // command was not processed because machine is paniced
#define STAT_KILL_JOB 207                       // ^d received (job kill)
#define STAT_NO_GPIO 208                        // no GPIO exists for this value

#define STAT_TEMPERATURE_CONTROL_ERROR 209      // temperature controls err'd out

#define STAT_G29_NOT_CONFIGURED 210
#define STAT_ERROR_211 211
#define	STAT_MACHINE_ALARMED 211//legacy
#define STAT_ERROR_212 212
#define STAT_ERROR_213 213
#define STAT_ERROR_214 214
#define STAT_ERROR_215 215
#define STAT_ERROR_216 216
#define STAT_ERROR_217 217
#define STAT_ERROR_218 218
#define STAT_ERROR_219 219

#define STAT_SOFT_LIMIT_EXCEEDED 220            // soft limit error - axis unspecified
#define STAT_SOFT_LIMIT_EXCEEDED_XMIN 221       // soft limit error - X minimum
#define STAT_SOFT_LIMIT_EXCEEDED_XMAX 222       // soft limit error - X maximum
#define STAT_SOFT_LIMIT_EXCEEDED_YMIN 223       // soft limit error - Y minimum
#define STAT_SOFT_LIMIT_EXCEEDED_YMAX 224       // soft limit error - Y maximum
#define STAT_SOFT_LIMIT_EXCEEDED_ZMIN 225       // soft limit error - Z minimum
#define STAT_SOFT_LIMIT_EXCEEDED_ZMAX 226       // soft limit error - Z maximum
#define STAT_SOFT_LIMIT_EXCEEDED_AMIN 227       // soft limit error - A minimum
#define STAT_SOFT_LIMIT_EXCEEDED_AMAX 228       // soft limit error - A maximum
#define STAT_SOFT_LIMIT_EXCEEDED_BMIN 229       // soft limit error - B minimum

#define STAT_SOFT_LIMIT_EXCEEDED_BMAX 230       // soft limit error - B maximum
#define STAT_SOFT_LIMIT_EXCEEDED_CMIN 231       // soft limit error - C minimum
#define STAT_SOFT_LIMIT_EXCEEDED_CMAX 232       // soft limit error - C maximum
#define STAT_SOFT_LIMIT_EXCEEDED_ARC 233        // soft limit err on arc

#define STAT_ERROR_234 234
#define STAT_ERROR_235 235
#define STAT_ERROR_236 236
#define STAT_ERROR_237 237
#define STAT_ERROR_238 238
#define STAT_ERROR_239 239

#define STAT_HOMING_CYCLE_FAILED 240            // homing cycle did not complete
#define STAT_HOMING_ERROR_BAD_OR_NO_AXIS 241
#define STAT_HOMING_ERROR_ZERO_SEARCH_VELOCITY 242
#define STAT_HOMING_ERROR_ZERO_LATCH_VELOCITY 243
#define STAT_HOMING_ERROR_TRAVEL_MIN_MAX_IDENTICAL 244
#define STAT_HOMING_ERROR_NEGATIVE_LATCH_BACKOFF 245
#define STAT_HOMING_ERROR_HOMING_INPUT_MISCONFIGURED 246
#define STAT_HOMING_ERROR_SWITCH_MISCONFIGURATION 246//legacy
#define STAT_HOMING_ERROR_MUST_CLEAR_SWITCHES_BEFORE_HOMING 247
#define STAT_ERROR_248 248
#define STAT_ERROR_249 249

#define STAT_PROBE_CYCLE_FAILED 250             // probing cycle did not complete
#define STAT_PROBE_TRAVEL_TOO_SMALL 251
#define STAT_NO_PROBE_INPUT_CONFIGURED 252
#define STAT_MULTIPLE_PROBE_SWITCHES_CONFIGURED 253
#define STAT_PROBE_IS_ALREADY_TRIPPED 254

#define STAT_ERROR_255 255

// ****** !!! Do not exceed 255 without also changing stat_t typedef ******

#endif // End of include guard: ERROR_H_ONCE
