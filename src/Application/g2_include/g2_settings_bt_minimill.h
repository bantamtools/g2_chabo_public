/**************************************************
TEMPORARY SETTING FILE FOR INTERNAL TESTING
****************************************************/
/*
 * settings_bt_minimill.h - Bantam Tools CNC Milling Machine Project
 * This file is part of the g2core project
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
/* Note: The values in this file are the default settings that are loaded
 *      into a virgin EEPROM, and can be changed using the config commands.
 *    After initial load the EEPROM values (or changed values) are used.
 *
 *    System and hardware settings that you shouldn't need to change
 *    are in hardware.h  Application settings that also shouldn't need
 *    to be changed are in g2core.h
 */

#ifndef SETTINGS_BT_MINIMILL_H_ONCE
#define SETTINGS_BT_MINIMILL_H_ONCE
#include "main.h"
#ifdef CHABO_MINIMILL

/***********************************************************************/
/**** BT_MINIMILL profile ***************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to BT MINIMILL settings"

//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_INTEGRATION_TIME   0.1     // cornering - between 0.10 and 2.00 (higher is faster)
#define CHORDAL_TOLERANCE           0.005    // chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE           0       // 0=off, 1=on
#define HARD_LIMIT_ENABLE           0       // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1       // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true

#define SPINDLE_SPINUP_DELAY        3.0     // after unpausing and turning the spindle on, dwell for 1.5s
#define SPINDLE_SPEED_CHANGE_PER_MS 10

#define ESC_BOOT_TIME               5000    // how long the ESC takes to boot, in milliseconds
#define ESC_LOCKOUT_TIME            900     // how long the interlock needs to be engaged before killing power... actually 1s, but be conservative


#define FEEDHOLD_Z_LIFT             -1       // mm to lift Z on feedhold, or -1 to go to Z-max
#define PROBE_REPORT_ENABLE         true

// Switch definitions for interlock & E-stop
#define ENABLE_INTERLOCK_AND_ESTOP
#undef PAUSE_DWELL_TIME
#define PAUSE_DWELL_TIME                3.0 //after unpausing and turning the spindle on, dwell for 1.5s // previously set to 1.5

// Communications and reporting settings

#define COMM_MODE                   AUTO_MODE           // one of: TEXT_MODE, JSON_MODE
#define XIO_ENABLE_FLOW_CONTROL     FLOW_CONTROL_RTS    // FLOW_CONTROL_OFF, FLOW_CONTROL_RTS

#define TEXT_VERBOSITY              TV_VERBOSE          // one of: TV_SILENT, TV_VERBOSE
#define JSON_VERBOSITY               JV_LINENUM//JV_MESSAGES         // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define QUEUE_REPORT_VERBOSITY      QR_OFF           // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY     SR_FILTERED         // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS        100                 // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS   250                 // milliseconds - set $SV=0 to disable
#define STATUS_REPORT_DEFAULTS      "mpox", "mpoy", "mpoz", \
                                    "ofsx", "ofsy", "ofsz", \
                                    "g55x", "g55y", "g55z", \
                                    "unit", "stat", "coor", "momo", "dist", \
                                    "home", "mots", "plan", "line", "path", \
                                    "frmo", "prbe", "safe", "estp", "spc", \
                                    "hold", "macs", "cycs", "sps"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS MILLIMETERS     // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE CANON_PLANE_XY  // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM G55      // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE

#define G54_X_OFFSET 0    // G54 is often set to all zeros
#define G54_Y_OFFSET 0
#define G54_Z_OFFSET 0
#define G54_A_OFFSET 0

#define G55_X_OFFSET 0    // G54 is often set to all zeros
#define G55_Y_OFFSET 0
#define G55_Z_OFFSET 0
#define G55_A_OFFSET 0
#define G55_C_OFFSET 0

#define G56_X_OFFSET 0    // G54 is often set to all zeros
#define G56_Y_OFFSET 0
#define G56_Z_OFFSET 0
#define G56_A_OFFSET 0

#define G57_X_OFFSET 0    // G54 is often set to all zeros
#define G57_Y_OFFSET 0
#define G57_Z_OFFSET 0
#define G57_A_OFFSET 0

#define G58_X_OFFSET 0    // G54 is often set to all zeros
#define G58_Y_OFFSET 0
#define G58_Z_OFFSET 0
#define G58_A_OFFSET 0

#define G59_X_OFFSET 0    // G54 is often set to all zeros
#define G59_Y_OFFSET 0
#define G59_Z_OFFSET 0
#define G59_A_OFFSET 0

// *** motor settings ************************************************************************************
































// NOTE: Motor numbers are reversed from TinyGv8 in order to maintain compatibility with wiring harnesses

#define M1_STEPS_PER_UNIT           0
#define M2_STEPS_PER_UNIT           0
#define M3_STEPS_PER_UNIT           0
#define M4_STEPS_PER_UNIT           0

#define M1_MOTOR_MAP                AXIS_X_EXTERNAL         // 1ma
#define M1_STEP_ANGLE               1.8                     // 1sa
#define M1_TRAVEL_PER_REV           8                       // 1tr
#define M1_MICROSTEPS               8                       // 1mi  1,2,4,8,16,32
#define M1_POLARITY                 1                       // 1po  0=normal, 1=reversed

#define M2_MOTOR_MAP                AXIS_Y_EXTERNAL
#define M2_STEP_ANGLE               1.8
#define M2_TRAVEL_PER_REV           8  
#define M2_MICROSTEPS               8
#define M2_POLARITY                 0

#define M3_MOTOR_MAP                AXIS_Z_EXTERNAL
#define M3_STEP_ANGLE               1.8
#define M3_TRAVEL_PER_REV           4
#define M3_MICROSTEPS               8
#define M3_POLARITY                 1

#define M4_MOTOR_MAP                AXIS_A_EXTERNAL
#define M4_STEP_ANGLE               1.8
#define M4_TRAVEL_PER_REV           90  // degrees moved per motor rev
#define M4_MICROSTEPS               8
#define M4_POLARITY                 1

#define M5_MOTOR_MAP                AXIS_B_EXTERNAL
#define M5_STEP_ANGLE               1.8
#define M5_TRAVEL_PER_REV           360  // degrees moved per motor rev
#define M5_MICROSTEPS               8
#define M5_POLARITY                 0

#define M6_MOTOR_MAP                AXIS_C_EXTERNAL
#define M6_STEP_ANGLE               1.8
#define M6_TRAVEL_PER_REV           360  // degrees moved per motor rev
#define M6_MICROSTEPS               8
#define M6_POLARITY                 0

// *** axis settings **********************************************************************************

#define JERK_MAX                    300//400                 // 650 million mm/(min^3)
#define JERK_HIGH_SPEED             1000                // 1000 million mm/(min^3) // Jerk during homing needs to stop *fast*
#define VELOCITY_MAX                3000
#define SEARCH_VELOCITY             400
#define LATCH_VELOCITY              25                  // reeeeally slow for accuracy
#define ROTARY_JERK_MAX             650
#define FEEDRATE_MAX                VELOCITY_MAX

#define X_AXIS_MODE                 AXIS_STANDARD       // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX              3000        // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX              X_VELOCITY_MAX      // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN                0                   // xtn  minimum travel for soft limits
#define X_TRAVEL_MAX                164                 // xtr  travel between switches or crashes
#define X_JERK_MAX                  JERK_MAX            // xjm
#define X_JERK_HIGH_SPEED           JERK_HIGH_SPEED     // xjh
#define X_HOMING_INPUT              1                   // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION          1                   // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY           SEARCH_VELOCITY     // xsv
#define X_LATCH_VELOCITY            LATCH_VELOCITY      // xlv  mm/min
#define X_LATCH_BACKOFF             2
#define X_ZERO_BACKOFF              1.4                 // xzb  mm



#define Y_AXIS_MODE                 AXIS_STANDARD
#define Y_VELOCITY_MAX              X_VELOCITY_MAX
#define Y_FEEDRATE_MAX              Y_VELOCITY_MAX
#define Y_TRAVEL_MIN                0
#define Y_TRAVEL_MAX                111//110
#define Y_JERK_MAX                  JERK_MAX
#define Y_JERK_HIGH_SPEED           JERK_HIGH_SPEED
#define Y_HOMING_INPUT              3
#define Y_HOMING_DIRECTION          0
#define Y_SEARCH_VELOCITY           SEARCH_VELOCITY
#define Y_LATCH_VELOCITY            LATCH_VELOCITY
#define Y_ZERO_BACKOFF              0.4
#define Y_LATCH_BACKOFF             1


#define Z_AXIS_MODE                 AXIS_STANDARD
#define Z_VELOCITY_MAX              1800
#define Z_FEEDRATE_MAX              Z_VELOCITY_MAX
#define Z_TRAVEL_MIN                -77.7
#define Z_TRAVEL_MAX                0
#define Z_JERK_MAX                  175
#define Z_JERK_HIGH_SPEED           JERK_HIGH_SPEED
#define Z_HOMING_INPUT              6
#define Z_HOMING_DIRECTION          1
#define Z_SEARCH_VELOCITY           SEARCH_VELOCITY
#define Z_LATCH_VELOCITY            LATCH_VELOCITY
#define Z_ZERO_BACKOFF              0.4
#define Z_LATCH_BACKOFF             1

#define A_AXIS_MODE                 AXIS_STANDARD
#define A_VELOCITY_MAX              2000
#define A_FEEDRATE_MAX              2000
#define A_TRAVEL_MIN                -1  // degrees
#define A_TRAVEL_MAX                -1  // same value means infinite, no limit
#define A_JERK_MAX                  ROTARY_JERK_MAX
#define A_JERK_HIGH_SPEED           A_JERK_MAX
#define A_RADIUS                    1.0
#define A_HOMING_INPUT              0
#define A_HOMING_DIRECTION          0
#define A_SEARCH_VELOCITY           SEARCH_VELOCITY
#define A_LATCH_VELOCITY            LATCH_VELOCITY
#define A_LATCH_BACKOFF             1
#define A_ZERO_BACKOFF              0.4

#ifdef REFACTOR_LIMIT_SWITCHES 
#define X_LIMIT_SW_ACTIVE_STATE SWITCH_NC_ACTIVE_STATE
#define Y_LIMIT_SW_ACTIVE_STATE SWITCH_NC_ACTIVE_STATE
#define Z_LIMIT_SW_ACTIVE_STATE SWITCH_NC_ACTIVE_STATE
#endif 
#define A_LIMIT_SW_ACTIVE_STATE SWITCH_NC_ACTIVE_STATE

#define PROBING_INPUT               5

// *** PWM SPINDLE CONTROL ***

#define P1_PWM_FREQUENCY         100     // in Hz
#define P1_SPEED_LO              10000   // in RPM (arbitrary units)
#define P1_SPEED_HI              24000//8-18-2022 based on observation 22,240 is max  was:28000
#define P1_PHASE_LO              0.1477//8-19-2022 based on observation was; 0.138  // phase [0..1]
#define P1_PHASE_HI              0.196//8-19-2022  based on observation was 0.193 // needs to be updated 
#define P1_PWM_PHASE_OFF         0.1000
#define P1_PWM_K_VALUE           1.0

//#define P1_USE_MAPPING_CUBIC
#define P1_MAPPING_CUBIC_X3     2.1225328766717546e-013
#define P1_MAPPING_CUBIC_X2    -7.2900167282605129e-009
#define P1_MAPPING_CUBIC_X1     8.5854646785876479e-005
#define P1_MAPPING_CUBIC_X0    -2.1301489219406905e-001

#endif  // SETTINGS_BT_MINIMILL_H_ONCE
#endif