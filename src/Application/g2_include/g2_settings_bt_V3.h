
/*
 * settings_bt_V3.h - V3 Industrial scale Milling Machine
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

#ifndef SETTINGS_BT_V3_H_ONCE
#define SETTINGS_T_V3_H_ONCE

#include "main.h"
#ifdef SEBRIGHT_STM32F777 
#include "optidrive_ctrlr.h"
#endif

/***********************************************************************/
/**** Otherlab OtherMill profile ***************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to Bantam Tools V3 settings"

//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_INTEGRATION_TIME   0.2     // was 0.1 cornering - between 0.10 and 2.00 (higher is faster)
#define CHORDAL_TOLERANCE           0.01    // chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE           0       // 0=off, 1=on
#define HARD_LIMIT_ENABLE           0       // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1       // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true

#ifdef MCB_PLUS//vfd does it's own internal ramp, not ramp config relevance now
#define SPINDLE_SPEED_CHANGE_PER_MS 0  
#define SPINDLE_SPINUP_DELAY_PADDING_SEC 1.0

#ifdef SEBRIGHT_STM32F777
#define SPINDLE_SPINUP_DELAY        (ODE2_DEFAULT_ACCEL_RAMP_TIME_SEC+SPINDLE_SPINUP_DELAY_PADDING_SEC)      // after unpausing and turning the spindle on, dwell for 1.5s
#else
#define SPINDLE_SPINUP_DELAY        1.5     // after unpausing and turning the spindle on, dwell for 1.5s
#endif

#else
#define SPINDLE_SPEED_CHANGE_PER_MS 7.0
#define SPINDLE_SPINUP_DELAY        0.1     // after unpausing and turning the spindle on, dwell for 1.5s
#endif
   
#define ESC_BOOT_TIME               5000    // how long the ESC takes to boot, in milliseconds
#define ESC_LOCKOUT_TIME            900     // how long the interlock needs to be engaged before killing power... actually 1s, but be conservative

#define FEEDHOLD_Z_LIFT             -1       // mm to lift Z on feedhold, or -1 to go to Z-max
#define PROBE_REPORT_ENABLE         true

// Switch definitions for interlock & E-stop
//#undef PAUSE_DWELL_TIME
#define PAUSE_DWELL_TIME                1.5 //after unpausing and turning the spindle on, dwell for 1.5s

// Communications and reporting settings

#define COMM_MODE                   AUTO_MODE           // one of: TEXT_MODE, JSON_MODE
#define XIO_ENABLE_FLOW_CONTROL     FLOW_CONTROL_RTS    // FLOW_CONTROL_OFF, FLOW_CONTROL_RTS

#define TEXT_VERBOSITY              TV_VERBOSE          // one of: TV_SILENT, TV_VERBOSE
#define JSON_VERBOSITY              JV_MESSAGES         // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
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

//For Development only: we are borrow the desktop mill (tinyG) steppers and spindle:
#define G54_X_OFFSET 0    // G54 is often set to all zeros
#define G54_Y_OFFSET 0
#define G54_Z_OFFSET 0

#define G55_X_OFFSET 0
#define G55_Y_OFFSET 0
#define G55_Z_OFFSET 0
#define G55_C_OFFSET 0

#define G56_X_OFFSET 0
#define G56_Y_OFFSET 0
#define G56_Z_OFFSET 0

#define G57_X_OFFSET 0
#define G57_Y_OFFSET 0
#define G57_Z_OFFSET 0

#define G58_X_OFFSET 0
#define G58_Y_OFFSET 0
#define G58_Z_OFFSET 0

#define G59_X_OFFSET 0
#define G59_Y_OFFSET 0
#define G59_Z_OFFSET 0
// *** motor settings ************************************************************************************

// NOTE: Motor numbers are reversed from TinyGv8 in order to maintain compatibility with wiring harnesses
#define M1_MOTOR_MAP                AXIS_X_EXTERNAL         // 1ma
#define M1_STEP_ANGLE               1.8                     // 1sa
#define M1_TRAVEL_PER_REV 	        5.0 
#define M1_MICROSTEPS               8                       // 1mi  1,2,4,8,16,32
#define M1_POLARITY                 0                      // 1po  0=normal, 1=reversed
#define M1_STEPS_PER_UNIT           0
                                      
#define M2_MOTOR_MAP                AXIS_Y_EXTERNAL
#define M2_STEP_ANGLE               1.8
#define M2_TRAVEL_PER_REV           5.0
#define M2_POLARITY                 1                                                                                         
#define M2_MICROSTEPS               8
#define M2_STEPS_PER_UNIT           0                                      

#define M3_MOTOR_MAP                AXIS_Z_EXTERNAL
#define M3_STEP_ANGLE               1.8
#define M3_TRAVEL_PER_REV 	        5.0                                        
#define M3_MICROSTEPS               8
#define M3_POLARITY                 1 
#define M3_STEPS_PER_UNIT           0

#define M4_MOTOR_MAP                AXIS_Y2_EXTERNAL
#define M4_STEP_ANGLE               1.8
#define M4_TRAVEL_PER_REV 	        5.0
#define M4_POLARITY                 1                                                                           
#define M4_MICROSTEPS               8 
#define M4_STEPS_PER_UNIT           0                                      

                                      
// *** axis settings **********************************************************************************

#define JERK_MAX                    500                 // 500 million mm/(min^3)
#define JERK_HIGH_SPEED             800                 // 1000 million mm/(min^3) // Jerk during homing needs to stop *fast*

#ifdef REDUCE_VMAX
#define VELOCITY_MAX                2500//8-30-2021 matches jog vmax                                
#else                                      
#define VELOCITY_MAX                3500//4000 
#endif

#define SEARCH_VELOCITY             300//800                                                                           
#define FEEDRATE_MAX                VELOCITY_MAX
#define LATCH_VELOCITY              25                  // reeeeally slow for accuracy
#define X_AXIS_MODE                 AXIS_STANDARD       // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX              VELOCITY_MAX        // xvm  G0 max velocity in mm/min

#ifdef REDUCE_VMAX//9-29-2021 DEBUG_JOG_DEFECT
#define X_FEEDRATE_MAX              2500//this matches jog_max                              
#else                                      
#define X_FEEDRATE_MAX              3000              // xfr  G1 max feed rate in mm/min
#endif

#define X_TRAVEL_MAX                576//572               // xtr  travel between switches or crashes                                    
#define X_TRAVEL_MIN                0                   // xtn  minimum travel for soft limits

#ifdef REDUCE_JERK_MAX
#define X_JERK_MAX                  300//350//300//9-14-2021  combatting stalls 650            // xjm
#else
#define X_JERK_MAX                  650            // xjm                                      
#endif

#define X_JERK_HIGH_SPEED           JERK_HIGH_SPEED     // xjh
#define X_HOMING_INPUT              1                   // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION          0                   // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY           SEARCH_VELOCITY     // xsv
#define X_LATCH_VELOCITY            LATCH_VELOCITY      // xlv  mm/min
#define X_LATCH_BACKOFF             2                   // xlb  mm
#define X_ZERO_BACKOFF              0.4                 // xzb  mm

//ATC homing uses X axis parameters except for homing direction which is opposite end
#define ATC_AXIS_MODE          X_AXIS_MODE  
#define ATC_VELOCITY_MAX       X_VELOCITY_MAX  
#define ATC_FEEDRATE_MAX       X_FEEDRATE_MAX  
#define ATC_TRAVEL_MIN         X_TRAVEL_MIN  
#define ATC_TRAVEL_MAX         X_TRAVEL_MAX  
#define ATC_JERK_MAX           X_JERK_MAX  
#define ATC_JERK_HIGH_SPEED    X_JERK_HIGH_SPEED 
#define ATC_HOMING_INPUT       X_HOMING_INPUT  
#define ATC_HOMING_DIRECTION   1
#define ATC_SEARCH_VELOCITY    X_SEARCH_VELOCITY 
#define ATC_LATCH_VELOCITY     X_LATCH_VELOCITY  
#define ATC_LATCH_BACKOFF      2
#define ATC_ZERO_BACKOFF       X_ZERO_BACKOFF  
                                      
#define Y_AXIS_MODE                 AXIS_STANDARD
#define Y_VELOCITY_MAX              VELOCITY_MAX                                     
#define Y_FEEDRATE_MAX              X_FEEDRATE_MAX
#define Y_HOMING_DIRECTION          1                                      
#define Y_TRAVEL_MAX                218//0//4-29-2021 polarity correction 218                                                                            
#define Y_TRAVEL_MIN                0// -218//4-29-2021 polarity correction0                                      
#define Y_JERK_MAX                  300
#define Y_JERK_HIGH_SPEED           JERK_HIGH_SPEED
#define Y_HOMING_INPUT              3
#define Y_SEARCH_VELOCITY           SEARCH_VELOCITY
#define Y_LATCH_VELOCITY            LATCH_VELOCITY
#define Y_LATCH_BACKOFF             2
#define Y_ZERO_BACKOFF              0.4

#define Z_AXIS_MODE                 AXIS_STANDARD
#define Z_VELOCITY_MAX              VELOCITY_MAX                                      
#define Z_FEEDRATE_MAX              X_FEEDRATE_MAX
#define Z_TRAVEL_MIN                -215                                                                           
#define Z_TRAVEL_MAX                0

#ifdef REDUCE_JERK_MAX                                      
#define Z_JERK_MAX                  300//400sme: debug 9-14-2021 500
#else //original was 500
#define Z_JERK_MAX                  500                                      
#endif
                                      
#define Z_JERK_HIGH_SPEED           JERK_HIGH_SPEED
#define Z_HOMING_INPUT              6
#define Z_HOMING_DIRECTION          1
#define Z_SEARCH_VELOCITY           SEARCH_VELOCITY
#define Z_LATCH_VELOCITY            LATCH_VELOCITY
#define Z_LATCH_BACKOFF             2
#define Z_ZERO_BACKOFF              0.4
                                      
// Rotary values are chosen to make the motor react the same as X for testing

// *** PWM SPINDLE CONTROL ***
#define P1_PWM_DUTY_CYCLE		0.0  //sme: directly set duty cycle 0.0..1.0  
#define P1_PWM_FREQUENCY		100  // in Hz// sme: FREQ (==1/pwm period) not adjustable 
                                                                          
#ifdef CHABO_ATSAMS70N20XB                                      
#define P1_SPEED_LO          10000
#define P1_SPEED_HI          28000   
#define P1_PHASE_LO			 0.138   
#define P1_PHASE_HI			 0.1937  
#else  //V3 VFD spindle                                        
#define P1_CW_SPEED_LO			ODE2_MIN_SPEED_RPM// VFD  min Speed ref to Optidrive 
#define P1_CW_SPEED_HI			ODE2_MAX_SPEED_RPM//VFD max Speed ref to Optidrive 
#endif


#define P1_CW_PHASE_LO			0.64  //measured 10-3-2019  duty_cycle [0..1.0]
#define P1_CW_PHASE_HI			0.87  //measured 10-3-2019
//#define P1_CCW_SPEED_LO			0     // sme: CCW not used
//#define P1_CCW_SPEED_HI			0     // sme: CCW not used
//#define P1_CCW_PHASE_LO			0.1   // sme: CCW not used
//#define P1_CCW_PHASE_HI			0.1   // sme: CCW not used
#define P1_PWM_PHASE_OFF		0.1   // sme: CCW not used
                                   
//*** Input / output settings ***
/*
    See gpio.h GPIO defines for options

    Homing and probing settings are independent of ACTION and FUNCTION settings
    but rely on proper switch MODE setting (i.e. NC or NO)

    IO_MODE_DISABLED
    IO_ACTIVE_LOW    aka NORMALLY_OPEN
    IO_ACTIVE_HIGH   aka NORMALLY_CLOSED

    INPUT_ACTION_NONE
    INPUT_ACTION_STOP
    INPUT_ACTION_FAST_STOP
    INPUT_ACTION_HALT
    INPUT_ACTION_RESET

    INPUT_FUNCTION_NONE
    INPUT_FUNCTION_LIMIT
    INPUT_FUNCTION_INTERLOCK
    INPUT_FUNCTION_SHUTDOWN
    INPUT_FUNCTION_PANIC
*/

#define PROBING_INPUT               5

// Xmin on v9 board                 // X homing - see X axis setup
#define DI1_ENABLED                 IO_ENABLED
#define DI1_POLARITY                IO_ACTIVE_HIGH      // Normally Closed
#define DI1_ACTION                  INPUT_ACTION_LIMIT

// Xmax                             // External ESTOP
#define DI2_ENABLED                 IO_ENABLED
#define DI2_POLARITY                IO_ACTIVE_HIGH
#define DI2_ACTION                  INPUT_ACTION_NONE   // SHUTDOWN handled by E-Stop handler

// Ymin                             // Y homing - see Y axis setup
#define DI3_ENABLED                 IO_ENABLED
#define DI3_POLARITY                IO_ACTIVE_HIGH
#define DI3_ACTION                  INPUT_ACTION_LIMIT

// Ymax                             // Safety interlock
#define DI4_ENABLED                 IO_ENABLED
#define DI4_POLARITY                IO_ACTIVE_HIGH
#define DI4_ACTION                  INPUT_ACTION_NONE   // (hold is performed by Interlock function)

// Zmin                             // Z probe
#define DI5_ENABLED                 IO_ENABLED
#define DI5_POLARITY                IO_ACTIVE_LOW       // Normally Open
#define DI5_ACTION                  INPUT_ACTION_NONE

// Zmax                             // Z homing - see Z axis for setup
#define DI6_ENABLED                 IO_ENABLED
#define DI6_POLARITY                IO_ACTIVE_HIGH
#define DI6_ACTION                  INPUT_ACTION_LIMIT

// Amin                             // Version pin - board type
#define DI7_ENABLED                 IO_ENABLED
#define DI7_POLARITY                IO_ACTIVE_LOW
#define DI7_ACTION                  INPUT_ACTION_NONE

// Amax                             // Version pin - board type
#define DI8_ENABLED                 IO_ENABLED
#define DI8_POLARITY                IO_ACTIVE_LOW
#define DI8_ACTION                  INPUT_ACTION_NONE

// Safety line w/HW timer           // Version pin - board type
#define DI9_ENABLED                 IO_DISABLED
#define DI9_POLARITY                IO_ACTIVE_LOW
#define DI9_ACTION                  INPUT_ACTION_NONE
#define DI9_EXTERNAL_NUMBER         10

// Silk marked as "CS2"             // Version pin - machine type
#define DI10_ENABLED                 IO_ENABLED
#define DI10_POLARITY                IO_ACTIVE_LOW
#define DI10_ACTION                  INPUT_ACTION_NONE
#define DI10_EXTERNAL_NUMBER         9

// *** PWM SPINDLE CONTROL ***

#define P1_PWM_FREQUENCY            100     // in Hz
//#define P1_CW_SPEED_LO              10500   // in RPM (arbitrary units)
//#define P1_CW_SPEED_HI              16400
//#define P1_CW_PHASE_LO              0.13  // phase [0..1]
//#define P1_CW_PHASE_HI              0.17
#define P1_CCW_SPEED_LO             0
#define P1_CCW_SPEED_HI             0
#define P1_CCW_PHASE_LO             0.1
#define P1_CCW_PHASE_HI             0.1
#define P1_PWM_PHASE_OFF            0.1

//#define P1_USE_MAPPING_CUBIC
#define P1_MAPPING_CUBIC_X3     2.1225328766717546e-013
#define P1_MAPPING_CUBIC_X2    -7.2900167282605129e-009
#define P1_MAPPING_CUBIC_X1     8.5854646785876479e-005
#define P1_MAPPING_CUBIC_X0    -2.1301489219406905e-001

#endif  //  SETTINGS_BT_V3_H_ONCE
