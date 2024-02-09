
/*
 * main.cpp - g2core - An embedded rs274/ngc CNC controller
 * This file is part of the g2core project.
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2018 Robert Giseburt
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
/* See github.com/Synthetos/G2 for code and docs on the wiki
 */

//#include "stm32f7xx_hal.h"
#include "g2core.h"  // #1 There are some dependencies
#include "g2_config.h"  // #2
#include "g2_controller.h"
#include "g2_canonical_machine.h"
#include "g2_json_parser.h"	// required for unit tests only
#include "g2_planner.h"
#include "g2_stepper.h"
#include "g2_encoder.h"
#include "g2_spindle.h"
#include "g2_util.h"
#include "g2_report.h"
#include "g2_persistence.h"

#include "g2_sd_persistence.h"  // needed for nvObj_t definition

#include "system_bantam.h" 
/***** NOTE: *****
   The actual main.cpp for g2core is in the Motate project.
   It calls the setup() and loop() functions in this file
*****/
/******************** System Globals *************************/
stat_t status_code;	// allocate a variable for the ritorno macro
/**** Status Messages ***************************************************************
 * get_status_message() - return the status message
 *
 * See tinyg.h for status codes. These strings must align with the status codes in tinyg.h
 * The number of elements in the indexing array must match the # of strings
 *
 * Reference for putting display strings and string arrays in AVR program memory:
 * http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/pgmspace.html
 */
extern stat_t status_code;	// allocate a variable for the ritorno macro
char global_string_buf[MESSAGE_LEN];	// allocate a string for global message use

/* sme: provide global str buf for replacing fprintf statements that use "stderr" with a destination buf used by sprintf */
char global_sprintf_dest_buf[SPRINTF_DEST_BUF_LEN];
 extern /*"C"*/ int g2_main(void);
/*** Status message strings ***/

/********************************
 **** Status message strings ****
 ********************************/
 
// NB: The ones that are never called are optimized out by the compiler (e.g. the placeholders)
static const char stat_00[] = "OK";
static const char stat_01[] = "Error";
static const char stat_02[] = "Eagain";
static const char stat_03[] = "No operation performed";
static const char stat_04[] = "Completed operation";
static const char stat_05[] = "System shutdown";
static const char stat_06[] = "System panic";
static const char stat_07[] = "End of line";
static const char stat_08[] = "End of file";
static const char stat_09[] = "File not open";

static const char stat_10[] = "Max file size exceeded";
static const char stat_11[] = "No such device";
static const char stat_12[] = "Buffer empty";
static const char stat_13[] = "Buffer full non-fatal";
static const char stat_14[] = "Buffer full FATAL";
static const char stat_15[] = "Initializing";
static const char stat_16[] = "Entering boot loader";
static const char stat_17[] = "Function is stubbed";
static const char stat_18[] = "System alarm";
static const char stat_19[] = "19";

static const char stat_20[] = "Internal error";
static const char stat_21[] = "Internal range error";
static const char stat_22[] = "Floating point error";
static const char stat_23[] = "Divide by zero";
static const char stat_24[] = "Invalid Address";
static const char stat_25[] = "Read-only address";
static const char stat_26[] = "Initialization failure";
static const char stat_27[] = "27";// was ALARMED in 0.97
static const char stat_28[] = "Failed to get planner buffer";
static const char stat_29[] = "Generic exception report";

static const char stat_30[] = "Move time is infinite";
static const char stat_31[] = "Move time is NAN";
static const char stat_32[] = "Float is infinite";
static const char stat_33[] = "Float is NAN";
static const char stat_34[] = "Persistence error";
static const char stat_35[] = "Bad status report setting";
static const char stat_36[] = "Failed to get planner buffer";

static const char stat_37[] = "Backplan hit running buffer";
static const char stat_38[] = "VFD Optidrive Modbus Comms Fault";//sme: 3-23-2021 new string;
static const char stat_39[] = "VFD Optidrive Tripped";

static const char stat_40[] = "40";
static const char stat_41[] = "41";
static const char stat_42[] = "42";
static const char stat_43[] = "43";
static const char stat_44[] = "44";
static const char stat_45[] = "45";
static const char stat_46[] = "46";
static const char stat_47[] = "47";
static const char stat_48[] = "48";
static const char stat_49[] = "49";
static const char stat_50[] = "50";
static const char stat_51[] = "51";
static const char stat_52[] = "52";
static const char stat_53[] = "53";
static const char stat_54[] = "54";
static const char stat_55[] = "55";
static const char stat_56[] = "56";
static const char stat_57[] = "57";
static const char stat_58[] = "58";
static const char stat_59[] = "59";
static const char stat_60[] = "60";
static const char stat_61[] = "61";
static const char stat_62[] = "62";
static const char stat_63[] = "63";
static const char stat_64[] = "64";
static const char stat_65[] = "65";
static const char stat_66[] = "66";
static const char stat_67[] = "67";
static const char stat_68[] = "68";
static const char stat_69[] = "69";
static const char stat_70[] = "70";
static const char stat_71[] = "71";
static const char stat_72[] = "72";
static const char stat_73[] = "73";
static const char stat_74[] = "74";
static const char stat_75[] = "75";
static const char stat_76[] = "76";
static const char stat_77[] = "77";
static const char stat_78[] = "78";
static const char stat_79[] = "79";
static const char stat_80[] = "80";
static const char stat_81[] = "81";
static const char stat_82[] = "82";
static const char stat_83[] = "83";
static const char stat_84[] = "84";
static const char stat_85[] = "85";

static const char stat_86[] = "Spindle assertion failure";
static const char stat_87[] = "mp_exec_aline() assertion failure";
static const char stat_88[] = "Buffer free assertion failure";
static const char stat_89[] = "State management assertion failure";
static const char stat_90[] = "Config assertion failure";
static const char stat_91[] = "XIO assertion failure";
static const char stat_92[] = "Encoder assertion failure";
static const char stat_93[] = "Stepper assertion failure";
static const char stat_94[] = "Planner assertion failure";
static const char stat_95[] = "Canonical machine assertion failure";
static const char stat_96[] = "Controller assertion failure";
static const char stat_97[] = "Stack overflow detected";
static const char stat_98[] = "Memory fault detected";
static const char stat_99[] = "Generic assertion failure";

static const char stat_100[] = "Unrecognized command or config name";
static const char stat_101[] = "Invalid or malformed command";
static const char stat_102[] = "Bad number format";
static const char stat_103[] = "Unsupported number or JSON type";
static const char stat_104[] = "Parameter is read-only";
static const char stat_105[] = "Parameter cannot be read";
static const char stat_106[] = "Command not accepted";
static const char stat_107[] = "Input exceeds max length";
static const char stat_108[] = "Input less than minimum value";
static const char stat_109[] = "Input exceeds maximum value";

static const char stat_110[] = "Input value range error";
static const char stat_111[] = "JSON syntax error";
static const char stat_112[] = "JSON has too many pairs";
static const char stat_113[] = "JSON string too long";
static const char stat_114[] = "JSON txt fields cannot be nested";
static const char stat_115[] = "JSON maximum nesting depth exceeded";
static const char stat_116[] = "JSON value does not agree with variable type";
static const char stat_117[] = "Input from a muted channel was ignored";
static const char stat_118[] = "The provided checksum didn't match";
static const char stat_119[] = "The provided line number was out of sequence";

static const char stat_120[] = "Missing line number with checksum";//sme: 3-23-2021 string was missing for a defined stat_code!
static const char stat_121[] = "121";
static const char stat_122[] = "122";
static const char stat_123[] = "123";
static const char stat_124[] = "124";
static const char stat_125[] = "125";
static const char stat_126[] = "126";
static const char stat_127[] = "127";
static const char stat_128[] = "128";
static const char stat_129[] = "129";

static const char stat_130[] = "Generic Gcode input error";
static const char stat_131[] = "Gcode command unsupported";
static const char stat_132[] = "M code unsupported";
static const char stat_133[] = "Gcode modal group violation";
static const char stat_134[] = "Axis word missing";
static const char stat_135[] = "Axis cannot be present";
static const char stat_136[] = "Axis invalid for this command";
static const char stat_137[] = "Axis disabled";
static const char stat_138[] = "Axis target position missing";
static const char stat_139[] = "Axis target position invalid";

static const char stat_140[] = "Selected plane missing";
static const char stat_141[] = "Selected plane invalid";
static const char stat_142[] = "Feedrate not specified";
static const char stat_143[] = "Inverse time mode cannot be used with this command";
static const char stat_144[] = "Rotary axes cannot be used with this command";
static const char stat_145[] = "G0 or G1 must be active for G53";
static const char stat_146[] = "Requested velocity exceeds limits";
static const char stat_147[] = "Cutter compensation cannot be enabled";
static const char stat_148[] = "Programmed point same as current point";
static const char stat_149[] = "Spindle speed below minimum";

static const char stat_150[] = "Spindle speed exceeded maximum";
static const char stat_151[] = "Spindle must be off for this command";
static const char stat_152[] = "Spindle must be turning for this command";
static const char stat_153[] = "Arc specification error";
static const char stat_154[] = "Arc specification error - impossible center point";
static const char stat_155[] = "Arc specification error - arc has rotary axis(es)";
static const char stat_156[] = "Arc specification error - missing axis(es)";
static const char stat_157[] = "Arc specification error - missing offset(s)";
//--------------------------------------1--------10--------20--------30--------40--------50--------60-64
static const char stat_158[] = "Arc specification error - radius arc out of tolerance";
static const char stat_159[] = "Arc specification error - endpoint is starting point";

static const char stat_160[] = "P word missing";
static const char stat_161[] = "P word invalid";
static const char stat_162[] = "P word zero";
static const char stat_163[] = "P word negative";
static const char stat_164[] = "P word not an integer";
static const char stat_165[] = "P word not a valid tool number";
static const char stat_166[] = "D word missing";
static const char stat_167[] = "D word invalid";
static const char stat_168[] = "E word missing";
static const char stat_169[] = "E word invalid";

static const char stat_170[] = "H word missing";
static const char stat_171[] = "H word invalid";
static const char stat_172[] = "L word missing";
static const char stat_173[] = "L word invalid";
static const char stat_174[] = "Q word missing";
static const char stat_175[] = "Q word invalid";
static const char stat_176[] = "R word missing";
static const char stat_177[] = "R word invalid";
static const char stat_178[] = "S word missing";
static const char stat_179[] = "S word invalid";

static const char stat_180[] = "T word missing";
static const char stat_181[] = "T word invalid";
static const char stat_182[] = "182";
static const char stat_183[] = "183";
static const char stat_184[] = "184";
static const char stat_185[] = "185";
static const char stat_186[] = "186";
static const char stat_187[] = "187";
static const char stat_188[] = "188";
static const char stat_189[] = "189";

static const char stat_190[] = "190";
static const char stat_191[] = "191";
static const char stat_192[] = "192";
static const char stat_193[] = "193";
static const char stat_194[] = "194";
static const char stat_195[] = "195";
static const char stat_196[] = "196";
static const char stat_197[] = "197";
static const char stat_198[] = "198";
static const char stat_199[] = "199";

static const char stat_200[] = "Generic error";
static const char stat_201[] = "Move < min length";
static const char stat_202[] = "Move < min time";
//--------------------------------------1--------10--------20--------30--------40--------50--------60-64
static const char stat_203[] = "Limit hit [$clear to reset, $lim=0 to override]";
static const char stat_204[] = "Command rejected by ALARM [$clear to reset]";
static const char stat_205[] = "Command rejected by SHUTDOWN [$clear to reset]";
static const char stat_206[] = "Command rejected by PANIC [^x to reset]";
static const char stat_207[] = "Kill job";
static const char stat_208[] = "No GPIO for this value";
static const char stat_209[] = "209";

static const char stat_210[] = "Marlin G29 command was not configured at compile-time";
static const char stat_211[] = "211";
static const char stat_212[] = "212";
static const char stat_213[] = "213";
static const char stat_214[] = "214";
static const char stat_215[] = "215";
static const char stat_216[] = "216";
static const char stat_217[] = "217";
static const char stat_218[] = "218";
static const char stat_219[] = "219";

static const char stat_220[] = "Soft limit";
static const char stat_221[] = "Soft limit - X min";
static const char stat_222[] = "Soft limit - X max";
static const char stat_223[] = "Soft limit - Y min";
static const char stat_224[] = "Soft limit - Y max";
static const char stat_225[] = "Soft limit - Z min";
static const char stat_226[] = "Soft limit - Z max";
static const char stat_227[] = "Soft limit - A min";
static const char stat_228[] = "Soft limit - A max";
static const char stat_229[] = "Soft limit - B min";
static const char stat_230[] = "Soft limit - B max";
static const char stat_231[] = "Soft limit - C min";
static const char stat_232[] = "Soft limit - C max";
static const char stat_233[] = "Soft limit during arc";
static const char stat_234[] = "234";
static const char stat_235[] = "235";
static const char stat_236[] = "236";
static const char stat_237[] = "237";
static const char stat_238[] = "238";
static const char stat_239[] = "239";

static const char stat_240[] = "Homing cycle failed";
static const char stat_241[] = "Homing Err - Bad or no axis specified";
static const char stat_242[] = "Homing Err - Search velocity is zero";
static const char stat_243[] = "Homing Err - Latch velocity is zero";
static const char stat_244[] = "Homing Err - Travel min & max are the same";
static const char stat_245[] = "245";
static const char stat_246[] = "Homing Err - Homing input is misconfigured";
static const char stat_247[] = "Homing Err - Must clear switches before homing";
static const char stat_248[] = "248";
static const char stat_249[] = "249";

static const char stat_250[] = "Probe cycle failed";
static const char stat_251[] = "Probe travel is too small";
static const char stat_252[] = "No probe switch configured";
static const char stat_253[] = "Multiple probe switches configured";
static const char stat_254[] = "Probe is already tripped";
static const char stat_255[] = "255";

const char *const stat_msg[] = {
    stat_00, stat_01, stat_02, stat_03, stat_04, stat_05, stat_06, stat_07, stat_08, stat_09,
    stat_10, stat_11, stat_12, stat_13, stat_14, stat_15, stat_16, stat_17, stat_18, stat_19,
    stat_20, stat_21, stat_22, stat_23, stat_24, stat_25, stat_26, stat_27, stat_28, stat_29,
    stat_30, stat_31, stat_32, stat_33, stat_34, stat_35, stat_36, stat_37, stat_38, stat_39,
    stat_40, stat_41, stat_42, stat_43, stat_44, stat_45, stat_46, stat_47, stat_48, stat_49,
    stat_50, stat_51, stat_52, stat_53, stat_54, stat_55, stat_56, stat_57, stat_58, stat_59,
    stat_60, stat_61, stat_62, stat_63, stat_64, stat_65, stat_66, stat_67, stat_68, stat_69,
    stat_70, stat_71, stat_72, stat_73, stat_74, stat_75, stat_76, stat_77, stat_78, stat_79,
    stat_80, stat_81, stat_82, stat_83, stat_84, stat_85, stat_86, stat_87, stat_88, stat_89,
    stat_90, stat_91, stat_92, stat_93, stat_94, stat_95, stat_96, stat_97, stat_98, stat_99,
    stat_100, stat_101, stat_102, stat_103, stat_104, stat_105, stat_106, stat_107, stat_108, stat_109,
    stat_110, stat_111, stat_112, stat_113, stat_114, stat_115, stat_116, stat_117, stat_118, stat_119,
    stat_120, stat_121, stat_122, stat_123, stat_124, stat_125, stat_126, stat_127, stat_128, stat_129,
    stat_130, stat_131, stat_132, stat_133, stat_134, stat_135, stat_136, stat_137, stat_138, stat_139,
    stat_140, stat_141, stat_142, stat_143, stat_144, stat_145, stat_146, stat_147, stat_148, stat_149,
    stat_150, stat_151, stat_152, stat_153, stat_154, stat_155, stat_156, stat_157, stat_158, stat_159,
    stat_160, stat_161, stat_162, stat_163, stat_164, stat_165, stat_166, stat_167, stat_168, stat_169,
    stat_170, stat_171, stat_172, stat_173, stat_174, stat_175, stat_176, stat_177, stat_178, stat_179,
    stat_180, stat_181, stat_182, stat_183, stat_184, stat_185, stat_186, stat_187, stat_188, stat_189,
    stat_190, stat_191, stat_192, stat_193, stat_194, stat_195, stat_196, stat_197, stat_198, stat_199,
    stat_200, stat_201, stat_202, stat_203, stat_204, stat_205, stat_206, stat_207, stat_208, stat_209,
    stat_210, stat_211, stat_212, stat_213, stat_214, stat_215, stat_216, stat_217, stat_218, stat_219,
    stat_220, stat_221, stat_222, stat_223, stat_224, stat_225, stat_226, stat_227, stat_228, stat_229,
    stat_230, stat_231, stat_232, stat_233, stat_234, stat_235, stat_236, stat_237, stat_238, stat_239,
    stat_240, stat_241, stat_242, stat_243, stat_244, stat_245, stat_246, stat_247, stat_248, stat_249,
    stat_250, stat_251, stat_252, stat_253, stat_254, stat_255
};
 
/*
 * get_status_message() - global support for status messages.
 */
 
extern /* "C" */ char *get_status_message(stat_t status)
//char *get_status_message(stat_t status)
{ 
    return ((char *)GET_TEXT_ITEM(stat_msg, status));
} 
/************* System Globals For Debugging and Diagnostics ****************/
// See also: util.h for debugging and diagnostics

/******************** Application Code ************************/

/*
 * _application_init_services()
 * _application_init_machine()
 * _application_init_startup()
 *
 * There are a lot of dependencies in the order of these inits.
 * Don't change the ordering unless you understand this.
 */

void application_init_services(void)
{
  setup_sd_persistence();
  persistence_init();		        // set up EEPROM or other NVM		- must be second
}

void application_init_machine(void)
{ 
    cm = &cm1;                          // set global canonical machine pointer to primary machine
    cm->machine_state = MACHINE_INITIALIZING;
    canonical_machine_inits();          // combined inits for CMs and planner
    stepper_init();                     // stepper st_pre, st_run data init 
    encoder_init();                     // virtual encoders
}

void application_init_startup(void)
{
  volatile static uint32_t start_time, delta_time;
    // start the application
    controller_init();                  // should be first startup init (requires xio_init())
    start_time=sys_get_time_ms();
    config_init();			// apply the config settings from persistence    
    delta_time=sys_get_delta_time_ms(start_time);//observed: 105 ms for confi init
    canonical_machine_reset(&cm1);      // initialize both CMs but only reset the primary
    gcode_parser_init();                // baseline Gcode parser
    spindle_init();       
    spindle_reset();

}

/*
 * main() - See Motate main.cpp for the actual main()
 */

void setup(void)
{
    // application setup
    application_init_services();//persistence setup occurs here
    application_init_machine();
    application_init_startup();
}
 
/*
 * _application_init()
 */

/******************************************************
 Below is the edge branch "main"
void _system_init(void)
{
    Motate::WatchDogTimer.disable();
}
 
int main(void) {
    _system_init();

    if (setup)
        setup();

    // main loop
    for (;;) {
        loop();
    }
    return 0;
}

************************************************************************/

int g2_main(void)
{
   setup();
   return 0;
}
