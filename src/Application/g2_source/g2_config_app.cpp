
/*
 * config_app.cpp - application-specific part of configuration data
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2019 Robert Giseburt
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
/* This file contains application specific data for the config system:
 *  - application-specific functions and function prototypes
 *  - application-specific message and print format strings
 *  - application-specific config array
 *  - any other application-specific data or functions
 *
 * See config_app.h for a detailed description of config objects and the config table
 */

#include "main.h" 
#include "bantam_hal.h"
#include "g2core.h"  // #1
#include "g2_config.h"  // #2
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
#include "g2_controller.h"
#include "g2_canonical_machine.h"
#include "g2_gcode.h"
#include "g2_json_parser.h"
#include "g2_text_parser.h"
#include "g2_settings.h"
#include "g2_planner.h"
#include "g2_plan_arc.h"
#include "g2_stepper.h"
#include "g2_spindle.h"
#include "spindle_ctrl.h"//8-12-2022 
#include "g2_pwm.h" 
#include "g2_report.h"
#include "g2_util.h"
#include "g2_help.h"
#include "g2_kinematics.h"
   
#include "tmc2660_mtr_ctrlr.h"//tn "tune" group
#include "sk6812.h"      
#include "system_bantam.h"
#include "kernel.h"
#include "leds_mgr.h"

/*** structures ***/

cfgParameters_t cfg;         // application specific configuration parameters
#define DIN_COUNT 16
bool din[DIN_COUNT]={0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0};//edge, input 7 represents presence of our mill device type
extern void rcc_csr_print(nvObj_t *nv);
extern uint16_t comms_wdog_update_period_secs;
#ifdef DEPLOY_HARD_FAULT_REPORTING
#define PRESERVE_VALUE 0
stat_t get_hard_fault_info(nvObj_t *nv);
stat_t clear_hard_fault_info(nvObj_t *nv);
#endif

/***********************************************************************************
 **** application-specific internal functions **************************************
 ***********************************************************************************/
// See config.cpp/.h for generic variables and functions that are not specific to
// g2core or the motion control application domain

// helpers (most helpers are defined immediately above their usage so they don't need prototypes here)
stat_t hw_flash_loader(nvObj_t *nv)
{
    return STAT_OK;
}
static stat_t _do_motors(nvObj_t *nv);      // print parameters for all motor groups
static stat_t _do_axes(nvObj_t *nv);        // print parameters for all axis groups
static stat_t _do_offsets(nvObj_t *nv);     // print offset parameters for G54-G59,G92, G28, G30
static stat_t _do_all(nvObj_t *nv);         // print all parameters

static const char fmt_fbs[] = "[fbs] firmware build%34s\n";
static const char fmt_fbc[] = "[fbc] firmware config%33s\n";
static const char fmt_fb[] = "[fb]  firmware build%18.2f\n";

static const char fmt_fv[] = "[fv]  firmware version%16.2f\n";
static const char fmt_cv[] = "[cv]  configuration version%11.2f\n";
void hw_print_fbs(nvObj_t *nv) { text_print_str(nv, fmt_fbs);}  // TYPE_STRING
void hw_print_fbc(nvObj_t *nv) { text_print_str(nv, fmt_fbc);}  // TYPE_STRING

void hw_print_fb(nvObj_t *nv);
void hw_print_hp(nvObj_t *nv);
void hw_print_hv(nvObj_t *nv);
void hw_print_id(nvObj_t *nv);
// communications settings and functions

static stat_t get_rx(nvObj_t *nv);          // get bytes in RX buffer
static stat_t get_tick(nvObj_t *nv);        // get system tick count
stat_t hw_get_fb(nvObj_t *nv); 
stat_t hw_get_fv(nvObj_t *nv); 
stat_t hw_get_hp(nvObj_t *nv);  
stat_t hw_get_hv(nvObj_t *nv);  
stat_t hw_get_fbs(nvObj_t *nv) ; 
void hw_print_fb(nvObj_t *nv);
void hw_print_fv(nvObj_t *nv);
void hw_print_cv(nvObj_t *nv);

stat_t hw_get_id(nvObj_t *nv) 
{
   return (STAT_OK);
}
/*
 * hw_get_fbc() - get configuration settings file
 */

stat_t hw_get_fbc(nvObj_t *nv)
{
    nv->valuetype = TYPE_STRING;
#ifdef SETTINGS_FILE
#define settings_file_string1(s) #s
#define settings_file_string2(s) settings_file_string1(s)
    ritorno(nv_copy_string(nv, settings_file_string2(SETTINGS_FILE)));
#undef settings_file_string1
#undef settings_file_string2
#else
    ritorno(nv_copy_string(nv, "settings_othermill_pro.h"));
#endif

    return (STAT_OK);
}
/**********************************************************************
 *
 * sme_pre_init_for_configs_sake()
 * 
 * Description: the set_defa() function operates within difficult to adjust
 *   array index ranges through an undocumented paradigm of Synthetos.
 *   failure to "crack the code" on this schema, especially with the over-arching incidences of commented
 *   out sections (by synthetos or tomc) that put to question the accuracy of the defined ranges anyway,
 *   This function initializes elements that needed to have been initialized prior to other init functions
 *   whose pre-conditions require the fields to be initialize, but in fact, in this por attempt, are not,
 *   which in turn results in a multitide of div-by-zero faults.
 ****************************************************************************/
void sme_pre_init_for_configs_sake(void)
{
st_init_motors();
}
/***********************************************************************************
 **** CONFIG TABLE  ****************************************************************
 ***********************************************************************************
 *
 *  Read the notes in config.h first
 *
 *  NOTES AND CAVEATS
 *
 *  - Token matching occurs from the most specific to the least specific. This means
 *    that if shorter tokens overlap longer ones the longer one must precede the
 *    shorter one. E.g. "gco" needs to come before "gc"
 *
 *  - Mark group strings for entries that have no group as nul -->  "".
 *    This is important for group expansion.
 *
 *  - Groups do not have groups. Neither do uber-groups, e.g.
 *    'x' is --> { "", "x",    and 'm' is --> { "", "m",
 *
 *  - Be careful not to define groups longer than GROUP_LEN [6] and tokens longer
 *    than TOKEN_LEN [12]. (See config.h for lengths). The combined group + token
 *    cannot exceed TOKEN_LEN. String functions working on the table assume these
 *    rules are followed and do not check lengths or perform other validation.
 *
 *  - The precision value 'p' only affects JSON responses. You need to also set
 *    the %f in the corresponding format string to set text mode display precision
 *
 *  - Unit conversions are now conditional, and handled by convert_incoming_float()
 *    and convert_outgoing_float(). Apply conversion flags to all axes, not just linear,
 *    as rotary axes may be treated as linear if in radius mode, so the flag is needed.
 ***************************************************************************************
  groups:
  sys,mpo,pos, ofs,hom,prb,jog,in,p1,vfd,1,2,3,4,z,y,a,g54,g55,g56,g57,g58, g59,92,g28,g30,jid,fxa,sp,uda, udb,udc,udd,tof, 
  tt1,tt2,tt3,tt4,tt5,tt6,tt7,tt8,tt9,tt10,tt11,tt12,tt13,tt14,tt15,tt16,tt17,tt18,tt19,tt20,tt21,tt22,tt23,tt24,tt25,tt26,tt27,tt28,tt29,tt30,tt31,tt32
  
  
 */
const cfgItem_t cfgArray[] = {
#define OPTIMIZE_JOB_RUN_TIME_JSON_CMDS
    { "sys", "fb", _fn,  2, hw_print_fb,  hw_get_fb,  set_ro, nullptr, 0 },   // MUST BE FIRST for persistence checking!
    { "sys", "fv", _fn,  2, hw_print_fv,  hw_get_fv,  set_ro, nullptr, 0 },
    { "sys", "fbs",_sn,  0, hw_print_fbs, hw_get_fbs, set_ro, nullptr, 0 },  
    { "sys", "hp", _sn,  0, hw_print_hp,  hw_get_hp,  set_ro, nullptr, 0 },
    { "sys", "hv", _sn,  0, hw_print_hv,  hw_get_hv,  set_ro, nullptr, 0 },
    { "sys", "id", _sn,  0, hw_print_id,  hw_get_id,  set_ro, nullptr, 0 },   // device ID (ASCII signature)
    { "sys","scscalef",_fip, 0, st_print_scscalef, get_flt, st_set_scscalef,&st_step_correction_factor,STEP_CORRECTION_FACTOR}, 
#ifdef OPTIMIZE_JOB_RUN_TIME_JSON_CMDS     
    { "sp","spo",  _fip, 3, sp_print_spo,  sp_get_spo,  sp_set_spo,  nullptr, SPINDLE_OVERRIDE_FACTOR},        
     // General system parameters
    { "sys","fro", _fin, 3, cm_print_fro,  cm_get_fro, cm_set_fro, nullptr, FEED_OVERRIDE_FACTOR},    
    { "sys","tro", _fin, 3, cm_print_tro,  cm_get_tro, cm_set_tro, nullptr, TRAVERSE_OVERRIDE_FACTOR},     

#if 1//8-16-2022 explicitly turn off wdog   
    { "sys","wdog", _ii, 0, comms_print_wdog, comms_get_wdog, comms_set_wdog, (uint16_t*) &comms_wdog_update_period_secs, 0},     
    { "sys","wdtime", _ii, 0, comms_print_wdtimeout, comms_get_wdog_timeout, comms_set_wdog_timeout,nullptr, 0 }, 
#else    
    { "sys","wdog", _in, 0, comms_print_wdog, comms_get_wdog, comms_set_wdog, nullptr, 0},     
    { "sys","wdtimeout", _in, 0, comms_print_wdtimeout, comms_get_wdog_timeout, comms_set_wdog_timeout,nullptr, 0 },     
#endif  
#endif
    
    // dynamic model attributes for reporting purposes (up front for speed)
#ifdef DEPLOY_GET_CAUSE_OF_RESET 
    // { "", "rcccsr",  _i0, 0, rcc_csr_print, get_ui8, set_noop,  (uint8_t*)&rcc_csr_value, 0 },    // Model line number   
#endif
    { "", "stat", _i0, 0, cm_print_stat, cm_get_stat,  set_ro,       nullptr, 0 },    // combined machine state
    { "","stat2", _i0, 0, cm_print_stat, cm_get_stat2, set_ro,       nullptr, 0 },    // combined machine state
    { "", "n",    _ii, 0, cm_print_line, cm_get_mline, set_noop,     nullptr, 0 },    // Model line number
    { "", "line", _ii, 0, cm_print_line, cm_get_line,  set_ro,       nullptr, 0 },    // Active line number - model or runtime line number
    { "", "vel",  _f0, 2, cm_print_vel,  cm_get_vel,   set_ro,       nullptr, 0 },    // current velocity
    { "", "feed", _f0, 2, cm_print_feed, cm_get_feed,  set_ro,       nullptr, 0 },    // feed rate
    { "", "macs", _i0, 0, cm_print_macs, cm_get_macs,  set_ro,       nullptr, 0 },    // raw machine state
    { "", "cycs", _i0, 0, cm_print_cycs, cm_get_cycs,  set_ro,       nullptr, 0 },    // cycle state
    { "", "mots", _i0, 0, cm_print_mots, cm_get_mots,  set_ro,       nullptr, 0 },    // motion state
    { "", "hold", _i0, 0, cm_print_hold, cm_get_hold,  set_ro,       nullptr, 0 },    // feedhold state
    { "", "unit", _i0, 0, cm_print_unit, cm_get_unit,  set_ro,       nullptr, 0 },    // units mode
    { "", "coor", _i0, 0, cm_print_coor, cm_get_coor,  set_ro,       nullptr, 0 },    // coordinate system
    { "", "momo", _i0, 0, cm_print_momo, cm_get_momo,  set_ro,       nullptr, 0 },    // motion mode
    { "", "plan", _i0, 0, cm_print_plan, cm_get_plan,  set_ro,       nullptr, 0 },    // plane select
    { "", "path", _i0, 0, cm_print_path, cm_get_path,  set_ro,       nullptr, 0 },    // path control mode
    { "", "dist", _i0, 0, cm_print_dist, cm_get_dist,  set_ro,       nullptr, 0 },    // distance mode
    { "", "admo", _i0, 0, cm_print_admo, cm_get_admo,  set_ro,       nullptr, 0 },    // arc distance mode
    { "", "frmo", _i0, 0, cm_print_frmo, cm_get_frmo,  set_ro,       nullptr, 0 },    // feed rate mode
    { "", "tool", _i0, 0, cm_print_tool, cm_get_toolv, set_ro,       nullptr, 0 },    // active tool
    { "", "safe", _i0, 0, cm_print_safe, cm_get_safe,  set_ro,       nullptr, 0 },    // interlock status
    { "", "estp", _i0, 0, cm_print_estp, cm_get_estp,  cm_ack_estop, nullptr, 0 },    // E-stop status (SET to ack)
    { "", "estpc",_i0, 0, cm_print_estp, cm_ack_estop, cm_ack_estop, nullptr, 0 },  // E-stop status clear (GET to ack)
    { "", "four", _i0, 0, cm_print_four, cm_get_four,  set_ro,       nullptr, 0 },    // fourth axis deployed
    
    { "", "g92e", _i0, 0, cm_print_g92e, cm_get_g92e,  set_ro,nullptr, 0 },    // G92 enable state
#ifdef TEMPORARY_HAS_LEDS
    { "", "_leds",_i0, 0, tx_print_nul, _get_leds,_set_leds, nullptr, 0 },    // TEMPORARY - change LEDs
#endif

    { "mpo","mpox",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // X machine position
    { "mpo","mpoy",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // Y machine position
    { "mpo","mpoz",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // Z machine position
    { "mpo","mpoa",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // A machine position

    { "pos","posx",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // X work position
    { "pos","posy",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // Y work position
    { "pos","posz",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // Z work position
    { "pos","posa",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // A work position

    { "ofs","ofsx",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // X work offset
    { "ofs","ofsy",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // Y work offset
    { "ofs","ofsz",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // Z work offset
    { "ofs","ofsa",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // A work offset

    { "hom","home",_i0, 0, cm_print_home,cm_get_home,cm_set_home,nullptr,0 }, // homing state, invoke homing cycle
    { "hom","homx",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // X homed - Homing status group
    { "hom","homy",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // Y homed
    { "hom","homz",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // Z homed
    { "hom","homa",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // A homed
    
    { "prb","prbe",_i0, 0, tx_print_nul, cm_get_prob,set_ro, nullptr, 0 },    // probing state
    { "prb","prbx",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // X probe results
    { "prb","prby",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // Y probe results
    { "prb","prbz",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // Z probe results
    { "prb","prba",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // A probe results
    { "prb","prbs",_i0, 0, tx_print_nul, get_nul, cm_set_probe, nullptr,0 },  // store probe
    { "prb","prbr",_bip, 0, tx_print_nul, cm_get_prbr, cm_set_prbr, nullptr, PROBE_REPORT_ENABLE }, // enable probe report. Init in cm_init
    { "prb","prbin",_iip, 0, tx_print_nul, cm_get_probe_input, cm_set_probe_input, nullptr, PROBING_INPUT }, // probing input

    { "jog","jogx",_f0, 5, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in X axis
    { "jog","jogy",_f0, 5, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in Y axis
    { "jog","jogz",_f0, 5, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in Z axis
    { "jog","joga",_f0, 5, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in A axis
 
    // END generated with ${PROJECT_ROOT}/Resources/generate_dins_cfgArray.js
 
/***************/
   

    //sme: use spindle_ctrl accessor functions, which will apply 
    // PWM settings://sme: "phase" == throttle pwm duty cycle min, max settings
    //1-22-2021: Even though th MCB+ replaces pwm with VFD for spindle speed control, we retain the pwm struct
    //for assigning and referencing spindle min and max speeds
    { "p1","p1frq",_fip, 5, pwm_print_p1frq, get_flt, set_flt, &pwm.c[PWM_SPINDLE].frequency,  P1_PWM_FREQUENCY }, 
    { "p1","p1dc", _fip, 5, pwm_print_p1dc,  get_flt, set_flt, &pwm.c[PWM_SPINDLE].duty_cycle, P1_PWM_PHASE_OFF }, //phase is duty cycle	
    { "p1","p1pl", _fip, 5, pwm_print_p1pl,  get_flt, set_flt, &pwm.c[PWM_SPINDLE].phase_lo,   P1_PHASE_LO },
    { "p1","p1ph", _fip, 5, pwm_print_p1ph,  get_flt, set_flt, &pwm.c[PWM_SPINDLE].phase_hi,   P1_PHASE_HI },
    { "p1","p1pof",_fip, 5, pwm_print_p1pof, get_flt, set_flt, &pwm.c[PWM_SPINDLE].phase_off,  P1_PWM_PHASE_OFF }, //phase is duty cycle	
    { "p1","p1sl", _fip, 5, pwm_print_p1sl,  get_flt, set_flt, &pwm.c[PWM_SPINDLE].speed_lo,   P1_SPEED_LO },
    { "p1","p1sh", _fip, 5, pwm_print_p1sh,  get_flt, set_flt, &pwm.c[PWM_SPINDLE].speed_hi,   P1_SPEED_HI },
    { "p1","p1k",  _fip, 5, pwm_print_p1k,   get_flt, set_flt, &pwm.c[PWM_SPINDLE].k_value,    P1_PWM_K_VALUE },

   
/******************/
  // Motor parameters
  // generated with ${PROJECT_ROOT}/Resources/generate_motors_cfgArray.js
    { "1","1mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M1_MICROSTEPS },       
    { "1","1ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M1_MOTOR_MAP },
    { "1","1sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M1_STEP_ANGLE },
    { "1","1tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M1_TRAVEL_PER_REV },
    { "1","1su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M1_STEPS_PER_UNIT },
    { "1","1po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M1_POLARITY },

    { "2","2mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M2_MICROSTEPS },
    { "2","2ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M2_MOTOR_MAP },
    { "2","2sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M2_STEP_ANGLE },
    { "2","2tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M2_TRAVEL_PER_REV },
    { "2","2su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M2_STEPS_PER_UNIT },
    { "2","2po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M2_POLARITY },

    { "3","3mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M3_MICROSTEPS },
    { "3","3ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M3_MOTOR_MAP },
    { "3","3sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M3_STEP_ANGLE },
    { "3","3tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M3_TRAVEL_PER_REV },
    { "3","3su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M3_STEPS_PER_UNIT },
    { "3","3po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M3_POLARITY },
    
    { "4","4mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M4_MICROSTEPS },
    { "4","4ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M4_MOTOR_MAP },
    { "4","4sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M4_STEP_ANGLE },
    { "4","4tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M4_TRAVEL_PER_REV },
    { "4","4su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M4_STEPS_PER_UNIT },
    { "4","4po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M4_POLARITY }, 

//tbd: add second stepper for Y axis    
#ifdef STALLGUARD_TESTING
     { "1", "1tnselect", _iip, 0, tn_print_select, get_ui8, tmc2660_set_select, (uint8_t *)&tmc2660_select_motor_id,           SELECT_NONE },//For stallguard tuning, poll a selected axis
#endif    
     { "1","1tnsemin", _iip, 0, tn_print_cs_thrsl, get_ui8, tmc2660_set_cs_lower_thrsh, (uint8_t *)&tn[MOTOR_1].cs_set_se_min_thresh,M1_TN_COOLSTEP_THRESH_MIN},//set coolstep min threshold, range 0-15[0=disabled]
     { "2","2tnsemin", _iip, 0, tn_print_cs_thrsl, get_ui8, tmc2660_set_cs_lower_thrsh, (uint8_t *)&tn[MOTOR_2].cs_set_se_min_thresh,M2_TN_COOLSTEP_THRESH_MIN},//set coolstep min threshold, range 0-15[0=disabled]
     { "3","3tnsemin", _iip, 0, tn_print_cs_thrsl, get_ui8, tmc2660_set_cs_lower_thrsh, (uint8_t *)&tn[MOTOR_3].cs_set_se_min_thresh,M3_TN_COOLSTEP_THRESH_MIN},//set coolstep min threshold, range 0-15[0=disabled]
     { "4","4tnsemin", _iip, 0, tn_print_cs_thrsl, get_ui8, tmc2660_set_cs_lower_thrsh, (uint8_t *)&tn[MOTOR_4].cs_set_se_min_thresh,M4_TN_COOLSTEP_THRESH_MIN},//set coolstep min threshold, range 0-15[0=disabled]
                                                                                                                                                                                                   
     { "1","1tnsemax", _iip, 0, tn_print_cs_thrsh, get_ui8, tmc2660_set_cs_upper_thrsh, (uint8_t *)&tn[MOTOR_1].cs_set_se_max_thresh,M1_TN_COOLSTEP_THRESH_MAX},//set coolstep , range 0-15
     { "2","2tnsemax", _iip, 0, tn_print_cs_thrsh, get_ui8, tmc2660_set_cs_upper_thrsh, (uint8_t *)&tn[MOTOR_2].cs_set_se_max_thresh,M2_TN_COOLSTEP_THRESH_MAX},//set coolstep , range 0-15
     { "3","3tnsemax", _iip, 0, tn_print_cs_thrsh, get_ui8, tmc2660_set_cs_upper_thrsh, (uint8_t *)&tn[MOTOR_3].cs_set_se_max_thresh,M3_TN_COOLSTEP_THRESH_MAX},//set coolstep , range 0-15
     { "4","4tnsemax", _iip, 0, tn_print_cs_thrsh, get_ui8, tmc2660_set_cs_upper_thrsh, (uint8_t *)&tn[MOTOR_4].cs_set_se_max_thresh,M4_TN_COOLSTEP_THRESH_MAX},//set coolstep , range 0-15
           
     { "1","1tnseup", _iip, 0, tn_print_cs_coil_i_up, get_ui8, tmc2660_set_cs_coil_incr_up, (uint8_t *)&tn[MOTOR_1].cs_set_i_incr_step_size,M1_TN_COIL_I_INCR_UP},//"1tnseup" coil current increments range 0-3
     { "2","2tnseup", _iip, 0, tn_print_cs_coil_i_up, get_ui8, tmc2660_set_cs_coil_incr_up, (uint8_t *)&tn[MOTOR_2].cs_set_i_incr_step_size,M2_TN_COIL_I_INCR_UP},//"1tnseup" coil current increments range 0-3
     { "3","3tnseup", _iip, 0, tn_print_cs_coil_i_up, get_ui8, tmc2660_set_cs_coil_incr_up, (uint8_t *)&tn[MOTOR_3].cs_set_i_incr_step_size,M3_TN_COIL_I_INCR_UP},//"1tnseup" coil current increments range 0-3
     { "4","4tnseup", _iip, 0, tn_print_cs_coil_i_up, get_ui8, tmc2660_set_cs_coil_incr_up, (uint8_t *)&tn[MOTOR_4].cs_set_i_incr_step_size,M4_TN_COIL_I_INCR_UP},//"1tnseup" coil current increments range 0-3

     { "1","1tnsedn", _iip, 0, tn_print_cs_coil_i_dn, get_ui8, tmc2660_set_cs_coil_incr_dn, (uint8_t *)&tn[MOTOR_1].cs_set_i_decr_speed,M1_TN_COIL_I_INCR_DN},//"1tnseup" coil current increments range 0-3
     { "2","2tnsedn", _iip, 0, tn_print_cs_coil_i_dn, get_ui8, tmc2660_set_cs_coil_incr_dn, (uint8_t *)&tn[MOTOR_2].cs_set_i_decr_speed,M2_TN_COIL_I_INCR_DN},//"1tnseup" coil current increments range 0-3
     { "3","3tnsedn", _iip, 0, tn_print_cs_coil_i_dn, get_ui8, tmc2660_set_cs_coil_incr_dn, (uint8_t *)&tn[MOTOR_3].cs_set_i_decr_speed,M3_TN_COIL_I_INCR_DN},//"1tnseup" coil current increments range 0-3
     { "4","4tnsedn", _iip, 0, tn_print_cs_coil_i_dn, get_ui8, tmc2660_set_cs_coil_incr_dn, (uint8_t *)&tn[MOTOR_4].cs_set_i_decr_speed,M4_TN_COIL_I_INCR_DN},//"1tnseup" coil current increments range 0-3
   
     { "1", "1tnseimin", _iip, 0, tn_print_standstill_imin, get_ui8, tmc2660_set_cs_standstill_imin, (uint8_t *)&tn[MOTOR_1].cs_set_i_standstill, M1_TN_SE_IMIN}, //range: 1= CS/4, 0 = CS/2
     { "2", "2tnseimin", _iip, 0, tn_print_standstill_imin, get_ui8, tmc2660_set_cs_standstill_imin, (uint8_t *)&tn[MOTOR_2].cs_set_i_standstill, M2_TN_SE_IMIN}, //range: 1= CS/4, 0 = CS/2
     { "3", "3tnseimin", _iip, 0, tn_print_standstill_imin, get_ui8, tmc2660_set_cs_standstill_imin, (uint8_t *)&tn[MOTOR_3].cs_set_i_standstill, M3_TN_SE_IMIN}, //range: 1= CS/4, 0 = CS/2
     { "4", "4tnseimin", _iip, 0, tn_print_standstill_imin, get_ui8, tmc2660_set_cs_standstill_imin, (uint8_t *)&tn[MOTOR_4].cs_set_i_standstill, M4_TN_SE_IMIN}, //range: 1= CS/4, 0 = CS/2
  
     { "1", "1tnsgt", _iip, 0, tn_print_sg_thrsh, get_ui8, tmc2660_set_sg_thresh, (uint8_t *)&tn[MOTOR_1].sg_set_stall_threshold, M1_TN_SG_THRESH_VAL },//SG==Stallguard, range -64-+63, advised to stay above -10
     { "2", "2tnsgt", _iip, 0, tn_print_sg_thrsh, get_ui8, tmc2660_set_sg_thresh, (uint8_t *)&tn[MOTOR_2].sg_set_stall_threshold, M2_TN_SG_THRESH_VAL },//SG==Stallguard, range -64-+63, advised to stay above -10 
     { "3", "3tnsgt", _iip, 0, tn_print_sg_thrsh, get_ui8, tmc2660_set_sg_thresh, (uint8_t *)&tn[MOTOR_3].sg_set_stall_threshold, M3_TN_SG_THRESH_VAL },//SG==Stallguard, range -64-+63, advised to stay above -10 
     { "4", "4tnsgt", _iip, 0, tn_print_sg_thrsh, get_ui8, tmc2660_set_sg_thresh, (uint8_t *)&tn[MOTOR_4].sg_set_stall_threshold, M4_TN_SG_THRESH_VAL },//SG==Stallguard, range -64-+63, advised to stay above -10 

     { "1", "1tnsfilt", _iip, 0, tn_print_filt_en, get_ui8, tmc2660_set_sg_filt_enable, (uint8_t *)&tn[MOTOR_1].sg_set_stall_filter_enable, M1_TN_SG_FILT_EN },//range: enable=1, disable=0 stall guard filtering 
     { "2", "2tnsfilt", _iip, 0, tn_print_filt_en, get_ui8, tmc2660_set_sg_filt_enable, (uint8_t *)&tn[MOTOR_2].sg_set_stall_filter_enable, M2_TN_SG_FILT_EN },//range: enable=1, disable=0 stall guard filtering 
     { "3", "3tnsfilt", _iip, 0, tn_print_filt_en, get_ui8, tmc2660_set_sg_filt_enable, (uint8_t *)&tn[MOTOR_3].sg_set_stall_filter_enable, M3_TN_SG_FILT_EN },//range: enable=1, disable=0 stall guard filtering 
     { "4", "4tnsfilt", _iip, 0, tn_print_filt_en, get_ui8, tmc2660_set_sg_filt_enable, (uint8_t *)&tn[MOTOR_4].sg_set_stall_filter_enable, M4_TN_SG_FILT_EN },//range: enable=1, disable=0 stall guard filtering 
    
     { "1", "1tnchm", _iip, 0, tn_print_chpr_mode, get_ui8, tmc2660_set_chpr_mode, (uint8_t *)&tn[MOTOR_1].chpr_set_mode, M1_TN_CHOP_CFG_MODE },//range: 0=spreadcycle, 1= constant off-time 
     { "2", "2tnchm", _iip, 0, tn_print_chpr_mode, get_ui8, tmc2660_set_chpr_mode, (uint8_t *)&tn[MOTOR_2].chpr_set_mode, M2_TN_CHOP_CFG_MODE },//range: 0=spreadcycle, 1= constant off-time  
     { "3", "3tnchm", _iip, 0, tn_print_chpr_mode, get_ui8, tmc2660_set_chpr_mode, (uint8_t *)&tn[MOTOR_3].chpr_set_mode, M3_TN_CHOP_CFG_MODE },//range: 0=spreadcycle, 1= constant off-time 
     { "4", "4tnchm", _iip, 0, tn_print_chpr_mode, get_ui8, tmc2660_set_chpr_mode, (uint8_t *)&tn[MOTOR_4].chpr_set_mode, M4_TN_CHOP_CFG_MODE },//range: 0=spreadcycle, 1= constant off-time  

     { "1", "1tntbl", _iip, 0, tn_print_blnkng_time, get_ui8, tmc2660_set_chpr_blanking_time, (uint8_t *)&tn[MOTOR_1].chpr_set_blanking_time, M1_TN_CHOP_CFG_BLNKNG_TIME },//range:0-3: 0=16,1=24,2=36,3=54
     { "2", "2tntbl", _iip, 0, tn_print_blnkng_time, get_ui8, tmc2660_set_chpr_blanking_time, (uint8_t *)&tn[MOTOR_2].chpr_set_blanking_time, M2_TN_CHOP_CFG_BLNKNG_TIME },//range:0-3: 0=16,1=24,2=36,3=54
     { "3", "3tntbl", _iip, 0, tn_print_blnkng_time, get_ui8, tmc2660_set_chpr_blanking_time, (uint8_t *)&tn[MOTOR_3].chpr_set_blanking_time, M3_TN_CHOP_CFG_BLNKNG_TIME },//range:0-3: 0=16,1=24,2=36,3=54
     { "4", "4tntbl", _iip, 0, tn_print_blnkng_time, get_ui8, tmc2660_set_chpr_blanking_time, (uint8_t *)&tn[MOTOR_4].chpr_set_blanking_time, M4_TN_CHOP_CFG_BLNKNG_TIME },//range:0-3: 0=16,1=24,2=36,3=54
     
     { "1", "1tnrndtf", _iip, 0, tn_print_rndm_off_time, get_ui8, tmc2660_set_chpr_rndm_off_time, (uint8_t *)&tn[MOTOR_1].chpr_set_rndm_off_time, M1_TN_CHOP_CFG_RNDM_OFF_TIME },//range: 0= disable, 1=enable needed when hearing audible beat frequency
     { "2", "2tnrndtf", _iip, 0, tn_print_rndm_off_time, get_ui8, tmc2660_set_chpr_rndm_off_time, (uint8_t *)&tn[MOTOR_2].chpr_set_rndm_off_time, M2_TN_CHOP_CFG_RNDM_OFF_TIME },//range: 0= disable, 1=enable needed when hearing audible beat frequency
     { "3", "3tnrndtf", _iip, 0, tn_print_rndm_off_time, get_ui8, tmc2660_set_chpr_rndm_off_time, (uint8_t *)&tn[MOTOR_3].chpr_set_rndm_off_time, M3_TN_CHOP_CFG_RNDM_OFF_TIME },//range: 0= disable, 1=enable needed when hearing audible beat frequency
     { "4", "4tnrndtf", _iip, 0, tn_print_rndm_off_time, get_ui8, tmc2660_set_chpr_rndm_off_time, (uint8_t *)&tn[MOTOR_4].chpr_set_rndm_off_time, M4_TN_CHOP_CFG_RNDM_OFF_TIME },//range: 0= disable, 1=enable needed when hearing audible beat frequency

     { "1", "1tnchfdt", _iip, 0, tn_print_fast_decay_time, get_ui8, tmc2660_set_chpr_chm1_fast_decay_time, (uint8_t *)&tn[MOTOR_1].chpr_set_chm1_fast_decay_time, M1_TN_CHOP_FAST_DECAY_TIME },//range 0= slow, 1-15 duration of fast decay, precondition: CHM=1 
     { "2", "2tnchfdt", _iip, 0, tn_print_fast_decay_time, get_ui8, tmc2660_set_chpr_chm1_fast_decay_time, (uint8_t *)&tn[MOTOR_2].chpr_set_chm1_fast_decay_time, M2_TN_CHOP_FAST_DECAY_TIME },//range 0= slow, 1-15 duration of fast decay, precondition: CHM=1 
     { "3", "3tnchfdt", _iip, 0, tn_print_fast_decay_time, get_ui8, tmc2660_set_chpr_chm1_fast_decay_time, (uint8_t *)&tn[MOTOR_3].chpr_set_chm1_fast_decay_time, M3_TN_CHOP_FAST_DECAY_TIME },//range 0= slow, 1-15 duration of fast decay, precondition: CHM=1 
     { "4", "4tnchfdt", _iip, 0, tn_print_fast_decay_time, get_ui8, tmc2660_set_chpr_chm1_fast_decay_time, (uint8_t *)&tn[MOTOR_4].chpr_set_chm1_fast_decay_time, M4_TN_CHOP_FAST_DECAY_TIME },//range 0= slow, 1-15 duration of fast decay, precondition: CHM=1 
     
     { "1", "1tnchfdm", _iip, 0, tn_print_chpr_chm1_fast_decay_mode, get_ui8, tmc2660_set_chpr_chm1_fast_decay_mode, (uint8_t *)&tn[MOTOR_1].chpr_set_chm1_fast_decay_mode, M1_TN_CHOP_FAST_DECAY_MODE },//range:0=enable termination, 1= end by time only, precondition: CHM=1
     { "2", "2tnchfdm", _iip, 0, tn_print_chpr_chm1_fast_decay_mode, get_ui8, tmc2660_set_chpr_chm1_fast_decay_mode, (uint8_t *)&tn[MOTOR_2].chpr_set_chm1_fast_decay_mode, M2_TN_CHOP_FAST_DECAY_MODE },//range:0=enable termination, 1= end by time only, precondition: CHM=1 
     { "3", "3tnchfdm", _iip, 0, tn_print_chpr_chm1_fast_decay_mode, get_ui8, tmc2660_set_chpr_chm1_fast_decay_mode, (uint8_t *)&tn[MOTOR_3].chpr_set_chm1_fast_decay_mode, M3_TN_CHOP_FAST_DECAY_MODE },//range:0=enable termination, 1= end by time only, precondition: CHM=1 
     { "4", "4tnchfdm", _iip, 0, tn_print_chpr_chm1_fast_decay_mode, get_ui8, tmc2660_set_chpr_chm1_fast_decay_mode, (uint8_t *)&tn[MOTOR_4].chpr_set_chm1_fast_decay_mode, M4_TN_CHOP_FAST_DECAY_MODE },//range:0=enable termination, 1= end by time only, precondition: CHM=1 

     { "1", "1tnswo", _iip, 0, tn_print_chpr_chm1_sine_offset, get_ui8, tmc2660_set_chpr_chm1_sine_offset, (uint8_t *)&tn[MOTOR_1].chpr_set_chm1_sine_offset, M1_TN_CHOP_SINE_OFFSET },//range: 0-2 negative, 3 no offset, 4-15 positive offset  precondition CH1=1  
     { "2", "2tnswo", _iip, 0, tn_print_chpr_chm1_sine_offset, get_ui8, tmc2660_set_chpr_chm1_sine_offset, (uint8_t *)&tn[MOTOR_2].chpr_set_chm1_sine_offset, M2_TN_CHOP_SINE_OFFSET },//range: 0-2 negative, 3 no offset, 4-15 positive offset  precondition CH1=1
     { "3", "3tnswo", _iip, 0, tn_print_chpr_chm1_sine_offset, get_ui8, tmc2660_set_chpr_chm1_sine_offset, (uint8_t *)&tn[MOTOR_3].chpr_set_chm1_sine_offset, M3_TN_CHOP_SINE_OFFSET },//range: 0-2 negative, 3 no offset, 4-15 positive offset  precondition CH1=1  
     { "4", "4tnswo", _iip, 0, tn_print_chpr_chm1_sine_offset, get_ui8, tmc2660_set_chpr_chm1_sine_offset, (uint8_t *)&tn[MOTOR_4].chpr_set_chm1_sine_offset, M4_TN_CHOP_SINE_OFFSET },//range: 0-2 negative, 3 no offset, 4-15 positive offset  precondition CH1=1
    
     { "1", "1tnhdec", _iip, 0, tn_print_chpr_chm0_hyst_decr_intrvl, get_ui8, tmc2660_set_chpr_chm0_hyst_decr_intrvl, (uint8_t *)&tn[MOTOR_1].chpr_set_chm0_hyst_decr_intrvl, M1_TN_CHOP_HYST_DECR_INTRVL },//range:0=16,1=32,2=48,3=64 precondition: CHM=0
     { "2", "2tnhdec", _iip, 0, tn_print_chpr_chm0_hyst_decr_intrvl, get_ui8, tmc2660_set_chpr_chm0_hyst_decr_intrvl, (uint8_t *)&tn[MOTOR_2].chpr_set_chm0_hyst_decr_intrvl, M2_TN_CHOP_HYST_DECR_INTRVL },//range:0=16,1=32,2=48,3=64 precondition: CHM=0 
     { "3", "3tnhdec", _iip, 0, tn_print_chpr_chm0_hyst_decr_intrvl, get_ui8, tmc2660_set_chpr_chm0_hyst_decr_intrvl, (uint8_t *)&tn[MOTOR_3].chpr_set_chm0_hyst_decr_intrvl, M3_TN_CHOP_HYST_DECR_INTRVL },//range:0=16,1=32,2=48,3=64 precondition: CHM=0
     { "4", "4tnhdec", _iip, 0, tn_print_chpr_chm0_hyst_decr_intrvl, get_ui8, tmc2660_set_chpr_chm0_hyst_decr_intrvl, (uint8_t *)&tn[MOTOR_4].chpr_set_chm0_hyst_decr_intrvl, M4_TN_CHOP_HYST_DECR_INTRVL },//range:0=16,1=32,2=48,3=64 precondition: CHM=0 

     { "1", "1tnhstrt", _iip, 0, tn_print_chpr_chm0_hyst_start, get_ui8, tmc2660_set_chpr_chm0_hyst_start, (uint8_t *)&tn[MOTOR_1].chpr_set_chm0_hyst_start, M1_TN_CHOP_HYST_START_DFLT },//range:HEND 0=1,1=2...7=8 precondition: Spreadcycle (CHM=0)
     { "2", "2tnhstrt", _iip, 0, tn_print_chpr_chm0_hyst_start, get_ui8, tmc2660_set_chpr_chm0_hyst_start, (uint8_t *)&tn[MOTOR_2].chpr_set_chm0_hyst_start, M2_TN_CHOP_HYST_START_DFLT },//range:HEND 0=1,1=2...7=8 precondition: Spreadcycle (CHM=0) 
     { "3", "3tnhstrt", _iip, 0, tn_print_chpr_chm0_hyst_start, get_ui8, tmc2660_set_chpr_chm0_hyst_start, (uint8_t *)&tn[MOTOR_3].chpr_set_chm0_hyst_start, M3_TN_CHOP_HYST_START_DFLT },//range:HEND 0=1,1=2...7=8 precondition: Spreadcycle (CHM=0)
     { "4", "4tnhstrt", _iip, 0, tn_print_chpr_chm0_hyst_start, get_ui8, tmc2660_set_chpr_chm0_hyst_start, (uint8_t *)&tn[MOTOR_4].chpr_set_chm0_hyst_start, M4_TN_CHOP_HYST_START_DFLT },//range:HEND 0=1,1=2...7=8 precondition: Spreadcycle (CHM=0) 

     { "1", "1tnhend", _iip, 0, tn_print_chpr_chm0_hyst_end, get_ui8, tmc2660_set_chpr_chm0_hyst_end, (uint8_t *)&tn[MOTOR_1].chpr_set_chm0_hyst_end, M1_TN_CHOP_HYST_END_DFLT },//range:0-15, precondition: CHM=0
     { "2", "2tnhend", _iip, 0, tn_print_chpr_chm0_hyst_end, get_ui8, tmc2660_set_chpr_chm0_hyst_end, (uint8_t *)&tn[MOTOR_2].chpr_set_chm0_hyst_end, M2_TN_CHOP_HYST_END_DFLT },//range:0-15, precondition: CHM=0 
     { "3", "3tnhend", _iip, 0, tn_print_chpr_chm0_hyst_end, get_ui8, tmc2660_set_chpr_chm0_hyst_end, (uint8_t *)&tn[MOTOR_3].chpr_set_chm0_hyst_end, M3_TN_CHOP_HYST_END_DFLT },//range:0-15, precondition: CHM=0
     { "4", "4tnhend", _iip, 0, tn_print_chpr_chm0_hyst_end, get_ui8, tmc2660_set_chpr_chm0_hyst_end, (uint8_t *)&tn[MOTOR_4].chpr_set_chm0_hyst_end, M4_TN_CHOP_HYST_END_DFLT },//range:0-15, precondition: CHM=0 

     { "1", "1tntoff", _iip, 0, tn_print_chpr_off_time, get_ui8, tmc2660_set_chpr_off_time, (uint8_t *)&tn[MOTOR_1].chpr_set_off_time, M1_TN_CHOP_OFF_TIME_DFLT },//range:1 to 15= slow decay time t=1/(f_clck)*((Toff*32)+12)
     { "2", "2tntoff", _iip, 0, tn_print_chpr_off_time, get_ui8, tmc2660_set_chpr_off_time, (uint8_t *)&tn[MOTOR_2].chpr_set_off_time, M2_TN_CHOP_OFF_TIME_DFLT },//range:1 to 15= slow decay time t=1/(f_clck)*((Toff*32)+12) 
     { "3", "3tntoff", _iip, 0, tn_print_chpr_off_time, get_ui8, tmc2660_set_chpr_off_time, (uint8_t *)&tn[MOTOR_3].chpr_set_off_time, M3_TN_CHOP_OFF_TIME_DFLT },//range:1 to 15= slow decay time t=1/(f_clck)*((Toff*32)+12)
     { "4", "4tntoff", _iip, 0, tn_print_chpr_off_time, get_ui8, tmc2660_set_chpr_off_time, (uint8_t *)&tn[MOTOR_4].chpr_set_off_time, M4_TN_CHOP_OFF_TIME_DFLT },//range:1 to 15= slow decay time t=1/(f_clck)*((Toff*32)+12) 
      
     { "1", "1tnslpl", _iip, 0, tn_print_slope_ctrl_ls, get_ui8, tmc2660_set_slope_ls, (uint8_t *)&tn[MOTOR_1].pwr_set_slope_ls, M1_TN_DRVR_CFG_SLOPE_CNTRL_LS },//range:0=Min 3=Max
     { "2", "2tnslpl", _iip, 0, tn_print_slope_ctrl_ls, get_ui8, tmc2660_set_slope_ls, (uint8_t *)&tn[MOTOR_2].pwr_set_slope_ls, M2_TN_DRVR_CFG_SLOPE_CNTRL_LS },//range:0=Min 3=Max 
     { "3", "3tnslpl", _iip, 0, tn_print_slope_ctrl_ls, get_ui8, tmc2660_set_slope_ls, (uint8_t *)&tn[MOTOR_3].pwr_set_slope_ls, M3_TN_DRVR_CFG_SLOPE_CNTRL_LS },//range:0=Min 3=Max
     { "4", "4tnslpl", _iip, 0, tn_print_slope_ctrl_ls, get_ui8, tmc2660_set_slope_ls, (uint8_t *)&tn[MOTOR_2].pwr_set_slope_ls, M4_TN_DRVR_CFG_SLOPE_CNTRL_LS },//range:0=Min 3=Max 

     { "1", "1tnslph",_iip, 0, tn_print_slope_ctrl_hs, get_ui8, tmc2660_set_slope_hs, (uint8_t *)&tn[MOTOR_1].pwr_set_slope_hs, M1_TN_DRVR_CFG_SLOPE_CNTRL_HS },//range: 0=Min, 1=Min+Temper compens, 2=Med+Temper compens,3=Max
     { "2", "2tnslph",_iip, 0, tn_print_slope_ctrl_hs, get_ui8, tmc2660_set_slope_hs, (uint8_t *)&tn[MOTOR_2].pwr_set_slope_hs, M2_TN_DRVR_CFG_SLOPE_CNTRL_HS },//range: 0=Min, 1=Min+Temper compens, 2=Med+Temper compens,3=Max 
     { "3", "3tnslph",_iip, 0, tn_print_slope_ctrl_hs, get_ui8, tmc2660_set_slope_hs, (uint8_t *)&tn[MOTOR_3].pwr_set_slope_hs, M3_TN_DRVR_CFG_SLOPE_CNTRL_HS },//range: 0=Min, 1=Min+Temper compens, 2=Med+Temper compens,3=Max
     { "4", "4tnslph",_iip, 0, tn_print_slope_ctrl_hs, get_ui8, tmc2660_set_slope_hs, (uint8_t *)&tn[MOTOR_4].pwr_set_slope_hs, M4_TN_DRVR_CFG_SLOPE_CNTRL_HS },//range: 0=Min, 1=Min+Temper compens, 2=Med+Temper compens,3=Max 

     { "1", "1tns2g",_iip, 0, tn_print_s2g_detct, get_ui8, tmc2660_set_short_to_gnd_detect, (uint8_t *)&tn[MOTOR_1].prtct_set_shrt2gnd, M1_TN_DRVR_CFG_S2G_PROT_DIS },//range:0=No open load detected,1= Open load detected
     { "2", "2tns2g",_iip, 0, tn_print_s2g_detct, get_ui8, tmc2660_set_short_to_gnd_detect, (uint8_t *)&tn[MOTOR_2].prtct_set_shrt2gnd, M2_TN_DRVR_CFG_S2G_PROT_DIS },//range:0=No open load detected,1= Open load detected
     { "3", "3tns2g",_iip, 0, tn_print_s2g_detct, get_ui8, tmc2660_set_short_to_gnd_detect, (uint8_t *)&tn[MOTOR_3].prtct_set_shrt2gnd, M3_TN_DRVR_CFG_S2G_PROT_DIS },//range:0=No open load detected,1= Open load detected
     { "4", "4tns2g",_iip, 0, tn_print_s2g_detct, get_ui8, tmc2660_set_short_to_gnd_detect, (uint8_t *)&tn[MOTOR_4].prtct_set_shrt2gnd, M4_TN_DRVR_CFG_S2G_PROT_DIS },//range:0=No open load detected,1= Open load detected

     { "1", "1tnts2g",_iip, 0, tn_print_s2g_detct_tmr, get_ui8, tmc2660_set_short_to_gnd_dly_time, (uint8_t *)&tn[MOTOR_1].prtct_set_shrt2gnd_dly, M1_TN_DRVR_CFG_S2G_DETCT_TMR },//range: 0=3.2uS, 1=1.6uS, 2=1.2uS, 3=0.8uS
     { "2", "2tnts2g",_iip, 0, tn_print_s2g_detct_tmr, get_ui8, tmc2660_set_short_to_gnd_dly_time, (uint8_t *)&tn[MOTOR_2].prtct_set_shrt2gnd_dly, M2_TN_DRVR_CFG_S2G_DETCT_TMR },//range: 0=3.2uS, 1=1.6uS, 2=1.2uS, 3=0.8uS
     { "3", "3tnts2g",_iip, 0, tn_print_s2g_detct_tmr, get_ui8, tmc2660_set_short_to_gnd_dly_time, (uint8_t *)&tn[MOTOR_3].prtct_set_shrt2gnd_dly, M3_TN_DRVR_CFG_S2G_DETCT_TMR },//range: 0=3.2uS, 1=1.6uS, 2=1.2uS, 3=0.8uS
     { "4", "4tnts2g",_iip, 0, tn_print_s2g_detct_tmr, get_ui8, tmc2660_set_short_to_gnd_dly_time, (uint8_t *)&tn[MOTOR_4].prtct_set_shrt2gnd_dly, M4_TN_DRVR_CFG_S2G_DETCT_TMR },//range: 0=3.2uS, 1=1.6uS, 2=1.2uS, 3=0.8uS

     { "1", "1tntst",_iip, 0, tn_print_test_mode_off, get_ui8, tmc2660_set_test_mode, (uint8_t *)&tn[MOTOR_1].diag_set_test_mode, M1_TN_DRVR_CFG_TEST_MODE_OFF },//range: 0-3  0= Normal operation
     { "2", "2tntst",_iip, 0, tn_print_test_mode_off, get_ui8, tmc2660_set_test_mode, (uint8_t *)&tn[MOTOR_2].diag_set_test_mode, M2_TN_DRVR_CFG_TEST_MODE_OFF },//range: 0-3  0= Normal operation 
     { "3", "3tntst",_iip, 0, tn_print_test_mode_off, get_ui8, tmc2660_set_test_mode, (uint8_t *)&tn[MOTOR_3].diag_set_test_mode, M3_TN_DRVR_CFG_TEST_MODE_OFF },//range: 0-3  0= Normal operation
     { "4", "4tntst",_iip, 0, tn_print_test_mode_off, get_ui8, tmc2660_set_test_mode, (uint8_t *)&tn[MOTOR_4].diag_set_test_mode, M4_TN_DRVR_CFG_TEST_MODE_OFF },//range: 0-3  0= Normal operation 

     { "1", "1tnintpol",_iip, 0, tn_print_sd_interpol, get_ui8, tmc2660_set_ustep_interpol, (uint8_t *)&tn[MOTOR_1].drv_set_step_interpl, M1_TN_SD_CFG_INTERPOL },//range:1-Enable, 0-Disable
     { "2", "2tnintpol",_iip, 0, tn_print_sd_interpol, get_ui8, tmc2660_set_ustep_interpol, (uint8_t *)&tn[MOTOR_2].drv_set_step_interpl, M2_TN_SD_CFG_INTERPOL },//range:1-Enable, 0-Disable 
     { "3", "3tnintpol",_iip, 0, tn_print_sd_interpol, get_ui8, tmc2660_set_ustep_interpol, (uint8_t *)&tn[MOTOR_3].drv_set_step_interpl, M3_TN_SD_CFG_INTERPOL },//range:1-Enable, 0-Disable
     { "4", "4tnintpol",_iip, 0, tn_print_sd_interpol, get_ui8, tmc2660_set_ustep_interpol, (uint8_t *)&tn[MOTOR_4].drv_set_step_interpl, M4_TN_SD_CFG_INTERPOL },//range:1-Enable, 0-Disable 

     { "1", "1tndedge",_iip, 0, tn_print_edges_per_puls, get_ui8, tmc2660_set_edges_per_pulse, (uint8_t *)&tn[MOTOR_1].drv_set_dbl_edge_step, M1_TN_EDGES_PER_PULSE },//range:0=rising only, 1: Both rising and falling STEP pulse edges are active
     { "2", "2tndedge",_iip, 0, tn_print_edges_per_puls, get_ui8, tmc2660_set_edges_per_pulse, (uint8_t *)&tn[MOTOR_2].drv_set_dbl_edge_step, M2_TN_EDGES_PER_PULSE },//range:0=rising only, 1: Both rising and falling STEP pulse edges are active
     { "3", "3tndedge",_iip, 0, tn_print_edges_per_puls, get_ui8, tmc2660_set_edges_per_pulse, (uint8_t *)&tn[MOTOR_3].drv_set_dbl_edge_step, M3_TN_EDGES_PER_PULSE },//range:0=rising only, 1: Both rising and falling STEP pulse edges are active
     { "4", "4tndedge",_iip, 0, tn_print_edges_per_puls, get_ui8, tmc2660_set_edges_per_pulse, (uint8_t *)&tn[MOTOR_4].drv_set_dbl_edge_step, M4_TN_EDGES_PER_PULSE },//range:0=rising only, 1: Both rising and falling STEP pulse edges are active

     { "1", "1tnsdoff",_iip, 0, tn_print_sd_en, get_ui8, tmc2660_set_step_dir_mode, (uint8_t *)&tn[MOTOR_1].drv_set_stpdir_off, M1_TN_DRVR_CFG_SD_EN },//range:0=Enable 1=Disable Step / dir interface 
     { "2", "2tnsdoff",_iip, 0, tn_print_sd_en, get_ui8, tmc2660_set_step_dir_mode, (uint8_t *)&tn[MOTOR_2].drv_set_stpdir_off, M2_TN_DRVR_CFG_SD_EN },//range:0=Enable 1=Disable Step / dir interface 
     { "3", "3tnsdoff",_iip, 0, tn_print_sd_en, get_ui8, tmc2660_set_step_dir_mode, (uint8_t *)&tn[MOTOR_3].drv_set_stpdir_off, M3_TN_DRVR_CFG_SD_EN },//range:0=Enable 1=Disable Step / dir interface 
     { "4", "4tnsdoff",_iip, 0, tn_print_sd_en, get_ui8, tmc2660_set_step_dir_mode, (uint8_t *)&tn[MOTOR_4].drv_set_stpdir_off, M4_TN_DRVR_CFG_SD_EN },//range:0=Enable 1=Disable Step / dir interface 

     { "1", "1tncs",_iip, 0, tn_print_i_scale, get_ui8, tmc2660_set_sg_iscale, (uint8_t *)&tn[MOTOR_1].sg_set_i_scale, M1_TN_SG_CFG_ISCALE },//range: 0-31, 0=1/32, 1=2/32,...31=32/32
     { "2", "2tncs",_iip, 0, tn_print_i_scale, get_ui8, tmc2660_set_sg_iscale, (uint8_t *)&tn[MOTOR_2].sg_set_i_scale, M2_TN_SG_CFG_ISCALE },//range: 0-31, 0=1/32, 1=2/32,...31=32/32 
     { "3", "3tncs",_iip, 0, tn_print_i_scale, get_ui8, tmc2660_set_sg_iscale, (uint8_t *)&tn[MOTOR_3].sg_set_i_scale, M3_TN_SG_CFG_ISCALE },//range: 0-31, 0=1/32, 1=2/32,...31=32/32
     { "4", "4tncs",_iip, 0, tn_print_i_scale, get_ui8, tmc2660_set_sg_iscale, (uint8_t *)&tn[MOTOR_4].sg_set_i_scale, M4_TN_SG_CFG_ISCALE },//range: 0-31, 0=1/32, 1=2/32,...31=32/32 
     
     { "1", "1tnvsense",_iip, 0, tn_print_vsense_fscale, get_ui8, tmc2660_set_vsense_fscale, (uint8_t *)&tn[MOTOR_1].drv_set_vsense, M1_TN_DRVR_CFG_VSENSE_FSCALE },//range:0-1, 0 = Vsense to 0.36V,1 = Vsense to 0.165V (for reduced power dissipation) 
     { "2", "2tnvsense",_iip, 0, tn_print_vsense_fscale, get_ui8, tmc2660_set_vsense_fscale, (uint8_t *)&tn[MOTOR_2].drv_set_vsense, M2_TN_DRVR_CFG_VSENSE_FSCALE },//range:0-1, 0 = Vsense to 0.36V,1 = Vsense to 0.165V (for reduced power dissipation)  
     { "3", "3tnvsense",_iip, 0, tn_print_vsense_fscale, get_ui8, tmc2660_set_vsense_fscale, (uint8_t *)&tn[MOTOR_3].drv_set_vsense, M3_TN_DRVR_CFG_VSENSE_FSCALE },//range:0-1, 0 = Vsense to 0.36V,1 = Vsense to 0.165V (for reduced power dissipation) 
     { "4", "4tnvsense",_iip, 0, tn_print_vsense_fscale, get_ui8, tmc2660_set_vsense_fscale, (uint8_t *)&tn[MOTOR_4].drv_set_vsense, M4_TN_DRVR_CFG_VSENSE_FSCALE },//range:0-1, 0 = Vsense to 0.36V,1 = Vsense to 0.165V (for reduced power dissipation)  

     //"Read/Gets"
     { "1", "1tnse",_i0, 0, tn_print_coolstep_val, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_1].cs_get_coolstep_value, 0 },//range:0..31
     { "2", "2tnse",_i0, 0, tn_print_coolstep_val, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_2].cs_get_coolstep_value, 0 },//range: 0..31
     { "3", "3tnse",_i0, 0, tn_print_coolstep_val, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_3].cs_get_coolstep_value, 0 },//range:0..31
     { "4", "4tnse",_i0, 0, tn_print_coolstep_val, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_4].cs_get_coolstep_value, 0 },//range: 0..31

     { "1", "1tnsg",_i0, 0, tn_print_stall_val, get_ui16, set_nul, (uint16_t *)&tn[MOTOR_1].sg_get_stall_val, 0 },//"1tnsgr" range 0-1023, high load is inversely proportional to value
     { "2", "2tnsg",_i0, 0, tn_print_stall_val, get_ui16, set_nul, (uint16_t *)&tn[MOTOR_2].sg_get_stall_val, 0 },//"1tnsgr" range 0-1023, high load is inversely proportional to value
     { "3", "3tnsg",_i0, 0, tn_print_stall_val, get_ui16, set_nul, (uint16_t *)&tn[MOTOR_3].sg_get_stall_val, 0 },//"1tnsgr" range 0-1023, high load is inversely proportional to value       
     { "4", "4tnsg",_i0, 0, tn_print_stall_val, get_ui16, set_nul, (uint16_t *)&tn[MOTOR_4].sg_get_stall_val, 0 },//"1tnsgr" range 0-1023, high load is inversely proportional to value       

     { "1", "1tnsgt",_i0, 0, tn_print_stall_flg, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_1].sg_get_stall_flag, 0 },//"1tnsgd"  range: 0=no stall, 1= stall happened
     { "2", "2tnsgt",_i0, 0, tn_print_stall_flg, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_2].sg_get_stall_flag, 0 },//"1tnsgd"  range: 0=no stall, 1= stall happened
     { "3", "3tnsgt",_i0, 0, tn_print_stall_flg, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_3].sg_get_stall_flag, 0 },//"1tnsgd"  range: 0=no stall, 1= stall happened
     { "4", "4tnsgt",_i0, 0, tn_print_stall_flg, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_4].sg_get_stall_flag, 0 },//"1tnsgd"  range: 0=no stall, 1= stall happened

     { "1", "1tns2gab",_i0, 0, tn_print_shrt2gnd_ab, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_1].prtct_get_a_b_shrt2gnd, 0},//"1tnlgr" Read S2GA and S2GB range 0=No short 1=Short    
     { "2", "2tns2gab",_i0, 0, tn_print_shrt2gnd_ab, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_2].prtct_get_a_b_shrt2gnd, 0},//"1tnlgr" Read S2GA and S2GB range 0=No short 1=Short    
     { "3", "3tns2gab",_i0, 0, tn_print_shrt2gnd_ab, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_3].prtct_get_a_b_shrt2gnd, 0},//"1tnlgr" Read S2GA and S2GB range 0=No short 1=Short    
     { "4", "4tns2gab",_i0, 0, tn_print_shrt2gnd_ab, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_4].prtct_get_a_b_shrt2gnd, 0},//"1tnlgr" Read S2GA and S2GB range 0=No short 1=Short    
                                                                                                                    
     { "1", "1tnolab",_i0, 0, tn_print_ol_ab_flgs, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_1].prtct_get_openld_a_b,0 },//"1tnlolr" Read OLA and OLB - Open load detection range 0=No open load detected,1= Open load detected     
     { "2", "2tnolab",_i0, 0, tn_print_ol_ab_flgs, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_2].prtct_get_openld_a_b,0 },//"1tnlolr" Read OLA and OLB - Open load detection range 0=No open load detected,1= Open load detected       
     { "3", "3tnolab",_i0, 0, tn_print_ol_ab_flgs, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_3].prtct_get_openld_a_b,0 },//"1tnlolr" Read OLA and OLB - Open load detection range 0=No open load detected,1= Open load detected     
     { "4", "4tnolab",_i0, 0, tn_print_ol_ab_flgs, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_4].prtct_get_openld_a_b,0 },//"1tnlolr" Read OLA and OLB - Open load detection range 0=No open load detected,1= Open load detected     
 
     { "1", "1tnotpw",_i0, 0, tn_print_otp_ot_flgs, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_1].prtct_get_ovrtmp_wrn, 0 },// Read OTPW and OT - Overtemp warning and shutdown   
     { "2", "2tnotpw",_i0, 0, tn_print_otp_ot_flgs, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_2].prtct_get_ovrtmp_wrn, 0 },// Read OTPW and OT - Overtemp warning and shutdown
     { "3", "3tnotpw",_i0, 0, tn_print_otp_ot_flgs, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_3].prtct_get_ovrtmp_wrn, 0 },// Read OTPW and OT - Overtemp warning and shutdown   
     { "4", "4tnotpw",_i0, 0, tn_print_otp_ot_flgs, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_4].prtct_get_ovrtmp_wrn, 0 },// Read OTPW and OT - Overtemp warning and shutdown

     { "1", "1tnstst",_i0, 0, tn_print_stndstll_flg, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_1].diag_get_stndstill_indic, 0 },// Read Standstill indicator (STST)range 0=No standstill, 1=Stand still   
     { "2", "2tnstt",_i0, 0, tn_print_stndstll_flg, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_2].diag_get_stndstill_indic, 0 },// Read Standstill indicator (STST)range 0=No standstill, 1=Stand still 
     { "3", "3tnstst",_i0, 0, tn_print_stndstll_flg, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_3].diag_get_stndstill_indic, 0 },// Read Standstill indicator (STST)range 0=No standstill, 1=Stand still 
     { "4", "4tnstst",_i0, 0, tn_print_stndstll_flg, get_ui8, set_nul, (uint8_t *)&tn[MOTOR_4].diag_get_stndstill_indic, 0 },// Read Standstill indicator (STST)range 0=No standstill, 1=Stand still 

     { "1", "1tnmstep",_i0, 0, tn_print_ustep_pos, get_ui16, set_nul, (uint16_t *)&tn[MOTOR_1].drv_get_mstep_position, 0},//Read ustep position for coil A in table range, 9 bits: 0..8 position,9 polarity:
     { "2", "2tnmstep",_i0, 0, tn_print_ustep_pos, get_ui16, set_nul, (uint16_t *)&tn[MOTOR_2].drv_get_mstep_position, 0},//Read ustep position for coil A in table range, 9 bits: 0..8 position,9 polarity:
     { "3", "3tnmstep",_i0, 0, tn_print_ustep_pos, get_ui16, set_nul, (uint16_t *)&tn[MOTOR_3].drv_get_mstep_position, 0},//Read ustep position for coil A in table range, 9 bits: 0..8 position,9 polarity:                                                                                                                         
     { "4", "4tnmstep",_i0, 0, tn_print_ustep_pos, get_ui16, set_nul, (uint16_t *)&tn[MOTOR_4].drv_get_mstep_position, 0},//Read ustep position for coil A in table range, 9 bits: 0..8 position,9 polarity:                                                                                                                         

       
  // END generated with ${PROJECT_ROOT}/Resources/generate_motors_cfgArray.js
    // Axis parameters

    { "x","xam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, X_AXIS_MODE },
    { "x","xvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, X_VELOCITY_MAX },
    { "x","xfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, X_FEEDRATE_MAX },
    { "x","xtn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, X_TRAVEL_MIN },
    { "x","xtm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, X_TRAVEL_MAX },
    { "x","xjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, X_JERK_MAX },
    { "x","xjh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, X_JERK_HIGH_SPEED },
    { "x","xhi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, X_HOMING_INPUT },
    { "x","xhd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, X_HOMING_DIRECTION },
    { "x","xsv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, X_SEARCH_VELOCITY },
    { "x","xlv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, X_LATCH_VELOCITY },
    { "x","xlb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, X_LATCH_BACKOFF },
#ifdef REFACTOR_LIMIT_SWITCHES   
    { "x","xlas",_iip, 0, cm_print_las, cm_get_las, cm_set_las, nullptr, X_LIMIT_SW_ACTIVE_STATE},
#endif
    { "x","xzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, X_ZERO_BACKOFF },

    { "y","yam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, Y_AXIS_MODE },
    { "y","yvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, Y_VELOCITY_MAX },
    { "y","yfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, Y_FEEDRATE_MAX },
    { "y","ytn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, Y_TRAVEL_MIN },
    { "y","ytm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, Y_TRAVEL_MAX },
    { "y","yjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, Y_JERK_MAX },
    { "y","yjh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, Y_JERK_HIGH_SPEED },
    { "y","yhi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, Y_HOMING_INPUT },
    { "y","yhd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, Y_HOMING_DIRECTION },
    { "y","ysv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, Y_SEARCH_VELOCITY },
    { "y","ylv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, Y_LATCH_VELOCITY },
    { "y","ylb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, Y_LATCH_BACKOFF },
#ifdef REFACTOR_LIMIT_SWITCHES   
    { "y","ylas",_iip, 0, cm_print_las, cm_get_las, cm_set_las, nullptr, Y_LIMIT_SW_ACTIVE_STATE },
#endif 
    { "y","yzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, Y_ZERO_BACKOFF },

    { "z","zam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, Z_AXIS_MODE },
    { "z","zvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, Z_VELOCITY_MAX },
    { "z","zfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, Z_FEEDRATE_MAX },
    { "z","ztn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, Z_TRAVEL_MIN },
    { "z","ztm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, Z_TRAVEL_MAX },
    { "z","zjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, Z_JERK_MAX },
    { "z","zjh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, Z_JERK_HIGH_SPEED },
    { "z","zhi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, Z_HOMING_INPUT },
    { "z","zhd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, Z_HOMING_DIRECTION },
    { "z","zsv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, Z_SEARCH_VELOCITY },
    { "z","zlv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, Z_LATCH_VELOCITY },
    { "z","zlb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, Z_LATCH_BACKOFF },
#ifdef REFACTOR_LIMIT_SWITCHES   
    { "z","zlas",_iip, 0, cm_print_las, cm_get_las, cm_set_las, nullptr,Z_LIMIT_SW_ACTIVE_STATE },
#endif 
    { "z","zzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, Z_ZERO_BACKOFF },

    { "a","aam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, A_AXIS_MODE },
    { "a","avm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, A_VELOCITY_MAX },   
    { "a","afr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, A_FEEDRATE_MAX },     
    { "a","atn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, A_TRAVEL_MIN },
    { "a","atm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, A_TRAVEL_MAX },   
    { "a","ajm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, A_JERK_MAX },      
    { "a","ajh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, A_JERK_HIGH_SPEED },   
    { "a","ara",_fipc, 5, cm_print_ra, cm_get_ra, cm_set_ra, nullptr, A_RADIUS},
    { "a","ahi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, A_HOMING_INPUT },  
    { "a","asv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, A_SEARCH_VELOCITY },
    { "a","alv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, A_LATCH_VELOCITY },
    { "a","alb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, A_LATCH_BACKOFF }, 
#ifdef REFACTOR_LIMIT_SWITCHES
    { "a","alas",_iip, 0, cm_print_las, cm_get_las, cm_set_las, nullptr, A_LIMIT_SW_ACTIVE_STATE},
#endif 
    { "a","azb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, A_ZERO_BACKOFF },      
    { "a","ahd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, A_HOMING_DIRECTION },

    
    // Coordinate system offsets (G54-G59 and G92)
    { "g54","g54x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_X_OFFSET },
    { "g54","g54y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_Y_OFFSET },
    { "g54","g54z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_Z_OFFSET },
    { "g54","g54a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_A_OFFSET },
  
    { "g55","g55x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_X_OFFSET },
    { "g55","g55y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_Y_OFFSET },
    { "g55","g55z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_Z_OFFSET },
    { "g55","g55a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_A_OFFSET },
    { "g55","g55c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_C_OFFSET },
   
    { "g56","g56x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_X_OFFSET },
    { "g56","g56y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_Y_OFFSET },
    { "g56","g56z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_Z_OFFSET },
    { "g56","g56a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_A_OFFSET },

    { "g57","g57x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_X_OFFSET },
    { "g57","g57y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_Y_OFFSET },
    { "g57","g57z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_Z_OFFSET },
    { "g57","g57a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_A_OFFSET },

    { "g58","g58x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_X_OFFSET },
    { "g58","g58y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_Y_OFFSET },
    { "g58","g58z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_Z_OFFSET },
    { "g58","g58a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_A_OFFSET },

    { "g59","g59x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_X_OFFSET },
    { "g59","g59y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_Y_OFFSET },
    { "g59","g59z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_Z_OFFSET },
    { "g59","g59a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_A_OFFSET },

    { "g92","g92x",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },// G92 handled differently
    { "g92","g92y",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92z",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92a",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },

    // Coordinate positions (G28, G30)
    { "g28","g28x",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },// g28 handled differently
    { "g28","g28y",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28z",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28a",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },

    { "g30","g30x",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },// g30 handled differently
    { "g30","g30y",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30z",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30a",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },

    // this is a 128bit UUID for identifying a previously committed job state
    { "jid","jida",_d0, 0, tx_print_nul, get_data, set_data, &cfg.job_id[0], 0 },
    { "jid","jidb",_d0, 0, tx_print_nul, get_data, set_data, &cfg.job_id[1], 0 },
    { "jid","jidc",_d0, 0, tx_print_nul, get_data, set_data, &cfg.job_id[2], 0 },
    { "jid","jidd",_d0, 0, tx_print_nul, get_data, set_data, &cfg.job_id[3], 0 },

    // fixturing information
    { "fxa","fxast",_fipc, 0, tx_print_nul, get_flt, set_flt, &cfg.fx_state_a, 0 },
    { "fxa","fxa1x",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[0][0], 0 },
    { "fxa","fxa1y",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[0][1], 0 },
    { "fxa","fxa2x",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[1][0], 0 },
    { "fxa","fxa2y",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[1][1], 0 },
    { "fxa","fxa3x",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[2][0], 0 },
    { "fxa","fxa3y",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[2][1], 0 },
    { "fxa","fxa4x",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[3][0], 0 },
    { "fxa","fxa4y",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[3][1], 0 },

    // Spindle functions
    { "sp","spmo", _i0,  0, sp_print_spmo, get_nul,     set_nul,     nullptr, 0 }, // keeping this key around, but it returns null and does nothing
    { "sp","spph", _bip, 0, sp_print_spph, sp_get_spph, sp_set_spph, nullptr, SPINDLE_PAUSE_ON_HOLD },
    { "sp","spde", _fip, 2, sp_print_spde, sp_get_spde, sp_set_spde, nullptr, SPINDLE_SPINUP_DELAY },
    { "sp","spsn", _fip, 2, sp_print_spsn, sp_get_spsn, sp_set_spsn, nullptr, SPINDLE_SPEED_MIN},
    { "sp","spsm", _fip, 2, sp_print_spsm, sp_get_spsm, sp_set_spsm, nullptr, SPINDLE_SPEED_MAX},
    { "sp","spep", _iip, 0, sp_print_spep, sp_get_spep, sp_set_spep, nullptr, SPINDLE_ENABLE_POLARITY },
    { "sp","spdp", _iip, 0, sp_print_spdp, sp_get_spdp, sp_set_spdp, nullptr, SPINDLE_DIR_POLARITY },
    { "sp","spoe", _bip, 0, sp_print_spoe, sp_get_spoe, sp_set_spoe, nullptr, SPINDLE_OVERRIDE_ENABLE},   
    { "sp","spc",  _i0,  0, sp_print_spc,  sp_get_spc,  sp_set_spc,  nullptr, 0 },   // spindle state
    { "sp","sps",  _f0,  0, sp_print_sps,  sp_get_sps,  sp_set_sps,  nullptr, 0 },   // spindle speed
#ifdef DEPLOY_SP_DLY_PER_RAMP_CFG_PARAM//8-12-2022
   { "sp","sprde",  _fip, 2, sp_print_sprde, sp_get_sprde, sp_set_sprde, nullptr, SPINDLE_MS_DLY_PER_RPM_INCR },
   { "sp","sprinc", _fip, 2, sp_print_sprinc, sp_get_sprinc, sp_set_sprinc, nullptr, SPINDLE_RPM_STEP_INCREMENT},
#endif
#ifndef OPTIMIZE_JOB_RUN_TIME_JSON_CMDS     
    { "sp","spo",  _fip, 3, sp_print_spo,  sp_get_spo,  sp_set_spo,  nullptr, SPINDLE_OVERRIDE_FACTOR},    
     // General system parameters
    { "sys","fro", _fin, 3, cm_print_fro,  cm_get_fro, cm_set_fro, nullptr, FEED_OVERRIDE_FACTOR},    
    { "sys","tro", _fin, 3, cm_print_tro,  cm_get_tro, cm_set_tro, nullptr, TRAVERSE_OVERRIDE_FACTOR},     
#endif
    { "sys","jt",  _fipn, 2, cm_print_jt,  cm_get_jt,  cm_set_jt,  nullptr, JUNCTION_INTEGRATION_TIME },
    { "sys","ct",  _fipnc,4, cm_print_ct,  cm_get_ct,  cm_set_ct,  nullptr, CHORDAL_TOLERANCE },
    { "sys","zl",  _fipnc,3, cm_print_zl,  cm_get_zl,  cm_set_zl,  nullptr, FEEDHOLD_Z_LIFT },
    { "sys","sl",  _bipn, 0, cm_print_sl,  cm_get_sl,  cm_set_sl,  nullptr, SOFT_LIMIT_ENABLE },
    { "sys","lim", _bipn, 0, cm_print_lim, cm_get_lim, cm_set_lim, nullptr, HARD_LIMIT_ENABLE },
    { "sys","saf", _bipn, 0, cm_print_saf, cm_get_saf, cm_set_saf, nullptr, SAFETY_INTERLOCK_ENABLE },
    { "sys","m48", _bin, 0, cm_print_m48,  cm_get_m48, cm_get_m48, nullptr, 1 },   // M48/M49 feedrate & spindle override enable
    { "sys","froe",_bin, 0, cm_print_froe, cm_get_froe,cm_set_froe,nullptr, FEED_OVERRIDE_ENABLE},   
    { "sys","troe",_bin, 0, cm_print_troe, cm_get_troe,cm_set_troe,nullptr, TRAVERSE_OVERRIDE_ENABLE},
    { "spot","r", _iip, 0, pwm_rgbw_print,  get_ui8, set_ui8, (uint8_t*)&pwm_rgbw_setting.red, RGBW_DEFAULT_SETTING },
    { "spot","g", _iip, 0, pwm_rgbw_print,  get_ui8, set_ui8, (uint8_t*)&pwm_rgbw_setting.green, RGBW_DEFAULT_SETTING },
    { "spot","b", _iip, 0, pwm_rgbw_print,  get_ui8, set_ui8, (uint8_t*)&pwm_rgbw_setting.blue, RGBW_DEFAULT_SETTING },
    { "spot","w", _iip, 0, pwm_rgbw_print,  get_ui8, set_ui8, (uint8_t*)&pwm_rgbw_setting.white, RGBW_DEFAULT_SETTING },
    { "spot","hz", _iip,0, pwm_rgbw_print,  get_ui8, set_ui8, (uint8_t*)&pwm_rgbw_setting.blink_hz, RGBW_DEFAULT_BLINK_HZ },       

    // Communications and reporting parameters
#ifdef __TEXT_MODE
    { "sys","tv", _iipn, 0, tx_print_tv, txt_get_tv, txt_set_tv, nullptr, TEXT_VERBOSITY },
#endif
 
    { "sys","ej", _iipn, 0, js_print_ej,  js_get_ej, js_set_ej, nullptr, COMM_MODE },
    { "sys","jv", _iipn, 0, js_print_jv,  js_get_jv, js_set_jv, nullptr, JSON_VERBOSITY },
     
    { "sys","qv", _iipn, 0, qr_print_qv,  qr_get_qv, qr_set_qv, nullptr, QUEUE_REPORT_VERBOSITY },
    { "sys","sv", _iipn, 0, sr_print_sv,  sr_get_sv, sr_set_sv, nullptr, STATUS_REPORT_VERBOSITY },
    { "sys","si", _iipn, 0, sr_print_si,  sr_get_si, sr_set_si, nullptr, STATUS_REPORT_INTERVAL_MS },
 
    // Gcode defaults
    // NOTE: The ordering within the gcode defaults is important for token resolution. gc must follow gco
    { "sys","gpl", _iipn, 0, cm_print_gpl, cm_get_gpl, cm_set_gpl, nullptr, GCODE_DEFAULT_PLANE },
    { "sys","gun", _iipn, 0, cm_print_gun, cm_get_gun, cm_set_gun, nullptr, GCODE_DEFAULT_UNITS },
    { "sys","gco", _iipn, 0, cm_print_gco, cm_get_gco, cm_set_gco, nullptr, GCODE_DEFAULT_COORD_SYSTEM },
    { "sys","gpa", _iipn, 0, cm_print_gpa, cm_get_gpa, cm_set_gpa, nullptr, GCODE_DEFAULT_PATH_CONTROL }, 
    { "sys","gdi", _iipn, 0, cm_print_gdi, cm_get_gdi, cm_set_gdi, nullptr, GCODE_DEFAULT_DISTANCE_MODE },
    { "",   "gc2", _s0,   0, tx_print_nul, gc_get_gc,  gc_run_gc,  nullptr, 0 },  // send gcode to secondary planner
    { "",   "gc",  _s0,   0, tx_print_nul, gc_get_gc,  gc_run_gc,  nullptr, 0 },  // gcode block - must be last in this group
    // Actions and Reports
    { "", "sr",   _n0, 0, sr_print_sr,   sr_get,    sr_set,    nullptr, 0 },    // request and set status reports
    { "", "qr",   _n0, 0, qr_print_qr,   qr_get,    set_nul,   nullptr, 0 },    // get queue value - planner buffers available
    { "", "qi",   _n0, 0, qr_print_qi,   qi_get,    set_nul,   nullptr, 0 },    // get queue value - buffers added to queue
    { "", "qo",   _n0, 0, qr_print_qo,   qo_get,    set_nul,   nullptr, 0 },    // get queue value - buffers removed from queue
    { "", "er",   _n0, 0, tx_print_nul,  rpt_er,    set_nul,   nullptr, 0 },    // get bogus exception report for testing    
    { "", "rx",   _n0, 0, tx_print_int,  get_rx,    set_nul,   nullptr, 0 },    // get RX buffer bytes or packets
    { "", "dw",   _i0, 0, tx_print_int,  st_get_dw, set_noop,  nullptr, 0 },    // get dwell time remaining
    { "", "msg",  _s0, 0, tx_print_str,  get_nul,   set_noop,  nullptr, 0 },    // no operation on messages
    { "", "alarm",_n0, 0, tx_print_nul,  cm_alrm,   cm_alrm,   nullptr, 0 },    // trigger alarm
    { "", "panic",_n0, 0, tx_print_nul,  cm_pnic,   cm_pnic,   nullptr, 0 },    // trigger panic
    { "", "shutd",_n0, 0, tx_print_nul,  cm_shutd,  cm_shutd,  nullptr, 0 },    // trigger shutdown
    { "", "clear",_n0, 0, tx_print_nul,  cm_clr,    cm_clr,    nullptr, 0 },    // GET "clear" to clear alarm state
    { "", "clr",  _n0, 0, tx_print_nul,  cm_clr,    cm_clr,    nullptr, 0 },    // synonym for "clear"
    { "", "tick", _n0, 0, tx_print_int,  get_tick,  set_nul,   nullptr, 0 },    // get system time tic
    { "", "tram", _b0, 0, cm_print_tram,cm_get_tram,cm_set_tram,nullptr,0 },    // SET to attempt setting rotation matrix from probes
    { "", "defa", _b0, 0, tx_print_nul,  help_defa,set_defaults,nullptr,0 },    // set/print defaults / help screen
    { "", "mark", _i0, 0, tx_print_nul,  get_int32, set_int32, &cfg.mark, 0 },
    { "", "hw_reset",_f0, 0, tx_print_nul, help_hw_reset, main_hw_reset, &cs.null, 0 }, 
    { "", "flash",_f0, 0, tx_print_nul, help_dfuboot_loader, main_set_dfu_bootloader_mode, &cs.null, 0 },
    { "", "dfuboot",_f0, 0, tx_print_nul, help_dfuboot_loader, main_set_dfu_bootloader_mode, &cs.null, 0 },    
#ifdef DEPLOY_HARD_FAULT_REPORTING
    { "", "hwfault",   _s0, 0, tx_print_str,  get_hard_fault_info, clear_hard_fault_info,  nullptr, PRESERVE_VALUE }, //get hwfault registers hex value
#endif
 
    
#ifdef DEPLOY_KERNEL_LOG_REPORTING
    { "", "kxt", _f0,  0, kernel_print_kxt,  kernel_get_kxt,  set_ro, nullptr, 0 },
    { "", "kfl", _f0,  0, kernel_print_kfl,  kernel_get_kfl,  set_ro, nullptr, 0 },
#endif
     
#if 1//debug only allow, irrelevant, trinamics manages power!        
    { "sys","mt", _fipn, 2, st_print_mt,  get_flt,   st_set_mt,  &st_cfg.motor_power_timeout,MOTOR_POWER_TIMEOUT},
    { "",   "me", _f0,   0, tx_print_str, st_set_me, st_set_me,  &cs.null, 0 },
    { "",   "md", _f0,   0, tx_print_str, st_set_md, st_set_md,  &cs.null, 0 },
#endif     
#ifdef __HELP_SCREENS
    { "", "help",_b0, 0, tx_print_nul, help_config, set_nul, nullptr, 0 },  // prints config help screen
    { "", "h",   _b0, 0, tx_print_nul, help_config, set_nul, nullptr, 0 },  // alias for "help"
#endif

#ifdef __USER_DATA
    // User defined data groups
#ifdef DEPLOY_INSTALLED_TOOL_PERSISTENCE 
    { "uda","uda0", _dip, 0, tx_print_int, get_data, set_uda_data, &cfg.user_data_a[0], USER_DATA_A0 },
    { "uda","uda1", _dip, 0, tx_print_int, get_data, set_uda_data, &cfg.user_data_a[1], USER_DATA_A1 },
    { "uda","uda2", _dip, 0, tx_print_int, get_data, set_uda_data, &cfg.user_data_a[2], USER_DATA_A2 },
    { "uda","uda3", _dip, 0, tx_print_int, get_data, set_uda_data, &cfg.user_data_a[3], USER_DATA_A3 },
#else
    { "uda","uda0", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[0], USER_DATA_A0 },
    { "uda","uda1", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[1], USER_DATA_A1 },
    { "uda","uda2", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[2], USER_DATA_A2 },
    { "uda","uda3", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[3], USER_DATA_A3 },
#endif
    { "udb","udb0", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[0], USER_DATA_B0 },
    { "udb","udb1", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[1], USER_DATA_B1 },
    { "udb","udb2", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[2], USER_DATA_B2 },
    { "udb","udb3", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[3], USER_DATA_B3 },

    { "udc","udc0", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[0], USER_DATA_C0 },
    { "udc","udc1", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[1], USER_DATA_C1 },
    { "udc","udc2", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[2], USER_DATA_C2 },
    { "udc","udc3", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[3], USER_DATA_C3 },

    { "udd","udd0", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[0], USER_DATA_D0 },
    { "udd","udd1", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[1], USER_DATA_D1 },
    { "udd","udd2", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[2], USER_DATA_D2 },
    { "udd","udd3", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[3], USER_DATA_D3 },

#endif 

 

#ifdef DEPLOY_UNUSED_G2CORE_TOF_GROUP
    // sme: 10-5-2021 Not deployed by Bantam: Tool table offsets
    { "tof","tofx",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofy",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofz",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofa",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
#endif    
//#ifdef ATC_ALIGN_ADJUST_PARAMS
 
#define TT_OFFSET 0//sme: quick over-simplification on observed 0 for all TT deined in g2_settings_default.h
    // Tool table
    { "tt1","tt1x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt1","tt1y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt1","tt1z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt1","tt1a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt2","tt2x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt2","tt2y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt2","tt2z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt2","tt2a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt3","tt3x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt3","tt3y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt3","tt3z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt3","tt3a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt4","tt4x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt4","tt4y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt4","tt4z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt4","tt4a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt5","tt5x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt5","tt5y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt5","tt5z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt5","tt5a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
 
    { "tt6","tt6x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt6","tt6y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt6","tt6z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt6","tt6a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt6","tt6b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt6","tt6c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt7","tt7x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt7","tt7y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt7","tt7z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt7","tt7a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt7","tt7b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt7","tt7c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt8","tt8x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt8","tt8y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt8","tt8z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt8","tt8a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    
    { "tt9","tt9x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt9","tt9y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt9","tt9z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt9","tt9a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt10","tt10x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt10","tt10y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt10","tt10z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt10","tt10a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt11","tt11x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt11","tt11y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt11","tt11z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt11","tt11a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt12","tt12x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt12","tt12y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt12","tt12z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt12","tt12a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt13","tt13x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt13","tt13y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt13","tt13z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt13","tt13a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt14","tt14x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt14","tt14y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt14","tt14z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt14","tt14a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt15","tt15x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt15","tt15y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt15","tt15z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt15","tt15a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    
    { "tt16","tt16x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt16","tt16y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt16","tt16z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt16","tt16a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt17","tt17x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt17","tt17y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt17","tt17z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt17","tt17a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt18","tt18x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt18","tt18y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt18","tt18z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt18","tt18a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt19","tt19x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt19","tt19y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt19","tt19z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt19","tt19a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt20","tt20x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt20","tt20y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt20","tt20z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt20","tt20a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt21","tt21x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt21","tt21y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt21","tt21z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt21","tt21a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt22","tt22x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt22","tt22y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt22","tt22z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt22","tt22a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
 
    { "tt23","tt23x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt23","tt23y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt23","tt23z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt23","tt23a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt24","tt24x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt24","tt24y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt24","tt24z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt24","tt24a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt25","tt25x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt25","tt25y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt25","tt25z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt25","tt25a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt26","tt26x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt26","tt26y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt26","tt26z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt26","tt26a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt27","tt27x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt27","tt27y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt27","tt27z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt27","tt27a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt28","tt28x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt28","tt28y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt28","tt28z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt28","tt28a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt29","tt29x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt29","tt29y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt29","tt29z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt29","tt29a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    
    { "tt30","tt30x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt30","tt30y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt30","tt30z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt30","tt30a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt31","tt31x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt31","tt31y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt31","tt31z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt31","tt31a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

    { "tt32","tt32x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt32","tt32y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },
    { "tt32","tt32z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET }, 
    { "tt32","tt32a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT_OFFSET },

 
    // Diagnostic parameters
#ifdef __DIAGNOSTIC_PARAMETERS
    { "",    "clc",_f0, 0, tx_print_nul, st_clc,  st_clc, nullptr, 0 },  // clear diagnostic step counters

    { "_te","_tex",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_X], 0 }, // X target endpoint
    { "_te","_tey",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_Y], 0 },
    { "_te","_tez",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_Z], 0 },
    { "_te","_tea",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_A/*Y2*/], 0 },

    { "_tr","_trx",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_X], 0 },  // X target runtime
    { "_tr","_try",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_Y], 0 },
    { "_tr","_trz",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_Z], 0 },
    { "_tr","_tra",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_A/*Y2*/], 0 },
 
    { "_ts","_ts1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_1], 0 },      // Motor 1 target steps
    { "_ps","_ps1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_1], 0 },    // Motor 1 position steps
    { "_cs","_cs1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_1], 0 },   // Motor 1 commanded steps (delayed steps)
    { "_es","_es1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_1], 0 },     // Motor 1 encoder steps
    { "_xs","_xs1",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_1].corrected_steps, 0 }, // Motor 1 correction steps applied
    { "_fe","_fe1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_1], 0 },   // Motor 1 following error in steps
 
    { "_ts","_ts2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_2], 0 },
    { "_ps","_ps2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_2], 0 },
    { "_cs","_cs2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_2], 0 },
    { "_es","_es2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_2], 0 },
    { "_xs","_xs2",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_2].corrected_steps, 0 },
    { "_fe","_fe2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_2], 0 },
 
    { "_ts","_ts3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_3], 0 },
    { "_ps","_ps3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_3], 0 },
    { "_cs","_cs3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_3], 0 },
    { "_es","_es3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_3], 0 },
    { "_xs","_xs3",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_3].corrected_steps, 0 },
    { "_fe","_fe3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_3], 0 },
#ifdef DEPLOY_FOURTH_AXIS  
    { "_ts","_ts4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_4], 0 },
    { "_ps","_ps4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_4], 0 },
    { "_cs","_cs4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_4], 0 },
    { "_es","_es4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_4], 0 },
    { "_xs","_xs4",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_4].corrected_steps, 0 },
    { "_fe","_fe4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_4], 0 },
#endif 
 
    
    //  __DIAGNOSTIC_PARAMETERS
    // Persistence for status report - must be in sequence
    // *** Count must agree with NV_STATUS_REPORT_LEN in report.h ***   
    { "","se00",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[0],0 }, // 950
    { "","se01",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[1],0 },
    { "","se02",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[2],0 },
    { "","se03",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[3],0 },
    { "","se04",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[4],0 },
    { "","se05",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[5],0 },
    { "","se06",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[6],0 },
    { "","se07",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[7],0 },
    { "","se08",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[8],0 },
    { "","se09",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[9],0 },
    { "","se10",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[10],0 },
    { "","se11",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[11],0 },
    { "","se12",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[12],0 },
    { "","se13",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[13],0 },
    { "","se14",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[14],0 },
    { "","se15",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[15],0 },
    { "","se16",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[16],0 },
    { "","se17",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[17],0 },
    { "","se18",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[18],0 },
    { "","se19",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[19],0 },
    { "","se20",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[20],0 },
    { "","se21",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[21],0 },
    { "","se22",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[22],0 },
    { "","se23",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[23],0 },
    { "","se24",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[24],0 },
    { "","se25",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[25],0 },
    { "","se26",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[26],0 },
    { "","se27",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[27],0 },
    { "","se28",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[28],0 },
    { "","se29",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[29],0 },
    { "","se30",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[30],0 },
    { "","se31",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[31],0 },
    { "","se32",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[32],0 },
    { "","se33",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[33],0 },
    { "","se34",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[34],0 },
    { "","se35",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[35],0 },
    { "","se36",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[36],0 },
  
    { "","se37",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[37],0 },
    { "","se38",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[38],0 },
    { "","se39",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[39],0 },
    // Count is 40, since se00 counts as one.
#endif

    // Group lookups - must follow the single-valued entries for proper sub-string matching
    // *** Must agree with NV_COUNT_GROUPS below ***
    // *** If you adjust the number of entries in a group you must also adjust the count for that group ***
    // *** COUNT STARTS FROM HERE ***
#define FIXED_GROUPS 4//3  8-29-2022: added "spot" group for spotlight pwm driven leds ring     
    { "","sys",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // system group
    { "","p1", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // PWM 1 group
    { "","sp", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // Spindle group 
#define SPOTLIGHT_GROUP 1   
    { "","spot", _i0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // "spot"light group    

#define AXIS_GROUPS AXES
    { "","x",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // axis groups
    { "","y",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","z",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","a",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },

#define MOTOR_GROUPS MOTORS
    { "","1",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // motor groups
    { "","2",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","3",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","4",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
 
#define COORDINATE_OFFSET_GROUPS 9
    { "","g54",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // coord offset groups
    { "","g55",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g56",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g57",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g58",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g59",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g92",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // origin offsets
    { "","g28",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // g28 home position
    { "","g30",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // g30 home position
#define TOOL_OFFSET_GROUPS (TOOLS+1)
    { "","tof",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // current tool offsets
    { "","tt1",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt2",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt3",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt4",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt5",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
#if (TOOLS > 5)
    { "","tt6",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt7",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt8",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt9",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt10",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt11",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt12",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt13",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt14",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt15",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt16",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt17",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt18",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt19",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt20",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt21",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt22",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt23",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt24",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt25",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt26",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt27",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt28",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt29",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt30",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt31",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt32",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
#endif

#define MACHINE_STATE_GROUPS 9
    { "","mpo",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // machine position group
    { "","pos",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // work position group
    { "","ofs",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // work offset group
    { "","hom",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // axis homing state group
    { "","prb",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // probing state group
    { "","pwr",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // motor power enagled group
    { "","jog",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // axis jogging state group
    { "","jid",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // job ID group
    { "","fxa",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // fixturing group af

#ifdef __USER_DATA
#define USER_DATA_GROUPS 4
    { "","uda", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // user data group
    { "","udb", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // user data group
    { "","udc", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // user data group
    { "","udd", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // user data group
#else
#define USER_DATA_GROUPS 0
#endif

#ifdef __DIAGNOSTIC_PARAMETERS
#define DIAGNOSTIC_GROUPS 8
    { "","_te",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // target axis endpoint group
    { "","_tr",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // target axis runtime group
    { "","_ts",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // target motor steps group
    { "","_ps",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // position motor steps group
    { "","_cs",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // commanded motor steps group
    { "","_es",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // encoder steps group
    { "","_xs",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // correction steps group
    { "","_fe",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // following error group
#endif

#define NV_COUNT_UBER_GROUPS 4
    // Uber-group (groups of groups, for text-mode displays only)
    // *** Must agree with NV_COUNT_UBER_GROUPS below ****
    { "", "m", _f0, 0, tx_print_nul, _do_motors, set_nul, nullptr, 0 },
    { "", "q", _f0, 0, tx_print_nul, _do_axes,   set_nul, nullptr, 0 },
    { "", "o", _f0, 0, tx_print_nul, _do_offsets,set_nul, nullptr, 0 },
    { "", "$", _f0, 0, tx_print_nul, _do_all,    set_nul, nullptr, 0 }
 };

/***** Make sure these defines line up with any changes in the above table *****/
#define NV_COUNT_GROUPS (FIXED_GROUPS \
                        + AXIS_GROUPS \
                        + MOTOR_GROUPS \
                        + COORDINATE_OFFSET_GROUPS \
                        + TOOL_OFFSET_GROUPS \
                        + MACHINE_STATE_GROUPS \
                        + USER_DATA_GROUPS \
                        + SPOTLIGHT_GROUP \
                        + DIAGNOSTIC_GROUPS)

/* <DO NOT MESS WITH THESE DEFINES> */
 
#define NV_INDEX_MAX (sizeof(cfgArray) / sizeof(cfgItem_t))
#define NV_INDEX_END_SINGLES    (NV_INDEX_MAX - NV_COUNT_UBER_GROUPS - NV_COUNT_GROUPS - NV_STATUS_REPORT_LEN)
#define NV_INDEX_START_GROUPS    (NV_INDEX_MAX - NV_COUNT_UBER_GROUPS - NV_COUNT_GROUPS)
#define NV_INDEX_START_UBER_GROUPS (NV_INDEX_MAX - NV_COUNT_UBER_GROUPS)
/* </DO NOT MESS WITH THESE DEFINES> */
volatile int nv_end_of_singles=NV_INDEX_END_SINGLES;
volatile int nv_start_of_groups=NV_INDEX_START_GROUPS;
 
index_t nv_index_max() { return ( NV_INDEX_MAX );}
bool nv_index_is_single(index_t index) { return ((index <= nv_end_of_singles/*NV_INDEX_END_SINGLES*/) ? true : false);}
bool nv_index_is_group(index_t index) { return (((index >= NV_INDEX_START_GROUPS) && (index < NV_INDEX_START_UBER_GROUPS)) ? true : false);}
bool nv_index_lt_groups(index_t index) { return ((index <= NV_INDEX_START_GROUPS) ? true : false);}
 
/***** APPLICATION SPECIFIC CONFIGS AND EXTENSIONS TO GENERIC FUNCTIONS *****/
/*
 * convert_incoming_float() - pre-process an incoming floating point number for canonical units
 * convert_outgoing_float() - pre-process an outgoing floating point number for units display
 *
 *  Incoming floats are destined for SET operations.
 *  Outgoing floats are the raw values from GET operations, destined for text or JSON display.
 *
 *  Apologies in advance for these twisty little functions. These functions are used to
 *  convert incoming floats into the native, canonical form of a parameter (mm, or whatever)
 *  and outgoing floats into a display format appropriate to the units mode in effect.
 *  They use the flags in the config table and other cues to determine what type of conversion
 *  to perform.
 *
 *  The conversions are complicated by the fact that only linear axes actually convert -
 *  rotaries do not - unless they are in radius mode. Plus, determining the axis for a motor
 *  requires unraveling the motor mapping (handled in cm_get_axis_type()). Also, there are
 *  global SYS group values that are not associated with any axis. Lastly, the
 *  steps-per-unit value (1su) is actually kept in inverse conversion form, as its native
 *  form would be units-per-step.
 */

static void _convert(nvObj_t *nv, float conversion_factor)
{
    volatile float fval=(float)nv->value_flt;
    volatile int ival=nv->value_int;
    volatile bool isnan_flag=isnan(fval);
    volatile bool isinf_flag=isinf(fval);
    volatile static int nan_or_inf_counter=0;
    if (nv->valuetype != TYPE_FLOAT) 
    { 
      return; 
    } // can be called non-destructively for any value type
#if 1
    if (isnan_flag ||isinf_flag)
    {
        nan_or_inf_counter++;
#define MAX_NAN_OR_INF_COUNT 1000//arbitrary
        if (nan_or_inf_counter>MAX_NAN_OR_INF_COUNT)
        {
            nan_or_inf_counter= -1;
        }
        return; 
    }
#else    
    if (isnan((double)nv->value_flt) || isinf((double)nv->value_flt)) 
    { 
       return; 
    } // trap illegal float values
#endif
    ///+++ transform these checks into NaN or INF strings with an error return?

    if (cm_get_units_mode(MODEL) == INCHES) 
    {
        cmAxisType axis_type = cm_get_axis_type(nv);        // linear, rotary, global or error
        if ((axis_type == AXIS_TYPE_LINEAR) || (axis_type == AXIS_TYPE_SYSTEM)) 
        {
            if (cfgArray[nv->index].flags & F_CONVERT) 
            {    // standard units conversion
                    nv->value_flt *= conversion_factor;
            } else
            if (cfgArray[nv->index].flags & F_ICONVERT) 
            {   // inverse units conversion
                nv->value_flt /= conversion_factor;
            }
        }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
}

void convert_incoming_float(nvObj_t *nv) { return(_convert (nv, MM_PER_INCH)); }
void convert_outgoing_float(nvObj_t *nv) { return(_convert (nv, INCHES_PER_MM)); }

/*
 * get_float()       - boilerplate for retrieving raw floating point value
 * set_float()       - boilerplate for setting a floating point value with unit conversion
 * set_float_range() - set a floating point value with inclusive range check
 *
 *  get_float() loads nv->value with 'value' in internal canonical units (e.g. mm, degrees)
 *  without units conversion. If conversion is required call convert_outgoing_float()
 *  afterwards. The text mode and JSON display routines do this, so you generally don't
 *  have to worry about this.
 *
 *  set_float() is designed to capture incoming float values, so it performs unit conversion.
 *  set_float_range() performs an inclusive range test on the CONVERTED value
 */

stat_t get_float(nvObj_t *nv, const float value) 
{
    nv->value_flt = value;
    nv->valuetype = TYPE_FLOAT;
    nv->precision = GET_TABLE_WORD(precision);
    return STAT_OK;
}

stat_t set_float(nvObj_t *nv, float &value) 
{
    convert_incoming_float(nv);
    value = nv->value_flt;
    return (STAT_OK);
}

stat_t set_float_range(nvObj_t *nv, float &value, float low, float high) 
{
    char msg[64];
    convert_incoming_float(nv);      // conditional unit conversion
    if (nv->value_flt < low) 
    {
        sprintf(msg, "Input is less than minimum value %0.4f", low);
        nv_add_conditional_message(msg);
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_flt > high) 
    {
        sprintf(msg, "Input is more than maximum value %0.4f", high);
        nv_add_conditional_message(msg);
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    //sme: 11/7/2022 is this still needed? nv->valuetype=TYPE_FLOAT;//sme: 11-3-2022
    value = nv->value_flt;
    return (STAT_OK);
}

/*
 * get_integer() - boilerplate for retrieving 8 and 32 bit integer values
 * set_int_u8() - boilerplate for setting 8 bit integer value with range checking
 * set_int32()   - boilerplate for setting 32 bit integer value with range checking
 */

stat_t _set_int_tests(nvObj_t *nv, int32_t low, int32_t high)
{
    char msg[64];

    if (nv->value_int < low) 
    {
        sprintf(msg, "Input less than minimum value %d", (int)low);
        nv_add_conditional_message(msg);
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_int > high) 
    {
        sprintf(msg, "Input more than maximum value %d", (int)high);
        nv_add_conditional_message(msg);
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    return (STAT_OK);
}

stat_t get_integer(nvObj_t *nv, const int32_t value)
{
    nv->value_int = value;
    nv->valuetype = TYPE_INTEGER;
    return STAT_OK;
}

stat_t set_int_u8(nvObj_t *nv, uint8_t &value, uint8_t low, uint8_t high)
{
    ritorno(_set_int_tests(nv, low, high))
    value = nv->value_int;
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}


stat_t set_int32(nvObj_t *nv, int32_t &value, int32_t low, int32_t high)
{
    ritorno(_set_int_tests(nv, low, high))
    value = nv->value_int;  // note: valuetype = TYPE_INT already set
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

stat_t set_uint32(nvObj_t *nv, uint32_t &value, int32_t low, int32_t high)
{
    ritorno(_set_int_tests(nv, low, high))
    value = nv->value_int;  // note: valuetype = TYPE_INT already set
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

/*
 * get_string() - boilerplate for retrieving a string value
 */

stat_t get_string(nvObj_t *nv, const char *str)
{
    nv->valuetype = TYPE_STRING;
    return (nv_copy_string(nv, str));
}

/*
 * nv_group_is_prefixed() - hack
 *
 *  This little function deals with the exception cases that some groups don't use
 *  the parent token as a prefix to the child elements; SYS being a good example.
 */
bool nv_group_is_prefixed(char *group)
{
    if (strcmp("sys", group) == 0) 
    {    // =0 means its a match
        return (false);
    }
    if (strcmp("sr", group) == 0) 
    {
        return (false);
    }
    return (true);
}
/*
 * hw_get_fb()  - get firmware build number
 * hw_get_fv()  - get firmware version number
 * hw_get_hp()  - get hardware platform string
 * hw_get_hv()  - get hardware version string
 * hw_get_fbs() - get firmware build string
 */

stat_t hw_get_fb(nvObj_t *nv) 
{ 
    return (get_float(nv, cs.fw_build)); 
}
stat_t hw_get_fv(nvObj_t *nv) { return (get_float(nv, cs.fw_version)); }
//to do refactor to only change the #defines, not the whole set of function calls
stat_t hw_get_hp(nvObj_t *nv) { return (get_string(nv, (const char *)CHABO_HARDWARE_PLATFORM)); }
stat_t hw_get_hv(nvObj_t *nv) { return (get_string(nv, (const char *)CHABO_HARDWARE_VERSION)); }
stat_t hw_get_fbs(nvObj_t *nv) { return (get_string(nv, (const char *)CHABO_FIRMWARE_BUILD_STRING)); }

#ifdef DEPLOY_HARD_FAULT_REPORTING

stat_t clear_hard_fault_info(nvObj_t *nv)
{
    if (nv->value_int<PRESERVE_VALUE)
    {
       memset((void *)&hard_fault_info,0, sizeof(HardFaultInfoS));
    }
    else if(nv->value_int>PRESERVE_VALUE)
    {
        hard_fault_info.fault_flag=true;
        hard_fault_info.hfsr=nv->value_int;
        hard_fault_info.cfsr=nv->value_int<<1;
        sprintf((char*)hard_fault_info.msg_text,"hfsr=0x%x, cfsr=0x%x\n\0",hard_fault_info.hfsr,hard_fault_info.cfsr);
    }
    return STAT_OK;
}
stat_t get_hard_fault_info(nvObj_t *nv)
{
#define FAULE_NONE_TXT (char *)"FAULT NONE"
   
  stat_t status=STAT_OK;
  volatile static uint32_t hfsr=0;
  volatile static uint32_t cfsr=0;
  volatile static bool flag=false;
  char *msgptr= FAULE_NONE_TXT;
  flag = hard_fault_info.fault_flag;
  hfsr=hard_fault_info.hfsr;
  cfsr=hard_fault_info.cfsr;
 
  if (flag==true)
  {
      sprintf((char*)hard_fault_info.msg_text,"hfsr=0x%x, cfsr=0x%x\n\0",hard_fault_info.hfsr,hard_fault_info.cfsr);    
      msgptr=(char*)&hard_fault_info.msg_text[0];
  }
  
  comms_mgr_write_msg(msgptr);
  return status;
}
#endif

#ifdef DEPLOY_KERNEL_LOG_REPORTING

void kernel_print_kxt(nvObj_t *nv)
{
  nv->valuetype=TYPE_PARENT;//remove this when fleshed out
}

stat_t kernel_get_kxt(nvObj_t *nv) 
{
  nv->valuetype=TYPE_PARENT;//remove this when fleshed out
  return STAT_OK;
}

void kernel_print_kfl(nvObj_t *nv)
{
  __NOP();//__no_operation();
}

stat_t kernel_get_kfl(nvObj_t *nv) 
{
  nv->valuetype=TYPE_INTEGER; 
#ifdef DEPLOY_KERNEL_TASK_FAULT_STATUS  
  nv->value_int= kernel_serialize_logfile(cs.out_buf, sizeof(cs.out_buf));  
#endif
  return STAT_OK; 
}
#endif
/**** UberGroup Operations ****************************************************
 * Uber groups are groups of groups organized for convenience:
 *  - motors    - group of all motor groups
 *  - axes      - group of all axis groups
 *  - offsets   - group of all offsets and stored positions
 *  - all       - group of all groups
 *
 * _do_group_list() - get and print all groups in the list (iteration)
 * _do_motors()     - get and print motor uber group 1-N
 * _do_axes()       - get and print axis uber group XYZABC
 * _do_offsets()    - get and print offset uber group G54-G59, G28, G30, G92
 * _do_inputs()     - get and print inputs uber group di1 - diN
 * _do_outputs()    - get and print outputs uber group do1 - doN
 * _do_all()        - get and print all groups uber group
 */

static void _do_group(nvObj_t *nv, char *group)   // helper to a group
{
    nv_reset_nv_list();
    nv = nv_body;
    strncpy(nv->token, group, TOKEN_LEN);
    nv->index = nv_get_index((const char *)"", nv->token);
    nv_get_nvObj(nv);
    nv_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
}

static stat_t _do_group_list(nvObj_t *nv, char list[][TOKEN_LEN+1]) // helper to print multiple groups in a list
{
    for (uint8_t i=0; i < NV_MAX_OBJECTS; i++) 
    {
        if (list[i][0] == NUL) 
        {
            return (STAT_COMPLETE);
        }
        _do_group(nv, list[i]);
    }
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses the normal response line
}

static stat_t _do_motors(nvObj_t *nv)  // print parameters for all motor groups
{
    char group[GROUP_LEN];
    for (uint8_t i=1; i < MOTORS+1; i++) 
    {
        sprintf(group, "%d", i);
        _do_group(nv, group);
    }
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses the normal response line
}

static stat_t _do_axes(nvObj_t *nv)  // print parameters for all axis groups
{

    char list[][TOKEN_LEN+1] = {"x","y","z","a","b","c",""}; // must have a terminating element

    return (_do_group_list(nv, list));
}

static stat_t _do_offsets(nvObj_t *nv)  // print offset parameters for G54-G59,G92, G28, G30
{
    char list[][TOKEN_LEN+1] = {"g54","g55","g56","g57","g58","g59","g92","g28","g30",""}; // must have a terminating element
    return (_do_group_list(nv, list));
}

static stat_t _do_heaters(nvObj_t *nv)  // print parameters for all heater groups
{
    char group[GROUP_LEN];
    for (uint8_t i=1; i < 4; i++) 
    {
        sprintf(group, "he%d", i);
        _do_group(nv, group);
    }
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses the normal response line
}
static stat_t _do_all(nvObj_t *nv)  // print all parameters
{
    _do_group(nv, (char *)"sys");   // System group
    _do_motors(nv);
    _do_axes(nv);
    _do_heaters(nv);                // there are no text mode prints for heaters
    _do_group(nv, (char *)"p1");    // PWM group
    _do_offsets(nv);                // coordinate system offsets
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses a second JSON write that would cause a fault
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * Most of these can be found in their respective modules.
 ***********************************************************************************/

/**** COMMUNICATIONS FUNCTIONS ******************************************************
 * get_rx()   - get bytes available in RX buffer
 * get_tick() - get system tick count
 */

static stat_t get_rx(nvObj_t *nv)
{
    nv->value_int = (float)254;        // ARM always says the serial buffer is available (max)
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

static stat_t get_tick(nvObj_t *nv)
{
    nv->value_int =sys_get_time_ms(); 
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

stat_t main_set_dfu_bootloader_mode(nvObj_t *nv)
{
     delay_ms(500);

      // Issue Clear GPNVM Bit command for Bit 1 (boot mode)
      EFC_REGS->EEFC_FCR = (EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_CGPB | EEFC_FCR_FARG(1));

      // Wait for command to complete
      while (!(EFC_REGS->EEFC_FSR & EEFC_FSR_FRDY_Msk))
      {
      }

      // Reset the device for the programmed fuse value to take effect
      RSTC_REGS->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST_Msk;  
      return STAT_OK;
}
//sme: 01-31-2023: this was missing in the cfg array for CAN==reset command
stat_t main_hw_reset(nvObj_t *nv)
{
      // Reset the device for the programmed fuse value to take effect
      RSTC_REGS->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST_Msk;  
      return STAT_OK;    
}

void get_out_jail_free_reset(void)
{
    RSTC_REGS->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST_Msk;  
}


/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_rx[] = "rx:%d\n";
static const char fmt_ex[] = "[ex]  enable flow control%10d [0=off,1=XON/XOFF, 2=RTS/CTS]\n";

void cfg_print_rx(nvObj_t *nv) { text_print(nv, fmt_rx);}       // TYPE_INT
void cfg_print_ex(nvObj_t *nv) { text_print(nv, fmt_ex);}       // TYPE_INT

#endif // __TEXT_MODE

//from V3
static const char fmt_hp[] = "[hp]  hardware platform%15.2f\n";
static const char fmt_hv[] = "[hv]  hardware version%16.2f\n";
static const char fmt_id[] = "[id]  TinyG ID%30s\n";

void hw_print_fb(nvObj_t *nv){ text_print_flt(nv, fmt_fb);}
void hw_print_fv(nvObj_t *nv){ text_print_flt(nv, fmt_fv);}
void hw_print_cv(nvObj_t *nv){  text_print_flt(nv, fmt_cv);}

void hw_print_hp(nvObj_t *nv){  text_print_flt(nv, fmt_hp);}
void hw_print_hv(nvObj_t *nv){ text_print_flt(nv, fmt_hv);}
void hw_print_id(nvObj_t *nv) {  text_print_str(nv, fmt_id);}


