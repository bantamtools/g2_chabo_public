
/*
 * spindle.h - spindle driver
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
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

#ifndef SPINDLE_H_ONCE
#define SPINDLE_H_ONCE
#include "g2_config.h"

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

#include "spindle_ctrl.h"
#include "g2_error.h"  // for stat_t   

#ifndef SPINDLE_DWELL_MAX
#define SPINDLE_DWELL_MAX   10000000.0      // maximum allowable dwell time. May be overridden in settings files
#endif

#ifndef SPINDLE_SPEED_MIN
#define SPINDLE_SPEED_MIN   P1_SPEED_LO     // {spsn:
#endif

#ifndef SPINDLE_SPEED_MAX
#define SPINDLE_SPEED_MAX     P1_SPEED_HI     // {spsm:
#endif


#define SPINDLE_OVERRIDE_ENABLE true//7-28-2022 false
#define SPINDLE_OVERRIDE_FACTOR 1.00
#define SPINDLE_OVERRIDE_MIN 0.05       // 5%
#define SPINDLE_OVERRIDE_MAX 2.00       // 200%
#define SPINDLE_OVERRIDE_RAMP_TIME 1    // change sped in seconds

typedef enum {
    SPINDLE_DISABLED = 0,       // spindle will not operate
    SPINDLE_PLAN_TO_STOP,       // spindle operating, plans to stop
    SPINDLE_CONTINUOUS,         // spindle operating, does not plan to stop
} spMode;
#define SPINDLE_MODE_MAX SPINDLE_CONTINUOUS

typedef enum {                  // how spindle controls are presented by the Gcode parser
    SPINDLE_OFF = 0,            // M5 
    SPINDLE_RUNNING,            //M3  SME
    SPINDLE_PAUSE,              // request PAUSE and store PAUSED state to spindle.state
    SPINDLE_PAUSED=SPINDLE_PAUSE,// bit to indicate that spindle is currently paused    
    SPINDLE_RESUME,             // request RESUME and revert spindle.state to CW, CCW
    SPINDLE_ACTION_MAX=SPINDLE_RESUME,
    SPINDLE_CTRL_COUNT        
} spControl;


// *** NOTE: The spindle polarity active hi/low values currently agree with ioMode in gpio.h
// These will all need to be changed to ACTIVE_HIGH = 0, ACTIVE_LOW = 1
// See: https://github.com/synthetos/g2_private/wiki/GPIO-Design-Discussion#settings-common-to-all-io-types

typedef enum {                  // Note: These values agree with
    SPINDLE_ACTIVE_LOW = 0,     // Will set output to 0 to enable the spindle or CW direction
    SPINDLE_ACTIVE_HIGH = 1,    // Will set output to 1 to enable the spindle or CW direction
} spPolarity;

typedef enum {                  // electronic speed controller for some spindles
    ESC_ONLINE = 0,
    ESC_OFFLINE,
    ESC_LOCKOUT,
    ESC_REBOOTING,
    ESC_LOCKOUT_AND_REBOOTING,
} ESCState;

/*
 * Spindle control structure
 */

typedef struct spSpindle {

    spControl   state;              // {spc:} OFF, ON, PAUSE, RESUME, WAIT
    float       speed;              // {sps:}  S in RPM
    float       speed_min;          // {spsn:} minimum settable spindle speed
    float       speed_max;          // {spsm:} maximum settable spindle speed
    float       speed_actual;       // hidden internal value used in speed ramping
    float       ramp_step_spinup_dly_ms;//ms time per accel/dece/ ramp step increment
    float       ramp_step_rpm_incr;
    //polarity is n/a for bantam--gut it soon
    spPolarity  enable_polarity;    // {spep:} 0=active low, 1=active high
    spPolarity  dir_polarity;       // {spdp:} 0=clockwise low, 1=clockwise high
   
    bool        pause_enable;       // {spph:} pause on feedhold
    float       spinup_delay;       // {spde:} optional delay on spindle start (set to 0 to disable)
    bool        override_enable;    // {spoe:} TRUE = spindle speed override enabled (see also m48_enable in canonical machine)
    float       override_factor;    // {spo:}  1.0000 x S spindle speed. Go up or down from there
} spSpindle_t;
extern spSpindle_t spindle;

/*
 * Global Scope Functions
 */

uint8_t g2_get_spindle_mode(void);
uint16_t g2_get_spindle_speed_rpm(void);
float g2_get_spindle_ramp_rpm_increment(void);
float g2_get_spindle_ramp_step_dwell_ms(void);
float g2_get_spindle_prev_speed_rpm(void);


void spindle_init();
void spindle_reset();

stat_t spindle_control_immediate(spControl control);
stat_t spindle_control_sync(spControl control);
stat_t spindle_speed_immediate(float speed);    // S parameter
stat_t spindle_speed_sync(float speed);         // S parameter
bool spindle_is_running(void);
bool spindle_ready_to_resume(); // if the spindle can resume at this time, return true
bool spindle_is_on_or_paused(); // returns if the spindle is on or paused - IOW would it try to resume from feedhold
bool spindle_is_running_or_paused(void);//more explicitly asking what bantam tool spindle state is without implied reference to REVERSE, CCW directions that bantab does not use
bool spindle_is_off(void);//11-14-2022--don't turn off if already off. 
bool spindle_speed_ramp_from_systick(); // used only in systick call from stepper.cpp
 
stat_t spindle_override_control(const float override_factor, const bool enable_flag);  // M51
void spindle_start_override(float override_factor);
void spindle_end_override(void);
 

stat_t sp_get_spep(nvObj_t *nv);
stat_t sp_set_spep(nvObj_t *nv);
stat_t sp_get_spdp(nvObj_t *nv);
stat_t sp_set_spdp(nvObj_t *nv);
stat_t sp_get_spph(nvObj_t *nv);
stat_t sp_set_spph(nvObj_t *nv);

stat_t sp_get_spde(nvObj_t *nv);
stat_t sp_set_spde(nvObj_t *nv);
#ifdef DEPLOY_SP_DLY_PER_RAMP_CFG_PARAM//8-12-2022
stat_t sp_get_sprde(nvObj_t *nv);
stat_t sp_set_sprde(nvObj_t *nv); 
stat_t sp_get_sprinc(nvObj_t *nv);
stat_t sp_set_sprinc(nvObj_t *nv); 
#endif
//stat_t sp_get_spdn(nvObj_t *nv);
//stat_t sp_set_spdn(nvObj_t *nv);

stat_t sp_get_spsn(nvObj_t *nv);
stat_t sp_set_spsn(nvObj_t *nv);
stat_t sp_get_spsm(nvObj_t *nv);
stat_t sp_set_spsm(nvObj_t *nv);

stat_t sp_get_spoe(nvObj_t* nv);
stat_t sp_set_spoe(nvObj_t* nv);
stat_t sp_get_spo(nvObj_t* nv);
stat_t sp_set_spo(nvObj_t* nv);

stat_t sp_get_spc(nvObj_t* nv);
stat_t sp_set_spc(nvObj_t* nv);
stat_t sp_get_sps(nvObj_t* nv);
stat_t sp_set_sps(nvObj_t* nv);

/*--- text_mode support functions ---*/

#ifdef __TEXT_MODE

    void sp_print_spmo(nvObj_t* nv);
    void sp_print_spep(nvObj_t* nv);
    void sp_print_spdp(nvObj_t* nv);
    void sp_print_spph(nvObj_t* nv);
#ifdef DEPLOY_SP_DLY_PER_RAMP_CFG_PARAM
    void sp_print_sprde(nvObj_t* nv);
    void sp_print_sprinc(nvObj_t* nv);
#endif
    void sp_print_spde(nvObj_t* nv);
//    void sp_print_spdn(nvObj_t* nv);
    void sp_print_spsn(nvObj_t* nv);
    void sp_print_spsm(nvObj_t* nv);
    void sp_print_spoe(nvObj_t* nv);
    void sp_print_spo(nvObj_t* nv);
    void sp_print_spc(nvObj_t* nv);
    void sp_print_sps(nvObj_t* nv);

#else

    #define sp_print_spmo tx_print_stub
    #define sp_print_spep tx_print_stub
    #define sp_print_spdp tx_print_stub
    #define sp_print_spph tx_print_stub
#ifdef DEPLOY_SP_DLY_PER_RAMP_CFG_PARAM
    void sp_print_sprde tx_print_stub
    void sp_print_sprinc tx_print_stub
#endif
    #define sp_print_spde tx_print_stub
//    #define sp_print_spdn tx_print_stub
    #define sp_print_spsn tx_print_stub
    #define sp_print_spsm tx_print_stub
    #define sp_print_spoe tx_print_stub
    #define sp_print_spo tx_print_stub
    #define sp_print_spc tx_print_stub
    #define sp_print_sps tx_print_stub

#endif  // __TEXT_MODE
    
#ifdef DEPLOY_SP_DLY_PER_RAMP_CFG_PARAM
float g2_get_spindle_rpm_step_incr(void);
float g2_get_spindle_rpm_step_incr_dly_ms(void);
#endif

#endif  // End of include guard: SPINDLE_H_ONCE

