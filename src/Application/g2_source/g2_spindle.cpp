
/*
 * spindle.cpp - canonical machine spindle driver
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
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
#include "main.h"  
#include "bantam_hal.h" 
#include "g2core.h"             // #1 dependency order
#include "g2_config.h"             // #2
#include "g2_canonical_machine.h"  // #3
#include "g2_text_parser.h"        // #4
#include "g2_spindle.h"
#include "g2_planner.h"
#include "g2_pwm.h"
#include "g2_util.h"
#include "g2_report.h"   

/**** Allocate structures ****/
spSpindle_t spindle;

/**** Static functions ****/
static float _get_spindle_pwm (spSpindle_t &_spindle, pwmControl_t &_pwm);
bool spindle_is_off(void)
{
    bool result = ((spindle.state==SPINDLE_OFF)
                  &&((int)spindle.speed==0)
                  &&((int)spindle.speed_actual==0));
    return( result);
}
bool spindle_is_running(void){return(spindle.state==SPINDLE_RUNNING);}
bool spindle_is_running_or_paused(void){return((spindle.state==SPINDLE_RUNNING)||(spindle.state==SPINDLE_PAUSED));}
//#define SPINDLE_DIRECTION_ASSERT \
//    if ((spindle.direction < SPINDLE_CW) || (spindle.direction > SPINDLE_CCW)) { \
 ///        spindle.direction = SPINDLE_CW; \
 //   }

#ifndef SPINDLE_SPEED_CHANGE_PER_MS
#define SPINDLE_SPEED_CHANGE_PER_MS 0
#endif

/****************************************************************************************
 * spindle_init()
 * spindle_reset() - stop spindle, set speed to zero, and reset values
 */
void spindle_init()
{       
    if( pwm.c[PWM_SPINDLE].frequency < 0 )
    {
        pwm.c[PWM_SPINDLE].frequency = 0;
    }
    pwm_set_freq(PWM_SPINDLE, pwm.c[PWM_SPINDLE].frequency);
    pwm_set_duty(PWM_SPINDLE, pwm.c[PWM_SPINDLE].phase_off);
}

void spindle_reset()
{
    spindle_speed_immediate(0);
    spindle_control_immediate(SPINDLE_OFF);
}

void _actually_set_spindle_speed() 
{
    volatile int entry_counter=0;
    float speed_lo, speed_hi;
 
    if (spindle.state == SPINDLE_RUNNING)
    {
        speed_lo = pwm.c[PWM_SPINDLE].speed_lo;
        speed_hi = pwm.c[PWM_SPINDLE].speed_hi;

        // clamp spindle speed to lo/hi range
        // Ensure that S0 is left alone
        if ((spindle.speed < speed_lo) && !fp_ZERO(spindle.speed)) 
        {
            spindle.speed = speed_lo;
        }

        // allow spindle.speed_actual to start at 0 to match physical spinup
        if (spindle.speed > speed_hi) 
        {
            spindle.speed = speed_hi;
        }

        if (spindle.speed_actual > speed_hi) 
        {
            spindle.speed_actual = speed_hi;
        }
    }  
    else
    {
        // off/disabled/paused
        spindle.speed_actual = 0;
       // Note: spindle_ctrl handles setting actual speed instead of here
        return;

    }
    
   //Apply dwell:
   if ((spindle.state!= SPINDLE_OFF )/*||(fp_NE(spindle.speed,0.0))*/) //debug try 3-3-2021
   {
      if (fp_NE(spindle.speed_actual, spindle.speed)) 
      {
          mp_request_out_of_band_dwell(spindle.spinup_delay);
      }
      else
      {
        __NOP();//allows breakpoint for debugging
      }
   }
   else
   {
     __NOP();//allows breakpoint for debugging
   }      
}

/****************************************************************************************
 * _exec_spindle_control()     - actually execute the spindle command
 * spindle_control_immediate() - execute spindle control immediately
 * spindle_control_sync()      - queue a spindle control to the planner buffer
 *
 *  Basic operation: Spindle function is executed by _exec_spindle_control().
 *  Spindle_control_immediate() performs the control as soon as it's received.
 *  Spindle_control_sync() inserts spindle move into the planner, and handles spinups.
 *
 *  Valid inputs to Spindle_control_immediate() and Spindle_control_sync() are:
 *
 *    - SPINDLE_OFF turns off spindle and sets spindle state to SPINDLE_OFF.
 *      This will also re-load enable and direction polarity to the pins if they have changed.
*      In spindle_control_sync() a non-zero spin up delay runs a dwell immediately
 *      following the spindle change, but only if the planner had planned the spindle
 *      operation to zero. (I.e. if the spindle controls / S words do not plan to zero
 *      the delay is not run). Spindle_control_immediate() has no spinup delay or
 *      dwell behavior.
 *
 *    - SPINDLE_PAUSE is only applicable to CW and CCW states. It forces the spindle OFF and
 *      sets spindle.state to PAUSE. A PAUSE received when not in CW or CCW state is ignored.
 *
 *    - SPINDLE_RESUME, if in a PAUSE state, reverts to RUNNING
 *      The SPEED is not changed, and if it were changed in the interim the "new" speed
 *      is used. If RESUME is received from spindle_control_sync() the usual spinup delay
 *      behavior occurs. If RESUME is received when not in a PAUSED state it is ignored.
 *      This recognizes that the main reason an immediate command would be issued - either
 *      manually by the user or by an alarm or some other program function - is to stop
 *      a spindle. So the Resume should be ignored for safety.
 */

/*
 Actions:
    - OFF       Turn spindle off. Even if it's already off (reloads polarities)
    - RUNNING
    - PAUSE     Turn off spindle, enter PAUSE state
    - RESUME    Turn spindle on CW or CCW as before
    - NOP       No operation, ignore
    - REVERSE   Reverse spindle direction (Q: need a cycle to spin down then back up again?)
 */

static void _exec_spindle_control(float *control, bool* )
{   
    spControl action = (spControl)control[0];
    if (action > SPINDLE_ACTION_MAX) 
    {
        return;
    }
    spControl state = spindle.state;
  
    switch (action)
    {
        case SPINDLE_OFF:          
            spindle.state = SPINDLE_OFF;    // the control might have been something other than SPINDLE_OFF  
            break;
         
        case SPINDLE_RUNNING:
            hal_write_pin(0,ESC_EN_PIN,(GPIO_PinState) 1);
            //spindle.direction = control;
            spindle.state = action;
            break;
         
        case SPINDLE_PAUSE : 
            spindle.state = SPINDLE_PAUSE;    
            break;  // enable bit is already set up to stop the move
            
        case SPINDLE_RESUME:            
            spindle.state = SPINDLE_RUNNING; 
            hal_write_pin(0,ESC_EN_PIN,(GPIO_PinState) 1);            
            break;
        
        default:     
           break;
    }        
    _actually_set_spindle_speed();   
}

/*
 * spindle_control_immediate() - execute spindle control immediately
 * spindle_control_sync()      - queue a spindle control to the planner buffer
 */

stat_t spindle_control_immediate(spControl control)
{
    float value[] = { (float)control };
    _exec_spindle_control(value, nullptr);
    return(STAT_OK);
}

stat_t spindle_control_sync(spControl control)  // uses spControl arg: OFF, CW, CCW
{
    // skip the PAUSE operation if pause-enable is not enabled (pause-on-hold)
    if ((control == SPINDLE_PAUSE) && (!spindle.pause_enable)) 
    {
        return (STAT_OK);
    }

    // ignore pause and resume if the spindle isn't even on
    if ((spindle.state == SPINDLE_OFF) && (control == SPINDLE_PAUSE || control == SPINDLE_RESUME)) 
    {
        return (STAT_OK);
    }
    if(spindle.state != SPINDLE_OFF)  //sme 10-26-2022 program has ended, spindle is already off and door is open, dont do a feed hold 
    if (spindle.speed > 0.0 && !spindle_ready_to_resume())
    {
        // request a feedhold immediately
        cm_request_feedhold(FEEDHOLD_TYPE_ACTIONS, FEEDHOLD_EXIT_CYCLE);
    }

    // queue the spindle control
    float value[] = { (float)control };
    mp_queue_command(_exec_spindle_control,value, nullptr);
    return(STAT_OK);
}

/****************************************************************************************
 * _exec_spindle_speed()     - actually execute the spindle speed command
 * spindle_speed_immediate() - execute spindle speed change immediately
 * spindle_speed_sync()      - queue a spindle speed change to the planner buffer
 *
 *  Setting S0 is considered as turning spindle off. Setting S to non-zero from S0
 *  will enable a spinup delay if spinups are npn-zero.
 */

static void _exec_spindle_speed(float *value, bool *flag)
{   
    spindle.speed = *value;
    _actually_set_spindle_speed();
}

static stat_t _spindle_range_check(float *speed) 
{
    if(fp_NOT_ZERO(*speed)==true)//sme: 10-26-2022 do not reject zero speed. let it through  
    if (*speed < spindle.speed_min) 
    { 
      *speed = spindle.speed_min;
    }
    if (*speed > spindle.speed_max) 
    { 
      *speed = spindle.speed_max;
    }
    return (STAT_OK);
}

stat_t spindle_speed_immediate(float speed)
{
    ritorno(_spindle_range_check(&speed));
    float value[] =
    { 
      speed 
    };

    _exec_spindle_speed(value, nullptr);
    return (STAT_OK);
}

stat_t spindle_speed_sync(float speed)
{
    ritorno(_spindle_range_check(&speed)); 
    float value[] = { speed };

    mp_queue_command(_exec_spindle_speed, value, nullptr);
    return (STAT_OK);
}

bool spindle_ready_to_resume() 
{
    if ((cm1.estop_state != 0) || (cm1.safety_state != 0)) 
    {
        return false;
    }
    return true;
}

bool spindle_is_on_or_paused() 
{
    if (spindle.state != SPINDLE_OFF) 
    {
        return true;
    }
    return false;
}

/****************************************************************************************
 * spindle_override_control()
 * spindle_start_override()
 * spindle_end_override()
 */
stat_t spindle_override_control(const float spo_factor, const bool spo_enable_flag) // M51
{
    bool new_enable = true;
    bool new_override = false;
    if (spo_enable_flag==true)
    {                            
        if (fp_ZERO(spo_factor)) 
        {
            new_enable = false;             // P0 disables override
        } 
        else 
        {
            if (spo_factor < SPINDLE_OVERRIDE_MIN) 
            {
                return (STAT_INPUT_LESS_THAN_MIN_VALUE);
            }
            if (spo_factor > SPINDLE_OVERRIDE_MAX) 
            {
                return (STAT_INPUT_EXCEEDS_MAX_VALUE);
            }
            spindle.override_factor = spo_factor;    // spo_factor is valid, store it.
            new_override = true;
        }
    }

    spindle.override_enable = new_enable;        // always update the enable state
    return (STAT_OK);
}

void spindle_start_override( float override_factor)
{
    return;
}

void spindle_end_override(void)
{
    return;
}

/****************************
 * END OF SPINDLE FUNCTIONS *
 ****************************/

/****************************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ****************************************************************************************/

/****************************************************************************************
 **** Spindle Settings ******************************************************************
 ****************************************************************************************/
 stat_t sp_get_spep(nvObj_t *nv) { return(get_integer(nv, spindle.enable_polarity)); }
 stat_t sp_set_spep(nvObj_t *nv) 
 {
    stat_t status = set_int_u8(nv, (uint8_t &) spindle.enable_polarity, 0, 1); 
    return (status);
 }

stat_t sp_get_spdp(nvObj_t *nv) { return(get_integer(nv, spindle.dir_polarity)); }
stat_t sp_set_spdp(nvObj_t *nv) 
{
     // Not relevant to V3.There is never a need to change polarity. only one direction, CW    
     return STAT_OK;         
}

stat_t sp_get_spph(nvObj_t *nv) { return(get_integer(nv, spindle.pause_enable)); }
stat_t sp_set_spph(nvObj_t *nv) { return(set_int_u8(nv, (uint8_t &) spindle.pause_enable, 0, 1)); }
stat_t sp_get_spde(nvObj_t *nv) { return(get_float(nv, spindle.spinup_delay)); }
stat_t sp_set_spde(nvObj_t *nv) 
{ 
  return(set_float_range(nv, spindle.spinup_delay, 0, SPINDLE_DWELL_MAX)); 
}
#ifdef DEPLOY_SP_DLY_PER_RAMP_CFG_PARAM//8-12-2022
stat_t sp_get_sprde(nvObj_t *nv) { return(get_float(nv, spindle.ramp_step_spinup_dly_ms)); }
stat_t sp_set_sprde(nvObj_t *nv){return(set_float_range(nv, spindle.ramp_step_spinup_dly_ms, SP_MIN_DLY_PER_RPM_STEP, SP_MAX_DLY_PER_RPM_STEP)); }
stat_t sp_get_sprinc(nvObj_t *nv) { return(get_float(nv, spindle.ramp_step_rpm_incr)); }
stat_t sp_set_sprinc(nvObj_t *nv){return(set_float_range(nv, spindle.ramp_step_rpm_incr, SP_MIN_RPM_STEP_INCR, SP_MIN_RPM_STEP_INCR)); }
#endif
stat_t sp_get_spsn(nvObj_t *nv) { return(get_float(nv, spindle.speed_min)); }
stat_t sp_set_spsn(nvObj_t *nv) { return(set_float_range(nv, spindle.speed_min, SPINDLE_SPEED_MIN, SPINDLE_SPEED_MAX)); }
stat_t sp_get_spsm(nvObj_t *nv) { return(get_float(nv, spindle.speed_max)); }
stat_t sp_set_spsm(nvObj_t *nv) { return(set_float_range(nv, spindle.speed_max, SPINDLE_SPEED_MIN, SPINDLE_SPEED_MAX)); }

stat_t sp_get_spoe(nvObj_t *nv) { return(get_integer(nv, spindle.override_enable)); }

stat_t sp_set_spoe(nvObj_t *nv) 
{ 
    stat_t result = set_int_u8(nv, (uint8_t &)spindle.override_enable, 0, 1); 
    return result;
}
stat_t sp_get_spo(nvObj_t *nv) { return(get_float(nv, spindle.override_factor)); }

stat_t sp_set_spo(nvObj_t *nv) 
{ 
#if 1
    volatile static float fval_before, fval_after=0;
    volatile float nv_fval=nv->value_flt;
    fval_before=spindle.override_factor;
    stat_t result = set_float_range(nv, spindle.override_factor, SPINDLE_OVERRIDE_MIN, SPINDLE_OVERRIDE_MAX);
    fval_after=spindle.override_factor; 
    return result;
#else
    return(set_float_range(nv, spindle.override_factor, SPINDLE_OVERRIDE_MIN, SPINDLE_OVERRIDE_MAX));
#endif
}

// These are provided as a way to view and control spindles without using M commands
stat_t sp_get_spc(nvObj_t *nv) { return(get_integer(nv, spindle.state)); }
stat_t sp_set_spc(nvObj_t *nv) { return(spindle_control_immediate((spControl)nv->value_int)); }
stat_t sp_get_sps(nvObj_t *nv) { return(get_float(nv, spindle.speed)); }
stat_t sp_set_sps(nvObj_t *nv) { return(spindle_speed_immediate(nv->value_flt)); }

/****************************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ****************************************************************************************/

#ifdef __TEXT_MODE

const char fmt_spc[]  = "[spc]  spindle control:%12d [0=OFF,1=CW,2=CCW]\n";
const char fmt_sps[]  = "[sps]  spindle speed:%14.0f rpm\n";
const char fmt_spmo[] = "[spmo] spindle mode%16d [0=disabled,1=plan-to-stop,2=continuous]\n";
const char fmt_spep[] = "[spep] spindle enable polarity%5d [0=active_low,1=active_high]\n";
const char fmt_spdp[] = "[spdp] spindle direction polarity%2d [0=CW_low,1=CW_high]\n";
const char fmt_spph[] = "[spph] spindle pause on hold%7d [0=no,1=pause_on_hold]\n";
const char fmt_spde[] = "[spde] spindle spinup delay%10.1f seconds\n";
const char fmt_spsn[] = "[spsn] spindle speed min%14.2f rpm\n";
const char fmt_spsm[] = "[spsm] spindle speed max%14.2f rpm\n";
const char fmt_spoe[] = "[spoe] spindle speed override ena%2d [0=disable,1=enable]\n";
const char fmt_spo[]  = "[spo]  spindle speed override%10.3f [0.050 < spo < 2.000]\n";

void sp_print_spc(nvObj_t *nv)  { text_print(nv, fmt_spc);}     // TYPE_INT
void sp_print_sps(nvObj_t *nv)  { text_print(nv, fmt_sps);}     // TYPE_FLOAT
void sp_print_spmo(nvObj_t *nv) { text_print(nv, fmt_spmo);}    // TYPE_INT
void sp_print_spep(nvObj_t *nv) { text_print(nv, fmt_spep);}    // TYPE_INT
void sp_print_spdp(nvObj_t *nv) { text_print(nv, fmt_spdp);}    // TYPE_INT
void sp_print_spph(nvObj_t *nv) { text_print(nv, fmt_spph);}    // TYPE_INT

void sp_print_spde(nvObj_t *nv) { text_print(nv, fmt_spde);}    // TYPE_FLOAT
#ifdef DEPLOY_SP_DLY_PER_RAMP_CFG_PARAM
const char fmt_sprde[] = "[sprde] spindle ramp step spinup delay%10.1f millisecs\n";
const char fmt_sprinc[] = "[sprinc] spindle ramp step incr %10.1f rpm\n";
void sp_print_sprde(nvObj_t* nv){ text_print(nv, fmt_sprde);}    // TYPE_FLOAT
void sp_print_sprinc(nvObj_t* nv){ text_print(nv, fmt_sprinc);}    // TYPE_FLOAT
#endif
void sp_print_spsn(nvObj_t *nv) { text_print(nv, fmt_spsn);}    // TYPE_FLOAT
void sp_print_spsm(nvObj_t *nv) { text_print(nv, fmt_spsm);}    // TYPE_FLOAT
void sp_print_spoe(nvObj_t *nv) { text_print(nv, fmt_spoe);}    // TYPE INT
void sp_print_spo(nvObj_t *nv)  { text_print(nv, fmt_spo);}     // TYPE FLOAT

#endif // __TEXT_MODE
 
uint8_t g2_get_spindle_mode(void)
{
  uint8_t result = 0;
 //to _integrate or abandon gm->gm.spindle_mode;
  return spindle.state;
 
}
uint16_t g2_get_spindle_speed_rpm(void)
{
  uint16_t result=0; 
  float speed_value=spindle.speed;
  
  if (spindle.override_enable==true)
  {
      speed_value = spindle.speed*spindle.override_factor;
      if (spindle.speed>=spindle.speed_min)
      { 
        if (speed_value<spindle.speed_min)
        {
          speed_value=spindle.speed_min;
        }
        else if (speed_value>spindle.speed_max)
        {
          speed_value=spindle.speed_max;
         }

      }
  }
  return speed_value; 
}

float g2_get_spindle_ramp_rpm_increment(void)
{
  float result = spindle.ramp_step_rpm_incr;
  return result; 
}

float g2_get_spindle_ramp_step_dwell_ms(void)
{
  float result=spindle.ramp_step_spinup_dly_ms;//SP_DLY_PER_RPM_STEP_MS;
  return result;
} 

#ifdef DEPLOY_SP_DLY_PER_RAMP_CFG_PARAM
float g2_get_spindle_rpm_step_incr(void)
{
    return spindle.ramp_step_rpm_incr;
}

float g2_get_spindle_rpm_step_incr_dly_ms(void)
{
    return spindle.ramp_step_spinup_dly_ms;
}
#endif
