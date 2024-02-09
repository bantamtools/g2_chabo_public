
/*
 * stepper.cpp - stepper motor controls
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2019 Robert Giseburt
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
 *
 *  This module provides the low-level stepper drivers and some related functions.
 *  See stepper.h for a detailed explanation of this module.
 */
 
#include "main.h"  
#include "bantam_hal.h" 
#include "g2core.h"
#include "g2_config.h"
#include "g2_stepper.h"
#include "g2_encoder.h"
#include "g2_planner.h"

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

#include "g2_text_parser.h"
#include "g2_util.h"
#include "g2_controller.h"
#include "g2_kinematics.h"
   
#include "system_bantam.h"
#include "motion_ctrl.h" //dda_interrupt start/stop 
#include "user_types.h"
extern  "C" void st_forward_plan_interrupt_callback(void);
extern  "C" void st_exec_move_interrupt_callback(void);
/**** Allocate structures ****/

stConfig_t st_cfg  ;
stPrepSingleton_t st_pre ;
static stRunSingleton_t st_run  ;

void st_init_motors(void)
{
  st_cfg.mot[MOTOR_1].motor_map=	M1_MOTOR_MAP;
  st_cfg.mot[MOTOR_1].step_angle=	M1_STEP_ANGLE;
  st_cfg.mot[MOTOR_1].travel_rev=	M1_TRAVEL_PER_REV;
  st_cfg.mot[MOTOR_1].microsteps=	M1_MICROSTEPS;
  st_cfg.mot[MOTOR_1].polarity=     M1_POLARITY; 
  
  st_cfg.mot[MOTOR_2].motor_map=	M2_MOTOR_MAP;
  st_cfg.mot[MOTOR_2].step_angle=	M2_STEP_ANGLE;
  st_cfg.mot[MOTOR_2].travel_rev=	M2_TRAVEL_PER_REV;
  st_cfg.mot[MOTOR_2].microsteps=	M2_MICROSTEPS;
  st_cfg.mot[MOTOR_2].polarity=     M2_POLARITY;
    
  st_cfg.mot[MOTOR_3].motor_map=	M3_MOTOR_MAP;
  st_cfg.mot[MOTOR_3].step_angle=	M3_STEP_ANGLE;
  st_cfg.mot[MOTOR_3].travel_rev=	M3_TRAVEL_PER_REV;
  st_cfg.mot[MOTOR_3].microsteps=	M3_MICROSTEPS;
  st_cfg.mot[MOTOR_3].polarity=     M3_POLARITY;
  
  st_cfg.mot[MOTOR_4].motor_map=	M4_MOTOR_MAP;
  st_cfg.mot[MOTOR_4].step_angle=	M4_STEP_ANGLE;
  st_cfg.mot[MOTOR_4].travel_rev=	M4_TRAVEL_PER_REV;
  st_cfg.mot[MOTOR_4].microsteps=	M4_MICROSTEPS;
  st_cfg.mot[MOTOR_4].polarity=     M4_POLARITY;

} 
//distinguish between stepper pulse signal state 1==OFF, 0==ON
static volatile  enum {STEP_ON,STEP_OFF} toggle_cnt=STEP_ON;
//Debug vars
static volatile FpScrU fpscr_u; 
static volatile int dda_step_on_isr_entry_cnt=0; 
static volatile int dda_step_off_isr_entry_cnt=0; 
static volatile uint32_t mot1_step_cnt, mot2_step_cnt,mot3_step_cnt=0;  
static volatile uint32_t mot1_total_step_cnt,mot2_total_step_cnt,mot3_total_step_cnt=0; 
 //   static uint32_t  x_axis_step_count, y_axis_step_count, z_axis_step_count=0;
void stepper_reset_total_step_count(void)
{
  mot1_total_step_cnt=0;
  mot2_total_step_cnt=0;
  mot3_total_step_cnt=0;
}
#ifdef STEP_CORRECTION_PARAM_ADJUSTMENTS //deploy run time-adjustable variables 
float step_correction_threshold= STEP_CORRECTION_THRESHOLD;
float st_step_correction_factor   = STEP_CORRECTION_FACTOR;
float st_step_correction_max      = STEP_CORRECTION_MAX;
float step_correction_holdoff  = STEP_CORRECTION_HOLDOFF;
#endif
/**** Static functions ****/

static void _load_move(void)  ;
/*int*/ void motion_request_exec_move(void);


typedef enum
{
  DWELL_TIMER_IDLE,
  DWELL_TIMER_ACTIVE,
  DWELL_TIMER_COMPLETED,
  DWELL_TIMER_CANCELLED,
  DWELL_TIMER_STATE_COUNT
}DwellTimerStateE;

static DwellTimerStateE st_dwell_timer_state = DWELL_TIMER_IDLE;
static uint32_t st_dwell_start_time_ms = 0; 
static uint32_t st_dwell_time_ms=0;

void st_print_scscalef(nvObj_t *nv)
{
  const char fmt_scscalef[] = "[%s] step correction scale factor: %0.2f range:[%0.2f..%0.2f]\n"; 
  sprintf( cs.out_buf,  fmt_scscalef, nv->token, nv->value,MIN_STEP_CORRECTION_FACTOR, MAX_STEP_CORRECTION_FACTOR);
  comms_mgr_write_msg(cs.out_buf);  
}

stat_t st_set_scscalef(nvObj_t *nv)
{
  if (nv->value<MIN_STEP_CORRECTION_FACTOR)
  {
    nv->value=MIN_STEP_CORRECTION_FACTOR;
  }
  st_step_correction_factor=nv->value;
  if(st_step_correction_max < st_step_correction_factor)
  {
       st_step_correction_max=nv->value;
  }
  return STAT_OK;
}
// SystickEvent (Tick events are 1 ms intervals)for handling dwells 
void st_dwell_timer_start(float _dwell_time_ms)
{
  st_dwell_timer_state=DWELL_TIMER_ACTIVE;
  st_dwell_start_time_ms=sys_get_time_ms(); 
  st_dwell_time_ms=(uint32_t)_dwell_time_ms;
}
void st_dwell_timer_cancel(void)
{
  st_dwell_timer_state=DWELL_TIMER_CANCELLED;
  st_dwell_start_time_ms=0; 
  st_dwell_time_ms=0;
}
#if 1//sme: obsolete, now forced to be only vacuous,debug only
stat_t st_set_md(nvObj_t *nv)	
{
  return (STAT_OK);
}

stat_t st_set_me(nvObj_t *nv)	
{ 
  return (STAT_OK);
}
stat_t st_set_mt(nvObj_t *nv)
{
  st_cfg.motor_power_timeout = min(MOTOR_TIMEOUT_SECONDS_MAX, max((float)nv->value, MOTOR_TIMEOUT_SECONDS_MIN));
  return (STAT_OK);
}
#endif
void st_dwell_timer_poll_callback(void)
{   
     
     if (st_dwell_timer_state==DWELL_TIMER_ACTIVE)
     { 
       /* Make available globally time remaining */
       st_run.dwell_ticks_downcount=sys_time_ms_time_remaining(st_dwell_start_time_ms,st_dwell_time_ms);

      // we're either in a dwell or a spindle speed ramp "dwell"
      // in either case, if a feedhold comes in, we need to bail, and since the dwell *is* the motion
      // move the state machine along from here
        if (cm->hold_state == FEEDHOLD_SYNC) 
        {
#define FEEDHOLD_SYNC_DWELL_TIME_MS 1.0000
          st_dwell_timer_start(FEEDHOLD_SYNC_DWELL_TIME_MS);          
          cm->hold_state = FEEDHOLD_MOTION_STOPPED;
        }

       if(st_run.dwell_ticks_downcount == 0)       
        {
            _load_move();  // load the next move at the current interrupt level
        }
     }
}  



/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
 
/**************************************************************************************
 * Function: st_exec_move_interrupt_callback
 *
 * Description:
 *
 *************************************************************************************/
 void st_exec_move_interrupt_callback(void)
 {
     //turn off this interrupt:
    TC3_CH1_TimerStop();
    if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_EXEC) 
    {
       if (mp_exec_move() != STAT_NOOP) 
       {
           st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; // flip it back
           st_request_load_move();
       }
       else
       {
         __NOP();//sme debug only
       }
     } 
     else
     {
       __NOP();//01-17-2023 sme debug probe stall in feedhold
     }     
 }/* End Function */
 

/*
 * stepper_init() - initialize stepper motor subsystem
 * stepper_reset() - reset stepper motor subsystem
 *
 *  Notes:
 *    - This init requires sys_init() to be run beforehand
 *    - microsteps are setup during config_init()
 *    - motor polarity is setup during config_init()
 */
void stepper_init()
{
    memset(&st_run, 0, sizeof(st_run));            // clear all values, pointers and status
    memset(&st_pre, 0, sizeof(st_pre));            // clear all values, pointers and status
    stepper_init_assertions();
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;
    st_dwell_start_time_ms =0;
    st_dwell_time_ms=0;
    st_dwell_timer_state = DWELL_TIMER_IDLE;
}

/*
 * stepper_reset() - reset stepper internals
 *
 * Used to initialize stepper and also to halt movement
 */

void stepper_reset()
{
   // stop all movement
    motion_ctrl_stop_dda_interrupt();
    st_dwell_start_time_ms =0;
    st_dwell_time_ms=0;
    st_dwell_timer_state = DWELL_TIMER_IDLE;

    st_run.dda_ticks_downcount = 0;               // signal the runtime is not busy
    st_run.dwell_ticks_downcount = 0;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;  // set to EXEC or it won't restart

    for (uint8_t motor=0; motor<MOTORS; motor++) 
    {
        st_pre.mot[motor].prev_direction = STEP_INITIAL_DIRECTION;
        st_pre.mot[motor].direction = STEP_INITIAL_DIRECTION;
        st_run.mot[motor].substep_accumulator = 0;      // will become max negative during per-motor setup;
        st_pre.mot[motor].corrected_steps = 0;          // diagnostic only - no action effect
    }
    mp_set_steps_to_runtime_position();                 // reset encoder to agree with the above
}

/*
 * stepper_init_assertions() - test assertions, return error code if violation exists
 * stepper_test_assertions() - test assertions, return error code if violation exists
 */

void stepper_init_assertions()
{
    st_run.magic_end = MAGICNUM;
    st_run.magic_start = MAGICNUM;
    st_pre.magic_end = MAGICNUM;
    st_pre.magic_start = MAGICNUM;
}

stat_t stepper_test_assertions()
{
    if ((BAD_MAGIC(st_run.magic_start)) || (BAD_MAGIC(st_run.magic_end)) ||
        (BAD_MAGIC(st_pre.magic_start)) || (BAD_MAGIC(st_pre.magic_end))) 
    {
        return(cm_panic(STAT_STEPPER_ASSERTION_FAILURE, "stepper_test_assertions()"));
    }
    return (STAT_OK);
}

/*
 * st_runtime_isbusy() - return TRUE if runtime is busy:
 *
 *  Busy conditions:
 *  - motors are running
 *  - dwell is running
 */

bool st_runtime_isbusy()
{
  // returns false if down count is zero
    return (st_run.dda_ticks_downcount || st_run.dwell_ticks_downcount);    
}

/*
 * st_clc() - clear counters
 */

stat_t st_clc(nvObj_t *nv)    // clear diagnostic counters, reset stepper prep
{
    stepper_reset();
    return(STAT_OK);
}

/******************************
 * Interrupt Service Routines *
 ******************************/
extern "C" void st_dda_interrupt_helper(StepperStepPulseStateE state);
/***** Stepper Interrupt Service Routine ************************************************
 * ISR - DDA timer interrupt routine - service ticks from DDA timer
 *	This interrupt is really 2 interrupts. It fires on timer overflow and also on match.
 *	Match interrupts are used to set step pins, and overflow interrupts clear step pins.
 *  When the timer starts (at 0), it does *not* fire an interrupt, but it will on match,
 *  and then again on overflow.
 *	This way the length of the stepper pulse can be controlled by setting the match value.
 *  Note that this makes the pulse timing the inverted duty cycle.
 *
 *	Note that the motor_N.step.isNull() tests are compile-time tests, not run-time tests.
 *	If motor_N is not defined that if{} clause (i.e. that motor) drops out of the complied code.
 *****************************************************************************************/
void st_dda_interrupt_helper(StepperStepPulseStateE state)
{ 
    // process last DDA tick after end of segment
    // edge: dda interrupt timer remains on all the time, so exit early when not stepping
    if (st_run.dda_ticks_downcount != 0)    
    if(state==STEP_PULSE_STATE_ON)
    {
       /* X-Axis stepper: */
       if ( (st_run.mot[MOTOR_1].substep_accumulator += st_run.mot[MOTOR_1].substep_increment) > 0) 
       { 
           mot1_total_step_cnt++;
           hal_set_pin(0, X_MTRDRV_STEP_PIN );            
           st_run.mot[MOTOR_1].substep_accumulator -= DDA_SUBSTEPS;        
           en.en[MOTOR_1].steps_run += en.en[MOTOR_1].step_sign;//pseudo-encode tracking
       }
       st_run.mot[MOTOR_1].substep_increment += st_run.mot[MOTOR_1].substep_increment_increment;
       
        /* Y-Axes stepper: */
       if ( (st_run.mot[MOTOR_2].substep_accumulator += st_run.mot[MOTOR_2].substep_increment) > 0) 
       {
            mot2_total_step_cnt++;            
            hal_set_pin(0, Y1_MTRDRV_STEP_PIN ); 
#ifdef CHABO_LFP             
            hal_set_pin(0, Y2_MTRDRV_STEP_PIN );  
#endif            
            st_run.mot[MOTOR_2].substep_accumulator -= DDA_SUBSTEPS; 
            en.en[MOTOR_2].steps_run += en.en[MOTOR_2].step_sign;//pseudo-encode tracking
       }
       st_run.mot[MOTOR_2].substep_increment += st_run.mot[MOTOR_2].substep_increment_increment;
       
       /* Z-Axis stepper: */
       if (  (st_run.mot[MOTOR_3].substep_accumulator += st_run.mot[MOTOR_3].substep_increment) > 0) 
       {
            mot3_total_step_cnt++;
            hal_set_pin(0, Z_MTRDRV_STEP_PIN );           
            st_run.mot[MOTOR_3].substep_accumulator -= DDA_SUBSTEPS;
            en.en[MOTOR_3].steps_run += en.en[MOTOR_3].step_sign;//pseudo-encode tracking
       } 
       st_run.mot[MOTOR_3].substep_increment += st_run.mot[MOTOR_3].substep_increment_increment; 
#ifdef DEPLOY_FOURTH_AXIS  
       /* A-Axis stepper uses "Y2" named motor control circuitry: */
       if ( (st_run.mot[MOTOR_4].substep_accumulator += st_run.mot[MOTOR_4].substep_increment) > 0) 
       {
            mot3_total_step_cnt++;
            hal_set_pin(0, Y2_MTRDRV_STEP_PIN );           
            st_run.mot[MOTOR_4].substep_accumulator -= DDA_SUBSTEPS;
            en.en[MOTOR_4].steps_run += en.en[MOTOR_4].step_sign;//pseudo-encode tracking
       } 
       st_run.mot[MOTOR_4].substep_increment += st_run.mot[MOTOR_4].substep_increment_increment;    
#endif       
    }
    else//state==STEP_PULSE_STATE_OFF
    {
       hal_reset_pin( 0, X_MTRDRV_STEP_PIN );     
       hal_reset_pin( 0, Y1_MTRDRV_STEP_PIN );       
       hal_reset_pin( 0, Z_MTRDRV_STEP_PIN );     
#if (MOTORS >= 4)  //4th motor applies to either y2 in chabo LFP or DCNC 4th axis    
       hal_reset_pin( 0, Y2_MTRDRV_STEP_PIN);  
#endif       
      // Process end of segment.
      // One more interrupt will occur to turn off any pulses set in this pass.          
      if (--st_run.dda_ticks_downcount == 0)
      {                 
         /* Close out the end of segment: */  
         motion_ctrl_stop_dda_interrupt();
         /* Load the next move at the current interrupt level*/
         _load_move();                   
      }
    }//End: else state==STEP_PULSE_STATE_OFF)    

} //End dda interrupt helper
/****************************************************************************************
 * Exec sequencing code		- computes and prepares next load segment
 * st_request_exec_move()	- SW interrupt to request to execute a move
 * exec_timer interrupt		- interrupt handler for calling exec function
 ***************************************************************************************/
void st_request_exec_move()//sme: identical to V3
{   
    if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_EXEC) 
    {// bother interrupting
        motion_request_exec_move();          
    }
}

/****************************************************************************************
 * st_request_forward_plan  - performs forward planning on penultimate block
 * fwd_plan interrupt       - interrupt handler for calling forward planning function
 */
void st_request_forward_plan()
{  // bother interrupting
   motion_request_forward_plan();  
}

 /**************************************************************************************
 * Function: st_forward_plan_interrupt_callback
 *
 * Description:
 *
 *************************************************************************************/
 void st_forward_plan_interrupt_callback(void)
 {
     stat_t result;
     //turn off this interrupt:
     TC3_CH2_TimerStop();
    result=mp_forward_plan();
     if ( result!= STAT_NOOP) 
     {   // We now have a move to exec.
         st_request_exec_move();
         return;
     }
     else
     {
       __NOP();//__no_operation();
     }
 }/* End Function */



/****************************************************************************************
 * Loader sequencing code
 * st_request_load_move() - fires a software interrupt (timer) to request to load a move
 * load_move interrupt    - interrupt handler for running the loader
 *
 *  _load_move() can only be called from an ISR at the same or higher level as
 *  the DDA or dwell ISR. A software interrupt has been provided to allow a non-ISR to
 *  request a load (see st_request_load_move())
 */

void st_request_load_move()
{
    if (st_runtime_isbusy()) 
    {   // don't request a load if the runtime is busy
        return;
    }
    if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_LOADER) 
    {       // bother interrupting
       _load_move();
    }
}
static void _set_motor_direction(const uint8_t motor, uint8_t direction)
{
   motion_set_step_direction((MotorAxisIdE)motor,(DirectionE)direction);
}

/****************************************************************************************
 * _load_move() - Dequeue move and load into stepper runtime structure
 *
 *  This routine can only be called be called from an ISR at the same or
 *  higher level as the DDA or dwell ISR. A software interrupt has been
 *  provided to allow a non-ISR to request a load (st_request_load_move())
 *
 *  In aline() code:
 *   - All axes must set steps and compensate for out-of-range pulse phasing.
 *   - If axis has 0 steps the direction setting can be omitted
 *   - If axis has 0 steps the motor power must be set accord to the power mode
 */

static void _load_move()
{
    // Be aware that dda_ticks_downcount must equal zero for the loader to run.
    // So the initial load must also have this set to zero as part of initialization
    if (st_runtime_isbusy()) 
    {
        return;                     // exit if the runtime is busy
    }

    // handle aline loads first (most common case)
    if (st_pre.block_type == BLOCK_TYPE_ALINE) 
    {
        //**** setup the new segment ****        
        // st_run.dda_ticks_downcount is setup right before turning on the interrupt, since we don't turn it off 
        // These sections are somewhat optimized for execution speed. The whole load operation
        // is supposed to take < 5 uSec (Arm M3 core). Be careful if you mess with this.

        // the following if() statement sets the runtime substep increment value or zeroes it
        if ((st_run.mot[MOTOR_1].substep_increment = st_pre.mot[MOTOR_1].substep_increment) != 0) 
        {
            // NB: If motor has 0 steps the following is all skipped. This ensures that state comparisons
            //     always operate on the last segment actually run by this motor, regardless of how many
            //     segments it may have been inactive in between.

            // Prepare the substep increment increment for linear velocity ramping
            st_run.mot[MOTOR_1].substep_increment_increment = st_pre.mot[MOTOR_1].substep_increment_increment;

            // Detect direction change and if so:
            //    Set the direction bit in hardware.
            //    Compensate for direction change by flipping substep accumulator value about its midpoint.
            // invert the accumulator for the direction change
            if (st_pre.mot[MOTOR_1].direction != st_pre.mot[MOTOR_1].prev_direction) 
            {
                st_pre.mot[MOTOR_1].prev_direction = st_pre.mot[MOTOR_1].direction;                       
                st_run.mot[MOTOR_1].substep_accumulator = -(DDA_SUBSTEPS + st_run.mot[MOTOR_1].substep_accumulator);
               _set_motor_direction(MOTOR_1, st_pre.mot[MOTOR_1].direction);               
            } 
            SET_ENCODER_STEP_SIGN(MOTOR_1, st_pre.mot[MOTOR_1].step_sign);
        } 
        else 
        {  // Motor has 0 steps; might need to energize motor for power mode processing
            st_run.mot[MOTOR_1].substep_increment_increment = 0;             
        }
        // accumulate counted steps to the step position and zero out counted steps for the segment currently being loaded
        ACCUMULATE_ENCODER(MOTOR_1);
 
        if ((st_run.mot[MOTOR_2].substep_increment = st_pre.mot[MOTOR_2].substep_increment) != 0) 
        {
            st_run.mot[MOTOR_2].substep_increment_increment = st_pre.mot[MOTOR_2].substep_increment_increment;
            if (st_pre.mot[MOTOR_2].direction != st_pre.mot[MOTOR_2].prev_direction) 
            {
                st_pre.mot[MOTOR_2].prev_direction = st_pre.mot[MOTOR_2].direction;          
                st_run.mot[MOTOR_2].substep_accumulator = -(DDA_SUBSTEPS + st_run.mot[MOTOR_2].substep_accumulator);              
                _set_motor_direction(MOTOR_2, st_pre.mot[MOTOR_2].direction);  
#ifdef CHABO_LFP                  
                _set_motor_direction(MOTOR_4, st_pre.mot[MOTOR_2].direction); 
                st_pre.mot[MOTOR_4].prev_direction = st_pre.mot[MOTOR_2].direction; 
#endif                
            }
            SET_ENCODER_STEP_SIGN(MOTOR_2, st_pre.mot[MOTOR_2].step_sign);
        } 
        else 
        {
            st_run.mot[MOTOR_2].substep_increment_increment = 0;            
        }
        ACCUMULATE_ENCODER(MOTOR_2);
  
        if ((st_run.mot[MOTOR_3].substep_increment = st_pre.mot[MOTOR_3].substep_increment) != 0) 
        {
            st_run.mot[MOTOR_3].substep_increment_increment = st_pre.mot[MOTOR_3].substep_increment_increment;
            if (st_pre.mot[MOTOR_3].direction != st_pre.mot[MOTOR_3].prev_direction) 
            {
                st_pre.mot[MOTOR_3].prev_direction = st_pre.mot[MOTOR_3].direction;
                st_run.mot[MOTOR_3].substep_accumulator = -(DDA_SUBSTEPS + st_run.mot[MOTOR_3].substep_accumulator);               
                _set_motor_direction(MOTOR_3, st_pre.mot[MOTOR_3].direction);          
            }          
            SET_ENCODER_STEP_SIGN(MOTOR_3, st_pre.mot[MOTOR_3].step_sign);
        } 
        else 
        {          
            st_run.mot[MOTOR_3].substep_increment_increment = 0;           
        }
        ACCUMULATE_ENCODER(MOTOR_3);
#ifdef DEPLOY_FOURTH_AXIS //only applies if DCNC
        if ((st_run.mot[MOTOR_4].substep_increment = st_pre.mot[MOTOR_4].substep_increment) != 0) 
        {
            st_run.mot[MOTOR_4].substep_increment_increment = st_pre.mot[MOTOR_4].substep_increment_increment;
            if (st_pre.mot[MOTOR_4].direction != st_pre.mot[MOTOR_4].prev_direction) 
            {
                st_pre.mot[MOTOR_4].prev_direction = st_pre.mot[MOTOR_4].direction;
                st_run.mot[MOTOR_4].substep_accumulator = -(DDA_SUBSTEPS + st_run.mot[MOTOR_4].substep_accumulator); // invert the accumulator for the direction change
               _set_motor_direction(MOTOR_4, st_pre.mot[MOTOR_4].direction);
            }         
            SET_ENCODER_STEP_SIGN(MOTOR_4, st_pre.mot[MOTOR_4].step_sign);
        } 
        else 
        {
            st_run.mot[MOTOR_4].substep_increment_increment = 0;           
        }
        ACCUMULATE_ENCODER(MOTOR_4);
#endif        
        //**** do this last ****
        st_run.dda_ticks_downcount = st_pre.dda_ticks;  
        motion_ctrl_start_dda_interrupt(); 
        //stepper_reset_total_step_count();//debug only
    // handle dwells and commands
    } 
    else if (st_pre.block_type == BLOCK_TYPE_DWELL) 
    {
        st_run.dwell_ticks_downcount = st_pre.dwell_ticks;
        st_dwell_timer_start(st_pre.dwell_ticks);

        // handle synchronous commands
    } 
    else if (st_pre.block_type == BLOCK_TYPE_COMMAND) 
    {
        mp_runtime_command(st_pre.bf);
    } // else null - which is okay in many cases

    // all other cases drop to here (e.g. Null moves after Mcodes skip to here)
    st_pre.block_type = BLOCK_TYPE_NULL;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;    // we are done with the prep buffer - flip the flag back
    st_request_exec_move();                             // exec and prep next move
}

/***********************************************************************************
 * st_prep_line() - Prepare the next move for the loader
 *
 *  This function does the math on the next pulse segment and gets it ready for
 *  the loader. It deals with all the DDA optimizations and timer setups so that
 *  loading can be performed as rapidly as possible. It works in joint space
 *  (motors) and it works in steps, not length units. All args are provided as
 *  floats and converted to their appropriate integer types for the loader.
 *
 * Args:
 *    - travel_steps[] are signed relative motion in steps for each motor. Steps are
 *      floats that typically have fractional values (fractional steps). The sign
 *      indicates direction. Motors that are not in the move should be 0 steps on input.
 *
 *    - following_error[] is a vector of measured errors to the step count. Used for correction.
 *
 *    - segment_time - how many minutes the segment should run. If timing is not
 *      100% accurate this will affect the move velocity, but not the distance traveled.
 *
 * NOTE:  Many of the expressions are sensitive to casting and execution order to avoid long-term
 *        accuracy errors due to floating point round off. One earlier failed attempt was:
 *          dda_ticks_X_substeps = (int32_t)((microseconds/1000000) * f_dda * dda_substeps);
 */

stat_t st_prep_line(const float start_velocity, const float end_velocity, const float travel_steps[], const float following_error[], const float segment_time)
{
  static volatile uint32_t prep_line_entry_count=0;
  static volatile float steps=0;
 
#define CORRECTION_STEP_BUF_COUNT 5
   volatile static float correction_steps_buf[CORRECTION_STEP_BUF_COUNT]={0,0,0,0,0};
   static int correction_steps_buf_ndx=0;
   volatile static uint8_t motor=0;
   prep_line_entry_count++;
   // trap assertion failures and other conditions that would prevent queuing the line
   
   if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_EXEC) 
    {     // never supposed to happen
        return (cm_panic(STAT_INTERNAL_ERROR, "st_prep_line() prep sync error"));
    } 
    else if (isinf(segment_time)) 
    {                           // never supposed to happen
        return (cm_panic(STAT_PREP_LINE_MOVE_TIME_IS_INFINITE, "st_prep_line()"));
    } 
    else if (isnan(segment_time)) 
    {                           // never supposed to happen
        return (cm_panic(STAT_PREP_LINE_MOVE_TIME_IS_NAN, "st_prep_line()"));
    }
    // setup segment parameters
    // - dda_ticks is the integer number of DDA clock ticks needed to play out the segment
    // - ticks_X_substeps is the maximum depth of the DDA accumulator (as a negative number)

    st_pre.dda_ticks = (int32_t)(segment_time * 60 * FREQUENCY_DDA);// NB: converts minutes to seconds

    // setup motor parameters
    // this is explained later
    double t_v0_v1 = (double)st_pre.dda_ticks * (start_velocity + end_velocity);

    float correction_steps;
    for (motor=0; motor<MOTORS; motor++) 
    {          // remind us that this is motors, not axes
        steps = travel_steps[motor];

        // Skip this motor if there are no new steps. Leave all other values intact.
        if (fp_ZERO(steps)) 
        {
            st_pre.mot[motor].substep_increment = 0;        // substep increment also acts as a motor flag
            continue;
        }

        // Setup the direction, compensating for polarity.
        // Set the step_sign which is used by the stepper ISR to accumulate step position

        if (steps >= 0) 
        {                    // positive direction
            st_pre.mot[motor].direction = DIRECTION_CW ^ st_cfg.mot[motor].polarity;
            st_pre.mot[motor].step_sign = 1;
        } 
        else 
        {
            st_pre.mot[motor].direction = DIRECTION_CCW ^ st_cfg.mot[motor].polarity;
            st_pre.mot[motor].step_sign = -1;
        }
         // 'Nudge' correction strategy. Inject a single, scaled correction value then hold off
        // NOTE: This clause can be commented out to test for numerical accuracy and accumulating errors
#ifndef DISABLE_STEP_CORRECTION//sme: if we disable, worse over-correction, that is, over-delivery occurs.
#ifdef STEP_CORRECTION_PARAM_ADJUSTMENTS //sme: run-time adjustment of step correction and related scale factors
        if ((--st_pre.mot[motor].correction_holdoff < 0) &&
            (std::abs(following_error[motor]) > step_correction_threshold)) 
        {
            st_pre.mot[motor].correction_holdoff =(int32_t)step_correction_holdoff;
            correction_steps = following_error[motor] * st_step_correction_factor;
            
            /* sme: track correction steps for diagnostics/debug */
            correction_steps_buf[correction_steps_buf_ndx++]=correction_steps;     
            
            if(correction_steps_buf_ndx>=CORRECTION_STEP_BUF_COUNT)
            {
              correction_steps_buf_ndx=0;
            }
            
            if (correction_steps > 0) 
            {
                correction_steps = /*std::*/min(/*std::*/min(correction_steps, std::abs(steps)), st_step_correction_max);
            } 
            else 
            {
                correction_steps = /*std::*/max(/*std::*/max(correction_steps, -std::abs(steps)), -st_step_correction_max);
            }
            st_pre.mot[motor].corrected_steps += correction_steps;
            steps -= correction_steps;
        }        
#else //original, compile-time adjustments

        if ((--st_pre.mot[motor].correction_holdoff < 0) &&
            (std::abs(following_error[motor]) > STEP_CORRECTION_THRESHOLD)) 
        {

            st_pre.mot[motor].correction_holdoff = STEP_CORRECTION_HOLDOFF;
            correction_steps = following_error[motor] * STEP_CORRECTION_FACTOR;

            if (correction_steps > 0) 
            {
                correction_steps = std::min(std::min(correction_steps, std::abs(steps)), STEP_CORRECTION_MAX);
            } 
            else 
            {
                correction_steps = std::max(std::max(correction_steps, -std::abs(steps)), -STEP_CORRECTION_MAX);
            }
            st_pre.mot[motor].corrected_steps += correction_steps;
            steps -= correction_steps;
        }
#endif 
#endif 
        // Compute substeb increment. The accumulator must be *exactly* the incoming
        // fractional steps times the substep multiplier or positional drift will occur.
        // Rounding is performed to eliminate a negative bias in the uint32 conversion
        // that results in long-term negative drift. (std::abs/round order doesn't matter)

        //  t is ticks duration of the move
        //  T is time duration of the move in minutes
        //  f is dda frequency, ticks/sec
        //  s is steps for the move
        //  n is unknown scale factor
        //       whatever the kinematics end up with to convert mm to steps for this motor and segment
        //  v_0 and v_1 are the start and end velocity (in mm/min)
        //
        //  t = T 60 f
        //  Note: conversion from minutes to seconds cancels out in n
        //  n = (s/(T 60))/(((v_0/60)+(v_1/60))/2) = (2 s)/(T(v_0 + v_1))
        //
        //  Needed is steps/tick
        //  1/m_0 = (n (v_0/60))/f
        //  1/m_1 = (n (v_1/60))/f
        //
        //  Substitute n:
        //  1/m_0 = ((2 s)/(T(v_0 + v_1)) (v_0/60))/f = (s v_0)/(T 30 f (v_0 + v_1)) = (2 s v_0)/(t (v_0 + v_1))
        //  1/m_1 = ((2 s)/(T(v_0 + v_1)) (v_1/60))/f = (s v_1)/(T 30 f (v_0 + v_1)) = (2 s v_1)/(t (v_0 + v_1))
        //  d = (1/m_1-1/m_0)/(t-1) = (2 s (v_1 - v_0))/((t - 1) t (v_0 + v_1))
        //  Some common terms:
        //  a = t (v_0 + v_1)
        //  b = 2 s
        //  c = 1/m_0 = (b v_0)/a
        // option 1:
        //  d = ((b v_1)/a - c)/(t-1)
        // option 2:
        //  d = (b (v_1 - v_0))/((t-1) a)

        double s_double = std::abs(steps * 2.0);

        // 1/m_0 = (2 s v_0)/(t (v_0 + v_1))
        st_pre.mot[motor].substep_increment = (int64_t)  (round(((s_double * start_velocity)/(t_v0_v1)) * (double)DDA_SUBSTEPS));
        // option 1:
        //  d = ((b v_1)/a - c)/(t-1)
        // option 2:
        //  d = (b (v_1 - v_0))/((t-1) a)
        st_pre.mot[motor].substep_increment_increment = (int64_t) (round(((s_double*(end_velocity-start_velocity))/(((double)st_pre.dda_ticks-1.0)*t_v0_v1)) * (double)DDA_SUBSTEPS));
    }
    st_pre.block_type = BLOCK_TYPE_ALINE;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;    // signal that prep buffer is ready

    return (STAT_OK);
}

// same as previous function, except it takes a different start and end velocity per motor
stat_t st_prep_line(const float start_velocities[], const float end_velocities[], const float travel_steps[], const float following_error[], const float segment_time)
{
    // TODO refactor out common parts of the two st_prep_line functions

    // trap assertion failures and other conditions that would prevent queuing the line
    if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_EXEC) 
    {     // never supposed to happen
        return (cm_panic(STAT_INTERNAL_ERROR, "st_prep_line() prep sync error"));
    } 
    else if (isinf(segment_time)) 
    {                           // never supposed to happen
        return (cm_panic(STAT_PREP_LINE_MOVE_TIME_IS_INFINITE, "st_prep_line()"));
    } 
    else if (isnan(segment_time)) 
    {                           // never supposed to happen
        return (cm_panic(STAT_PREP_LINE_MOVE_TIME_IS_NAN, "st_prep_line()"));
    }
    // setup segment parameters
    // - dda_ticks is the integer number of DDA clock ticks needed to play out the segment
    // - ticks_X_substeps is the maximum depth of the DDA accumulator (as a negative number)
    st_pre.dda_ticks = (int32_t)(segment_time * 60 * FREQUENCY_DDA);// NB: converts minutes to seconds

    float correction_steps;
    for (uint8_t motor=0; motor<MOTORS; motor++) 
    {          // remind us that this is motors, not axes
        float steps = travel_steps[motor];

        // setup motor parameters
        double t_v0_v1 = (double)st_pre.dda_ticks * (start_velocities[motor] + end_velocities[motor]);

        // Skip this motor if there are no new steps. Leave all other values intact.
        if (fp_ZERO(steps)) 
        {
            st_pre.mot[motor].substep_increment = 0;        // substep increment also acts as a motor flag
            continue;
        }

        // Setup the direction, compensating for polarity.
        // Set the step_sign which is used by the stepper ISR to accumulate step position

        if (steps >= 0) 
        {                    // positive direction
            st_pre.mot[motor].direction = DIRECTION_CW ^ st_cfg.mot[motor].polarity;
            st_pre.mot[motor].step_sign = 1;
        } 
        else 
        {
            st_pre.mot[motor].direction = DIRECTION_CCW ^ st_cfg.mot[motor].polarity;
            st_pre.mot[motor].step_sign = -1;
        }


        // 'Nudge' correction strategy. Inject a single, scaled correction value then hold off
        // NOTE: This clause can be commented out to test for numerical accuracy and accumulating errors
        if ((--st_pre.mot[motor].correction_holdoff < 0) &&
            (std::abs(following_error[motor]) > STEP_CORRECTION_THRESHOLD)) 
        {

            st_pre.mot[motor].correction_holdoff = STEP_CORRECTION_HOLDOFF;
            correction_steps = following_error[motor] * STEP_CORRECTION_FACTOR;

            if (correction_steps > 0) 
            {
                correction_steps = /*std::*/min(/*std::*/min(correction_steps, std::abs(steps)), STEP_CORRECTION_MAX);
            } 
            else 
            {
                correction_steps = /*std::*/max(/*std::*/max(correction_steps, -std::abs(steps)), -STEP_CORRECTION_MAX);
            }
            st_pre.mot[motor].corrected_steps += correction_steps;
            steps -= correction_steps;
        }

        // All math is explained in the previous function

        double s_double = std::abs(steps * 2.0);
        st_pre.mot[motor].substep_increment = (uint32_t)round(((s_double * start_velocities[motor])/(t_v0_v1)) * (double)DDA_SUBSTEPS);
        st_pre.mot[motor].substep_increment_increment = (int64_t)round(((s_double*(end_velocities[motor]-start_velocities[motor]))/(((double)st_pre.dda_ticks-1.0)*t_v0_v1)) * (double)DDA_SUBSTEPS);
    }
    st_pre.block_type = BLOCK_TYPE_ALINE;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;    // signal that prep buffer is ready
    
    return (STAT_OK);
}
/*
 * st_prep_null() - Keeps the loader happy. Otherwise performs no action
 */

void st_prep_null()
{
    st_pre.block_type = BLOCK_TYPE_NULL;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;    // signal that prep buffer is empty
}

/*
 * st_prep_command() - Stage command to execution
 */

void st_prep_command(void *bf)
{
    st_pre.block_type = BLOCK_TYPE_COMMAND;
    st_pre.bf = (mpBuf_t *)bf;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;    // signal that prep buffer is ready
}

/*
 * st_prep_dwell()      - Add a dwell to the move buffer
 */

void st_prep_dwell(float milliseconds)
{
    st_pre.block_type = BLOCK_TYPE_DWELL;
    // we need dwell_ticks to be at least 1
    st_pre.dwell_ticks = /*std::*/max((uint32_t)((milliseconds/1000.0) * FREQUENCY_DWELL), (uint32_t)1UL);
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;    // signal that prep buffer is ready
}

/*
 * st_prep_out_of_band_dwell()
 *
 * Add a dwell to the loader without going through the planner buffers.
 * Only usable while exec isn't running, e.g. in feedhold or stopped states.
 * Otherwise it is skipped.
 */

void st_prep_out_of_band_dwell(float milliseconds)
{
    st_prep_dwell(milliseconds);
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;    // signal that prep buffer is ready
    st_request_load_move();
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/* HELPERS
 * _motor() - motor number as an index or -1 if na
  * sme: legacy version was named _get_motor
 */

static int8_t _motor(const index_t index)
{
    char c = cfgArray[index].token[0];
    return (isdigit(c) ? c-0x31 : -1 ); // 0x30 + 1 offsets motor 1 to == 0
}

/*
 * _set_motor_steps_per_unit() - what it says
 * This function will need to be rethought if microstep morphing is implemented
 */

static float _set_motor_steps_per_unit(nvObj_t *nv)
{
    uint8_t m = _motor(nv->index);
    st_cfg.mot[m].units_per_step = (st_cfg.mot[m].travel_rev * st_cfg.mot[m].step_angle) /
                                   (360 * st_cfg.mot[m].microsteps);
    if (fp_NOT_ZERO(st_cfg.mot[m].units_per_step))
    {
       st_cfg.mot[m].steps_per_unit = 1/st_cfg.mot[m].units_per_step;
       kn_config_changed();
    }
    else
    {
      __NOP();//__no_operation();
    }
    return (st_cfg.mot[m].steps_per_unit);
}

/* PER-MOTOR FUNCTIONS
 *
 * st_get_ma() - get motor axis mapping
 * st_set_ma() - set motor axis mapping
 * st_get_sa() - get motor step angle
 * st_set_sa() - set motor step angle
 * st_get_tr() - get travel per motor revolution
 * st_set_tr() - set travel per motor revolution
 * st_get_mi() - get motor microsteps
 * st_set_mi() - set motor microsteps
 */

/*
 * st_get_ma() - get motor axis mapping
 *
 *  Legacy axis numbers are     XYZABC    for axis 0-5
 *  External axis numbers are   XYZABCUVW for axis 0-8
 *  Internal axis numbers are   XYZUVWABC for axis 0-8 (for various code reasons)
 *
 *  This function retrieves an internal axis number and remaps it to an external axis number
 */
stat_t st_get_ma(nvObj_t *nv)
{
    uint8_t remap_axis[6] = { 0,1,2,3,4,5 };
    ritorno(get_integer(nv, st_cfg.mot[_motor(nv->index)].motor_map));
    nv->value_int = remap_axis[nv->value_int];
    return(STAT_OK);
}

/*
 * st_set_ma() - set motor axis mapping
 *
 *  Legacy axis numbers are     XYZABC    for axis 0-5
 *  External axis numbers are   XYZABCUVW for axis 0-8
 *  Internal axis numbers are   XYZUVWABC for axis 0-8 (for various code reasons)
 *
 *  This function accepts an external axis number and remaps it to an external axis number,
 *  writes the internal axis number and returns the external number in the JSON response.
 */
stat_t st_set_ma(nvObj_t *nv)
{
    if (nv->value_int < 0) 
    {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_int > AXES) 
    {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    uint8_t external_axis = nv->value_int;   
    uint8_t remap_axis[6] = { 0,1,2,3,4,5 };   
    nv->value_int = remap_axis[nv->value_int];
    ritorno(set_int_u8(nv, st_cfg.mot[_motor(nv->index)].motor_map, 0, AXES));
    nv->value_int = external_axis;
    return(STAT_OK);
}

// step angle
stat_t st_get_sa(nvObj_t *nv) 
{ 
  return(get_float(nv, st_cfg.mot[_motor(nv->index)].step_angle)); 
}
stat_t st_set_sa(nvObj_t *nv)
{
    ritorno(set_float_range(nv, st_cfg.mot[_motor(nv->index)].step_angle, 0.001, 360));
    _set_motor_steps_per_unit(nv);
    return(STAT_OK);
}

// travel per revolution
stat_t st_get_tr(nvObj_t *nv) 
{ 
  return(get_float(nv, st_cfg.mot[_motor(nv->index)].travel_rev)); 
}
stat_t st_set_tr(nvObj_t *nv)
{
    ritorno(set_float_range(nv, st_cfg.mot[_motor(nv->index)].travel_rev, 0.0001, 1000000));
    _set_motor_steps_per_unit(nv);
    return(STAT_OK);
}

// microsteps
stat_t st_get_mi(nvObj_t *nv) 
{ 
  return(get_integer(nv, st_cfg.mot[_motor(nv->index)].microsteps)); 
}
/***************************************************************** 
 * Function: st_set_mi at point of this call, a range-checked, qualified  
 *     ustep setting has already been sent to trinamics.
 *      what remains is to apply the new ustep value to steps per unit parameter.
************************************************************************************/
stat_t st_set_mi(nvObj_t *nv)	// motor microsteps
{
  static Boolean_t debug_apply_st_reset_flag = true;
#ifndef REMOVE_MICROSTEP_SET_DEFECT// 10-18-2021
    set_ui16(nv); //this corrupts memory, and it makes sense because there is not target data for microsteps in the config array
#endif    
   _set_motor_steps_per_unit(nv); 
   if (debug_apply_st_reset_flag==true)
   {
       stepper_reset();//sme: 12-6-2019
   }

  return (STAT_OK);
}


// motor steps per unit (direct)
stat_t st_get_su(nvObj_t *nv)
{
    return(get_float(nv, st_cfg.mot[_motor(nv->index)].steps_per_unit));
}

stat_t st_set_su(nvObj_t *nv)
{
    // Don't set a zero or negative value - just calculate based on sa, tr, and mi
    // This way, if STEPS_PER_UNIT is set to 0 it is unused and we get the computed value
    if(nv->value_flt <= 0) 
    {
        nv->value_flt = _set_motor_steps_per_unit(nv);
        return(STAT_OK);
    }

    // Do unit conversion here because it's a reciprocal value (rather than process_incoming_float())
    if (cm_get_units_mode(MODEL) == INCHES) 
    {
        if (cm_get_axis_type(nv) == AXIS_TYPE_LINEAR) 
        {
            nv->value_flt *= INCHES_PER_MM;
        }
    }
    uint8_t m = _motor(nv->index);
    st_cfg.mot[m].steps_per_unit = nv->value_flt;
    st_cfg.mot[m].units_per_step = 1.0/st_cfg.mot[m].steps_per_unit;

    // Scale TR so all the other values make sense
    // You could scale any one of the other values, but TR makes the most sense
    st_cfg.mot[m].travel_rev = (360.0 * st_cfg.mot[m].microsteps) /
                               (st_cfg.mot[m].steps_per_unit * st_cfg.mot[m].step_angle);
    return(STAT_OK);
}

// polarity
stat_t st_get_po(nvObj_t *nv) 
{ 
  return(get_integer(nv, st_cfg.mot[_motor(nv->index)].polarity)); 
}
stat_t st_set_po(nvObj_t *nv) 
{ 
  return(set_int_u8(nv, st_cfg.mot[_motor(nv->index)].polarity, 0, 1)); 
}

stat_t st_get_dw(nvObj_t *nv)
{
    nv->value_int = st_run.dwell_ticks_downcount;
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/
#ifdef __TEXT_MODE

#ifndef SINGLE_TRANSLATION_BUILD
static const char msg_units0[] = " in";    // used by generic print functions
static const char msg_units1[] = " mm";
static const char msg_units2[] = " deg";
static const char *const msg_units[] = { msg_units0, msg_units1, msg_units2 };
#define DEGREE_INDEX 2
#endif

static const char fmt_0ma[] = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
static const char fmt_0sa[] = "[%s%s] m%s step angle%20.3f%s\n";
static const char fmt_0tr[] = "[%s%s] m%s travel per revolution%10.4f%s\n";
static const char fmt_0mi[] = "[%s%s] m%s microsteps%16d [1,2,4,8,16,32]\n";
static const char fmt_0su[] = "[%s%s] m%s steps per unit %17.5f steps per%s\n";
static const char fmt_0po[] = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
 
static void _print_motor_int(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, (int)nv->value_int);
   comms_mgr_write_msg(cs.out_buf) ;    
}
 
#if 0//sme not used after eliminating irrelevant motor power related print functs??
static void _print_motor_flt(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->value_flt);
   comms_mgr_write_msg(cs.out_buf) ;       
}
#endif
static void _print_motor_flt_units(nvObj_t *nv, const char *format, uint8_t units)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->value_flt, GET_TEXT_ITEM(msg_units, units));
   comms_mgr_write_msg(cs.out_buf)  ;     
}
static const char fmt_mt[] = "[mt] motor idle timeout%14.2f Sec\n";
void st_print_mt(nvObj_t *nv) { text_print_flt(nv, fmt_mt);}

void st_print_ma(nvObj_t *nv) { _print_motor_int(nv, fmt_0ma);}
void st_print_sa(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0sa, DEGREE_INDEX);}
void st_print_tr(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0tr, cm_get_units_mode(MODEL));}
void st_print_mi(nvObj_t *nv) { _print_motor_int(nv, fmt_0mi);}
void st_print_su(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0su, cm_get_units_mode(MODEL));}
void st_print_po(nvObj_t *nv) { _print_motor_int(nv, fmt_0po);}
#endif // __TEXT_MODE
