
/*
 * cycle_probing.c - probing cycle extension to canonical_machine.c
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S Hart, Jr., Sarah Tappon, Tom Cauchois, Robert Giseburt
 * With contributions from Other Machine Company.
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
#include "g2core.h"
#include "g2_config.h"
#include "g2_json_parser.h"
#include "g2_text_parser.h"
#include "g2_canonical_machine.h"
#include "g2_kinematics.h"
#include "g2_encoder.h"
#include "g2_spindle.h"
#include "g2_report.h"
#include "g2_gpio.h"
#include "g2_planner.h"
#include "g2_util.h"  
#include "motion_ctrl.h"
#include "system_bantam.h"
/**** Local stuff ****/

int probe_motion_end_callback_qnbr=0;//sme: 11-11-2022

#define MINIMUM_PROBE_TRAVEL 0.254      // mm of travel below which the probe will err out
#define PROBE_2ND_STAGE_FEEDRATE 300  //sme: 4-20-2021 observed  
#define PROBE_LAST_STAGE_FEEDRATE 100  //sme:4-20-2021 observed 
struct pbProbingSingleton 
{   
    // persistent probing runtime variables
    // probe target
    float target[AXES];
    bool  flags[AXES];

    // controls for probing cycle
    int8_t probe_input;                 // digital input to read
    bool trip_sense;                    // true if contact CLOSURE trips probe  (true for G38.2 and G38.3)
    bool alarm_flag;                    // true if failure triggers alarm       (true for G38.2 and G38.4)
    bool waiting_for_motion_complete;   // true if waiting for a motion to complete
    bool probe_tripped;                 // record if we saw the probe tripped (in case it bounces)
    stat_t (*func)();                   // binding for callback function state machine

    // saved gcode model state
    cmUnitsMode saved_units_mode;       // G20,G21 setting
    cmDistanceMode saved_distance_mode; // G90,G91 global setting
    bool saved_soft_limits;             // turn off soft limits during probing
    // float saved_jerk[AXES];          // saved and restored for each axis
#ifdef DEPLOY_PROBE_FEEDHOLD_TIMEOUT//12-19-2022 probe stalled bug--apply a timeout
    uint32_t feedhold_start_time_ms;
    int feedhold_delta_time_ms;
    bool feedhold_active_flag;
    bool feedhold_timeout_event_flag;
#endif
};
static struct pbProbingSingleton pb;
bool probing_in_progress(void){return (pb.waiting_for_motion_complete==true) ;}
#ifdef DEPLOY_PROBE_FEEDHOLD_TIMEOUT
int probe_feedhold_min_time_ms;
int probe_feedhold_max_time_ms;
#define OBSERVED_FEEDHOLD_COMPLETION_TIME_MS 54
#define PROBE_FEEDHOLD_TIMEOUT_LIMIT_MS 1000//arbitrary
void cycle_probing_feedhold_off(void)
{
    pb.feedhold_active_flag=false;
    pb.feedhold_delta_time_ms=sys_get_delta_time_ms(pb.feedhold_start_time_ms);
    /* Build up stats */
    if(probe_feedhold_min_time_ms>pb.feedhold_delta_time_ms)
    {
       probe_feedhold_min_time_ms=pb.feedhold_delta_time_ms; 
    }
    if(probe_feedhold_max_time_ms<pb.feedhold_delta_time_ms)
    {
       probe_feedhold_max_time_ms=pb.feedhold_delta_time_ms; 
    }    
}
bool probe_check_feedhold_timeout(void)
{    
    if (pb.feedhold_active_flag==true)
    {
        pb.feedhold_delta_time_ms=sys_get_delta_time_ms(pb.feedhold_start_time_ms);
        if (pb.feedhold_delta_time_ms>PROBE_FEEDHOLD_TIMEOUT_LIMIT_MS)
        {
             pb.feedhold_timeout_event_flag=true;
             if (cm->hold_state!= FEEDHOLD_SYNC)
             {
                 __NOP();//sme 01-17-2023
             }
        }
    }
    return pb.feedhold_timeout_event_flag;
}
#endif

/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static stat_t _probing_start();
static stat_t _probing_backoff();
static stat_t _probing_finish();
static stat_t _probing_exception_exit(stat_t status);
static stat_t _probe_move(const float target[], const bool flags[]);
static void _send_probe_report(void);

void _prepare_for_probe();
void _store_probe_position();
//sme: 4-23-2021
bool cm_probe_cycle_active(void)
{
  bool result=false;
  if ((cm->cycle_type == CYCLE_PROBE) || (cm->probe_state[0] == PROBE_WAITING)) 
  {
    result=true;
  }
    return  result;
}
/**** JSON INTERFACE *******************************************************************
 * cm_set_probe() - a command to tell it to store the current point as a probe point
 */

stat_t cm_set_probe(nvObj_t *nv)
{
    if (!fp_ZERO(nv->value_int)) 
    {
        nv->valuetype = TYPE_BOOLEAN;
        nv->value_int = true;

        _prepare_for_probe();
        cm->probe_state[0] = PROBE_SUCCEEDED;
        _store_probe_position();
    }
    return (STAT_OK);
}

/**** HELPERS ***************************************************************************
 * _prepare_for_probe() - rotate the stored probes in preparation for storing a new probe
 * _store_probe_position() - store the position as a finalized probe
 */

void _prepare_for_probe() 
{
    // if the previous probe succeeded, roll probes to the next position
    if (cm->probe_state[0] == PROBE_SUCCEEDED) 
    {
        for (uint8_t n = PROBES_STORED - 1; n > 0; n--) 
        {
            cm->probe_state[n] = cm->probe_state[n - 1];
            for (uint8_t axis = 0; axis < AXES; axis++) 
            {
                cm->probe_results[n][axis] = cm->probe_results[n - 1][axis];
            }
        }
        __NOP();
    }
#if 1//sme: 11-8-2022 
    else
    {
        __NOP();
    }
#endif
}

void _store_probe_position() 
{
    for (uint8_t axis = 0; axis < AXES; axis++) 
    {
        cm->probe_results[0][axis] = cm_get_absolute_position(ACTIVE_MODEL, axis);
    }
}

// helper
static void _motion_end_callback(float* vect, bool* flag)
{//sme: 01-23-2023 observe: args not used.

    pb.waiting_for_motion_complete = false;
}

/*
 * _probing_handler - a gpioDigitalInputHandler to capture pin change events
 *   Will be registered only during homing mode - see gpio.h for more info
 */
bool _probing_handler( bool state)
{ 
  bool result=GPIO_NOT_HANDLED; 
     if (cm->cycle_type == CYCLE_PROBE) 
     {                
        // If the probe tripped, and the pin changes again, don't unset it! So, use |=
        pb.probe_tripped |= (state == pb.trip_sense);        
        en_take_encoder_snapshot();   
#ifdef DEPLOY_PROBE_FEEDHOLD_TIMEOUT//12-19-2022 probe stalled bug--apply a timeout
        pb.feedhold_start_time_ms=sys_get_time_ms();
         pb.feedhold_active_flag=true;
#endif        
        cm_request_feedhold(FEEDHOLD_TYPE_SKIP, FEEDHOLD_EXIT_STOP);
        result=GPIO_HANDLED;  
     }
     return result;
} 


/***********************************************************************************
 **** G38.x Probing Cycle **********************************************************
 ***********************************************************************************/

/***********************************************************************************
 * cm_probing_cycle_start()    - G38.x probing cycle using contact (digital input)
 *
 *  cm_probe_cycle_start() is the entry point for a probe cycle. It checks for
 *  some errors, sets up the cycle, then prevents any new commands from queuing
 *  to the planner so that the planner can move to a stop and report motion stopped.
 *
 *  --- Some further details ---
 *
 *  Start with the G38.x documentation, which is not repeated here.
 *  https://github.com/synthetos/g2/wiki/Gcode-Probes
 *
 *  When the probe input fires the input interrupt takes a snapshot of the internal
 *  encoders, then requests a "high speed" feedhold. We then run forward kinematics
 *  on the encoder snapshot to get the reported position. We also execute a move
 *  from the final position (after the feedhold) back to the point we report.
 *
 *  Additionally, we record the last PROBES_STORED (at least 3) probe points that
 *  succeeded. The current or most recent probe (be it success, failure, or
 *  in-progress) occupies one of those positions, which is the one reported by the
 *  "prb" JSON.
 *
 *  Internally the active/most recent probe is stored in cm->probe_results[0] and
 *  cm->probe_state[0]. Before a new probe is started, if cm->probe_state[0] ==
 *  PROBE_SUCCEEDED, then 0 rolls to 1, and 1 to 2, up to PROBES_STORED-1.
 *  The oldest probe is "lost."
 *
 *  Alarms and exceptions: It is *not* necessarily an error condition for the
 *  probe not to trigger, depending on the G38.x command received. It is an error
 *  for the limit or homing switches to fire, or for some other configuration error.
 *  These are trapped and cause Alarms.
 *
 *  Note: Spindle and coolant are not affected during probing. Some probes require
 *  the spindle to be turned on.
 *
 *  Note: When coding a cycle (like this one) you get to perform one queued
 *  move per entry into the continuation, then you must exit. We put two buffer
 *  items into the queue: We queue a move, then we queue a "command" that simply
 *  sets a flag in the probing object (pb.waiting_for_motion_end) to tell us that
 *  the move has finished. The runtime has a special exception for probing and
 *  homing where if a move is interrupted it clears it out of the queue.
 *
 *  You must also wait until the last move has actually completed before declaring
 *  the cycle to be done. Otherwise there is a nasty race condition in the
 *  _controller_HSM() that may accept the next command before the position of the
 *  final move has been recorded in the Gcode model. That's part of what what the
 *  wait_for_motion_end callback is about.
 */

uint8_t cm_straight_probe(float target[], bool flags[], bool trip_sense, bool alarm_flag)
{
    int feedrate=(int)cm->gm.feed_rate;
  
    if (cm->cycle_type == CYCLE_PROBE) 
    {
        return(cm_alarm(STAT_PROBE_CYCLE_FAILED, "Already probing - cannot start another probe"));
    }

    // error if zero feed rate
    if (fp_ZERO(cm->gm.feed_rate)) 
    {
        return(cm_alarm(STAT_FEEDRATE_NOT_SPECIFIED, "Feedrate is zero"));
    }
    
    if(feedrate<PROBE_2ND_STAGE_FEEDRATE)
    {
     
    }
    // error if no axes specified
    if (!(flags[AXIS_X] | flags[AXIS_Y] | flags[AXIS_Z] | flags[AXIS_A] )) 
    {
        return(cm_alarm(STAT_AXIS_IS_MISSING, "Axis is missing"));
    }

    // initialize the probe input; error if no probe input specified
    if ((pb.probe_input = cm->probe_input) == -1) 
    {
        return(cm_alarm(STAT_NO_PROBE_INPUT_CONFIGURED, "Probe input not configured"));
    }

    // setup
    pb.alarm_flag = alarm_flag;             // set true to enable probe fail alarms (all exceptions alarm regardless)
    pb.trip_sense = trip_sense;             // set to sense of "tripped" contact
    pb.func = _probing_start;               // bind probing start function

    cm_set_model_target(target, flags);     // convert target to canonical form taking all offsets into account
    copy_vector(pb.target, cm->gm.target);   // cm_set_model_target() sets target in gm, move it to pb
    copy_vector(pb.flags, flags);           // set axes involved in the move

     _prepare_for_probe();

    // clear the old probe results
    clear_vector(cm->probe_results[0]);      // NOTE: relying on cm->probe_results will not detect a probe to 0,0,0.

    // queue a function to let us know when we can start probing
    cm->probe_state[0] = PROBE_WAITING;      // wait until planner queue empties before starting movement
    pb.waiting_for_motion_complete = true;  
    pb.probe_tripped = false;
    mp_queue_command(_motion_end_callback, nullptr, nullptr);  // note: these args are ignored
#if 0//sme:2-9-2023 NOT HERE 11-11-2022 added for debugging
    probe_motion_end_callback_qnbr=mp->q.write_buf_nbr;
#endif    
 
#ifdef DEPLOY_PROBE_FEEDHOLD_TIMEOUT
#define LARGE_INIT_VAL_MS 1000
    probe_feedhold_min_time_ms=LARGE_INIT_VAL_MS;
    probe_feedhold_max_time_ms=0;
    pb.feedhold_active_flag=false;
    pb.feedhold_timeout_event_flag=false;
#endif
    return (STAT_OK);
}

/***********************************************************************************
 *  cm_probing_cycle_callback() - handle probing progress
 *
 *  This is called regularly from the controller. If we report NOOP, the controller
 *  will continue with other tasks. Otherwise the controller will not execute any
 *  later tasks, including read any more "data".
 */

uint8_t cm_probing_cycle_callback(void)
{
    uint8_t status= STAT_OK;
    
    static bool fault_reported_flag=false;
    if ((cm->cycle_type != CYCLE_PROBE) && (cm->probe_state[0] != PROBE_WAITING)) 
    {     
        return (STAT_NOOP);                 // exit if not in a probing cycle
    }

    if (pb.waiting_for_motion_complete) 
    {   // sync to planner move ends (using callback)
        // check for alarm or shutdown and recover
        // expect the alarm or shutdown to flush the queue, so don't worry about that
        if (cm->machine_state == MACHINE_ALARM || cm->machine_state == MACHINE_SHUTDOWN) 
        {
            cm_abort_probing(cm);
            return (STAT_OK);
        }
       return (STAT_EAGAIN);
    }

    return (pb.func());                     // execute the current probing move
}

/***********************************************************************************
 * cm_abort_probing() - something big happened, the queue is flushing, reset to non-probing state
 *
 *  The task here is to stop sending homing moves to the planner, and ensure we can re-enter
 *  homing fresh without issue.
 *
 *  Note that this should always be called after any feedhold and planner reset.
 *
 */

void cm_abort_probing(cmMachine_t *_cm) 
{
    // The queue has been emptied, the callback is lost, and all of the states we saved are reset
    pb.waiting_for_motion_complete = false;

    // Also clean up the latest probe record
    if (cm->probe_state[0] == PROBE_WAITING) 
    {
        // we can stop waiting
        cm->probe_state[0] = PROBE_FAILED;
        // and, if we abort a probe, we report normally but do NOT alarm
        pb.alarm_flag = false;
    }

    // The cycle_type may have already been changed, but if it hasn't do so now
    if (_cm->cycle_type == CYCLE_PROBE) 
    {
        _probing_finish();
    }

    pb.func = nullptr;
}

/***********************************************************************************
 * _probe_move()          - function to execute probing moves
 * _motion_end_callback() - callback completes when motion has stopped
 *
 *  target[] must be provided in machine canonical coordinates (absolute, mm)
 *  cm_set_absolute_override() also zeros work offsets, which are restored on exit.
 */

static stat_t _probe_move(const float target[], const bool flags[])
{  
    cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_OFFSETS);
    pb.waiting_for_motion_complete = true;          // set this BEFORE the motion starts
    cm_straight_feed(target, flags, PROFILE_FAST);  // NB: feed rate was set earlier, so it's OK
    mp_queue_command(_motion_end_callback, nullptr, nullptr); // the last two arguments are ignored anyway
    return (STAT_EAGAIN);
}

/***********************************************************************************
 * _probing_start() - start the probe or skip it if contact is already active
 */

static uint8_t _probing_start()
{
    volatile static float axis_length;
    // These initializations are required before starting the probing cycle but must
    // be done after the planner has exhausted all current moves as they affect the
    // runtime (specifically the digital input modes). Side effects would include
    // limit switches initiating probe actions instead of just killing movement

    cm->probe_state[0] = PROBE_FAILED;
    cm->machine_state  = MACHINE_CYCLE;
    cm->cycle_type     = CYCLE_PROBE;

    // save relevant non-axis parameters from Gcode model
    pb.saved_distance_mode = (cmDistanceMode)cm_get_distance_mode(ACTIVE_MODEL);
    pb.saved_units_mode = (cmUnitsMode)cm_get_units_mode(ACTIVE_MODEL);
    pb.saved_soft_limits = cm_get_soft_limits();
    cm_set_soft_limits(false);

    // set working values
    cm_set_distance_mode(ABSOLUTE_DISTANCE_MODE);
    cm_set_units_mode(MILLIMETERS);
     
    // Error if the probe target is too close to the current position
    axis_length=get_axis_vector_length(cm->gmx.position, pb.target);
    if (axis_length < MINIMUM_PROBE_TRAVEL) 
    {
        __NOP();
          _probing_exception_exit(STAT_PROBE_TRAVEL_TOO_SMALL);
        __NOP();
        return 0;
    }

    // Get initial probe state, and don't probe if we're already tripped.
    // If the initial input is the same as the trip_sense it's an error.
    if ((pb.trip_sense == motion_probe_switch_active())==true)       
    {     // == is exclusive nor for booleans
         __NOP();
       _probing_exception_exit(STAT_PROBE_IS_ALREADY_TRIPPED); 
       __NOP();
        return 0;
    }
   
    // Everything checks out. Run the probe move
    _probe_move(pb.target, pb.flags);
    pb.func = _probing_backoff;
 
    return (STAT_EAGAIN);
}

/***********************************************************************************
 * _probing_backoff() - runs after the probe move, whether it contacted or not
 *
 * Back off to the measured touch position captured by encoder snapshot
 */

static stat_t _probing_backoff()
{
    // Test if we've contacted. If so, do the backoff. Convert the contact position
    // captured from the encoder in step space to steps to mm. The encoder snapshot
    // was taken by input interrupt at the time of closure.

    if (pb.probe_tripped) 
    {
        cm->probe_state[0] = PROBE_SUCCEEDED;       
        float contact_position[AXES];      
        kn_forward_kinematics(en_get_encoder_snapshot_vector(), contact_position);    
        _probe_move(contact_position, pb.flags);   // NB: feed rate is the same as the probe move
         pb.func = _probing_finish;
    }  
    else 
    {
        cm->probe_state[0] = PROBE_FAILED;
    }
  
    pb.func = _probing_finish;
 
    return (STAT_EAGAIN);
}

/***********************************************************************************
 * _probe_restore_settings() - helper for both exits
 * _probing_exception_exit() - exit for probes that hit an exception
 * _probing_finish()         - exit for successful and non-contacted (failed) probes
 */

static void _probe_restore_settings()
{
    cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_OFF); // release abs override and restore work offsets
    cm_set_distance_mode(pb.saved_distance_mode);
    cm_set_units_mode(pb.saved_units_mode);
    cm_set_soft_limits(pb.saved_soft_limits);
    cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);// cancel feed modes used during probing
    cm_canned_cycle_end();
    sr_request_status_report(SR_REQUEST_IMMEDIATE);         // do this last
}

static stat_t _probing_exception_exit(stat_t status)
{
    _probe_restore_settings();          // cleanup first
    return (cm_alarm(status, "probe error"));
}

static stat_t _probing_finish()
{
    _probe_restore_settings();          // cleanup first
    _store_probe_position();

    // handle failed probes - successful probes already set the flag
    if (cm->probe_state[0] == PROBE_FAILED) 
    {
        if (pb.alarm_flag) 
        {
            cm_alarm(STAT_PROBE_CYCLE_FAILED, "probing failed");
        }
    }
    _send_probe_report();
    return (STAT_OK);
}

/*
 * _probe_report() - report probe results - must update results vector first
 */

static void _send_probe_report() 
{
    static int send_probe_report_entry_count=0;
  
    if (cm->probe_report_enable) 
    {
      send_probe_report_entry_count++;
        // If probe was successful the 'e' word == 1, otherwise e == 0 to signal an error
        char  buf[256];
        char* bufp = buf;
        bufp += sprintf(bufp, "{\"prb\":{\"e\":%i,", (int)cm->probe_state[0]);
        bufp += sprintf(bufp, "\"x\":%0.5f,", cm->probe_results[0][AXIS_X]);
        bufp += sprintf(bufp, "\"y\":%0.5f,", cm->probe_results[0][AXIS_Y]);
        bufp += sprintf(bufp, "\"z\":%0.5f" ,cm->probe_results[0][AXIS_Z]);
#ifdef DEPLOY_FOURTH_AXIS        
        bufp += sprintf(bufp, ",\"a\":%0.5f", cm->probe_results[0][AXIS_A]);
#endif 
        bufp += sprintf(bufp, "}}\n");  
        volatile int len=strlen(bufp);
        comms_mgr_write_msg(buf);  
    }
}

/*
 * cm_get_prbr() - get probe report enable setting
 * cm_set_prbr() - set probe report enable setting
 */

stat_t cm_get_prbr(nvObj_t *nv)
{
    nv->value_int = cm->probe_report_enable;
    nv->valuetype = TYPE_INTEGER;               // ++++ should probably be type boolean
    return (STAT_OK);
}

stat_t cm_set_prbr(nvObj_t *nv)
{
    cm->probe_report_enable = nv->value_int;
    return (STAT_OK);
}

