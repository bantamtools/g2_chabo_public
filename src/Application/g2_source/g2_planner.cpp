
/*
 * planner.cpp - Cartesian trajectory planning and motion execution
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2019 Rob Giseburt
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
/* --- Planner Notes ----
 *
 *  The planner works below the canonical machine and above the motor mapping and stepper
 *  execution layers. A rudimentary multitasking capability is implemented for long-running
 *  commands such as lines, arcs, and dwells. These functions are coded as non-blocking
 *  continuations - which are simple state machines that are re-entered multiple times
 *  until a particular operation is complete. These functions have 2 parts - the initial call,
 *  which sets up the local context (closure), and callbacks (continuations) that are called
 *  from the main loop (in controller.c). These tasks only support a single instantiation
 *  and are therefore also not re-entrant - as they rely on singletons for closure.
 *
 *  One important concept is isolation of state at the three layers of the data model -
 *  the Gcode model (gm), motion planner model (bf queue & mm), and motion runtime model (mr).
 *  These are designated as "model", "planner" and "runtime" in function names.
 *
 *  The Gcode model is owned by the canonical machine and should only be accessed by cm_xxxx()
 *  functions. Data from the Gcode model is transferred to the motion planner by the mp_xxxx()
 *  functions called by the canonical machine.
 *
 *  The planner should only use data in the planner model. When a move (buffer) is ready for
 *  execution the relevant data from the planner is transferred to the runtime model,
 *  which should also be isolated.
 *
 *  Models at different levels should never use data from other levels as the data may have
 *  changed or be out-of-sync and lead to unpredictable results.
 */
#include "g2core.h"
#include "g2_config.h"
#include "g2_canonical_machine.h"
#include "g2_plan_arc.h"
#include "g2_planner.h"
#include "g2_kinematics.h"
#include "g2_stepper.h"
#include "g2_encoder.h"
#include "g2_report.h"
#include "g2_util.h"
#include "g2_json_parser.h"
#include "main.h" 
#include "system_bantam.h"

// Allocate planner structures
#if 1//sme 11-11-2023
extern int probe_motion_end_callback_qnbr;//sme: 11-11-2022 help locate position in queue
#endif

mpPlanner_t *mp;                            // currently active planner (global variable)
mpPlanner_t mp1;                            // primary planning context
mpPlanner_t mp2;                            // secondary planning context

mpPlannerRuntime_t *mr;                     // context for planner block runtime
mpPlannerRuntime_t mr1;                     // primary planner runtime context
mpPlannerRuntime_t mr2;                     // secondary planner runtime context

mpBuf_t mp1_queue[PLANNER_QUEUE_SIZE];      // storage allocation for primary planner queue buffers
mpBuf_t mp2_queue[SECONDARY_QUEUE_SIZE];    // storage allocation for secondary planner queue buffers

json_commands_t *jc;                        // currently active JSON command buffer
json_commands_t jc1;                        // primary JSON command buffer
json_commands_t jc2;                        // secondary JSON command buffer

// Execution routines (NB: These are called from the LO interrupt)
static stat_t _exec_dwell(mpBuf_t *bf);
static stat_t _exec_command(mpBuf_t *bf);

// DIAGNOSTICS
//static void _planner_time_accounting();
static void _audit_buffers();

/****************************************************************************************
 * JSON planner objects
 */




/****************************************************************************************
 * planner_init() - initialize MP, MR and planner queue buffers
 * planner_reset() - selective reset MP and MR structures
 * planner_assert() - test planner assertions, PANIC if violation exists
 */

// initialize a planner queue
void _init_planner_queue(mpPlanner_t *_mp, mpBuf_t *queue, uint16_t size)
{
    mpBuf_t *pv, *nx;
    uint16_t i, nx_i;
    mpPlannerQueue_t *q = &(_mp->q);
    memset(q, 0, sizeof(mpPlannerQueue_t)); // clear values, pointers and status
    q->magic_start = MAGICNUM;
    q->magic_end = MAGICNUM;

    memset(queue, 0, sizeof(mpBuf_t)*size); // clear all buffers in queue
    q->bf = queue;                          // link the buffer pool first
    q->w = queue;                           // init all buffer pointers
    q->r = queue;
    q->queue_size = size;
    q->buffers_available = size;

    pv = &q->bf[size-1];
    for (i=0; i < size; i++) {
        q->bf[i].buffer_number = i;         // number is for diagnostics only (otherwise not used)
        nx_i = ((i<size-1) ? (i+1) : 0);    // buffer increment & wrap
        nx = &q->bf[nx_i];
        q->bf[i].nx = nx;                   // setup circular list pointers
        q->bf[i].pv = pv;
        pv = &q->bf[i];
    }
    q->bf[size-1].nx = queue;
}

void planner_init(mpPlanner_t *_mp, mpPlannerRuntime_t *_mr, mpBuf_t *queue, uint16_t queue_size)
{
    // init planner master structure
    memset(_mp, 0, sizeof(mpPlanner_t));    // clear all values, pointers and status
    _mp->magic_start = MAGICNUM;            // set boundary condition assertions
    _mp->magic_end = MAGICNUM;
    _mp->mfo_factor = 1.00;

    // init planner queues
    _mp->q.bf = queue;                      // assign puffer pool to queue manager structure
    _init_planner_queue(_mp, queue, queue_size);

    // init runtime structs
    _mp->mr = _mr;
    memset(_mr, 0, sizeof(mpPlannerRuntime_t)); // clear all values, pointers and status
    
    _mr->magic_start = MAGICNUM;            // mr assertions
    _mr->magic_end = MAGICNUM;

    _mr->block[0].nx = &_mr->block[1];      // Handle the two "stub blocks" in the runtime structure
    _mr->block[1].nx = &_mr->block[0];
    _mr->r = &_mr->block[0];
    _mr->p = &_mr->block[1];
}

void planner_reset(mpPlanner_t *_mp)        // reset planner queue, cease MR activity, but leave positions alone
{
    // selectively reset mpPlanner and mpPlannerRuntime w/o actually wiping them
    _mp->reset();
    _mp->mr->reset();
    jc->reset();
    _init_planner_queue(_mp, _mp->q.bf, _mp->q.queue_size); // reset planner buffers
}

stat_t planner_assert(const mpPlanner_t *_mp)
{
   
    if ((BAD_MAGIC(_mp->magic_start))     || (BAD_MAGIC(_mp->magic_end)) ||
        (BAD_MAGIC(_mp->mr->magic_start)) || (BAD_MAGIC(_mp->mr->magic_end))) {
        return (cm_panic(STAT_PLANNER_ASSERTION_FAILURE, "planner_assert()"));
    }
    for (uint16_t i=0; i < _mp->q.queue_size; i++) {
        if ((_mp->q.bf[i].nx == nullptr) || (_mp->q.bf[i].pv == nullptr)) {
            return (cm_panic(STAT_PLANNER_ASSERTION_FAILURE, "planner buffer is corrupted"));
        }
    }
    return (STAT_OK);
}

/****************************************************************************************
 * mp_halt_runtime() - stop runtime movement immediately
 */
void mp_halt_runtime()
{
    stepper_reset();                // stop the steppers and dwells
    planner_reset(mp);              // reset the active planner
}

/****************************************************************************************
 * mp_set_planner_position() - set planner position for a single axis
 * mp_set_runtime_position() - set runtime position for a single axis
 * mp_set_steps_to_runtime_position() - set encoder counts to the runtime position
 *
 *  Since steps are in motor space you have to run the position vector through inverse
 *  kinematics to get the right numbers. This means that in a non-Cartesian robot
 *  changing any position can result in changes to multiple step values. So this operation
 *  is provided as a single function and always uses the new position vector as an input.
 *
 *  Keeping track of position is complicated by the fact that moves exist in several
 *  reference frames. The scheme to keep this straight is:
 *
 *     - mp->position - start and end position for planning
 *     - mr->position - current position of runtime segment
 *     - mr->target   - target position of runtime segment
 *
 *  The runtime keeps a lot more data, such as waypoints, step vectors, etc.
 *  See struct mpMoveRuntimeSingleton for details.
 *
 *  Note that position is set immediately when called and may not be not an accurate
 *  representation of the tool position. The motors are still processing the action
 *  and the real tool position is still close to the starting point.
 */

void mp_set_planner_position(uint8_t axis, const float position) { mp->position[axis] = position; }
void mp_set_runtime_position(uint8_t axis, const float position) { mr->position[axis] = position; }

void mp_set_steps_to_runtime_position()
{
    if (mr == nullptr) {
        return;
    }

    float step_position[MOTORS];
    // There use to be a call to kn_inverse_kinematics here, but we don't do that now, instead we call kn->sync_encoders()
    // after handling the steps, allowing the kinematics to intelligently handle the offset of step and position.

    // Reset everything to match the internal encoders
    for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) 
    {
        mr->encoder_steps[motor] = en_read_encoder(motor);
        step_position[motor] = mr->encoder_steps[motor];
        mr->target_steps[motor] = step_position[motor];
        mr->position_steps[motor] = step_position[motor];
        mr->commanded_steps[motor] = step_position[motor];

        // These must be zero:
        mr->following_error[motor] = 0;
        st_pre.mot[motor].corrected_steps = 0;
    }   
    kn_sync_encoders(mr->encoder_steps, mr->position);
}


/***********************************************************************************
 * mp_set_target_steps() - set target steps and prep the steppers
 *
 *  This function used to be a portion of _exec_aline_segment().
 *  It handles the step-count bucket brigade, and the call to st_prep_line.
 *  Note that one of the goals is that stepper code and code that wants to set
 * the stepper count (i.e. kinematics idle) doesn't know or care about about mr->
 */

// global, for use locally in various functions
float mp_travel_steps[MOTORS];

stat_t mp_set_target_steps(const float target_steps[MOTORS])
{
  volatile static uint32_t following_error_count=0;
  stat_t result=STAT_OK;
    // Bucket-brigade the old target down the chain before getting the new target from kinematics
    //
    // NB: The direct manipulation of steps to compute travel_steps only works for Cartesian kinematics.
    //       Other kinematics may require transforming travel distance as opposed to simply subtracting steps.

    for (uint8_t m=0; m<MOTORS; m++) 
    {
        mr->commanded_steps[m] = mr->position_steps[m];       // previous segment's position, delayed by 1 segment
        mr->position_steps[m] = mr->target_steps[m];          // previous segment's target becomes position
        mr->target_steps[m] = target_steps[m];               // set the new target
        mp_travel_steps[m] = mr->target_steps[m] - mr->position_steps[m];
        mr->encoder_steps[m] = en_read_encoder(m);           // get current encoder position (time aligns to commanded_steps)
        mr->following_error[m] = mr->encoder_steps[m] - mr->commanded_steps[m];
       if  (mr->following_error[m] !=0)
       {
         following_error_count++;
       }
    }
    __NOP();
    result =st_prep_line(mr->segment_velocity, mr->target_velocity, mp_travel_steps, mr->following_error, mr->segment_time);
    __NOP();
    return result;
}
#if 0////sme: 8-8-2022 code review: this is not used.

stat_t mp_set_target_steps(const float target_steps[MOTORS], const float start_velocities[MOTORS], const float end_velocities[MOTORS], const float segment_time)
{
    mr->segment_time = segment_time;

    // Bucket-brigade the old target down the chain before getting the new target from kinematics
    //
    // NB: The direct manipulation of steps to compute travel_steps only works for Cartesian kinematics.
    //       Other kinematics may require transforming travel distance as opposed to simply subtracting steps.

    for (uint8_t m=0; m<MOTORS; m++) 
    {
        mr->commanded_steps[m] = mr->position_steps[m];       // previous segment's position, delayed by 1 segment
        mr->position_steps[m] = mr->target_steps[m];          // previous segment's target becomes position
        mr->target_steps[m] = target_steps[m];               // set the new target
        mp_travel_steps[m] = mr->target_steps[m] - mr->position_steps[m];
        mr->encoder_steps[m] = en_read_encoder(m);           // get current encoder position (time aligns to commanded_steps)
        mr->following_error[m] = mr->encoder_steps[m] - mr->commanded_steps[m];
    }

    return st_prep_line(start_velocities, end_velocities, mp_travel_steps, mr->following_error, mr->segment_time);
}
#endif

/****************************************************************************************
 * mp_queue_command() - queue a synchronous Mcode, program control, or other command
 * _exec_command()    - callback to execute command
 *
 *  How this works:
 *    - The command is called by the Gcode interpreter (cm_<command>, e.g. an M code)
 *    - cm_ function calls mp_queue_command which puts it in the planning queue (bf buffer).
 *      This involves setting some parameters and registering a callback to the
 *      execution function in the canonical machine.
 *    - the planning queue gets to the function and calls _exec_command()
 *    - ...which puts a pointer to the bf buffer in the prep struct (st_pre)
 *    - When the runtime gets to the end of the current activity (sending steps, counting a dwell)
 *      if executes mp_runtime_command...
 *    - ...which uses the callback function in the bf and the saved parameters in the vectors
 *    - To finish up mp_runtime_command() needs to free the bf buffer
 *
 *  Doing it this way instead of synchronizing on an empty queue simplifies the
 *  handling of feedholds, feed overrides, buffer flushes, and thread blocking,
 *  and makes keeping the queue full much easier - therefore avoiding Q starvation
 */

void mp_queue_command(cm_exec_t cm_exec, float *value, bool *flag)
{
    volatile mpBuf_t *bf;
     volatile float value_copy[4] = {value[0],value[1],value[2],value[3]};
    // Never supposed to fail as buffer availability was checked upstream in the controller
    if ((bf = mp_get_write_buffer()) == NULL) 
    {
        cm_panic(STAT_FAILED_GET_PLANNER_BUFFER, "mp_queue_command()");
        return;
    }
    
    mp->write_buf_nbr=bf->buffer_number;//sme: debug
    mp->q.write_buf_nbr=bf->buffer_number;//sme: debug
    probe_motion_end_callback_qnbr=mp->q.write_buf_nbr;//sme: debug
    bf->block_type = BLOCK_TYPE_COMMAND;
    memcpy((void*)&bf->gm, &cm->gm, sizeof(GCodeState_t)); // snapshot the active gcode state     
    bf->bf_func = _exec_command;      // callback to planner queue exec function
    bf->cm_func = cm_exec;            // callback to canonical machine exec function

    for (uint8_t axis = AXIS_X; axis < AXES; axis++) 
    {
        bf->unit[axis] = value_copy[axis];//value[axis];               // use the unit vector to store command values
        bf->axis_flags[axis] = flag[axis];
    }
   
    mp_commit_write_buffer(BLOCK_TYPE_COMMAND);     // must be final operation before exit
}

static stat_t _exec_command(mpBuf_t *bf)
{
    st_prep_command(bf);
    return (STAT_OK);
}

stat_t mp_runtime_command(mpBuf_t *bf)
{
    bf->cm_func(bf->unit, bf->axis_flags);          // 2 vectors used by callbacks
    if (mp_free_run_buffer()==true) //queue is empty when true
    {
        cm_cycle_end();                             // free buffer & perform cycle_end if planner is empty
    }
    return (STAT_OK);
}

/****************************************************************************************
 * _exec_json_command() - execute json string (from exec system)
 * mp_json_command()    - queue a json command
 * mp_json_command_immediate() - execute a json command with response suppressed
 */

static void _exec_json_command(float *value, bool *flag)
{
    char *json_string = jc->read_buffer();
    json_parse_for_exec(json_string, true);         // process it
    jc->free_buffer();
}

stat_t mp_json_command(char *json_string)
{
    // Never supposed to fail, since we stopped parsing when we were full
    jc->write_buffer(json_string);
    mp_queue_command(_exec_json_command, nullptr, nullptr);
    return (STAT_OK);
}

stat_t mp_json_command_immediate(char *json_string)
{
    return json_parser(json_string);
}

/****************************************************************************************
 * _exec_json_wait() - execute json wait string
 * mp_json_wait()    - queue a json wait command
 */

static stat_t _exec_json_wait(mpBuf_t *bf)
{
    char *json_string = jc->read_buffer();

    // process it
    json_parse_for_exec(json_string, false); // do NOT execute

    nvObj_t *nv = nv_exec;
    while ((nv != NULL) && (nv->valuetype != TYPE_EMPTY)) 
    {
        // For now we ignore non-BOOL
        if (nv->valuetype == TYPE_BOOLEAN) 
        {
            bool old_value = (bool)nv->value_int;         // force it to bool

            nv_get_nvObj(nv);
            bool new_value = (bool)nv->value_int;
            if (old_value != new_value) 
            {
                st_prep_dwell(1.0); // 1ms exactly
                return STAT_OK;
            }
        }
        nv = nv->nx;
    }
    jc->free_buffer();

    if (mp_free_run_buffer()==true) //queue is empty when true)
    {
        cm_cycle_end();     // free buffer & perform cycle_end if planner is empty
    }
    return (STAT_OK);
}

stat_t mp_json_wait(char *json_string)
{
    // Never supposed to fail, since we stopped parsing when we were full
    jc->write_buffer(json_string);

    mpBuf_t *bf;

    // Never supposed to fail as buffer availability was checked upstream in the controller
    if ((bf = mp_get_write_buffer()) == NULL) 
    {
        cm_panic(STAT_FAILED_GET_PLANNER_BUFFER, "mp_json_wait()");
        return STAT_ERROR;
    }
    bf->block_type = BLOCK_TYPE_COMMAND;
    bf->bf_func = _exec_json_wait;      // callback to planner queue exec function
    mp_commit_write_buffer(BLOCK_TYPE_COMMAND);            // must be final operation before exit
    return (STAT_OK);
}


/****************************************************************************************
 * mp_dwell()    - queue a dwell
 * _exec_dwell() - dwell execution
 *
 * Dwells are performed by passing a dwell move to the stepper drivers.
 * When the stepper driver sees a dwell it times the dwell on a separate
 * timer than the stepper pulse timer.
 */

stat_t mp_dwell(float seconds)
{
    mpBuf_t *bf;

    if ((bf = mp_get_write_buffer()) == NULL) 
    {     // get write buffer or fail
        return(cm_panic(STAT_FAILED_GET_PLANNER_BUFFER, "mp_dwell()")); // not ever supposed to fail
    }
    bf->bf_func = _exec_dwell;                      // register callback to dwell start
    bf->block_time = seconds;                       // in seconds, not minutes
    bf->block_state = BLOCK_INITIAL_ACTION;
    mp_commit_write_buffer(BLOCK_TYPE_DWELL);       // must be final operation before exit
    return (STAT_OK);
}

static stat_t _exec_dwell(mpBuf_t *bf)
{
    st_prep_dwell(bf->block_time * 1000.0);// convert seconds to ms
    if (mp_free_run_buffer()==true) //queue is empty when true) 
    {
        cm_cycle_end();                             // free buffer & perform cycle_end if planner is empty
    }
    return (STAT_OK);
}

/****************************************************************************************
 * mp_request_out_of_band_dwell() - request a dwell outside of the planner queue
 *
 *  This command is used to request that a dwell be run outside of the planner.
 *  The dwell will only be queued if the time is non-zero, and will only be executed
 *  if the runtime has been stopped. This function is typically called from an exec
 *  such as _exec_spindle_control(). The dwell move is executed from mp_exec_move().
 *  This is useful for queuing a dwell after a spindle change.
 */

void mp_request_out_of_band_dwell(float seconds)
{
    if (fp_NOT_ZERO(seconds)) 
    {
        mr->out_of_band_dwell_flag = true;
        mr->out_of_band_dwell_seconds = seconds;
    }
}

/****************************************************************************************
 * Planner helpers
 *
 * mp_get_planner_buffers()  - return # of available planner buffers
 * mp_planner_is_full()      - true if planner has no room for a new block
 * mp_has_runnable_buffer()  - true if next buffer is runnable, indicating motion has not stopped.
 * mp_is_it_phat_city_time() - test if there is time for non-essential processes
 */

uint8_t mp_get_planner_buffers(const mpPlanner_t *_mp)  // which planner are you interested in?
{
    return (_mp->q.buffers_available);
}

bool mp_planner_is_full(const mpPlanner_t *_mp)         // which planner are you interested in?
{
  bool result = false;
    // We also need to ensure we have room for another JSON command
  if ((_mp->q.buffers_available < PLANNER_BUFFER_HEADROOM) || (jc->available == 0))
  {
    result = true;
  }
    return result ;
}

bool mp_has_runnable_buffer(const mpPlanner_t *_mp)     // which planner are you interested in?)
{
  bool result = false;
  if (_mp->q.r->buffer_state != MP_BUFFER_EMPTY)
  {
    result = true;
  }
    return result;    // anything other than MP_BUFFER_EMPTY returns true
}

bool mp_is_phat_city_time()
{
  bool result = false;
  if (cm->hold_state == FEEDHOLD_HOLD) 
  {
     result =true;
  }  
  else if ((mp->plannable_time <= 0.0) || (PHAT_CITY_TIME < mp->plannable_time))
  {
    result =true;
  }  
  return result;
}

/****************************************************************************************
 * mp_planner_callback()
 *
 *  mp_planner_callback()'s job is to invoke backward planning intelligently.
 *  The flow of control and division of responsibilities for planning is:
 *
 *  - mp_aline() receives new Gcode moves and initializes the local variables
 *    for the new buffer.
 *
 *  - mp_planner_callback() is called regularly from the main loop.
 *    It's job is to determine whether or not to call mp_plan_block_list(),
 *    which will back-plan as many blocks as are ready for processing.
 *
 *    mp_planner_callback() also manages planner state - whether the planner
 *    is IDLE, in STARTUP or in one of the running states.
 *
 *  - _plan_block() is the backward planning function for a single buffer.
 *
 *  - Just-in-time forward planning is performed by mp_plan_move() in the
 *    plan_exec.cpp runtime executive
 *
 *  Some Items to note:
 *
 *  - At the start of a job the planner should fill up with unplanned blocks before
 *    motion starts. This eliminates an initial move that plans to zero and ensures
 *    the planner gets a "head start" on managing the time in the planner queue.
 *
 *  - It's important to distinguish between the case where the new block is actually
 *    a startup condition and where it's the first block after a stop or a stall.
 *    The planner wants to perform a STARTUP in the first case, but start planning
 *    immediately in the latter cases.
 *
 *  - Feedholds require replanning to occur
 */

stat_t mp_planner_callback()
{
    // Test if the planner has transitioned to an IDLE state
    if (mp_get_planner_buffers(mp) == mp->q.queue_size) 
    {

        // Edge case: If there's no runnable buffer, FEEDHOLD_SYNC will never exit
        if (cm->hold_state == FEEDHOLD_SYNC) 
        {
            cm->hold_state = FEEDHOLD_MOTION_STOPPED;
        }

        // detect and set IDLE state
        if ((cm->motion_state == MOTION_STOP) && (cm->hold_state == FEEDHOLD_OFF)) 
        {
            mp->planner_state = PLANNER_IDLE;
            return (STAT_OK);
        }
    }
    //sme: 2-13-2023 this was prior incorrectly using time_remaining whcih returns int, and 0 when timeout out and the bool is exppecting the opposite 
    bool _timed_out = sys_time_ms_limit_reached(mp->block_timeout,BLOCK_TIMEOUT_MS);
  
    if (!mp->request_planning && !_timed_out) 
    {      // Exit if no request or timeout
        return (STAT_OK);
    }

    // Process a planner request or timeout
    if (mp->planner_state == PLANNER_IDLE) 
    {
        mp->p = mp_get_r();                         // initialize planner pointer to run buffer
        mp->planner_state = PLANNER_STARTUP;
    }
    if (mp->planner_state == PLANNER_STARTUP) 
    {
        if (!mp_planner_is_full(mp) && !_timed_out) 
        {
            return (STAT_OK);                       // remain in STARTUP
        }
        mp->planner_state = PLANNER_PRIMING;
    }
    mp_plan_block_list();
    return (STAT_OK);
}

/*
 *  mp_replan_queue() - reset the blocks in the planner queue and request a planner run
 *
 *  We don't actually need to invalidate back-planning. Only forward planning.
 *
 */

void mp_replan_queue(mpBuf_t *bf, bool back_too/*=false*/)
{
    // if (back_too) {
    //     mp->p = bf; // reset the plan-start pointer for back-planning
    // }
    do {
        if (back_too) {
            if (bf->buffer_state == MP_BUFFER_BACK_PLANNED) {  // mark back-planned moves as (re)plannable
                bf->plannable = true;
            } else {    // If it's not at least back-planned, then we can stop.
                break;  // We don't need to adjust it.
            }
        } else {
            if (bf->buffer_state >= MP_BUFFER_FULLY_PLANNED) {  // revert from FULLY PLANNED state
                bf->buffer_state = MP_BUFFER_BACK_PLANNED;
            } else {    // If it's not fully-planned then it's either backplanned or earlier.
                break;  // We don't need to adjust it.
            }
        }
    } while ((bf = mp_get_next_buffer(bf)) != mp_get_r());

    mp->request_planning = true;
}
void mp_replan_queue(mpBuf_t *bf)
{
    do 
    {
        if (bf->buffer_state >= MP_BUFFER_FULLY_PLANNED) 
        {  // revert from FULLY PLANNED state
            bf->buffer_state = MP_BUFFER_BACK_PLANNED;
        } 
        else 
        {           // If it's not "planned" then it's either backplanned or earlier.
            break;  // We don't need to adjust it.
        }
    } while ((bf = mp_get_next_buffer(bf)) != mp_get_r());

    mp->request_planning = true;
}

/*
 *  mp_start_feed_override() - gradually adjust existing and new buffers to target override percentage
 *  mp_end_feed_override() - gradually adjust existing and new buffers to no override percentage
 *
 *  Variables:
 *    - 'mfo_factor' is the override scaling factor normalized to 1.0 = 100%
 *      Values < 1.0 are speed decreases, > 1.0 are increases. Upper and lower limits are checked.
 *
 *    - 'ramp_time' is approximate, as the ramp dynamically changes move execution times
 *      The ramp will attempt to meet the time specified but it will not be exact.
 */
/*  Function:
 *  The override takes effect as close to real-time as possible. Practically, this means about 20 or
 *  so behind the current running move. How it works:
 *
 *    - If the planner is idle just apply the override factor and be done with it. That's easy.
 *    - Otherwise look for the "break point" at 20 ms
 */

void mp_start_feed_override(/*const*/ float ramp_time, /*const*/ float override_factor)
{
    cm->mfo_state = MFO_REQUESTED;

    if (mp->planner_state == PLANNER_IDLE) 
    {
        mp->mfo_factor = override_factor; // that was easy
        return;
    }

    // Assume that the min and max values for override_factor have been validated upstream
    // SUVAT: V = U+AT ==> A = (V-U)/T
    mp->ramp_target = override_factor;
    mp->ramp_dvdt = (override_factor - mp->c->override_factor) / ramp_time;
    mp->mfo_active = true;

    if (fp_NOT_ZERO(mp->ramp_dvdt)) 
    {    // do these things only if you actually have a ramp to run
        mp->p = mp->c;                    // re-position the planner pointer
        mp->ramp_active = true;
        mp->request_planning = true;
    }
}

void mp_end_feed_override(/*const*/ float ramp_time)
{
    mp_start_feed_override (FEED_OVERRIDE_RAMP_TIME, 1.00);
}

void mp_start_traverse_override(/*const*/ float ramp_time, /*const*/ float override_factor)
{
    return;
}

void mp_end_traverse_override(/*const*/ float ramp_time)
{
    return;
}

/*
 * mp_planner_time_accounting() - gather time in planner
 */
bool _was_phat_city = true; // phat-city means there's time to do non-essentials
void mp_planner_time_accounting() {
    mpBuf_t *bf = mp_get_r();                       // start with run buffer

    // check the run buffer to see if anything is running. Might not be
    if (bf->buffer_state != MP_BUFFER_RUNNING) {    // this is not an error condition
        return;
    }
    mp->plannable_time = 0; //UPDATE_BF_MS(bf);     // DIAGNOSTIC
    while ((bf = bf->nx) != mp_get_r()) {
        if (bf->buffer_state == MP_BUFFER_EMPTY || bf->plannable == true) {
            break;
        }
        mp->plannable_time += bf->block_time;
    }
    UPDATE_MP_DIAGNOSTICS  // DIAGNOSTIC

    bool is_phat_city = mp_is_phat_city_time();

    // note if we switched to non-phat-city
    if (!is_phat_city && _was_phat_city) {
        rpt_exception(STAT_GENERIC_EXCEPTION_REPORT,(char*) "exiting phat city");
    } else if (is_phat_city && !_was_phat_city) {
        rpt_exception(STAT_GENERIC_EXCEPTION_REPORT, (char*)"entering phat city");
    }

    _was_phat_city = is_phat_city;
}

/**** PLANNER BUFFER PRIMITIVES ************************************************************
 *
 *  Planner buffers are used to queue and operate on Gcode blocks. Each buffer contains
 *  one Gcode block which may be a move, an M code, or other command that must be
 *  executed synchronously with movement.
 *
 *  The planner queue (mb) is a circular queue of planner buffers (bf's). Each block has a
 *  pointer to the next block (nx), and one to the previous block (pv).
 *
 *  It's useful to get terms straight or it can get confusing.
 *
 *    - The "run" block is the block that is currently executing (i.e. in mr). Since
 *      it's a circular FIFO queue the running block is considered the "first block".
 *
 *    - The "write" block (aka "new" block) is the block that was just put on the queue.
 *      The new block is at the other end of the queue from the run block.
 *
 *    - Moving "forward" is advancing to the next block (nx), which is in the direction
 *      of the new block. Moving "backwards" backs up to the previous block (pv) in the
 *      direction of the running block. Since the queue is a doubly linked circular list
 *      the ends connect, and blocks "outside" of the running and new blocks may be empty.
 *
 *    - The "planning" block is the block currently pointed to by the planner. This
 *      starts out right next to the running block and advances towards the new block
 *      as planning executes. Planning executes predominantly in the forward direction.
 *
 *  New blocks are populated by (1) getting a write buffer, (2) populating the buffer,
 *  then (3) placing it in the queue (commit write buffer). If an exception occurs
 *  during step (2) you can unget the write buffer before queuing it, which returns
 *  it to the pool of available buffers. (NB: Unget is currently unused but left in.)
 *
 *  The RUN buffer may be retrieved once for simple commands, or multiple times for
 *  long-running commands such as moves that get called multiple times. The first
 *  retrieval (get run buffer) will return the new run buffer. Subsequent retrievals
 *  will return the same buffer until it's state changes to complete. When the command
 *  is complete the run buffer is returned to the pool by freeing it.
 *
 * Notes:
 *  The write buffer pointer only moves forward on mp_commit_write_buffer, and the
 *  run buffer pointer only moves forward on mp_free_run_buffer().
 *  Tests, gets and unget have no effect on the pointers.
 *
 * Functions Provided:
 *   _clear_buffer(bf)        Zero the contents of a buffer
 *
 *   mp_get_prev_buffer(bf)   Return pointer to the previous buffer in the linked list
 *   mp_get_next_buffer(bf)   Return pointer to the next buffer in the linked list
 *
 *   mp_get_write_buffer()    Get pointer to next available write buffer
 *                            Return pointer or NULL if no buffer available.
 *
 *   mp_unget_write_buffer()  Free write buffer if you decide not to commit it.
 *
 *   mp_commit_write_buffer() Commit the write buffer to the queue.
 *                            Advance write pointer & changes buffer state.
 *
 *                            *** WARNING *** The calling routine must NOT use the write
 *                            buffer once it has been committed as it may be processed
 *                            and freed (cleared) before the commit function returns.
 *
 *   mp_get_run_buffer()      Get pointer to the next or current run buffer.
 *                            Return a new run buffer if prev buf was ENDed.
 *                            Return same buf if called again before ENDing.
 *                            Return NULL if no buffer available.
 *                            This behavior supports continuations (iteration).
 *
 *   mp_free_run_buffer()     Release the run buffer & return to buffer pool.
 *                            Return true if queue is empty, false otherwise.
 *                            This is useful for doing queue empty / end move functions.
 *
 * UNUSED BUT PROVIDED FOR REFERENCE:
 *   mp_copy_buffer(bf,bp)    Copy the contents of bp into bf - preserves links.
 */

// Also clears unlocked, so the buffer cannot be used
static inline void _clear_buffer(mpBuf_t *bf)
{
    bf->reset();    // Call a reset method on the buffer object.
}

/*
 * These GET functions are defined here but we use the macros in planner.h instead
mpBuf_t * mp_get_prev_buffer(const mpBuf_t *bf) { return (bf->pv); }
mpBuf_t * mp_get_next_buffer(const mpBuf_t *bf) { return (bf->nx); }
 */

mpBuf_t * mp_get_w() { return (mp->q.w); }
mpBuf_t * mp_get_r() 
{
   volatile static int run_buffer_number2 =0;//sme : debug
   run_buffer_number2= mp->q.r->buffer_number; 
   if (run_buffer_number2!=0)
   if (run_buffer_number2==probe_motion_end_callback_qnbr)
   {
      __NOP();
   }
   return (mp->q.r); 
}

mpBuf_t * mp_get_write_buffer()     // get & clear a buffer
{
    mpPlannerQueue_t *q = &(mp->q);

    if (q->w->buffer_state == MP_BUFFER_EMPTY) 
    {
        _clear_buffer(q->w);        // NB: this is redundant if the buffer was cleared mp_free_run_buffer()
        q->w->buffer_state = MP_BUFFER_INITIALIZING;
        q->buffers_available--;
        return (mp_get_w());
    }
    // The no buffer condition always causes a panic - invoked by the caller
    rpt_exception(STAT_FAILED_TO_GET_PLANNER_BUFFER, (char*)"mp_get_write_buffer()");
    return (NULL);
}

void mp_unget_write_buffer()        // mark buffer as empty and adjust free buffer count
{
    mpPlannerQueue_t *q = &(mp->q);

    if (q->w->buffer_state != MP_BUFFER_EMPTY) 
    {  // safety. Can't unget an empty buffer
        q->w->buffer_state = MP_BUFFER_EMPTY;
        q->buffers_available++;
    }
}

/*** WARNING ***
* The function calling mp_commit_write_buffer() must NOT use the write buffer once it has
* been committed. Interrupts may use the buffer immediately, invalidating its contents.
*/

void mp_commit_write_buffer(const blockType block_type)
{
    mpPlannerQueue_t *q = &(mp->q);

    q->w->block_type = block_type;
    q->w->block_state = BLOCK_INITIAL_ACTION;

    if (block_type != BLOCK_TYPE_ALINE) 
    {
         
        if ((mp->planner_state > PLANNER_STARTUP) && (cm->hold_state == FEEDHOLD_OFF)) 
        {
            // NB: BEWARE! the requested exec may result in the planner buffer being
            // processed IMMEDIATELY and then freed - invalidating the contents
         
            st_request_forward_plan();      // request an exec if the runtime is not busy
        }
        else
        {          
          __NOP();
        }
       
    }
    q->w->plannable = true;                 // enable block for planning
    mp->request_planning = true;
    q->w = q->w->nx;                        // advance write buffer pointer
    
    mp->block_timeout=sys_get_time_ms();
    qr_request_queue_report(+1);            // request QR and add to "added buffers" count
}

// Note: mp_get_run_buffer() is only called by mp_exec_move()
//--------
// sme: 11-14-2022: No, it is also called by mp_forward_plan(), _check_motion_stopped())
//--------
//which is inside an interrupt
// EMPTY and INITALIZING are the two cases where nothing is returned. This is not an error
// Otherwise return the buffer. Let mp_exec_move() manage the state machine to sort out:
//  (1) is the the first time the run buffer has been retrieved?
//  (2) is the buffer in error - i.e. not yet ready for running?
mpBuf_t * mp_get_run_buffer()
{  
    mpBuf_t *r = mp->q.r;

    if (r->buffer_state == MP_BUFFER_EMPTY || r->buffer_state == MP_BUFFER_INITIALIZING) 
    {
        return (NULL);
    }
     mp->q.run_buf_nbr=r->buffer_number;//sme: debug   

    return (r);
}

// Note: mp_free_run_buffer() is only called from mp_exec_XXX, which are within an interrupt
// Clearing and advancing must be done atomically as other interrupts may be using the run buffer
bool mp_free_run_buffer()           // EMPTY current run buffer & advance to the next
{
  bool result = false;
    mpPlannerQueue_t *q = &(mp->q);
    mpBuf_t *r_now = q->r;          // save this pointer is to avoid a race condition when clearing the buffer

    _audit_buffers();               // DIAGNOSTIC audit for buffer chain integrity (only runs in DEBUG mode)
    q->r = q->r->nx;                // advance to next run buffer first...
    _clear_buffer(r_now);           // ... then clear out the old buffer (& set MP_BUFFER_EMPTY)
//    r_now->buffer_state = MP_BUFFER_EMPTY; //... then mark the buffer empty while preserving content for debug inspection
    q->buffers_available++;
    qr_request_queue_report(-1);    // request a QR and add to the "removed buffers" count
   result =(q->w == q->r);
#if 1//sme: 11-11-2022; -->01-17-2023 observe: retval is not used
   if (result == true)
   {
       __NOP();
   }
#endif   
    return  result;         // return true if the queue emptied
}

/* UNUSED FUNCTIONS - left in for completeness and for reference
void mp_copy_buffer(mpBuf_t *bf, const mpBuf_t *bp)
{
    // copy contents of bp to bf while preserving pointers in bp
    memcpy((void *)(&bf->bf_func), (&bp->bf_func), sizeof(mpBuf_t) - (sizeof(void *) * 2));
}
*/


/************************************************************************************
 *** DIAGNOSTICS ********************************************************************
 ************************************************************************************/

/************************************************************************************
 * mp_dump_planner
 * _planner_report()
 * _audit_buffers()
 */

//#define __DUMP_PLANNER
//#define __PLANNER_REPORT_ENABLED
//#define __AUDIT_BUFFERS

#ifdef __DUMP_PLANNER
void mp_dump_planner(mpBuf_t *bf_start)   // starting at bf
{
    mpBuf_t *bf = bf_start;

    printf ("Buf, Line, State, Hint, Planbl, Iter, Tmove, Tplan, Ovr, Thr, Len, Ve, Vc, Vx, Vemax, Vcset, Vcmax, Vxmax, Vjt\n");

    do {
        printf ("%d,",    (int)bf->buffer_number);
        printf ("%d,",    (int)bf->linenum);
        printf ("%d,",    (int)bf->buffer_state);
        printf ("%d,",    (int)bf->hint);
        printf ("%d,",    (int)bf->plannable);
        printf ("%d,",    (int)bf->iterations);

        printf ("%1.2f,", bf->block_time_ms);
        printf ("%1.2f,", bf->plannable_time_ms);
        printf ("%1.3f,", bf->override_factor);
        printf ("%1.3f,", bf->throttle);
        printf ("%1.5f,", bf->length);
        printf ("%1.0f,", bf->pv->exit_velocity);
        printf ("%1.0f,", bf->cruise_velocity);
        printf ("%1.0f,", bf->exit_velocity);
        printf ("%1.0f,", bf->pv->exit_vmax);
        printf ("%1.0f,", bf->cruise_vset);
        printf ("%1.0f,", bf->cruise_vmax);
        printf ("%1.0f,", bf->exit_vmax);
        printf ("%1.0f\n", bf->junction_vmax);

        bf = bf->nx;
    } while (bf != bf_start);
}
#endif // __DUMP_PLANNER

//#if 0 && defined(DEBUG)
//#warning DEBUG TRAPS ENABLED

/*
 * _audit_buffers() - diagnostic to determine if buffers are sane
 * _planner_report() - a detailed report for buffer audits
 */

#ifndef __AUDIT_BUFFERS

static void _audit_buffers()
{
    // empty stub
}
#else

static void _planner_report(const char *msg)
{
    #ifdef __PLANNER_REPORT_ENABLED
    rpt_exception(STAT_PLANNER_ASSERTION_FAILURE, msg);

    for (uint8_t i=0; i<PLANNER_QUEUE_SIZE; i++) 
    {
        printf("{\"er\":{\"stat\":%d, \"type\":%d, \"lock\":%d, \"plannable\":%d",
            mb.bf[i].buffer_state,
            mb.bf[i].block_type,
            mb.bf[i].locked,
            mb.bf[i].plannable);
        
            if (&mb.bf[i] == mb.r) 
            {
              printf(", \"RUN\":t");
            }
              
            if (&mb.bf[i] == mb.w) 
            {
               printf(", \"WRT\":t");
            }
            
            printf("}}\n");
    }
    #endif
}

static void _audit_buffers()
{
    __disable_irq();

    // Current buffer should be in the running state.
    if (mb.r->buffer_state != MP_BUFFER_RUNNING) 
    {
        _planner_report("buffer audit1");
        debug_trap("buffer audit1");
    }

    // Check that the next from the previous is correct.
    if (mb.r->pv->nx != mb.r || mb.r->nx->pv != mb.r)
    {
        _planner_report("buffer audit2");
        debug_trap("buffer audit2");
    }

    // Now check every buffer, in order we would execute them.
    mpBuf_t *bf = mb.r->nx;
    while (bf != mb.r) 
    {
        // Check that the next from the previous is correct.
        if (bf->pv->nx != bf || bf->nx->pv != bf)
        {
            _planner_report("buffer audit3");
            debug_trap("buffer audit3");
        }

        // Order should be:
        //  - MP_BUFFER_RUNNING
        //  - MP_BUFFER_PLANNED (zero or more)
        //  - MP_BUFFER_NOT_PLANNED (zero or more)
        //  - MP_BUFFER_EMPTY (zero or more up until mb.r)
        //  - no more

        // After RUNNING, we can PREPPED, PLANNED, INITED, IN_PROCESS, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_RUNNING &&
            bf->buffer_state != MP_BUFFER_PREPPED &&
            bf->buffer_state != MP_BUFFER_PLANNED &&
            bf->buffer_state != MP_BUFFER_INITIALIZING &&
            bf->buffer_state != MP_BUFFER_IN_PROCESS &&
            bf->buffer_state != MP_BUFFER_EMPTY) 
        {
            // Exception: MP_BUFFER_INITIALIZING and MP_BUFFER_IN_PROCESS are allowed, but we may want to watch for it:
            if ((bf->buffer_state == MP_BUFFER_INITIALIZING) || (bf->buffer_state == MP_BUFFER_IN_PROCESS)) 
            {
                __NOP();
            } 
            else 
            {
                _planner_report("buffer audit4");
                debug_trap("buffer audit4");
            }
        }

        // After PLANNED, we can see PREPPED, INITED, IN_PROCESS, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_PLANNED &&
            bf->buffer_state != MP_BUFFER_PREPPED &&
            bf->buffer_state != MP_BUFFER_INITIALIZING &&
            bf->buffer_state != MP_BUFFER_IN_PROCESS &&
            bf->buffer_state != MP_BUFFER_EMPTY) 
        {
            _planner_report("buffer audit5");
            debug_trap("buffer audit5");
        }

        // After PREPPED, we can see PREPPED, INITED, IN_PROCESS, or EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_PREPPED &&
            bf->buffer_state != MP_BUFFER_PREPPED &&
            bf->buffer_state != MP_BUFFER_INITIALIZING &&
            bf->buffer_state != MP_BUFFER_IN_PROCESS &&
            bf->buffer_state != MP_BUFFER_EMPTY) 
        {
            _planner_report("buffer audit6");
            debug_trap("buffer audit6");
        }

        // After EMPTY, we should only see EMPTY
        if (bf->pv->buffer_state == MP_BUFFER_EMPTY && bf->buffer_state != MP_BUFFER_EMPTY) 
        {
            _planner_report("buffer audit7");
            debug_trap("buffer audit7");
        }
        // Now look at the next one.
        bf = bf->nx;
    }
    __enable_irq();
}

#endif // __AUDIT_BUFFERS

/****************************
 * END OF PLANNER FUNCTIONS *
 ****************************/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/
