
/*
 * controller.cpp - g2core controller and top level parser
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
 */
#include "main.h" 
#include "bantam_hal.h"
#include "g2core.h"  // #1
#include "g2_config.h"  // #2
#include "g2_controller.h"
#include "g2_json_parser.h"
#include "g2_text_parser.h"
#include "g2_gcode.h"
#include "g2_xio.h"// - extended IO functions
#include "g2_canonical_machine.h"
#include "g2_plan_arc.h"
#include "g2_planner.h"
#include "g2_stepper.h"
#include "g2_encoder.h"
#include "g2_report.h"
#include "g2_help.h"
#include "g2_util.h"
#include "g2_gpio.h"
#include "g2_spindle.h" 
#include "g2_switch.h" 
#include "g2_persistence.h"   
#include "system_bantam.h"   
#include "leds_mgr.h"
#include "comms_mgr.h"
#include "stepper.h"//matt
 



/****************************************************************************************
 **** STRUCTURE ALLOCATIONS *************************************************************
 ****************************************************************************************/

controller_t cs;        // controller state structure

/****************************************************************************************
 **** STATICS AND LOCALS ****************************************************************
 ****************************************************************************************/
#define CLIENT_BUF_SIZE 1024//me: allocate separate copy of incoming commands
static char client_buf[CLIENT_BUF_SIZE]; 
#define SPECIAL_CHAR_BUF_SIZE 10//arbitrarilly set--adjust as needed

static void _controller_HSM(void);
//static stat_t _led_indicator(void);             // twiddle the LED indicator
static stat_t _shutdown_handler(void);          // new (replaces _interlock_estop_handler)
static stat_t _interlock_estop_handler(void);
static stat_t _limit_switch_handler(void);      // revised for new GPIO code
bool planning_buffer_is_full=false;
static void _init_assertions(void);
static stat_t _test_assertions(void);
static stat_t _test_system_assertions(void);

static stat_t _sync_to_planner(void);
static stat_t _sync_to_tx_buffer(void);
static stat_t _dispatch_command(void);
static stat_t _dispatch_control(void);
static void _dispatch_kernel(void);
static stat_t _controller_state(void);          // manage controller state transitions
#ifdef DONT_REPORT_MODBUS_OFFLINE_WHEN_DOORS_OPEN
bool cm_interlock_is_opened(void)
{
  bool result=(cm1.safety_state & SAFETY_INTERLOCK_OPEN);
  return result;
}
bool cm_interlock_is_closed(void)
{
  bool result=(cm1.safety_state == SAFETY_INTERLOCK_CLOSED);
  return result;
}   
#endif
static stat_t _usb_comms_timeout_handler(void);

/*
 * _controller_set_connected(bool) - hook for xio to tell the controller that we
 * have/don't have a connection.
 */
void _controller_set_connected(bool is_connected) 
{
    // turn off reports while no-one's listening or we determine what dialect they speak
    sr.status_report_verbosity = SR_OFF;
    qr.queue_report_verbosity = QR_OFF;

    if (is_connected) 
    {
        cs.controller_state = CONTROLLER_CONNECTED; // we JUST connected
    } 
    else 
    {  // we just disconnected from the last device, we'll expect a banner again
        _reset_comms_mode();
        cs.controller_state = CONTROLLER_NOT_CONNECTED;
    }
}

GpioHandlerStatusE _shutdown_input_handler (const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number)
{    
     if (edge != INPUT_EDGE_LEADING) 
     { 
       return GPIO_NOT_HANDLED; 
     }
     cm->shutdown_requested = triggering_pin_number;
     return GPIO_HANDLED;    
}

GpioHandlerStatusE _limit_input_handler (const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) 
{    
     if (edge != INPUT_EDGE_LEADING) 
     { 
       return GPIO_NOT_HANDLED; 
     }
     cm->limit_requested = triggering_pin_number;
     return GPIO_HANDLED;  // allow others to see this notice
} 

void _interlock_input_handler(const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number)  
{
  if (edge != INPUT_EDGE_LEADING)
  {
      cm1.safety_interlock_disengaged = triggering_pin_number;
  } 
  else 
  {   // edge == INPUT_EDGE_TRAILING
      cm1.safety_interlock_reengaged = triggering_pin_number;
  }
} 

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/*
 * controller_init() - controller init
 */

void controller_init()
{
    // preserve settable parameters that may have already been set up
    commMode comm_mode = cs.comm_mode;
   
    memset(&cs, 0, sizeof(controller_t));           // clear all values, job_id's, pointers and status
    _init_assertions();

    cs.comm_mode = comm_mode; 
    cs.fw_build = CHABO_FIRMWARE_BUILD;            // set up identification
    cs.fw_version = CHABO_FIRMWARE_VERSION;       
    cs.controller_state = CONTROLLER_STARTUP;       // ready to run startup lines

#ifdef ROSECOMB_MVP
    if (comms_mgr_usb_connected() || uart_mgr_exp_uart_connected() ==true) 
#else
    if (comms_mgr_usb_connected()==true)      
#endif
    {
        cs.controller_state = CONTROLLER_CONNECTED;
    } 
    if( hal_read_pin(0,ITRLK_LOOP_FLAG1_Pin)==GPIO_PIN_SET)   
    {
        cm1.safety_state |= SAFETY_INTERLOCK_OPEN;
    } 
}
 
void controller_request_enquiry()
{
    sprintf(cs.out_buf, "{\"ack\":true}\n");   
    comms_mgr_write_msg(cs.out_buf) ;   
}

/*
 * controller_run() - MAIN LOOP - top-level controller
 *
 * The order of the dispatched tasks is very important.
 * Tasks are ordered by increasing dependency (blocking hierarchy).
 * Tasks that are dependent on completion of lower-level tasks must be
 * later in the list than the task(s) they are dependent upon.
 *
 * Tasks must be written as continuations as they will be called repeatedly,
 * and are called even if they are not currently active.
 *
 * The DISPATCH macro calls the function and returns to the controller parent
 * if not finished (STAT_EAGAIN), preventing later routines from running
 * (they remain blocked). Any other condition - OK or ERR - drops through
 * and runs the next routine in the list.
 *
 * A routine that had no action (i.e. is OFF or idle) should return STAT_NOOP
 */
#define DISPATCH(func) if (func == STAT_EAGAIN) return;
//#define SME_MEASURE_EXEC_TIME
#ifdef SME_MEASURE_EXEC_TIME
volatile uint32_t start_time_ms, end_time_ms;
volatile uint32_t delta_time_ms;
volatile uint32_t  min_exec_time_ms=0xFFFF;
volatile uint32_t  max_exec_time_ms=0;
enum {bin0,bin1,bin2,bin3,bin4,bin5,bin6,bin7,bin8,bin9,bin_count};
volatile uint32_t histogram[bin_count]={0};

void controller_run( )
{
    min_exec_time_ms=0xFFFF;
    max_exec_time_ms=0;

    start_time_ms=sys_get_time_ms();

    _controller_HSM();

    end_time_ms=sys_get_time_ms();
    delta_time_ms=end_time_ms-start_time_ms;
              
    if (delta_time_ms<min_exec_time_ms)
    {
       min_exec_time_ms=delta_time_ms;
    }

    if (delta_time_ms>max_exec_time_ms)
    {
           max_exec_time_ms=delta_time_ms;
    }

    if (delta_time_ms <1)
    {
          histogram[bin0]++;
          if(histogram[bin0]  >0xFFFFFFF0)
          {
            __NOP();//__no_operation();
          }

    }
    else if (delta_time_ms <2)
    {
          histogram[bin1]++;
          if(histogram[bin1]  >0xFFFFFFF0)
          {
            __NOP();//__no_operation();
          }                      
    }
    else if (delta_time_ms <3)
    {
            histogram[bin2]++;
    }
    else if (delta_time_ms <4)
    {
          histogram[bin3]++;
    }
    else if (delta_time_ms <5)
    {
          histogram[bin4]++;
    }
    else if (delta_time_ms <6)
    {
          histogram[bin5]++;
    }
    else if (delta_time_ms <7)
    {
          histogram[bin6]++;
    }
    else if (delta_time_ms <8)
    {
          histogram[bin7]++;
    }	
    else if (delta_time_ms <9)
    {
          histogram[bin8]++;
    }
    else //if (delta_time_ms <100)
    {
          histogram[bin9]++;
    }									
  
}
#else 
void controller_run( )
{  
    _controller_HSM();
}
#endif

static void _controller_HSM()
{

//----- Interrupt Service Routines are the highest priority controller functions ----//
//      See hardware.h for a list of ISRs and their priorities.
//
//----- kernel level ISR handlers ----(flags are set in ISRs)------------------------//

    // Order is important, and line breaks indicate dependency groups
//--Bantam V3 non-motate, unconditionally called main loop routines -------------------//  
    st_dwell_timer_poll_callback();             //when dwell is active, this polls the countdown
    poll_switches();
 
//-- motate-compliant "dispathed routines    ---------------------------------------//
    DISPATCH(_shutdown_handler());              // invoke shutdown         
    DISPATCH(_usb_comms_timeout_handler());    
    DISPATCH(_interlock_estop_handler());       // interlock or estop have been thrown
    DISPATCH(_limit_switch_handler());          // invoke limit switch
    DISPATCH(_controller_state());              // controller state management
    DISPATCH(_test_system_assertions());        // system integrity assertions  
    DISPATCH(_dispatch_control());              // read any control messages prior to executing cycles      
    //----- planner hierarchy for gcode and cycles ---------------------------------------//
    DISPATCH(sr_status_report_callback());      // conditionally send status report
    DISPATCH(qr_queue_report_callback());       // conditionally send queue report

    // these 3 must be in this exact order:
    DISPATCH(mp_planner_callback());            // motion planner
    DISPATCH(cm_operation_runner_callback());   // operation action runner      
    DISPATCH(cm_arc_callback(cm));              // arc generation runs as a cycle above lines
    DISPATCH(cm_homing_cycle_callback());       // homing cycle operation (G28.2)
    DISPATCH(cm_probing_cycle_callback());      // probing cycle operation (G38.2)
    DISPATCH(cm_jogging_cycle_callback());      // jog cycle operation
    DISPATCH(cm_deferred_write_callback());     // persist G10 changes when not in machining cycle
    DISPATCH(cm_feedhold_command_blocker());    // blocks new Gcode from arriving while in feedhold
     
    //Not here. This is now done in sd_card_mgr: 
#ifndef DEPLOY_SD_TASK
    DISPATCH(write_persistent_values_callback());
#endif

//----- command readers and parsers --------------------------------------------------//
    DISPATCH(_sync_to_planner());               // ensure there is at least one free buffer in planning queue
    DISPATCH(_sync_to_tx_buffer());             // sync with TX buffer (pseudo-blocking)
    DISPATCH(_dispatch_command());              // MUST BE LAST - read and execute next command
}

/****************************************************************************************
 * command dispatchers
 * _dispatch_control - entry point for control-only dispatches
 * _dispatch_command - entry point for control and data dispatches
 * _dispatch_kernel - core dispatch routines
 *
 *  Reads next command line and dispatches to relevant parser or action
 *
 *  Note: The dispatchers must only read and process a single line from the
 *        RX queue before returning control to the main loop.
 */
#define ACCEPT_CONTROL_ONLY_MSGS true //sme: Mnemonics are so nice!
static stat_t _dispatch_control()
{ 
    volatile char *bufptr=&client_buf[0];//sme

    if (cs.controller_state == CONTROLLER_READY) 
    {        
        cs.linelen= comms_mgr_get_next_msg((char *)bufptr,ACCEPT_CONTROL_ONLY_MSGS);     
        if (cs.linelen>0) 
        {
            cs.bufp=(char *)bufptr; 
            cs.linelen+=2;//sme: adjust for desktop app bytes read
           _dispatch_kernel();
        }
        else
        {
           __NOP();
        }       
    }
    return (STAT_OK);
}
#define ACCEPT_CONTROL_AND_DATA_MSGS false
static stat_t _dispatch_command()
{
   char *bufptr=&client_buf[0]; 

    if (cs.controller_state == CONTROLLER_READY) 
    {
        if ((!mp_planner_is_full(mp))          
        && (cs.linelen = comms_mgr_get_next_msg(bufptr, ACCEPT_CONTROL_AND_DATA_MSGS)) > 0)  
        {  
          cs.bufp=bufptr;
          cs.bufp[cs.linelen]=0;//sme: 4-2-2020 was getting add-on trash
          cs.linelen += 2;//sme: 3-18-2020 needed to pass on the bytes read to desktop app, in reply message footer  
          _dispatch_kernel();
        }
    }
    return (STAT_OK);
}

static void _dispatch_kernel(void)
{
    stat_t status;

    while ((*cs.bufp == SPC) || (*cs.bufp == TAB)) 
    {        // position past any leading whitespace
        cs.bufp++;
    }
    strncpy(cs.saved_buf, cs.bufp, SAVED_BUFFER_LEN-1);     // save input buffer for reporting

    if (*cs.bufp == NUL) 
    {   // blank line - just a CR or the 2nd termination in a CRLF
        if (js.json_mode == TEXT_MODE) 
	    {
            text_response(STAT_OK, cs.saved_buf);
            return;
        }
    }
    
    // trap single character commands
    if (*cs.bufp == '!') //==CHAR_FEEDHOLD
    { 
        cm_request_feedhold(FEEDHOLD_TYPE_ACTIONS, FEEDHOLD_EXIT_CYCLE); 
    }
    
    /* SME: Criticality of job kill dictates response taken in rx interrupt */
    else if (*cs.bufp == CHAR_ALARM_KILL_JOB) 
    { 
       cm_request_job_kill(); 
       comms_mgr_assign_cmd_flush_ndx();//sme added 7-22-2022 experiment
       comms_mgr_flushToCommand();//added 11-25-2020 to follow g2core's implementation
    } 

     /* SME: Complexity of cycle processing in main control loop 
      * "command" vs "data" dictates "Resume" from "Paused" state
      * response taken in rx interrupt. 
      */    
    else if (*cs.bufp == '~') //==CHAR_CYCLE_START
    { 
        cm_request_cycle_start();
    }
    
    else if (*cs.bufp == '%') //==CHAR_QUEUE_FLUSH 
    { 
       cm_request_queue_flush();    
       comms_mgr_flushToCommand();
    }
    else if (*cs.bufp == CAN) // == hw reset immediately 
    {
      nvObj_t nv;
      nv.value_int=1;
      nv.value_flt=1.0;
      main_hw_reset(&nv);    
    }                    
    else if (*cs.bufp == CHAR_WDOG ) 
    {
      __NOP();//__no_operation();
    }     
 
    else if (*cs.bufp == ENQ) 
    { 
       controller_request_enquiry(); 
    }        
    else if (*cs.bufp == '{') 
    {   // process as JSON mode
        if (cs.comm_mode == AUTO_MODE) 
	    {
            js.json_mode = JSON_MODE; // switch to JSON mode
        }
        cs.comm_request_mode = JSON_MODE;// mode of this command
        json_parser(cs.bufp);
    }
    else if (strchr("$?Hh", *cs.bufp) != NULL) 
    {   // process as text mode
        if (cs.comm_mode == AUTO_MODE) 
        { 
           js.json_mode = TEXT_MODE; 			
        } // switch to text mode
       
        cs.comm_request_mode = TEXT_MODE;   // mode of this command
        status = text_parser(cs.bufp);
		
        if (js.json_mode == TEXT_MODE)
	    {   // needed in case mode was changed by $EJ=1
            text_response(status, cs.saved_buf);
        }
    }
    else if (js.json_mode == TEXT_MODE)     
    {   
        // anything else is interpreted as Gcode        
        cs.comm_request_mode = TEXT_MODE;   // mode of this command
        text_response(gcode_parser(cs.bufp), cs.saved_buf);
    }  
    else 
    {  // anything else is interpreted as Gcode
        cs.comm_request_mode = JSON_MODE;    // mode of this command

        // this optimization bypasses the standard JSON parser and does what it needs directly
        nvObj_t *nv = nv_reset_nv_list();   // get a fresh nvObj list
        strcpy(nv->token, "gc");             // label is as a Gcode block (do not get an index - not necessary)
        nv_copy_string(nv, cs.bufp);         // copy the Gcode line
        nv->valuetype = TYPE_STRING;
        status = gcode_parser(cs.bufp);
        nv_print_list(status, TEXT_NO_PRINT, JSON_RESPONSE_FORMAT);
        sr_request_status_report(SR_REQUEST_TIMED);         // generate incremental status report to show any changes
    }
}

/**** Local Functions ******************************************************************/

/*
 * _reset_comms_mode() - reset the communications mode (and other effected settings) after connection or disconnection
 */
void _reset_comms_mode() 
{
    // reset the communications mode
    cs.comm_mode = COMM_MODE;
    js.json_mode = (COMM_MODE < AUTO_MODE) ? COMM_MODE : JSON_MODE;
 
    sr.status_report_verbosity = STATUS_REPORT_VERBOSITY;
    qr.queue_report_verbosity = QUEUE_REPORT_VERBOSITY;
}

/* CONTROLLER STATE MANAGEMENT
 * _controller_state() - manage controller connection, startup, and other state changes
 */

static stat_t _controller_state()
{
    if (cs.controller_state == CONTROLLER_CONNECTED)
    {   // first time through after reset
        cs.controller_state = CONTROLLER_STARTUP;        
    }

    // first time through after reset
    else if (cs.controller_state == CONTROLLER_STARTUP)     
    {     
        _reset_comms_mode();       
        cs.controller_state = CONTROLLER_READY;
        rpt_print_system_ready_message();
    }
    return (STAT_OK);
}

/*
 * controller_set_connected(bool) - hook for xio to tell the controller that we
 * have/don't have a connection.
 */

void controller_set_connected(bool is_connected) 
{
    // turn off reports while no-one's listening or we determine what dialect they speak
    sr.status_report_verbosity = SR_OFF;
    qr.queue_report_verbosity = QR_OFF;

    if (is_connected) 
	{
        cs.controller_state = CONTROLLER_CONNECTED; // we JUST connected
    } 
	else 
	{  // we just disconnected from the last device, we'll expect a banner again
        _reset_comms_mode();
        cs.controller_state = CONTROLLER_NOT_CONNECTED;
    }
}


/*
 * controller_parse_control() - return true if command is a control (versus data)
 * Note: parsing for control is somewhat naiive. This will need to get better
 */

bool controller_parse_control(char *p) 
{
    if (strchr("{$?!~%Hh", *p) != NULL) 
    {   // a match indicates control line
        return (true);
    }
    return (false);
}

/*
 * _led_indicator() - blink an LED to show it we are normal, alarmed, or shut down
 */
#if 0//sme: Future: integrate led_indicator()
static stat_t _led_indicator()
{
    uint32_t blink_rate;
    if (cm_get_machine_state() == MACHINE_ALARM) 
    {
        blink_rate = LED_ALARM_BLINK_RATE;
    } 
    else if (cm_get_machine_state() == MACHINE_SHUTDOWN) 
    {
        blink_rate = LED_SHUTDOWN_BLINK_RATE;
    } 
    else if (cm_get_machine_state() == MACHINE_PANIC) 
    {
        blink_rate = LED_PANIC_BLINK_RATE;
    } 
    else 
    {
        blink_rate = LED_NORMAL_BLINK_RATE;
    }

    if (blink_rate != cs.led_blink_rate) 
    {
        cs.led_blink_rate =  blink_rate;
        cs.led_timer = 0;
    }
    if (SysTickTimer.getValue() > cs.led_timer) 
    {
        cs.led_timer = SysTickTimer.getValue() + cs.led_blink_rate;
        IndicatorLed.toggle();
    }  
    return (STAT_OK);
}
#endif 
/*
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 */
static stat_t _sync_to_tx_buffer()
{
    return (STAT_OK);
}

static stat_t _sync_to_planner()
{
    if (mp_planner_is_full(mp)) 
    {   // allow up to N planner buffers for this line
        planning_buffer_is_full=true;
        return (STAT_EAGAIN);
    }
    planning_buffer_is_full=false;
    return (STAT_OK);
}

/****************************************************************************************
 * ALARM STATE HANDLERS
 *
 * _shutdown_handler() - put system into shutdown state
 * _limit_switch_handler() - shut down system if limit switch fired
 * _interlock_handler() - feedhold and resume depending on edge
 *
 *    Some handlers return EAGAIN causing the control loop to never advance beyond that point.
 *
 * _interlock_handler() reacts the following ways:
 *   - safety_interlock_requested == INPUT_EDGE_NONE is normal operation (no interlock)
 *   - safety_interlock_requested == INPUT_EDGE_LEADING is interlock onset
 *   - safety_interlock_requested == INPUT_EDGE_TRAILING is interlock offset
 */
static stat_t _shutdown_handler(void)
{
    if (cm->shutdown_requested != 0) 
    {  // request may contain the (non-zero) input number
        char msg[10];
        sprintf(msg, "input %d", (int)cm->shutdown_requested);
        cm->shutdown_requested = false; // clear limit request used here ^
        cm_shutdown(STAT_SHUTDOWN, msg);
    }
    return(STAT_OK);
}

static stat_t _limit_switch_handler(void)
{
    auto machine_state = cm_get_machine_state();
    if ((machine_state != MACHINE_ALARM) &&
        (machine_state != MACHINE_PANIC) &&
        (machine_state != MACHINE_SHUTDOWN)) 
    {
#if 0//integrate  safe_pin.toggle        
        safe_pin.toggle();
#endif        
    }
    if ((cm->limit_enable == true) && (cm->limit_requested != 0)) 
    {
        char msg[10];
        sprintf(msg, "input %d", (int)cm->limit_requested);
        cm->limit_requested = false; // clear limit request used here ^
        cm_alarm(STAT_LIMIT_SWITCH_HIT, msg);
    }
    return (STAT_OK);
}

/*******************************************************************************
 *
 *  Function: _usb_comms_timeout_handler
 *
 *  Description: Active/relevant when spindle is running. If comms are lost
 *     when spindle is running, spindle is turned off for safety considerations
 *
 ******************************************************************************/
static stat_t _usb_comms_timeout_handler(void)
{
   static bool timed_out_action_taken = false;
   if (spindle.state == SPINDLE_RUNNING)
   if (((comms_wdog_enabled()==true)&&(comms_timed_out() == true))
   ||   ( usb_comms_disconnected()==true))
   {
       if (timed_out_action_taken !=true)
       {
          timed_out_action_taken =true;//take action only once per timeout

          if (cm1.machine_state == MACHINE_CYCLE)
          {
               cm_request_feedhold(FEEDHOLD_TYPE_ACTIONS, FEEDHOLD_EXIT_CYCLE);
          } 
          else  
          {
               spindle_control_immediate(SPINDLE_OFF);
          }                        
          sr_request_status_report(SR_REQUEST_IMMEDIATE);              
          if (usb_comms_disconnected()==true) 
          {
              rpt_exception(STAT_USB_COMMS_DISCONNECTED,(char*)"USB Comms lost (Disconnected)");        
          }
          else
          {
              rpt_exception(STAT_USB_COMMS_WDOG_TIMEOUT,(char *)"USB Comms lost (Watchdog Timeout)");
          }
       }       
   }
   else
   {
     /* reset the action flag */
     timed_out_action_taken = false;
   }
     return STAT_OK;
}

#define DELAY_TIME_MS  500
static stat_t _interlock_estop_handler(void) 
{
    bool report = false;
    static  bool apply_qflush_on_estop=true;
    static volatile int  re_init_tmc2660_counter=0;
    static uint32_t start_time_ms, end_time_ms=0;
    
    // Process E-Stop and Interlock signals 
    // Door was closed but has now opened:
    if ((cm1.safety_state & SAFETY_INTERLOCK_MASK) == SAFETY_INTERLOCK_CLOSED     
    && ((hal_read_pin(0,ITRLK_LOOP_FLAG0_Pin) == GPIO_PIN_RESET)
    ||  (hal_read_pin(0,ITRLK_LOOP_FLAG1_Pin) == GPIO_PIN_RESET)))
    {
        cm1.safety_state |= SAFETY_INTERLOCK_OPEN;

        // Check if the spindle is on
        if (spindle.state != SPINDLE_OFF) 
        {
            if (cm1.machine_state == MACHINE_CYCLE)
            {
                cm_request_feedhold(FEEDHOLD_TYPE_ACTIONS, FEEDHOLD_EXIT_CYCLE);
            } 
            else 
            {
                spindle_control_immediate(SPINDLE_OFF);
            }
        }     
        // If we just entered interlock and we're not off, start the lockout timer
        if ((cm1.safety_state & SAFETY_ESC_MASK) == SAFETY_ESC_ONLINE ||
            (cm1.safety_state & SAFETY_ESC_MASK) == SAFETY_ESC_REBOOTING) 
        {
            cm->esc_lockout_timer=sys_get_time_ms();            
            cm1.safety_state |= SAFETY_ESC_LOCKOUT;
        }
        report = true;

    
    } // Door was open but has now closed:
   else if (((cm1.safety_state & SAFETY_INTERLOCK_MASK) == SAFETY_INTERLOCK_OPEN)          
    && (hal_read_pin(0,ITRLK_LOOP_FLAG0_Pin) == GPIO_PIN_SET)
    && (hal_read_pin(0,ITRLK_LOOP_FLAG1_Pin) == GPIO_PIN_SET)) 
    {
        cm1.safety_state &= ~SAFETY_INTERLOCK_OPEN;
        // If we just left interlock, stop the lockout timer
        if ((cm1.safety_state & SAFETY_ESC_LOCKOUT) == SAFETY_ESC_LOCKOUT) 
        {
            cm1.safety_state &= ~SAFETY_ESC_LOCKOUT;            
            cm->esc_lockout_timer=0;            
        }
        report = true;
    }

    // EStop was pressed
    if (((cm1.estop_state & ESTOP_PRESSED_MASK) == ESTOP_RELEASED)     
    && (hal_read_pin(0,ESTOP_FLAG_Pin) == GPIO_PIN_RESET)) 
    {
        cm1.estop_state = ESTOP_PRESSED | ESTOP_UNACKED | ESTOP_ACTIVE;
        cm_shutdown(STAT_SHUTDOWN, "e-stop pressed");
        leds_mgr_set_rgbw_id(PWM_LEDS_ESTOP_PRESSED);
        if(apply_qflush_on_estop==true)
        {
           comms_mgr_assign_cmd_flush_ndx();//10-14-2021
           comms_mgr_flushToCommand();//10-14-2021
        }  
        // E-stop always sets the ESC to off
        cm1.safety_state &= ~SAFETY_ESC_MASK;
        cm1.safety_state |= SAFETY_ESC_OFFLINE;
        report = true;
    } 
    else 
    // Check if EStop was released:
    if (((cm1.estop_state & ESTOP_PRESSED_MASK) == ESTOP_PRESSED)  
    &&        (hal_read_pin(0,ESTOP_FLAG_Pin) == GPIO_PIN_SET )) 
    {
        cm1.estop_state &= ~ESTOP_PRESSED;
        leds_mgr_set_rgbw_id(PWM_LEDS_ESTOP_RELEASED);
        report = true;
        start_time_ms=sys_get_time_ms();
        
#if 1   //sme: 11-3-2022: to do: replace this blocking timer with non-blocking delay of restart of trinamics stepper drivers
        while ((sys_get_time_ms()-start_time_ms)<DELAY_TIME_MS) 
        {
            __NOP();
        }
        motion_ctrl_restart_tmc2660_setup();
#endif
    }

    // if E-Stop and Interlock are both 0, and we're off, go into "ESC Reboot"
    if (((cm1.safety_state & SAFETY_ESC_MASK) == SAFETY_ESC_OFFLINE) 
        && ((cm1.estop_state & ESTOP_PRESSED) == 0 )
        && ((cm1.safety_state & SAFETY_INTERLOCK_OPEN) == 0)) 
    {
        cm1.safety_state &= ~SAFETY_ESC_MASK;
        cm1.safety_state |= SAFETY_ESC_REBOOTING;
        cm->esc_boot_timer=sys_get_time_ms();        
        report = true;
    }

    // Check if ESC lockout timer or reboot timer have expired
    if (((cm1.safety_state & SAFETY_ESC_LOCKOUT) != 0)        
    &&  (sys_time_ms_limit_reached(cm->esc_lockout_timer,ESC_LOCKOUT_TIME)))    
    {
        cm1.safety_state &= ~SAFETY_ESC_MASK;
        cm1.safety_state |= SAFETY_ESC_OFFLINE;
        report = true;
    }
    if (((cm1.safety_state & SAFETY_ESC_MASK) == SAFETY_ESC_REBOOTING)
    && (sys_time_ms_limit_reached(cm->esc_boot_timer,ESC_BOOT_TIME)))                        
    {
        cm1.safety_state &= ~SAFETY_ESC_MASK;
        report = true;
    }

    // If we've successfully ended all the ESTOP conditions, then end ESTOP
    if (cm1.estop_state == ESTOP_ACTIVE) 
    {
        cm1.estop_state = 0;
        report = true;
    }

    if (report) 
    {
        sr_request_status_report(SR_REQUEST_IMMEDIATE);
    }
    return (STAT_OK);
}
 

/****************************************************************************************
 * _init_assertions() - initialize controller memory integrity assertions
 * _test_assertions() - check controller memory integrity assertions
 * _test_system_assertions() - check assertions for entire system
 */

static void _init_assertions()
{
    cs.magic_start = MAGICNUM;
    cs.magic_end = MAGICNUM;
}

static stat_t _test_assertions()
{
    if ((cs.magic_start != MAGICNUM) || (cs.magic_end != MAGICNUM)) 
    {
        return(cm_panic(STAT_CONTROLLER_ASSERTION_FAILURE, "controller_test_assertions()"));
    }
    return (STAT_OK);
}

stat_t _test_system_assertions()
{
    // these functions will panic if an assertion fails
    _test_assertions();         // controller assertions (local)
    config_test_assertions();
    canonical_machine_test_assertions(&cm1);
    canonical_machine_test_assertions(&cm2);
    planner_assert(&mp1);
    planner_assert(&mp2);
    stepper_test_assertions();
    encoder_test_assertions();  
    return (STAT_OK);
}

