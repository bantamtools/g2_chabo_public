 
/*
 * g2_switch.cpp - switch handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2014 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2014 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.
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
/* Switch Modes
 *
 *	The switches are considered to be homing switches when cycle_state is
 *	CYCLE_HOMING. At all other times they are treated as limit switches:
 *	  - Hitting a homing switch puts the current move into feedhold
 *	  - Hitting a limit switch causes the machine to shut down and go into lockdown until reset
 *
 * 	The normally open switch modes (NO) trigger an interrupt on the falling edge
 *	and lockout subsequent interrupts for the defined lockout period. This approach
 *	beats doing debouncing as an integration as switches fire immediately.
 *
 * 	The normally closed switch modes (NC) trigger an interrupt on the rising edge
 *	and lockout subsequent interrupts for the defined lockout period. Ditto on the method.
 */
#include "main.h"
#include "bantam_hal.h"
#include "g2core.h"
#include "g2_config.h"
#include "g2_settings.h"
#include "g2_switch.h"
#include "g2_canonical_machine.h"
#include "g2_text_parser.h"
#include "system_bantam.h"

switches_t sw;//sme:
/*******************************************************************
 * switch_init() - initialize homing/limit switches
 **********************************************************************/
void switch_init(void)
{
   switch_reset();
}
/*******************************************************************
 
 * switch_reset() - reset homing/limit switches (no initialization)
 *
 *	This function assumes all Motate pins have been set up and that
 *	SW_PAIRS and SW_POSITIONS is accurate
 *
 *	Note: `type` and `mode` are not initialized as they should be 
 *            set from configuration
 **********************************************************************/
void switch_reset(void)
{
  switch_t *s;	// shorthand
  
  for (uint8_t axis=0; axis<SW_PAIRS; axis++) 
  {
    for (uint8_t position=0; position<SW_POSITIONS; position++) 
    {
      s = &sw.s[axis][position];
      s->state = false;
     
    }/* End For */
  }/* End For */
}/* End function*/

static uint8_t axis_X_min_pin,axis_Y_min_pin,axis_Z_min_pin,  axis_Z_max_pin=0;
static uint8_t axis_x_new_transition,axis_y_new_transition,axis_z_new_transition,axis_probe_new_transition;
#ifdef DEPLOY_FOURTH_AXIS
static uint8_t axis_A_min_pin=0;
static uint8_t axis_a_new_transition;
#endif
/**************************************************************************
 * poll_switches() - run a polling cycle on all switches
 *
  *  Polling encapsulates the a call to _homing_handler, when a homing switch
 * experiences an new transition. Closure triggers  "back-off" which reverss direction to release
 * the pressed switch. There are no other switches besides homing, until
 * V3 deploys probing, wherein, the z axis is moving in the forward direction to meet a surface of the table or workpiece
 * whereas in homing, the z axis "homes" to the opposite direction.
 *************************************************************************/
stat_t poll_switches()
{
    volatile bool active_flag=false;
    int8_t presently_homed_axis =0;
    /* The helper function, encapsulates details of active state and new transitions 
     *  of all switches, not just limit switches 
     */
    active_flag=motion_poll_limit_switches();
    if (active_flag==true)
    {
      __NOP();
    }     
    /* Flag new transitions, active states: */
    
    /* Check the homing limit switches: */
    axis_x_new_transition= motion_xlimit_new_transition();
    axis_y_new_transition= motion_ylimit_new_transition();
    axis_z_new_transition= motion_zlimit_new_transition();
    
    axis_probe_new_transition=motion_probe_new_transition();
    
    axis_X_min_pin       = motion_xlimit_active();
    axis_Y_min_pin       = motion_ylimit_active();
    axis_Z_max_pin       = motion_zlimit_active();
    axis_Z_min_pin       = motion_probe_switch_active();
#ifdef DEPLOY_FOURTH_AXIS
    axis_A_min_pin       = motion_alimit_active();
#endif
   
   /* Assign and apply any new transitions, active states */
   if (cm->cycle_type == CYCLE_HOMING)  
   {       
       /* Apply the state of homing limit switches  */
       /*  Remedy: Observed active flag true on axis not presently being homed 
        * spuriously triggering present homing operation
        */
       presently_homed_axis=homing_get_present_axis();
       switch (presently_homed_axis)
       {
           case LIMIT_SW_AXIS_X:
              apply_switch_event(&sw.s[LIMIT_SW_AXIS_X][SW_MIN], (bool)axis_X_min_pin,axis_x_new_transition);           
                break;
           case LIMIT_SW_AXIS_Y:
               apply_switch_event(&sw.s[LIMIT_SW_AXIS_Y][SW_MIN], (bool)axis_Y_min_pin,axis_y_new_transition);           
                break;
           case LIMIT_SW_AXIS_Z: 
                apply_switch_event(&sw.s[LIMIT_SW_AXIS_Z][SW_MAX], (bool)axis_Z_max_pin,axis_z_new_transition);
                break;
#ifdef DEPLOY_FOURTH_AXIS
           case LIMIT_SW_AXIS_A: 
                apply_switch_event(&sw.s[LIMIT_SW_AXIS_A][SW_MIN], (bool)axis_A_min_pin,axis_a_new_transition);
                break;
#endif
           default:
               __NOP();
               break;
       }
   }

   else if(cm->cycle_type == CYCLE_PROBE)
   {  
     if (axis_probe_new_transition==true)
     {
       __NOP();//__no_operation();
     }
     /* During probing, the Z axis is moving in the the opposite direction from the home switch  */   
     apply_switch_event(&sw.s[LIMIT_SW_AXIS_Z][SW_MIN], (bool)axis_Z_min_pin,axis_probe_new_transition);
   }
   
   return (STAT_OK);

}/* End function */

/*********************************************************************
 * Function: apply_switch_event() - 
 *
 *  Description: Applies switch event in context of system state or cycle;
 *  
 *	Returns true if switch state changed - 
 *     e.g. leading or falling edge detected.
 *
 *	Assumes pin_value **input** = 1 means open, 0 is closed,
 *        
 **************************************************************************/
int8_t apply_switch_event(switch_t *s, uint8_t pin_value, uint8_t new_transition_flag)
{  
   uint8_t result = false;

   // The switch legitimately changed state - process edges
   s->state = pin_value;//sme: boolean: true==PRESSED, false == RELEASED
 
   if (cm->cycle_type == CYCLE_HOMING)
   { 
      if ((new_transition_flag == true) 
      &&  (pin_value==true)) //true == active state
      {
         result = _homing_handler();         
      }
   }
 
   else if(cm->cycle_type == CYCLE_PROBE)
   { 
      if (new_transition_flag == true)
      {
         _probing_handler(true);
      }
   }
  return (result);
}/* End function */

/***************************************************************
 * read_switch() - read switch state from the switch structure
 *NOTE: This does NOT read the pin itself. See poll_switch
 ***************************************************************/
int8_t read_switch(uint8_t axis, uint8_t position)
{
   return (sw.s[axis][position].state);
}/* End function */

/*******************************************************************
 *  sw_get_ss() - get switch state
 *
 *	Switches map to:
 *	  0 = Xmin, 1= Xmax
 *	  2 = Ymin, 3= Ymax
 *	  4 = Zmin, 5= Zmax
 *	  6 = Amin, 7= Amax
 *****************************************************************/
stat_t sw_get_ss(nvObj_t *nv)	// switch number (0-7)
{
     if (nv->value_int >= (SW_PAIRS * SW_POSITIONS)) 
     { 
       return (STAT_INPUT_EXCEEDS_MAX_VALUE/*sme:STAT_INPUT_VALUE_UNSUPPORTED*/);
     }
     uint8_t number = ((uint8_t)nv->token[0] & 0x0F);	// change from ASCII to a number 0-9 (A-F, too)
     nv->value_int =  read_switch( number/2, number&0x01 );
     nv->valuetype = TYPE_FLOAT;
     return (STAT_OK);
}/* End function */

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

  static const char fmt_st[]  = "[st]  switch type%18.0f [0=NO,1=NC]\n";
  void sw_print_st(nvObj_t *nv) { text_print_flt(nv, fmt_st);}

  static const char fmt_ss[]  = "Switch ss%s state:     %1.0f\n";
  void sw_print_ss(nvObj_t *nv) 
  { 
    fprintf( STDERR_SUBSTITUTE, fmt_ss, nv->token, nv->value);
    comms_mgr_write_msg(STDERR_SUBSTITUTE);
  }

#endif
