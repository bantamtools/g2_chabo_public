/*******************************************************************************
* File:   motionCtrl.c
*
* Description: 
*
*    Stepper motor setup/tuning/operation/feedback
*
*******************************************************************************/
#include "main.h" 
#include "bantam_hal.h"
#include "comms_mgr.h"
#include "user_types.h"
#include "globdefs.h"
#include "g2core.h" 
#include "g2_error.h" 
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

#include "tmc2660_mtr_ctrlr.h" //Setup/tuning of stepper motor control IC 
#include "motion_ctrl.h"
#include "system_bantam.h"//tim14 provides system time for velocity and rpm calcs
#include "g2_canonical_machine.h"
#include "g2_controller.h"
#include "g2_planner.h"
#include "g2_report.h"
#include "kernel.h"
#include <string.h>

//volatile static ErrCodeE motion_ctrl_err_status = ERR_NONE;
//volatile static ErrCodeE motion_ctrl_stpr_setup_status = ERR_NONE;
#if 1//sme 11-11-2023
extern int probe_motion_end_callback_qnbr;//sme: 11-11-2022 help locate position in queue
#endif
/* Define a struct for tracking probe normally open, contact "switch" */
typedef struct
{
  GPIO_PinState no_state;
  bool active_flg;/* active == "pressed" == "closed" == "falling edge" */
  bool new_transition_flg; /* if state has just transitioned between active, inactive */
}ProbeNoSwitchStateS;
  
/* Apply the Probe switch state struct to the Tool release toggle switch */
typedef ProbeNoSwitchStateS ToolReleaseNoSwitchStateS;

/* Enumerate motion control task states */
  typedef enum
  {
    MOTION_CTRL_IDLE,
    MOTION_SETUP_TMC2660_DRIVERS,//this is the actual order of progress
    MOTION_ENABLE_STEPPER_DRIVES,  
    MOTION_CTRL_RUN,   
    MOTION_CTRL_STATE_COUNT
  }MotionCtrlTaskStateE;
  
/* Enumerate chip setup state */
 typedef enum
 {
   SETUP_INACTIVE,      
   SETUP_REGISTER_INIT,
   SETUP_SEND_COMMANDS,
   SETUP_COMPLETED,
   SETUP_ON_ALL_DRIVES_COMPLETED,
   SETUP_COUNT  
 }MotorCtrlChipSetupStateE;

#ifdef CHABO_USE_MATTS_TMC2660_INIT
#define MOTION_CTRL_START_STATE MOTION_CTRL_IDLE
#else
#define MOTION_CTRL_START_STATE MOTION_SETUP_TMC2660_DRIVERS
#endif
 
volatile static MotionCtrlTaskStateE motion_ctrl_state =MOTION_CTRL_START_STATE;//MOTION_ENABLE_STEPPER_DRIVES;//cannot and only use  matt's setup 
static MotorCtrlChipSetupStateE setup_state=SETUP_REGISTER_INIT;
static int motor_axis_id=MTR_AXIS_X;
void motion_ctrl_restart_tmc2660_setup(void)
{
#ifdef CHABO_USE_MATTS_TMC2660_INIT
    tmc2660_re_init_matt();
#else
  setup_state=SETUP_REGISTER_INIT;
  motor_axis_id=MTR_AXIS_X;
  motion_ctrl_state=MOTION_SETUP_TMC2660_DRIVERS;
#endif
}
bool motion_ctrl_running(void) {return (motion_ctrl_state==MOTION_CTRL_RUN);}
bool motion_ctrl_tmc2660_setup_completed(void) {return (motion_ctrl_state>MOTION_ENABLE_STEPPER_DRIVES);}

/* Allocate switch polling state data store: */
#ifdef REFACTOR_LIMIT_SWITCHES
 LimitSwitchInfoS motion_limit_sw[POLLED_STATE_COUNT][LIMIT_SW_COUNT];
 SwitchActiveStateE motion_switch_active_state[LIMIT_SW_COUNT]=
 {
    X_LIMIT_SW_ACTIVE_STATE,
    Y_LIMIT_SW_ACTIVE_STATE,
    Z_LIMIT_SW_ACTIVE_STATE    
 };
#endif
volatile static SwitchPolledStateTrackerE poll_ndx = PRESENT_POLLED_STATE;

/* Probe electrical continuity contact with pallet is monitored as it were a switch */
static ProbeNoSwitchStateS motion_probe_sw[POLLED_STATE_COUNT];
#ifdef REFACTOR_LIMIT_SWITCHES
void motion_set_limit_sw_active_state(uint8_t axis, uint8_t state )
{
    
    if (axis<LIMIT_SW_COUNT)
    {
       motion_switch_active_state[axis]=(SwitchActiveStateE)state; 
       motion_limit_sw[PRESENT_POLLED_STATE][axis].active_state =( GPIO_PinState)state;
    }
}
#endif

 /***************************************************************************
 * 
 * Function: motion_ctrl_init 
 * 
 * Description:   
 * Controls  
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
void motion_ctrl_init(void)
{ 
  int  i=0;
  memset((void*)&motion_probe_sw[0],0,sizeof(motion_probe_sw));
#ifdef REFACTOR_LIMIT_SWITCHES
  memset((void*)&motion_limit_sw[0],0,sizeof(motion_limit_sw));
  /* limit switches are pulled high. NO, NC active state assignments are LOW */
  
  motion_switch_active_state[LIMIT_SW_AXIS_X]= X_LIMIT_SW_ACTIVE_STATE;
  motion_switch_active_state[LIMIT_SW_AXIS_Y]= Y_LIMIT_SW_ACTIVE_STATE;
  motion_switch_active_state[LIMIT_SW_AXIS_Z]= Z_LIMIT_SW_ACTIVE_STATE;  
  
  motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_X].active_state = (GPIO_PinState)X_LIMIT_SW_ACTIVE_STATE;
  motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_Y].active_state = (GPIO_PinState)Y_LIMIT_SW_ACTIVE_STATE;
  motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_Z].active_state = (GPIO_PinState)Z_LIMIT_SW_ACTIVE_STATE;
 
#endif
}/* End Function */


/* External function reference */
void st_dda_interrupt_helper(StepperStepPulseStateE state);
/*
   "Break events can also be generated by software using BG and B2G bits in the TIMx_EGR
   register. The software break generation using BG and BG2 is active whatever the BKE and
   BKE2 enable bits values" [STM32F77x Reference Manual]
*/
/****************************************************************************
 * 
 * Function:   motion_request_exec_move()
 * Description:  

 *************************************************************************/
void motion_request_exec_move(void)
{               
     TC3_CH1_TimerStart();
}
 
/****************************************************************************
 * 
 * Function:   motion_request_load_move()
 * Description:  
 *************************************************************************/
void motion_request_forward_plan(void)
{          
     TC3_CH2_TimerStart();
}
 /****************************************************************************
 * 
 * Function:  motion_set_step_direction
 * 
 * Description:  
 *************************************************************************/
 void motion_set_step_direction( MotorAxisIdE axis, DirectionE direction)
 {
  volatile GPIO_PinState pin_state = (GPIO_PinState)direction;
  switch (axis)
  {
    case MTR_AXIS_X:
      hal_write_pin(0, X_STPR_DIR_PIN , pin_state);      
      break;
      
    case MTR_AXIS_Y:
#ifdef CHABO_LFP
       hal_write_pin(0, Y2_STPR_DIR_PIN, pin_state);    
#endif 
       hal_write_pin(0, Y1_STPR_DIR_PIN , pin_state);
      break;  
      
    case MTR_AXIS_Z:
       hal_write_pin(0, Z_STPR_DIR_PIN , pin_state);  
      break; 
      
#ifdef DEPLOY_FOURTH_AXIS
    case MTR_AXIS_A:          
        hal_write_pin(0, Y2_STPR_DIR_PIN, pin_state);
        break;  
#endif
        
  default:
      break;      
  }/* End Switch */
 }/* End function*/


/***************************************************************************
 * 
 * Function: stepper_driver_chip_setup 
 * 
 * Description:   
 * Controls  
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
MotorCtrlChipSetupStateE stepper_driver_chip_setup(void)
{
    int status=STAT_OK; 
    
    switch(setup_state)
    {
      case SETUP_INACTIVE:
          break;  
          
      case SETUP_REGISTER_INIT:
          /* Init this motor drive's setup registers */ 
          tmc2660_register_init();          
          setup_state = SETUP_SEND_COMMANDS;
          break;
        
      case SETUP_SEND_COMMANDS:
          /* Repeatedly call tmc2660_setup_registers_task until it's internal state
           *  (return value) indicates it has completed setup off all the trinamics control
           *  registers for a given motor (designated by the spi bus slave select variable: 
           *   which mapped to using the motor_axis_id. 
          */ 
          status=tmc2660_setup_registers_task(0,(MotorAxisIdE)motor_axis_id);
         
          if (status == TMC2660_SETUP_COMPLETE)
          { 
             /* Onto the next motor/axis: */   
             motor_axis_id++;
             if(motor_axis_id <MTR_AXES_COUNT)                           
             {
               setup_state =SETUP_REGISTER_INIT;
             }
             else
             {
               setup_state = SETUP_ON_ALL_DRIVES_COMPLETED;
             }            
          }
         
          break;          
          
      case SETUP_ON_ALL_DRIVES_COMPLETED:
          //TBD: any cleanup/status reporting...
          //setup_state = SETUP_INACTIVE;
          break;
       
      default:
         status =-1;//ERR_MOTION_CTRL_UNDEF_STPR_SETUP_STATE;          
         break;     
         
    }/* End switch */
    
   if(status !=STAT_OK)
   {
     __NOP();//__no_operation();
   }
   
   return setup_state;
}/* End function */

/* Provide implementation independent access functions for clients to determine switch status */
bool motion_probe_new_transition(void){return (motion_probe_sw[PRESENT_POLLED_STATE].new_transition_flg==true);}

#ifdef REFACTOR_LIMIT_SWITCHES
/* Active state : Pressed==Closed==Falling Edge*/
bool motion_xlimit_active(void){return (motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_X].active_flg ==true);}
bool motion_ylimit_active(void){return (motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_Y].active_flg ==true);}
bool motion_zlimit_active(void){return (motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_Z].active_flg ==true);}
#ifdef DEPLOY_FOURTH_AXIS
bool motion_alimit_active(void){return (motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_A].active_flg ==true);}
#endif

/* Fresh transition detected in most recent polling */
bool motion_xlimit_new_transition(void){return (motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_X].new_transition_flg==true);}
bool motion_ylimit_new_transition(void){return (motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_Y].new_transition_flg==true);}
bool motion_zlimit_new_transition(void){return (motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_Z].new_transition_flg==true);}
#ifdef DEPLOY_FOURTH_AXIS
bool motion_alimit_new_transition(void){return (motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_A].new_transition_flg==true);}
#endif
#endif


/**************************************************************
*
* Function: motion_any_limit_active
*
*****************************************************************/ 
bool motion_any_limit_active(void)
{
#ifdef DEPLOY_LIMIT_SWITCH_FAULT_CHECKING
 volatile bool result = false;  
/* Report the first active limit switch */
   volatile static bool lim_switch_active[HOMING_AXES]={false};
#define AXIS_NAMES_STR_LEN 40
#define FAULT_STR_LEN 80
    char faulty_switch_names_string[FAULT_STR_LEN]={0};
    char faulty_switch_msg[FAULT_STR_LEN]={0};
#ifdef REPEAT_PERSISTENT_SWITCH_ANOMALIES
#define PERSISTENT_FAULT_REPORT_INTERVAL_COUNT 300 //100 counts are est. 1 second interval at 10ms scheduling of motion control
   static int fault_reported_interval_counter=0;
   static uint32_t start_time_ms, end_time_ms=0;
   volatile static  uint16_t delta_time_ms=0;
#else
   static bool fault_reported=false;
#endif
     lim_switch_active[AXIS_X]= motion_xlimit_active()==true; 
     lim_switch_active[AXIS_Y]= motion_ylimit_active()==true;
     lim_switch_active[AXIS_Z]= motion_zlimit_active()==true;
#ifdef DEPLOY_FOURTH_AXIS
     lim_switch_active[AXIS_A]= motion_alimit_active()==true;
#endif

  result = ((lim_switch_active[AXIS_X]==true)
     ||(lim_switch_active[AXIS_Y]==true)
     ||(lim_switch_active[AXIS_Z]==true)
#ifdef DEPLOY_FOURTH_AXIS         
     ||(lim_switch_active[AXIS_A]==true)
#endif          
     );
   
   if (result==true)
   {
      if (lim_switch_active[AXIS_X]==true)
      {
        strncat(faulty_switch_names_string, "X,",3);
      }
      if (lim_switch_active[AXIS_Y]==true)
      {
       strncat(faulty_switch_names_string, "Y,",3);
      }
      if (lim_switch_active[AXIS_Z]==true)
      {
        strncat(faulty_switch_names_string, "Z,",3);
      }
#ifdef DEPLOY_FOURTH_AXIS      
      if (lim_switch_active[AXIS_A]==true)
      {
        strncat(faulty_switch_names_string, "A,",3);
      }
#endif      
#ifdef REPEAT_PERSISTENT_SWITCH_ANOMALIES
      if (fault_reported_interval_counter++ >= PERSISTENT_FAULT_REPORT_INTERVAL_COUNT)
      {
         fault_reported_interval_counter=0;
         /* To check counting interval is desired time interval:*/     
         end_time_ms=sys_get_time_ms();
         delta_time_ms=end_time_ms-start_time_ms;
         start_time_ms=end_time_ms;
         sprintf(faulty_switch_msg,"Homing Limit switch(es): %s active or possibly defective, outside of homing cycle: ", faulty_switch_names_string);
         rpt_exception(STAT_ACTIVE_SWITCH_ANOMALY,faulty_switch_msg);
      }
   } 
#else
     if (fault_reported!=true)
     {
        fault_reported=true;
        sprintf(faulty_switch_msg,"Homing Limit switch(es): %s active or possibly defective, outside of homing cycle: ", faulty_switch_names_string);
        rpt_exception(STAT_ACTIVE_SWITCH_ANOMALY,faulty_switch_msg);
     }
   }
   else
   {
     fault_reported=false;
   }
#endif
#else //original
  bool result = 
    ((motion_xlimit_active()==true) 
         ||(motion_ylimit_active()==true)
         ||(motion_zlimit_active()==true)
#ifdef DEPLOY_FOURTH_AXIS  
         ||(motion_alimit_active()==true)
#endif
  );        
#endif
  return result;
}
/**************************************************************
*
* function: motion_probe_switch_active(axis)
*
*****************************************************************/ 
bool motion_probe_switch_active(void)
{
   bool result= motion_probe_sw[PRESENT_POLLED_STATE].active_flg==true;
   return result;
}
/***************************************************************************
 * 
 * Function: motion_poll_limit_switches
 * 
 * Description:
 * Not just for limit switches but all "switch"-like events: limits probe, tool release toggle 
 * Called periodically from motion_ctrl task.
 * Interprets limit switch states true true (asserted)   
 * when dual redundant NC,NO pairs are consistent ( open, closed respectively)
 * false otherwise.
 *
 *  "Limit" in V3 architecture is applied differently in homing versus probing.
 *  1. Three Homing limit is Min travel (home) for homing.
 *     a. Xmin :furthest "reverse" movement until contacting Xaxi home switch
 *     b. Ymin :furthest "reverse" movement until contacting Yaxis home switch
 *     c. Zmax :furthest "reverse", "up" movement until topped out contacting Z axis switch
 *  2. Three ATC limit switches: 
        a. Tool Carriage engaged/released to/from Tool Carriage Dock, 
        b. ATC homing/Seek "docked" Tool Carriage 
        c. Tool Holder in spindle, "Drawbar" Engaged/Released

 *  3. One Probing "switch": but will be conductive/continuity based contact event
 *     a. Zmin :furthers "forward", "down" movement until comin into contact with work piece of work table
 *
 *  4. One user toggle switch to release/engage a tool from/to the "Drawbar"   
 *
 * Assumptions/Requirement: 
 * 
 ****************************************************************************/
#ifdef REFACTOR_LIMIT_SWITCHES
int motion_poll_limit_switches(void)
{
  volatile int result = 0;
  int size_memcpy = sizeof(motion_limit_sw)/2;
  volatile int limit_sw_id =  LIMIT_SW_AXIS_X;

  /* Save the previous polled switch state: */
  memcpy((void *)&motion_limit_sw[PREVIOUS_POLLED_STATE],
          (void *)&motion_limit_sw[PRESENT_POLLED_STATE], size_memcpy );
  memcpy((void *)&motion_probe_sw[PREVIOUS_POLLED_STATE],(void *)
          &motion_probe_sw[PRESENT_POLLED_STATE],sizeof(ProbeNoSwitchStateS));

  motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_X].state = hal_read_pin(0,X_ENDSTOP_SW_NC_PIN);                                                                          
  motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_Y].state = hal_read_pin(0,Y_ENDSTOP_SW_NC_PIN);                                                                         
  motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_Z].state = hal_read_pin(0,Z_ENDSTOP_SW_NC_PIN);
#ifdef DEPLOY_FOURTH_AXIS
  motion_limit_sw[PRESENT_POLLED_STATE][LIMIT_SW_AXIS_A].state = hal_read_pin(0,A_ENDSTOP_SW_NC_PIN);
#endif
  /* Limit switches only:no or nc states:
   * Inspect each limit switch for active state, or transition 
   *  between active and inactive 
  */
  for(limit_sw_id=LIMIT_SW_AXIS_X;limit_sw_id<LIMIT_SW_COUNT; limit_sw_id++)
  {
      /* Check for active state: NC contact is open, NO contact is closed */
      motion_limit_sw[PRESENT_POLLED_STATE][limit_sw_id].active_flg =
      (motion_limit_sw[PRESENT_POLLED_STATE][limit_sw_id].state ==
              motion_limit_sw[PRESENT_POLLED_STATE][limit_sw_id].active_state) ;  
      
     /* Check for fresh transition, in either direction: */
     motion_limit_sw[PRESENT_POLLED_STATE][limit_sw_id].new_transition_flg =
     motion_limit_sw[PRESENT_POLLED_STATE][limit_sw_id].active_flg != 
             motion_limit_sw[PREVIOUS_POLLED_STATE][limit_sw_id].active_flg; 
    
     /* Provide an over-simplified return value: true when any switch is 
      * in the active state, whether or not is a fresh transition 
      */
    if(motion_limit_sw[PRESENT_POLLED_STATE][limit_sw_id].active_flg==true)
    {      
      result=true;               
    } 

    /* The remaining switches are not part of the motion_limit_sw_pair[][] array*/ 
    /* Probing contact event: moved along the Zaxis, */
    motion_probe_sw[PRESENT_POLLED_STATE].no_state   = hal_read_pin(0,PROBE_SW_NO_Pin);
    motion_probe_sw[PRESENT_POLLED_STATE].active_flg = motion_probe_sw[PRESENT_POLLED_STATE].no_state==(GPIO_PinState)SWITCH_NO_ACTIVE_STATE;
    motion_probe_sw[PRESENT_POLLED_STATE].new_transition_flg = 
    motion_probe_sw[PRESENT_POLLED_STATE].active_flg != motion_probe_sw[PREVIOUS_POLLED_STATE].active_flg;
    
    if(motion_limit_sw[PRESENT_POLLED_STATE][limit_sw_id].new_transition_flg==true)
    {
      result=true;
    } 

 }/* End for loop on true (NO, NC redundant pair) limit switches */
 return result;
 
}/* End function */

#endif
/*******************************************************************************/
/*******************************************************************************/


/**************************************************************
*
* function: motion_switch_active(axis)
*
*****************************************************************/ 
bool motion_switch_active(int8_t axis)
{
  Boolean_t result = false;
  int8_t switch_id =axis;
  if (axis < LIMIT_SW_COUNT)
  { 
#ifdef REFACTOR_LIMIT_SWITCHES
     result = (motion_limit_sw[PRESENT_POLLED_STATE][switch_id].active_flg ==true);
#endif
  }
   return result;
}

 /****************************************************************************
 * 
 * Function:  motion_ctrl_task 
 * 
 * Description:   
 *      Monitors via encoder feedback, direction, distance, velocity 
 *      of stepper motors involved in :
 *         1. executing a GCode block of code
 *         2. manual jogging to a position, 
 *         3. manual probing
 * Assumptions/Requirement: 

 *****************************************************************************/
void motion_ctrl_task(void)
{
    volatile static MotorCtrlChipSetupStateE setup_state=SETUP_INACTIVE;
    volatile static MotorAxisIdE axis = MTR_AXIS_X;
    
    enum {STOP_ALL,START_SW_INT1,START_SW_INT2};
    volatile static int test_val = STOP_ALL;
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_in();
#endif 
     /* Apply the   */
     switch(motion_ctrl_state )
     {
           case MOTION_CTRL_IDLE:
               break;
               
           case MOTION_SETUP_TMC2660_DRIVERS: 
             setup_state=stepper_driver_chip_setup();
             if(setup_state==SETUP_ON_ALL_DRIVES_COMPLETED)
             {
               motion_ctrl_state =MOTION_ENABLE_STEPPER_DRIVES;
             }
             break; 
             
          case MOTION_ENABLE_STEPPER_DRIVES:
             hal_write_pin(0,X_STPR_ENn_PIN,GPIO_PIN_RESET );     
             hal_write_pin(0,Y1_STPR_ENn_PIN,GPIO_PIN_RESET );   
             hal_write_pin(0,Y2_STPR_ENn_PIN,GPIO_PIN_RESET );             
             hal_write_pin(0,Z_STPR_ENn_PIN,GPIO_PIN_RESET );                         
             
             motion_ctrl_state = MOTION_CTRL_RUN;
             break; 
                           
         case MOTION_CTRL_RUN: 
              /* 1. Monitor/respond to JSON or Text Mode commands to adjust Trinamics parameters: */
             tmc2660_spi_comm_task();
#if 1//test debug only
             if ( test_val ==START_SW_INT1)
             {
               TC3_CH1_TimerStart(); 
             }
             else if  (test_val == START_SW_INT2)
             { 
                TC3_CH2_TimerStart(); 
             }
             else
             {
                TC3_CH1_TimerStop();
                TC3_CH2_TimerStop();
             }
#endif

#ifdef MAIN_DEPLOY_UART7_STALL_EVENT_PRINTS             
             main_monitor_stall_events();
#endif  
             break;
                         
         default:
             /* to do: raise error: corrupt data:*/
              
             break;
     }/* End Switch */
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_out();
#endif     
}/* End Task */
 /* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**************************************************************************
 * 
 * Function: motion_ctrl_start_dda_interrupt
 *
 * Encapsulates activation enabling of two-stage timer interrupt
 * that performs the dda gcode "moves"
 *
 **************************************************************************/
void motion_ctrl_start_dda_interrupt(void)
{
    TC0_CH1_TimerStart();
}/* End Function */

/**************************************************************************
 * 
 * Function: motion_ctrl_stop_dda_interrupt
 *
 * Encapsulates activation enabling of two-stage timer interrupt
 * that performs the dda gcode "moves"
 *
 **************************************************************************/
void motion_ctrl_stop_dda_interrupt(void)
{
    TC0_CH1_TimerStop();
}/* End Function */
 