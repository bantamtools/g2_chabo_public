/*******************************************************************************
* File:    spindle_ctrl.c
*
* Description: 
*     
*
*******************************************************************************/
#include "main.h"  
#include "bantam_hal.h"

#include "g2_util.h"
#include "user_types.h"
#include "globdefs.h" 
#include "g2_config.h"
#include "system_bantam.h"// for tracking elapsed msec time  
#include "g2_canonical_machine.h"
#include "spindle_ctrl.h"

#ifndef ERR_NONE //sme this was in exception_mgr.h
#define ERR_NONE 0
#endif

#include "g2_spindle.h"
#include "g2_pwm.h"//User commands to JSON parser set pwm linear equation max, min rpm and pwm duty cycle 


#define MOV_AVG_SAMPLES_COUNT 16 
#define MOV_AVG_DIV_SHIFTS 4 //divide mov avg samples by right shifting 
#define DUTY_CYCLE_INCREM_SIZE 1  
#define SPINDLE_INIT_ON_DURATION_MS 800
#define SPINDLE_INIT_OFF_DURATION_MS 200

/* Enumerate Spindle task states: */
typedef enum
{
     SPNDL_IDLE,
     SPNDL_INIT,
     SPNDL_STARTUP_PULSE_WIDTH_ON,      //duration 800 ms
     SPNDL_STARTUP_PULSE_WIDTH_OFF,     //duration 200 ms
     SPNDL_READY,                       //Initialized, ready for RUN command, not moving/paused, equivalent to TinyG SPINDLE_OFF
     SPNDL_RUNNING,                     //TinyG spindle_mode==RUNNING==GCode:M03|M04==CW |CCW rotation
     SPNDL_RAMPING,                     //TinyG soft_start ramp sequence equivalent
     SPNDL_PAUSED,                      //TinyG spindle_mode==PAUSED-- e.g. was RUNNING but an interlock was opened
     SPNDL_STATE_COUNT
}SpindleDrvStateE;

typedef enum {RAMPING_UP, RAMPING_DOWN} SpindleRampDirectionE;
static volatile uint16_t spindle_init_iteration_count=0;
static volatile uint16_t spindle_low_speed_setting=SPINDLE_LOW_SPEED_DUTY_CYCLE_SETTING;
static volatile uint16_t spindle_mid_speed_setting=SPINDLE_MID_SPEED_DUTY_CYCLE_SETTING;
static volatile uint16_t spindle_high_speed_setting=SPINDLE_HIGH_SPEED_DUTY_CYCLE_SETTING; 
static volatile uint16_t SPINDL_INIT_ITERATION_COUNT = 0;//3;//00;
volatile static  uint16_t spindle_duty_cycle_counts = SPINDLE_PWM_STARTUP_COUNT; 
volatile static  uint16_t spindle_idle_duty_cycle_counts = SPINDLE_PWM_STARTUP_COUNT;
volatile static SpindleDrvStateE spndl_state=SPNDL_INIT;
volatile static UINT32 systick_start_count=0;
volatile static UINT32 delta_systick_count=0; 
volatile static uint32_t timer_count, timer_count_save;
volatile static signed long mov_avg_sum =0;
volatile static signed long mov_avg_rpm = 0;//metric: 100 hz=6000 rpm for 2-pole motors, 3000 rpm for 4-poles
volatile static signed long mov_avg_max_val_rpm = 0;//metric: 100 hz=6000 rpm for 2-pole motors, 3000 rpm for 4-poles
volatile static signed long mov_avg_sample[MOV_AVG_SAMPLES_COUNT]; 
volatile static uint16_t sample_ndx=0;
volatile static signed long mov_avg_capture_cnt_per_second =0; 
volatile static uint16_t event_count=0;//increment on each event
volatile static uint16_t spillover_timer_counts=0;//attempt to sample event counts along one second intervals
volatile static int spindle_new_target_speed_rpm = 0;
volatile static int spindle_present_target_speed_rpm  = 0;
volatile static int16_t spindle_target_speed_pwm = 0;
volatile static Boolean_t spindle_new_target_rpm_pending_flag=false;
volatile static int16_t spindle_delta_target_rpm =0;//difference in present and new rpm setting
volatile static uint16_t spindle_ramp_step_count = 0;
volatile static uint16_t spindle_ramp_step_ndx = 0;//
volatile static SpindleRampDirectionE spindle_ramp_direction_e=RAMPING_UP;
volatile static int16_t spindle_ramp_final_step_increment = 0;//modulo remainder after evenly dividing ramp-to target rpm into n steps
//volatile static int16_t  spindle_ramp_step_increment = 0;//plus or minus whole step increment
volatile static int16_t  spindle_ramp_step_rpm_ramp_value =0;//accumulates new stap value every new time interval completion
volatile static uint32_t spindle_ramp_start_time_ms =0;
volatile static float spndl_min_duty_cycle, spndl_min_duty_cycle_saved;
volatile static float spndl_max_duty_cycle, spndl_max_duty_cycle_saved; 
volatile static float spndl_min_speed_rpm,  spndl_min_speed_rpm_saved;
volatile static float spndl_max_speed_rpm,  spndl_max_speed_rpm_saved;
volatile static float spndl_ramp_step_delay_ms = SPINDLE_MS_DLY_PER_RPM_INCR;
volatile static float spndl_ramp_rpm_increment = SPINDLE_RPM_STEP_INCREMENT;
volatile static Boolean_t pwm_calc_param_updated_flag = false;
volatile static float spindle_duty_cycle_fraction=0;
Boolean_t spindle_is_ramping(void){return (spndl_state ==SPNDL_RAMPING);}
volatile static uint16_t spndl_pwm_period_counts=0;
void spindle_enable_power(void){hal_write_pin(0,ESC_EN_PIN,(GPIO_PinState) 1);}
void spindle_disable_power(void){hal_write_pin(0,ESC_EN_PIN,(GPIO_PinState) 0);}
void spindle_startup_pwm(void)
{
    spindle_enable_power();
    PWM0_ChannelsStart(PWM_CHANNEL_3_MASK);
}

 
 /***********************************************************************************
 * 
 *  Function: spindle_set_frequency_pwm
 *
 *  Description: Applies frequency to pwm output hardware register
 *             
 *  Assumes:  Calling function obeys: freq in Hz.            
 *
 **********************************************************************************/            
int spindle_set_frequency_pwm(float freq)
{
  int status=0;

  // Update count based on new frequency
  //htim11.Instance->ARR = ((float)SPINDLE_PWM_TMR_BASE_FREQ_HZ / freq) - 1;

  return status;
} /* End Function */
 /***********************************************************************************
 * 
 *  Function: spindle_set_duty_count_pwm
 *
 *  Description: Applies the duty cycle count to pwm output hardware register
 *             
 *  Assumes:  Calling function obeys: duty_count is a timer count, NOT percent!            
 *
 **********************************************************************************/            
int spindle_set_duty_count_pwm(uint16_t new_duty_cycle_count)
{
  int status=0;
  // Update count based on new value
  PWM0_SetSpindleDutyCycleCounts(new_duty_cycle_count);
  return status;
} /* End Function */
 /***************************************************************************
 * 
 * Function:  spindle_ctrl_init 
 * 
 * Description:   
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
void spindle_ctrl_init(void)
{
  spindle_present_target_speed_rpm = 0;
  spndl_state = SPNDL_INIT;
  spindle_new_target_rpm_pending_flag = false;
  spindle_delta_target_rpm = 0; 
  spindle_ramp_step_count = 0;
  spindle_ramp_step_ndx = 0; 
  spindle_ramp_final_step_increment = 0;  
  spndl_pwm_period_counts=PWM0_GetSpindlePwmPeriodCounts();
  spndl_min_duty_cycle = pwm_get_min_duty_cycle();
  spndl_max_duty_cycle = pwm_get_max_duty_cycle();
  spndl_min_speed_rpm  = pwm_get_min_rpm();
  spndl_max_speed_rpm  = pwm_get_max_rpm();
  spndl_min_duty_cycle_saved = spndl_min_duty_cycle;
  spndl_max_duty_cycle_saved = spndl_max_duty_cycle;
  spndl_min_speed_rpm_saved =  spndl_min_speed_rpm; 
  spndl_max_speed_rpm_saved =  spndl_max_speed_rpm; 
  spndl_ramp_step_delay_ms = SPINDLE_MS_DLY_PER_RPM_INCR;
  spndl_ramp_rpm_increment = SPINDLE_RPM_STEP_INCREMENT;
  spindle_ramp_direction_e=RAMPING_UP;
  //spindle_ramp_step_increment = SPINDLE_RPM_STEP_INCREMENT;
  spindle_idle_duty_cycle_counts = SPINDLE_PWM_STARTUP_COUNT;
#if 0//to do reconcile
  /* Set spindle to initial frequency and duty cycle */
  spindle_set_frequency_pwm(pwm_get_frequency());
  spindle_set_duty_count_pwm(pwm_get_phase_off() * spndl_pwm_period_counts);
#endif
}/* End function */


/************************************************************************************
 *
 * Function: spndl_calculate_duty_cycle
 *
 * Description: Applies linear interpolation to arrive at a duty_cycle for the
 *              designated rpm
 * Note: The generic linear equation  y = mx +b where m = (Ymax-Ymin)/Xmax-Xmin), 
 *        is not used. Instead, Tiny g uses a modified slope called a lerpfactor
 *   Lerpfactor = (X-Xmin)/Xmax-Xmin) ^ K
 *    Y = Lerpfactor * (Ymax-Ymin) + Ymin
 *   
 *
 ***********************************************************************************/
int spndl_calculate_duty_cycle(float rpm)
{
    int status = 0;  
    volatile float range_rpm = spndl_max_speed_rpm - spndl_min_speed_rpm;
   
    if (fp_NOT_ZERO(range_rpm))
    {
       volatile float lerpfactor                  = (rpm-spndl_min_speed_rpm)/range_rpm;//Direct from TinyG, 
       volatile float lerpfactor_curved           = pow(lerpfactor,pwm_get_k_value()); 
       volatile float range_duty_cycle_counts     = (spndl_max_duty_cycle-spndl_min_duty_cycle)* spndl_pwm_period_counts; 
       volatile float min_duty_cycle_counts       = spndl_min_duty_cycle * spndl_pwm_period_counts; 
       volatile float calculated_duty_cycle_counts=(lerpfactor_curved*range_duty_cycle_counts)+min_duty_cycle_counts;
      
       /* Assign file scope variables: */
       spindle_duty_cycle_counts = (uint16_t)calculated_duty_cycle_counts;
       spindle_duty_cycle_fraction =(calculated_duty_cycle_counts/(float)spndl_pwm_period_counts);
    }
    else
    {
      status = -1; //to do: apply correct errorcode
    }  
   return status;
}

/* sme: 8-18-2022: in order to use the ramp, a direct duty_cycle assignment needs to be converted t an equivalent rpm*/
int spndl_rpm_from_duty_cycle(float duty_cycle)
{
    int status = 0;
    volatile float range_rpm = spndl_max_speed_rpm - spndl_min_speed_rpm;
    volatile float result = 0;
    volatile float duty_cycle_counts           = duty_cycle * spndl_pwm_period_counts;
    volatile float range_duty_cycle_counts     = (spndl_max_duty_cycle-spndl_min_duty_cycle)* spndl_pwm_period_counts;
    volatile float min_duty_cycle_counts       = spndl_min_duty_cycle * spndl_pwm_period_counts; 
    if (fp_NOT_ZERO(range_duty_cycle_counts))
    {
        volatile float lerpfactor  = (duty_cycle_counts-min_duty_cycle_counts)/range_duty_cycle_counts;//Direct from TinyG, 
        volatile float lerpfactor_curved = pow(lerpfactor,pwm_get_k_value()); 
        volatile float calculated_rpm=(lerpfactor_curved*range_rpm)+spndl_min_speed_rpm;
        spindle_set_speed_rpm(calculated_rpm);
    } 
    
    //volatile float lerp_factor=(duty_cycle-min_duty_cycle)/;
    //float delta_duty_cycle=
    return status;
}
/************************************************************************************
 *
 * Function: spindle_feedback
 *
 * Description: Encapsulates all feedback  
 *
 ***********************************************************************************/
 void spindle_feedback(void) 
 {
   volatile uint32_t esc_scale_factor = SPINDLE_ESC_RPM_PER_100HZ;
     /* Capture time between this and previous event */
     timer_count_save = timer_count;
//To do: integrate Chabo mcu pwm :      timer_count      = __HAL_TIM_GetCounter(htim);
    
     /* Accumulate events for over the 1 Hz sampling interval */
     event_count++;
    
     /* Check if at end of present one second sampling interval */
     if (timer_count >= SPINDLE_FEEDBACK_TMR_TICS_PER_SEC )
     {
         //hal_toggle_pin(GPIO_DBG2_GPIO_PortId,GPIO_DBG2_Pin );
         /* Apply any spillover to the next sample interval */
         spillover_timer_counts = timer_count - SPINDLE_FEEDBACK_TMR_TICS_PER_SEC ;
//To do: integrate Chabo mcu pwm :          __HAL_TIM_SetCounter(htim,spillover_timer_counts);

         /* Maintain the moving average: */
         mov_avg_sum += event_count;
         mov_avg_sum -= mov_avg_sample[sample_ndx];
         mov_avg_sample[sample_ndx++] = event_count;     
         if (sample_ndx >= MOV_AVG_SAMPLES_COUNT)
         {
            sample_ndx=0;     
         }
         
         mov_avg_capture_cnt_per_second = mov_avg_sum >> MOV_AVG_DIV_SHIFTS;   
         mov_avg_rpm = mov_avg_capture_cnt_per_second * SPINDLE_ESC_RPM_PER_100HZ; 
         /* moving avg is is surprisingly unsteady-look for a max, evenutally get the mode statistic */
         if (mov_avg_max_val_rpm<mov_avg_rpm)
         {
           mov_avg_max_val_rpm=mov_avg_rpm;
         }
         /* Reset the event count for the next sampling interval*/
         event_count = 0;

         /* Apply any spillover to the new sample interval data collection */
         if (spillover_timer_counts > 0 )
         {
             /* Deduct the spillover event from the completed sample interval data */
             if(mov_avg_sample[sample_ndx]>0)
             {
               mov_avg_sample[sample_ndx]--;
             }
             
             /* New sample interval gets this latest event */
             event_count = 1;
         }/* End if */                  
    }/* End if */
 }/* End Function */

 /***********************************************************************************
 * 
 *  Function: spindle_set_speed_pwm
 *
 *  Description: Applies rpm arg to rpm-to-pwm lookup table
 *             
 *               Applies rpm setting to duty cycle hardware register 
 *  Assumes:  Calling function obeys: spindle_mode == SPINDLE_RUNNING.            
 *
 **********************************************************************************/            
int spindle_set_speed_pwm(uint16_t rpm)
{
  int status=0;

  /* File scope var, duty_cycle_count is set in: */
  spndl_calculate_duty_cycle(rpm);
  spindle_target_speed_pwm = spindle_duty_cycle_counts;//file scope  is set in: spndl_calculate_duty_cycle    
   
  /* Set the new duty cycle */
  if (spindle_target_speed_pwm >= 0)
  {
       //pwm_refresh_duty_cycle_pct(spindle_duty_cycle_fraction);
      spindle_set_duty_count_pwm(spindle_target_speed_pwm);
  }
  else
  {  
     __NOP(); //error 
  }
   return status;
} /* End Function */

/************************************************************************************** 
*
*
*  Function: spindle_start_ramp
*
* Description: Called each time spindle speed is changed, including the initial setting.
*
* Assumptions: spindle_mode must be SPINDLE_RUNNING, and that is the responsibility 
* of the calling function
*
**************************************************************************************/
int spindle_start_ramp(void)
{
   int status = 0;  
   float spindle_ramp_step_increment = (int)g2_get_spindle_ramp_rpm_increment();//get latest--could change via JSON cmd
   /* The rpm ramp will either increment to a higher taarget rpm or decrement to a lower target rpm*/
   if(spindle_new_target_speed_rpm > spindle_present_target_speed_rpm )
   {
      if (spindle_present_target_speed_rpm < spndl_min_speed_rpm)
      {
        spindle_present_target_speed_rpm = (int)(spndl_min_speed_rpm-spindle_ramp_step_increment );
      }
      spindle_delta_target_rpm =(spindle_new_target_speed_rpm - spindle_present_target_speed_rpm );     
      spindle_ramp_step_rpm_ramp_value = spindle_present_target_speed_rpm  + spindle_ramp_step_increment;
      spindle_ramp_direction_e = RAMPING_UP;
   }
   else
   {
    //not here: spindle_ramp_step_increment = -SPINDLE_RPM_STEP_INCREMENT;
     spindle_delta_target_rpm =(spindle_present_target_speed_rpm - spindle_new_target_speed_rpm);  
     spindle_ramp_step_rpm_ramp_value = spindle_present_target_speed_rpm - spindle_ramp_step_increment;
     spindle_ramp_direction_e = RAMPING_DOWN;
   }
   if(spindle_delta_target_rpm > SPINDLE_MIN_RPM_SPEED_CHANGE)
   {
      /* Calculate the accel/decel ramp parameters */
      spindle_ramp_step_count= (uint16_t)(spindle_delta_target_rpm / spindle_ramp_step_increment); 
      spindle_ramp_final_step_increment = spindle_delta_target_rpm % (int)spindle_ramp_step_increment;
      if(spindle_ramp_final_step_increment>0)
      {
         spindle_ramp_step_count++;
      }
      spindle_ramp_step_ndx = 0;    
      spndl_state = SPNDL_RAMPING;
      spindle_ramp_start_time_ms = sys_get_time_ms();
      spindle_set_speed_pwm(spindle_ramp_step_rpm_ramp_value);     
   }
   else
   {
     status=-1;//ERR_SPINDLE_SPEED_CHANGE_TOO_SMALL;
   }
   return status;
}/* End function */
/**************************************************************************
 * 
 * Function: spindle_set_speed_rpm(float value) 
 *
 * Description:  Range checks, corrects rpm setting and
 *              translates to duty cycle.
 *  Returns: Error code if range error,   0 otherwise            
 *
 **************************************************************************/
 int spindle_set_speed_rpm(float value)
 {
   int status = 0;

   /* Set spindle target speed */
   spindle_new_target_speed_rpm = (uint16_t)value;
   
   /* Alert the spindle_ctrl task of new speed value */
   spindle_new_target_rpm_pending_flag=true;
 
   return status;
 }/* End function */
 
 
/************************************************************************************
 *
 * Function: spndl_refresh_pwm_calc_params
 *
 * Description:a usr can use JSON commands to change the parameters
 * used to perform linear interpolation for calculating a pwm duty cycle
 * that corresponds to a designated target rpm
 *
 **********************************************************************************/
int spndl_refresh_pwm_calc_params(void)
{
  int status = 0;
  #define NUM_RANGE_CHECKS 4
  BOOL fault_flag[NUM_RANGE_CHECKS] = {0,0,0,0};
  BOOL change_flag = false;
  
  /* Retrieve JSON-set persistance values: min, max: rpm, duty cycle, and range check these */
  spndl_min_duty_cycle = pwm_calc_get_min_duty_cycle();
  if (spndl_min_duty_cycle_saved != spndl_min_duty_cycle  )
  {
     spndl_min_duty_cycle_saved = spndl_min_duty_cycle;
     change_flag = true;
  }
  
  spndl_max_duty_cycle = pwm_calc_get_max_duty_cycle();
  if (spndl_max_duty_cycle_saved != spndl_max_duty_cycle )
  {
    spndl_max_duty_cycle_saved = spndl_max_duty_cycle;
    change_flag = true;    
  }
    
  spndl_min_speed_rpm = pwm_calc_get_min_rpm();
  if (spndl_min_speed_rpm_saved  != spndl_min_speed_rpm)
  {
    spndl_min_speed_rpm_saved  = spndl_min_speed_rpm;
    change_flag = true;
  }  
  
  spndl_max_speed_rpm = pwm_calc_get_max_rpm();
  if (spndl_max_speed_rpm_saved != spndl_max_speed_rpm)
  {
    spndl_max_speed_rpm_saved = spndl_max_speed_rpm;
    change_flag = true;
  }  
  if ((fault_flag[0]==true) 
  ||  (fault_flag[1]==true) 
  ||  (fault_flag[2]==true) 
  ||  (fault_flag[3]==true)) 
  {
    status = -1;//ERR_RANGE_FAULT_JSON_CMD_PWM_CALC_PARAMS;//to do: provide a mnemonic err code    
  }
  /* Provide a breakpoint, and allow for additional processing with persistent flag */
  if (change_flag == true)
  {
    pwm_calc_param_updated_flag=true;
  }
  return status;
}
/************************************************************************************
 *
 * Function: spindle_execute_ramp_step
 *
 * Description: Iterates the spindle accell/decel ramp function until 
 *  all individual sequential ramp stages leading to target rpm have been performed
 *  (Note: this is performed open loop
 *
 **********************************************************************************/
Boolean_t spindle_execute_ramp_step(void)
{ 
  Boolean_t completed_flag = false;
  float spindle_ramp_step_increment = (int)g2_get_spindle_ramp_rpm_increment();//get latest--could change via JSON cmd
  bool ramp_step_dwell_time_completed_flag = false;
  /* The ramp step increment will be negative when the target rpm is less than the present*/
  if (spindle_ramp_direction_e == RAMPING_DOWN)
  {
    spindle_ramp_step_increment = -spindle_ramp_step_increment;  
  } 
  /* Poll for completion of dwell time for present ramp step: */
  ramp_step_dwell_time_completed_flag = sys_time_ms_limit_reached(spindle_ramp_start_time_ms,
                                                                  (int)spndl_ramp_step_delay_ms);
  if (ramp_step_dwell_time_completed_flag == true)
  {
       /* Perform next ramp step */
       spindle_ramp_step_ndx++;//note: start ramp executed the initial ramp ste so the 0th ndx has aalready completed
       if (spindle_ramp_step_ndx >= spindle_ramp_step_count)
       {
         /*Re-adjust the pre-adjusted present target rpm to stay with the final target */
         spindle_present_target_speed_rpm  = spindle_new_target_speed_rpm;//-spindle_ramp_step_increment;
         spindle.speed_actual=spindle_present_target_speed_rpm;
         completed_flag = true;
       }
       else /* There are more ramp steps to perform */
       {
          if(spindle_ramp_step_ndx < spindle_ramp_step_count-1) 
          {
            spindle_ramp_step_rpm_ramp_value += spindle_ramp_step_increment;
          }
          else 
          {    /* handle special case of final step being a remainder amount, from integer division */
               if(spindle_ramp_final_step_increment>0)
               {
                  spindle_ramp_step_rpm_ramp_value += spindle_ramp_final_step_increment;
               }
               else
               {
                  spindle_ramp_step_rpm_ramp_value += spindle_ramp_step_increment;
               }
          }/* End else executing final step */
          /* Update the actual speed*/
          spindle.speed_actual=spindle_ramp_step_rpm_ramp_value;
          /* Set the next step's pwm duty cycle: */
          spindle_set_speed_pwm(spindle_ramp_step_rpm_ramp_value);
        
          /* Restart the step dwell timer for next ramp step: */
          spindle_ramp_start_time_ms = sys_get_time_ms();
       }/*End else there are more ramp steps to perform */
  }
  return completed_flag;
}/* End function */


 /****************************************************************************
 * 
 * Function:  spindle_ctrl_task 
 * 
 * Description:   
 *
 * Assumptions/Requirement: 
 *
 *****************************************************************************/
void spindle_ctrl_task(void)
{
  volatile int status = 0;
  volatile static Boolean_t spindle_new_target_duty_cycle_flag=false; 
  volatile static uint16_t g2_applied_spindle_speed_rpm=0;
  volatile static Boolean_t spindle_ramp_completed_flag=false;
  
  /* Poll for any changes in tinyG2 spindle_mode and spindle_speed state variables */
  volatile uint8_t g2_spindle_mode = g2_get_spindle_mode();
  volatile uint16_t g2_latest_spindle_speed_rpm = g2_get_spindle_speed_rpm(); 
  static float previous_duty_cycle_setting = -1;
  static uint16_t previous_spindle_speed_rpm = -1;
    /* Apply new speed from G2 state machine if changed */
  if (previous_spindle_speed_rpm != g2_latest_spindle_speed_rpm)
  {
    spindle_set_speed_rpm(g2_latest_spindle_speed_rpm);
    previous_spindle_speed_rpm = g2_latest_spindle_speed_rpm;
  }

  /* Poll for user input for duty cycle setting: */
  float duty_cycle_setting = pwm_get_duty_cycle();


  /* Flag new inut value when as needed */
  if ( previous_duty_cycle_setting != duty_cycle_setting )
  {
     previous_duty_cycle_setting = duty_cycle_setting;
     spindle_duty_cycle_counts = (int)(duty_cycle_setting * spndl_pwm_period_counts);
     spindle_new_target_duty_cycle_flag = true;
  }
  
  if((status=spndl_refresh_pwm_calc_params())!= ERR_NONE)//sets filescope flag, pwm_calc_param_updated_flag
  {
    __NOP(); 
  }
  if (pwm_calc_param_updated_flag == true)
  { 
    __NOP(); 
  }
  
  /* Execute the task's state machine:  */
  switch(spndl_state)
  {
   case SPNDL_IDLE:       
         break;
     
     case SPNDL_INIT:  
          /* Start Spindle PWM Timer generator */
          spindle_duty_cycle_counts = (pwm_get_phase_off() * spndl_pwm_period_counts); //"Off" == init/idle  ==10% duty cycle
          systick_start_count =sys_get_time_ms();
          spndl_state = SPNDL_STARTUP_PULSE_WIDTH_ON;
          break;
       
     case SPNDL_STARTUP_PULSE_WIDTH_ON:
           delta_systick_count=sys_get_time_ms()-systick_start_count;
          
          /* Maintain  "Initialization setting" duty cycle for 800 ms */
          if (delta_systick_count >=  SPINDLE_INIT_ON_DURATION_MS)
          {
             systick_start_count = sys_get_time_ms();
#define SPINDL_INIT_ITERATION_COUNT 0 //refine per observation
 
           if (spindle_init_iteration_count++ > SPINDL_INIT_ITERATION_COUNT)
           {
                spndl_state = SPNDL_READY;                 
           }
           else
           {
             systick_start_count =  sys_get_time_ms();
             spindle_set_duty_count_pwm(spindle_duty_cycle_counts);
             spindle_new_target_duty_cycle_flag = false;
             spindle_new_target_rpm_pending_flag= false;
             spndl_state = SPNDL_STARTUP_PULSE_WIDTH_ON; 
           }
         }
         break;
   
   case SPNDL_READY:

     if (g2_spindle_mode == SPINDLE_RUNNING)
     {
       spndl_state = SPNDL_RUNNING;
     }
    
      break;
      
    case SPNDL_RUNNING:

     /* Check for change of spindle state: */
     if (( g2_spindle_mode == SPINDLE_PAUSED)
     ||  ( g2_spindle_mode == SPINDLE_OFF))
     {  
        /* Spindle present target speed and duty cycle get zeroed */ 
        spindle_present_target_speed_rpm =0;
        spindle_duty_cycle_counts = spindle_idle_duty_cycle_counts;//SPINDLE_PWM_STARTUP_COUNT;
        spindle_set_duty_count_pwm(spindle_duty_cycle_counts);        
        /* Set next state: */
        spndl_state = SPNDL_READY;//default---change to pause as needed below
        if ( g2_spindle_mode == SPINDLE_PAUSED)
        {          
          spndl_state = SPNDL_PAUSED;
        }
        
     }
     else if (spindle_new_target_duty_cycle_flag==true)
     {
        ///spindle_set_duty_count_pwm(spindle_duty_cycle_counts);
        spndl_rpm_from_duty_cycle(duty_cycle_setting);
        spindle_new_target_duty_cycle_flag=false;
        spindle_start_ramp();
        spndl_state =SPNDL_RAMPING;
     }
     /* If spindle is RUNNING, check for new speed setting: */
     else if((spindle_new_target_rpm_pending_flag==true)||(spindle.speed_actual==0))
     {
        spindle_new_target_rpm_pending_flag=false;
        spindle_start_ramp();
        spndl_state =SPNDL_RAMPING;
     }

   
      break; 
   case SPNDL_PAUSED:
      if ( g2_spindle_mode == SPINDLE_RUNNING)
      {  
         /* Resume running by starting back up from 0 rpm to the latest speed setting
          *  which is retained in the variable:spindle_new_target_rpm_pending_flag
          *  even if it isns't very "new"
          */
         spindle_new_target_rpm_pending_flag=true;
         spndl_state = SPNDL_RUNNING;         
      }
      break;
      
   case SPNDL_RAMPING:
     /* Iteratively execute the ramp function until all ramp steps are completed: */
     spindle_ramp_completed_flag=spindle_execute_ramp_step();
     if (spindle_ramp_completed_flag == true )
     {
       /* Next state: */
       spndl_state = SPNDL_RUNNING;
     }
     break;
    
   default:
     break;
     
   }/* End switch*/
 
}/* End Task */
 