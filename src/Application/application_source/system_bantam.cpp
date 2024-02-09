/*******************************************************************************
* File:    system.c
*
* Description: 
*
*   provides time services and other system scope oversight
*
*******************************************************************************/
#include "main.h"
#include "bantam_hal.h"
#include "user_types.h"
#include "globdefs.h"
#include "system_bantam.h"
#include "kernel.h"

/* Defines when using usec granularity system timer interrupt: */
#define SYSTEM_USEC_X100_PER_MS 10
#define SYSTEM_USEC_X1000_PER_MS 1 

/* Defines when using accumulated times, msec and seconds granularity: */
#define SYSTEM_MSEC_PER_SEC 1000
#define SYSTEM_SEC_PER_MIN 60

/* Allocate time units to provide to time services client tasks */
static volatile uint32_t system_time_ms = 0;    //16 bit range: 0..999
static volatile uint32_t system_time_x100_us = 0;//or tenths of ms increments
static volatile uint32_t system_time_seconds = 0; //range 0..59
static volatile uint32_t system_time_minutes = 0; //range 0..59

/* Allocate internal divider counters for accumulating various time units */
static volatile uint16_t system_usec_x100_count=0;
static volatile uint16_t system_kernel_interrupt_count=0;
static volatile uint16_t system_msec_count=0;
static volatile uint16_t system_seconds_count=0;

   /***********************************************************************************
  *
  * Function: get_usec_elapsed_time
  * Description:  provided for precision timing when developing the encoder functions
  *
  ************************************************************************************/
  uint16_t get_usec_elapsed_time(void)
  {  
     volatile uint16_t timer_count;
     return   timer_count;            
  }
/***********************************************************************************
  *
  * Function:start timer from zero
  * Description: 
  *
  ************************************************************************************/
  void start_usec_timer(void)
  {   
  }
/***************************************************************************
 * 
 * Function:  sys_blocking_delay_ms 
 * 
 * Description: Blocks execution, expected usage limited to pre-kernel launch
 *              or during system initialization
 * 
 * 
 ****************************************************************************/
 void sys_blocking_delay_ms(uint32_t delay_ms) 
 {
     uint32_t start_time_ms=system_time_ms;
     uint32_t end_time_ms=start_time_ms+delay_ms;
     
     while(system_time_ms<end_time_ms)
     {
         __NOP();
     }
 }
/***************************************************************************
 * 
 * Function:  sys_get_time_ms 
 * 
 * Description:   returns the accumlated ms since system init
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
unsigned long sys_get_time_ms(void)
{
#if 1//10-18-2022-- strange apparent corruption when called in sr_status_report_callback(
return system_time_ms;
#else
 unsigned long result=system_time_ms;
  return result;
#endif
}

/***************************************************************************
 * 
 * Function:  sys_get_delta_time_ms 
 * 
 * Description:   returns the delta time ms since system prior saved time
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
short sys_get_delta_time_ms(uint32_t start_time_arg)
{
  short delta =system_time_ms-start_time_arg;
  if (delta<0)
  {
    //should never happen!
    delta=0;
  }
  return delta;
}

/***************************************************************************
 * 
 * Function:  sys_time_ms_limit_reached 
 * 
 * Description:   returns true when delta(present_time,start_time_arg) 
 *  >=time_limit arg
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
Boolean_t sys_time_ms_limit_reached(uint32_t start_time, int32_t time_limit)
{  
  volatile uint32_t elapsed_time = system_time_ms-start_time;
  Boolean_t result= elapsed_time >= time_limit ;
  return result;
}
/***************************************************************************
 * 
 * Function:  sys_time_ms_time_remaining 
 * 
 * Description:   returns int time remaining to reach limit
 *  
 * 
 ****************************************************************************/
int sys_time_ms_time_remaining(uint32_t start_time, int32_t time_limit)
{  
  int32_t elapsed_time = system_time_ms-start_time;
  int32_t remaining_time_ms = time_limit-elapsed_time;  
  if (remaining_time_ms<0)
  {
    remaining_time_ms=0;//clamp to zero
  }
  return remaining_time_ms;
}
/***************************************************************************
 * 
 * Function:   systick_interrupt_callback_helper
 * 
 * Description:  
*     Triggered by SystTick Interrupt  at ISR_PERIOD_INCREMENT
*      ms time intervals.
*      calls: Kernel_"interrupt"
 * 
 ****************************************************************************/
void systick_interrupt_callback_helper(void)
{
  #ifdef DEPLOY_KERNEL_FROM_1KHZ_SYSTICK_INTERRUPT
 
   /* Call the kernel interrupt every other ms, to match the duration of a kernel slot */
   #define KERNEL_INT_CALL_INTERVAL (KERNEL_ONE_SCHEDULING_INCREMENT_MS-1)  
   if(system_kernel_interrupt_count++ >=  KERNEL_INT_CALL_INTERVAL ) 
   {   
       // hal_toggle_pin(GPIO_DBG2_GPIO_PortId,GPIO_DBG2_Pin); //Confirmed: 250Hz toggle rate, 500 Hz entry rate 
       kernel_interrupt();
       system_kernel_interrupt_count=0;
   }
 
#endif   
   /* Update elapsed msec count, used in:sys_get_time_ms(), sys_time_ms_limit_reached() functions*/
   system_time_ms++;
}

/***************************************************************************
 * 
 * Function:  system_init 
 * 
 * Description:   
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
void system_init(void)
{
  system_time_ms = 0;
  system_time_x100_us = 0; 
  system_time_seconds = 0;
  system_time_minutes = 0;
  system_usec_x100_count=0;
  system_msec_count=0;
  system_seconds_count=0;
}
#ifdef DEPLOY_HI_RESOLUTION_SYSTIMER
/****************************************************************************
 * 
 * Function:  system_timer_interrupt
 * 
 * Description:  called from tim14 interrupt 

 * Assumptions/Requirement: 

 *****************************************************************************/
void system_timer_interrupt(void)
{
//To do: re-visit using a more finely resolved system timer interrupt (TIM14) rather than 1 KHz Systick  
#ifndef DEPLOY_KERNEL_FROM_1KHZ_SYSTICK_INTERRUPT

    system_time_x100_us++;
    system_usec_x100_count++;
    system_kernel_interrupt_count++;
    
    //hal_toggle_pin(GPIO_DBG2_GPIO_PortId,GPIO_DBG2_Pin ); //102 usec period measured
    
    /* Call Kernel interrupt per timing interval: */
   if(system_kernel_interrupt_count>=
#ifdef DEPLOY_KERNEL_FROM_1KHZ_TIM14_INTERRUPT//DEPLOY_KERNEL_FROM_10KHZ_TIM14_INTERRUPT    
    KERNEL_SLOT_DURATION_x1000_USECS)
#else   
    KERNEL_SLOT_DURATION_x100_USECS)
#endif
    {     
       //hal_toggle_pin(GPIO_DBG2_GPIO_PortId,GPIO_DBG2_Pin );//204 usec period measured 
       kernel_interrupt();
       system_kernel_interrupt_count=0;
    }    
    /* Update system time counters: */
    if(system_usec_x100_count >= SYSTEM_USEC_X1000_PER_MS)//SYSTEM_USEC_X100_PER_MS)
    {
       system_usec_x100_count=0;
       system_time_ms++;
       system_msec_count++;
      
       if (system_msec_count >= SYSTEM_MSEC_PER_SEC)
       {
           system_msec_count=0;
           system_time_seconds++;
           system_seconds_count++;
           
           if (system_seconds_count>=SYSTEM_SEC_PER_MIN)
           {
             system_seconds_count=0;
             system_time_minutes++;
           } 
        
       }/* End if */
    }/* End if */
#endif
}/* End Function */

#endif
