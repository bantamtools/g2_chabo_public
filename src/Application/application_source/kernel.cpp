/******************************************************************
* 
* Module:   kernel.c
*
* $Revision: 1.2 $
* $Date:   $
* Release Notes:
******************************************************************/
#include "main.h"
#include "bantam_hal.h"
#include "user_types.h"
#include "globdefs.h"
#include "time.h"
#include "system_bantam.h"
#include "kernel.h"     
#include "tasks.h"


/* Enumerate the states for a task */
typedef enum
{
   SUSPENDED,
   ASLEEP,
   READY,
   RUNNING
} KernelTaskStateE;

#ifdef DEPLOY_KERNEL_TASK_FAULT_STATUS
typedef enum
{
  FAULT_NONE,
  FAULT_OVERRAN_TIMESLOT,//kernel detects this at kernel interrupt time--will repeat into each slot that is overrun
  FAULT_OVERRAN_TIMESLOT_AT_CHECKOUT,//In addition to above, the time of checkout is compared against one slot period.
  FAULT_STARVED_BY_AN_OVERRUN,
  FAULT_CHECKIN_INVALID_STATE,
  FAULT_CHECKOUT_INVALID_STATE,
  FAULT_COUNT
}KernelTaskFaultE;

typedef struct
{
 TaskIdE taskid;
 KernelTaskFaultE fault;
 FourByteDataU data;
 time_t timestamp;
 //uint32_t incidence_count; 
}KernelTaskFaultInfoS;
#define KERNEL_TASK_FAULT_LOG_SIZE 1024//arbitrary, adjust as needed
typedef struct
{
  KernelTaskFaultInfoS  buf[KERNEL_TASK_FAULT_LOG_SIZE];
  short write_ndx;
  bool overwrite_flag; 
}KernelFaultLogS;
KernelFaultLogS kernel_fault_log; 

   volatile const char *probe_prog_str[PROBE_PROG_COUNT]=
   {
       "IDLE",
       "INIT",
       "START",
       "MOVE",
       "COMPLETED",
       "REPORT"
   };
//=====================================================================
//
// Function:  kernel_log_fault
//
// Description:  
//             
// =====================================================================
void kernel_log_fault( TaskIdE taskid, KernelTaskFaultE fault,FourByteDataU data )
{
  kernel_fault_log.buf[kernel_fault_log.write_ndx].taskid = taskid;
  kernel_fault_log.buf[kernel_fault_log.write_ndx].fault = fault;
  kernel_fault_log.buf[kernel_fault_log.write_ndx].data = data;
  kernel_fault_log.buf[kernel_fault_log.write_ndx].timestamp =sys_get_time_ms();
  kernel_fault_log.write_ndx++;
  if (kernel_fault_log.write_ndx>=KERNEL_TASK_FAULT_LOG_SIZE)
  {
    kernel_fault_log.write_ndx=0;
    //todo: send out message stating fault log is about to be overwritten
    kernel_fault_log.overwrite_flag=true;
    
    //by the next entry
  }
}

#ifdef DEPLOY_KERNEL_LOG_REPORTING
int kernel_get_log_entry_count(void){return kernel_fault_log.write_ndx;}
//=====================================================================
//
// Function:  kernel_serialize_logfile
//
// Description: Uses sprintf to print out the contents of the 
//              kernel fault log in JSON format, similar to json_serialize
//              but independent from nv object paradigm
//  Assumes: log has not overwritten itself--has not looped around past ndx 0.
//
//  Returns: number of log entries written out           
// =====================================================================
int kernel_serialize_logfile( char *out_buf, uint16_t size) 
{
    int num_log_entries = kernel_fault_log.write_ndx;
    char *str = out_buf;
    int str_max = size-sizeof(KernelTaskFaultInfoS);
    int str_len_accum = 0;
    int i;
    uint32_t *u32_dataptr=0;
    /* if log has been overwritten, write out the entire log. 
     *  Only the oldest entries: 0 to kernel_fault_log.write_ndx
     *  have been overwritten
    */
    if (kernel_fault_log.overwrite_flag==true)
    {
      num_log_entries=KERNEL_TASK_FAULT_LOG_SIZE;
    }
    
    /* start with an empty outbuf: */
    memset(out_buf,0,size);
    
    /*  produce the invariant header:*/
    str += sprintf(str, "{\"sr\": {\"kfl\": {");

    /* write out each log entry in hex byte format */
    if (num_log_entries>0) 
    {
        for (i=0;i<num_log_entries; i++)
        {
          u32_dataptr=(uint32_t*)&kernel_fault_log.buf[i].data;
          str += sprintf(str,"\"%d\" : \"0x%02x%02x%08lx%08lx\" ",
                         i,kernel_fault_log.buf[i].taskid,
                         kernel_fault_log.buf[i].fault,
                         *u32_dataptr,// eliminates compiler warning kernel_fault_log.buf[i].data,
                         (uint32_t)kernel_fault_log.buf[i].timestamp);
          
          /* prevent cs.out_buf overrun.*/
          str_len_accum=strlen(str);
          if (str_len_accum>=str_max)
          {
            break;
          }
          /* Comma separate all be the last log entry*/
          if (i<kernel_fault_log.write_ndx-1)
          {
             *str++ = ',';
          }         
        } 
        /* terminate the message */
        str += sprintf(str,"}}}\n\\0");
    }

    /* send the message */
    comms_mgr_write_msg(out_buf);
    return num_log_entries;
}
#endif

#endif
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT
typedef enum
{
  CHECK_UNDEFINED=0,
  CHECK_IN =0xAA55,
  CHECK_OUT=0x55AA,
  CHECK_COUNT =3 //defines three states, not in simple enum 0,1, 2,...order  
}KernelTaskCheckInE;
#endif
// Define the task control block
typedef struct
{
  KernelTaskStateE state;//current state: asleep,running, suspended
#ifdef DEPLOY_KERNEL_TASK_FAULT_STATUS
  KernelTaskFaultE fault_status;
#endif
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT
  KernelTaskCheckInE check_status;
#endif
  
#ifdef DEPLOY_KERNEL_TASK_TIMING
uint32_t start_time_ms;
uint16_t delta_time_ms;
uint16_t min_time_ms;
uint16_t max_time_ms;
uint16_t run_time_us;
uint16_t min_time_us;
uint16_t max_time_us;
#endif

  SINT16 sleep_count; // e.g sched increment downto 0
  UINT16 sleep_interval;//e.g 112 ms for 9 Hz interval
  KernelSlotIdE slot_id; // the slot this task will execute in
#ifdef KERNEL_ALLOW_MULTISLOTS_PER_TASK
  bool extra_slot;
#endif
  void (*taskptr)(void); //pointer to task function name
} KernelTaskControlBlockS;
 static volatile Boolean_t kernel_enable_int_flag = true;
 

//To do: implement below for tim14 interrupt
//if tim14 does system time, then don't literally disable tim14, simply set a flag 
//for tim14 interrupt to skip over calling the kernel interrupt
 
#ifndef DISABLE_KERNEL_INTERRUPT 
#define DISABLE_KERNEL_INTERRUPT() kernel_enable_int_flag =false;
#endif

#ifndef ENABLE_KERNEL_INTERRUPT 
#define ENABLE_KERNEL_INTERRUPT() kernel_enable_int_flag =true;
#endif

static KernelTaskControlBlockS kernel_task_control_block[TASK_ID_COUNT];
static /*KernelSlotIdE*/ int kernel_int_slot; 
static /*KernelSlotIdE*/ int kernel_saved_int_slot=-1; 
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING 
static int sched_running_task_slot_id=0; 
static TaskIdE sched_taskid_e = (TaskIdE) 0;
static TaskIdE select_task_id_e=(TaskIdE) 0;
#endif
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT
void kernel_task_check_in(void);
void kernel_task_check_out(void);
#endif
// Prototypes:
void kernel_init(void);
void kernel_init_task_control_block(void);
void kernel_ready_task(TaskIdE id);
void kernel_suspend_task(TaskIdE id);
BOOL kernel_task_is_suspended(TaskIdE id);
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
//=====================================================================
//
// Function:  kernel_task_check_in
//
// Description: Called by task at start of it's execution. 
//             Starts execution timer
//             Checks for abnormal ceck in 
// Precondition: check status must be checked out or idle.
//               otherwise fault is assigned
//
// =====================================================================
void kernel_task_check_in(void)
{
 FourByteDataU data;
  KernelTaskControlBlockS *task_ctrl_blck_ptr=&kernel_task_control_block[sched_taskid_e];
  
  /* Start execution timer  */
  start_usec_timer();
  task_ctrl_blck_ptr->start_time_ms=sys_get_time_ms();
 
  /* Check for abnormal checkin  */
  if (task_ctrl_blck_ptr->check_status==CHECK_IN)
  {
    task_ctrl_blck_ptr->fault_status= FAULT_CHECKIN_INVALID_STATE;
    data.u16_data[0]=task_ctrl_blck_ptr->check_status;
    kernel_log_fault(sched_taskid_e,FAULT_CHECKIN_INVALID_STATE,data);
  }
  /* Check in */
  task_ctrl_blck_ptr->check_status=CHECK_IN;
}
//=====================================================================
//
// Function:  kernel_task_check_out
//
// Description: Called by task on completion of execution.
//              Tracks execution time and maintains min, max stats
// Precondition: Check status must be checked in
//               otherwise fault is assigned
//
// =====================================================================
void kernel_task_check_out(void) 
{
   FourByteDataU data;
  /* Track execution time, gather stats on min, max times*/
   KernelTaskControlBlockS *task_ctrl_blck_ptr=&kernel_task_control_block[sched_taskid_e];
 
   task_ctrl_blck_ptr->delta_time_ms = sys_get_delta_time_ms(task_ctrl_blck_ptr->start_time_ms);
   task_ctrl_blck_ptr->run_time_us=get_usec_elapsed_time();
   
   /* do msec stats: */
   if (task_ctrl_blck_ptr->delta_time_ms<task_ctrl_blck_ptr->min_time_ms)
   {
     task_ctrl_blck_ptr->min_time_ms=task_ctrl_blck_ptr->delta_time_ms;
   }
   if (task_ctrl_blck_ptr->delta_time_ms>task_ctrl_blck_ptr->max_time_ms)
   {
     task_ctrl_blck_ptr->max_time_ms=task_ctrl_blck_ptr->delta_time_ms;
   }
  
  /* do usec stats: */
  if (task_ctrl_blck_ptr->run_time_us<task_ctrl_blck_ptr->min_time_us)
   {
     task_ctrl_blck_ptr->min_time_us=task_ctrl_blck_ptr->run_time_us;
   }
   if (task_ctrl_blck_ptr->run_time_us>task_ctrl_blck_ptr->max_time_us)
   {
     task_ctrl_blck_ptr->max_time_us=task_ctrl_blck_ptr->run_time_us;
   }
   if ((task_ctrl_blck_ptr->delta_time_ms>=KERNEL_SLOT_DURATION_MS)
   ||(task_ctrl_blck_ptr->run_time_us>(KERNEL_SLOT_DURATION_MS*1000)))
   {
#define USEC_NDX 0
#define MSEC_NDX 1
     data.u16_data[USEC_NDX]=task_ctrl_blck_ptr->run_time_us;
     data.u16_data[MSEC_NDX]=task_ctrl_blck_ptr->delta_time_ms;
#ifdef KERNEL_ALLOW_MULTISLOTS_PER_TASK
     if ((task_ctrl_blck_ptr->extra_slot!=true) 
     || (task_ctrl_blck_ptr->delta_time_ms >(KERNEL_SLOT_DURATION_MS*2)))
#endif
         kernel_log_fault(sched_taskid_e,FAULT_OVERRAN_TIMESLOT_AT_CHECKOUT,data);
   }
   /* Check for invalid checkout state:*/
   if (task_ctrl_blck_ptr->check_status!=CHECK_IN)  
    {
      task_ctrl_blck_ptr->fault_status= FAULT_CHECKOUT_INVALID_STATE;
      kernel_log_fault(sched_taskid_e,FAULT_CHECKIN_INVALID_STATE,data);
    }
    /* Apply checkout action*/
    task_ctrl_blck_ptr->check_status=CHECK_OUT;  
}
#endif
//=====================================================================
//
// Function:  kernel_init
//
// Description:Init file scope variables including the task control block
//
// =====================================================================
void kernel_init(void)
{
  kernel_enable_int_flag = true;
 
   // Init task control block
   kernel_init_task_control_block();
   
   // Init other variables
   kernel_int_slot = SLOT_0;

}// End Function

//=====================================================================
//
//Function: kernel_ready_task
//
// Description:
//
// =====================================================================
void kernel_ready_task(TaskIdE id)
{
    kernel_task_control_block[id].sleep_count = 
    kernel_task_control_block[id].sleep_interval;
    
    kernel_task_control_block[id].state  = READY;
}// End Function

//=====================================================================
//
//Function:kernel_suspend_task
//
// Description:
//
// =====================================================================
void kernel_suspend_task(TaskIdE id)
{
    kernel_task_control_block[id].state  = SUSPENDED;
}// End Function

//=====================================================================
//
//Function: kernel_task_is_suspended
//
// Description: Returns True if task not sleeping, ready or running
//              but actually suspended.
//
// =====================================================================
BOOL kernel_task_is_suspended(TaskIdE id)
{
    return (kernel_task_control_block[id].state == SUSPENDED);
}// End Function


//==============================================================================
// Initialization
//==============================================================================

//==============================================================================
//
// Function:  init_task_control_block
//
// Description:
//
// Returns:
//
//==============================================================================
void kernel_init_task_control_block(void)
{
    int taskid;
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING
    /* Adding memset here allows init of new field in task control block under the scope of just one condition compile flag in one place*/    
    memset((void*)&kernel_task_control_block[0], 0, sizeof(kernel_task_control_block));
#define INIT_MIN_MSECS 0xFFFF
#define INIT_MIN_USECS 0xFFFF
#endif
    // Common inits
    for (taskid = 0; taskid < TASK_ID_COUNT;taskid++)
    {
        kernel_task_control_block[taskid].state   = SUSPENDED;
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING        
        kernel_task_control_block[taskid].min_time_ms=INIT_MIN_MSECS;
        kernel_task_control_block[taskid].min_time_us=INIT_MIN_USECS;
#endif        
    }
// Specific initialization, by slots
    //=SLOT 0 ==================================================      
    kernel_task_control_block[TASK_COMMS_MGR].state                           = ASLEEP;
    kernel_task_control_block[TASK_COMMS_MGR].sleep_count                     = KERNEL_SCHED_10_MS_PERIOD;//10-27-2022 KERNEL_SCHED_30_MS_PERIOD;
    kernel_task_control_block[TASK_COMMS_MGR].sleep_interval                  = KERNEL_SCHED_10_MS_PERIOD;//10-27-2022 KERNEL_SCHED_30_MS_PERIOD;
    kernel_task_control_block[TASK_COMMS_MGR].slot_id                         = SLOT_0;
    kernel_task_control_block[TASK_COMMS_MGR].taskptr                         = comms_mgr_task;
 
    kernel_task_control_block[TASK_MAIN_CONTROLLER].state                   = ASLEEP;   
    kernel_task_control_block[TASK_MAIN_CONTROLLER].sleep_count             = KERNEL_SCHED_10_MS_PERIOD;
    kernel_task_control_block[TASK_MAIN_CONTROLLER].sleep_interval          = KERNEL_SCHED_10_MS_PERIOD;
    kernel_task_control_block[TASK_MAIN_CONTROLLER].slot_id                 = SLOT_0;    
#ifdef KERNEL_ALLOW_MULTISLOTS_PER_TASK
    kernel_task_control_block[TASK_MAIN_CONTROLLER].extra_slot              = true;
#endif    
    kernel_task_control_block[TASK_MAIN_CONTROLLER].taskptr                 = main_controller_task;
 
    //=SLOT 1 ==================================================  
    //per KERNEL_ALLOW_MULTISLOTS_PER_TASK, slot 1 is reserved for spillover of slot 0 by main controller 
#ifndef CHABO_USE_MATTS_TMC2660_INIT
    //=SLOT 2 ==================================================            
    kernel_task_control_block[TASK_MOTION_CTRL].state                       = ASLEEP;  
    kernel_task_control_block[TASK_MOTION_CTRL].sleep_count                 = KERNEL_SCHED_10_MS_PERIOD;
    kernel_task_control_block[TASK_MOTION_CTRL].sleep_interval              = KERNEL_SCHED_10_MS_PERIOD;
    kernel_task_control_block[TASK_MOTION_CTRL].slot_id                     = SLOT_2;
    kernel_task_control_block[TASK_MOTION_CTRL].taskptr                     = motion_ctrl_task;    
#endif

    //Not used with vfd:    
    kernel_task_control_block[TASK_SPINDLE_CTRL].state                      = ASLEEP;
    kernel_task_control_block[TASK_SPINDLE_CTRL].sleep_count                = KERNEL_SCHED_20_MS_PERIOD ;
    kernel_task_control_block[TASK_SPINDLE_CTRL].sleep_interval             = KERNEL_SCHED_20_MS_PERIOD ;
    kernel_task_control_block[TASK_SPINDLE_CTRL].slot_id                    = SLOT_2;
    kernel_task_control_block[TASK_SPINDLE_CTRL].taskptr                    = spindle_ctrl_task;

 
    //=SLOT 3 ================================================== 
    kernel_task_control_block[TASK_LEDS_MGR].state                          = ASLEEP;
    kernel_task_control_block[TASK_LEDS_MGR].sleep_count                    = KERNEL_SCHED_100_MS_PERIOD ;
    kernel_task_control_block[TASK_LEDS_MGR].sleep_interval                 = KERNEL_SCHED_100_MS_PERIOD ;
#ifdef REDUCE_KERNEL_SLOT_COUNT
    kernel_task_control_block[TASK_LEDS_MGR].slot_id                        = SLOT_MAX;
#else
    kernel_task_control_block[TASK_LEDS_MGR].slot_id                        = SLOT_3;
#endif
    kernel_task_control_block[TASK_LEDS_MGR].taskptr                        = leds_mgr_task;

#ifndef ROSECOMB_MVP
    kernel_task_control_block[TASK_UART_MGR].state                          = ASLEEP;   
    kernel_task_control_block[TASK_UART_MGR].sleep_count                    = KERNEL_SCHED_30_MS_PERIOD ;
    kernel_task_control_block[TASK_UART_MGR].sleep_interval                 = KERNEL_SCHED_30_MS_PERIOD ; 
#ifdef REDUCE_KERNEL_SLOT_COUNT
    kernel_task_control_block[TASK_UART_MGR].slot_id                        = SLOT_MAX;
#else    
    kernel_task_control_block[TASK_UART_MGR].slot_id                        = SLOT_3;
#endif
    kernel_task_control_block[TASK_UART_MGR].taskptr                        = comms_mgr_task;
#endif
    
#ifdef DEPLOY_SD_TASK   
    //=SLOT 4 ==================================================       
    kernel_task_control_block[TASK_SDMMC_MGR].state                         = ASLEEP;  
    kernel_task_control_block[TASK_SDMMC_MGR].sleep_count                   = KERNEL_SCHED_110_MS_PERIOD ;
    kernel_task_control_block[TASK_SDMMC_MGR].sleep_interval                = KERNEL_SCHED_110_MS_PERIOD ;    

#ifdef REDUCE_KERNEL_SLOT_COUNT
    kernel_task_control_block[TASK_SDMMC_MGR].slot_id                        = SLOT_MAX;
#else
    kernel_task_control_block[TASK_SDMMC_MGR].slot_id                       = SLOT_4;
#endif
    kernel_task_control_block[TASK_SDMMC_MGR].taskptr                       = sd_card_file_mgr_task;          
#endif
    
#ifdef DEPLOY_FUTURE_EXPANSION_BRD    
    kernel_task_control_block[TASK_SPI_MGR].state                          = ASLEEP;  
    kernel_task_control_block[TASK_SPI_MGR].sleep_count                    = KERNEL_SCHED_300_MS_PERIOD ;
    kernel_task_control_block[TASK_SPI_MGR].sleep_interval                 = KERNEL_SCHED_300_MS_PERIOD ;
    kernel_task_control_block[TASK_SPI_MGR].slot_id                        = SLOT_4;
    kernel_task_control_block[TASK_SPI_MGR].taskptr                        = spi_comm_task;   
#endif
    
   //=SLOT 5 ==================================================  
   //=SLOT 6 ==================================================   
   //=SLOT 7 ==================================================  
   //=SLOT 8 ==================================================      
}//End function
//=====================================================================
//
//Function: kernel_get_task_period_ms
//
// Description: returns scheduling period. 
//      
//
//=====================================================================
 uint16_t kernel_get_task_period_ms(TaskIdE id)
{
    volatile uint16_t _id=(uint16_t)id;
    volatile uint16_t kernel_exec_period_ms=KERNEL_EXEC_PERIOD_MS;
    volatile uint16_t sleep_interval=kernel_task_control_block[_id].sleep_interval;
    volatile uint16_t sheduling_period_ms=((sleep_interval+1)*kernel_exec_period_ms);
    return sheduling_period_ms;
} 
//=====================================================================
//
//Function: kernel_interrupt
//
// Description:
//     Triggered by Timer0 interrupt at ISR_PERIOD_INCREMENT
//      ms time intervals.
//      Counts down sleeping task timers
//      Sets task state from ASLEEP to READY when sleep time
//      counts down to 0
//
//
//=====================================================================
void kernel_interrupt(void) 
{
   static int i;
   
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING 
   
   FourByteDataU data;
   volatile static TaskIdE kern_taskid_e = (TaskIdE) 0;
   bool task_slot_overrun_flag=false;
#define SELECT_OVERRUN_TIME_MS (KERNEL_SLOT_DURATION_MS*2)
   uint32_t elapsed_run_time_ms;
#define SELECTED_ELAPSED_SYS_TIME_MS 1000
   uint32_t elapsed_sys_time_ms = sys_get_time_ms();
   static uint32_t select_elaps_sys_time_ms=SELECTED_ELAPSED_SYS_TIME_MS; 
   static uint32_t select_overrun_time_ms = SELECT_OVERRUN_TIME_MS;
#endif   
   //hal_toggle_pin(GPIO_DBG2_GPIO_PortId,GPIO_DBG2_Pin );
   if (kernel_enable_int_flag == true)
   {
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING 
      if (sched_running_task_slot_id==SLOT_UNASSIGNED) 
      {
        ///__no_operation();
      }
      else
      {
         if (kernel_task_control_block[sched_taskid_e].state == RUNNING)
         {
 
            kernel_task_control_block[sched_taskid_e].fault_status=FAULT_OVERRAN_TIMESLOT;
#define ASSIGNED_TIMESLOT_NDX 0
#define OVERRUN_TIMESLOT_NDX 1            
            data.u16_data[ASSIGNED_TIMESLOT_NDX]=sched_running_task_slot_id;
            data.u16_data[OVERRUN_TIMESLOT_NDX]=kernel_int_slot;
           
            if (sched_taskid_e==select_task_id_e)
            {
              elapsed_run_time_ms=sys_get_delta_time_ms(kernel_task_control_block[sched_taskid_e].start_time_ms);
              if ((elapsed_run_time_ms > select_overrun_time_ms)
              &&  (elapsed_sys_time_ms>select_elaps_sys_time_ms))
              {
#if 0 
                 __no_operation();
#else//log it                
                 kernel_log_fault(sched_taskid_e,FAULT_OVERRAN_TIMESLOT,data);            
                 task_slot_overrun_flag=true;
#endif
              }
            }
         }
      }
#endif  //overrun fault logging   
      /* Update the execution slot per this 2ms increment */
      kernel_int_slot++;
      
      if (kernel_int_slot >SLOT_MAX)
      {
        //hal_toggle_pin(GPIO_DBG2_GPIO_PortId,GPIO_DBG2_Pin);
        kernel_int_slot=SLOT_0;       
      }

      for (i =(TaskIdE) 0/*TASK_ID_FIRST*/; i<TASK_ID_COUNT; i++)
      {
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING 
         kern_taskid_e = (TaskIdE)i;
#endif          
#ifdef REDEFINE_KERNEL_SCHEDULING
          if (true)
#else         
          if (kernel_task_control_block[i].slot_id == kernel_int_slot)
#endif
          {
            if (kernel_task_control_block[i].state == ASLEEP)
            {
               /* 
                * Count down all sleeping task's sleep counts and
                * set state to READY when sleep count is down to 0
                */
                if (kernel_task_control_block[i].sleep_count > 0)
                {
                    kernel_task_control_block[i].sleep_count--;
                }
                else//Task has slept enough. Time to wake up and be READY
                {
                    kernel_task_control_block[i].sleep_count = kernel_task_control_block[i].sleep_interval;
                    kernel_task_control_block[i].state = READY;
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING                     
                   if( task_slot_overrun_flag==true)
                   {
                     kernel_task_control_block[i].fault_status=FAULT_STARVED_BY_AN_OVERRUN;
                     kernel_log_fault(kern_taskid_e,FAULT_STARVED_BY_AN_OVERRUN,data);
                   }
#endif                     
                } /* End if */
            }/* End if */
            else
            {
              __NOP();
            }

          }/* End if */
      }/* End For */
 
   }/* End if kernel_enable_int_flag ==true*/
   
}/* End Interrupt*/

 
#ifdef REPLACE_KERNEL_WITH_LOOPING
 
 
void kernel_scheduler(void)
{ 
   static uint32_t leds_start_time_ms, sdcard_start_time_ms, uart_start_time_ms,spindle_start_time_ms=0;
   volatile static uint32_t leds_delta_time_ms, sdcard_delta_time_ms, uart_delta_time_ms,spindle_delta_time_ms=0;
   while(FOREVER)//loop forever
   {
#define SPINDLE_SLEEP_TIME_MS 20
#define LEDS_SLEEP_TIME_MS 100 
#define UART_SLEEP_TIME_MS 210
#define SDCARD_SLEEP_TIME_MS 110   
      
     /* high priority tasks execute */ 
     comms_mgr_task(); 
     main_controller_task();       
 
     /* low priority tasks execute when their sleep counts (see kernel interrupt) are fulfilled: */ 
     if (sys_time_ms_limit_reached(spindle_start_time_ms, SPINDLE_SLEEP_TIME_MS))
     {
         spindle_start_time_ms=sys_get_time_ms();
         spindle_ctrl_task();
     }
     if (sys_time_ms_limit_reached(leds_start_time_ms, LEDS_SLEEP_TIME_MS))
     {            
        leds_start_time_ms=sys_get_time_ms();
        leds_mgr_task();
     }
     if (sys_time_ms_limit_reached(sdcard_start_time_ms, SDCARD_SLEEP_TIME_MS))
     {
         sdcard_start_time_ms=sys_get_time_ms();
        sd_card_file_mgr_task();
     }
     if (sys_time_ms_limit_reached(uart_start_time_ms, UART_SLEEP_TIME_MS))
     {            
        uart_start_time_ms=sys_get_time_ms();
        uart_mgr_task();
     }
   
   }

}
#else //intended scheduler
//=====================================================================
//
// Function: kernel_scheduler
//
// Description: Loops through each task's control block to
//              launch any READY task whose time slot INT_Slot,
//              has come.
//              Must disable interrupts to set task control block's
//              state variable, because that item is also written
//              by the main_interrupt
//
//=====================================================================
void kernel_scheduler(void)
{
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               static int j;
   static  /*(KernelSlotIdE*/ int test_slot_id=0;
   static UINT8 this_task_slot_id=0;

   while(FOREVER)//loop forever
   {  
      __NOP();

#if 0 // large jitter when disabling/enabling kernel interrupt entry 
      DISABLE_KERNEL_INTERRUPT();
      test_slot_id = kernel_int_slot;
      ENABLE_KERNEL_INTERRUPT(); 
#else //jitter eliminated. downside: kernel interrupt could be changing the slot number at same time as this function reading it.  
      test_slot_id = kernel_int_slot;
#endif
    
   if (kernel_saved_int_slot != test_slot_id) //Do not repeat the same slot.
   {
         kernel_saved_int_slot = test_slot_id; 
         for (j = 0; j < TASK_ID_COUNT; j++)
         {
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING           
              sched_taskid_e=(TaskIdE)j;
#endif
             this_task_slot_id = kernel_task_control_block[j].slot_id;
             if (this_task_slot_id == test_slot_id)
             {
                if (kernel_task_control_block[j].state == READY)
                {
                     /* Make the READY task RUNNING whose slot has come */
                     DISABLE_KERNEL_INTERRUPT();
                     kernel_task_control_block[j].state = RUNNING;
                     ENABLE_KERNEL_INTERRUPT();
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING 
                     sched_running_task_slot_id= this_task_slot_id; 
#endif
                      /* launch the ready task whose slot has come */
                      kernel_task_control_block[j].taskptr();
                      
                     /* if task did not self-suspend (which is needed only for one-shot, non periodic tasks), then put the task back to sleep --no errors, it can run again */
                     if (kernel_task_control_block[j].state == RUNNING)
                     {
                          DISABLE_KERNEL_INTERRUPT();
                          kernel_task_control_block[j].state = ASLEEP;
#ifdef DEPLOY_KERNEL_TASK_OVERRUN_CHECKING                          
                          sched_running_task_slot_id=SLOT_UNASSIGNED;
#endif
                          ENABLE_KERNEL_INTERRUPT();
                     }

                 }/* End If ==READY */

             } /* End if If this_task_slot_id  ==Int_Slot */

            } /* End For */

      } /* End if (Saved_Int_Slot !=Int_Slot) */

   }/* End while */
 
}/* kernel_scheduler */
#endif
 