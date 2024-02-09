/*******************************************************************************
                      
Module:  kernel.h
 $Revision:  $
 $Date:  $

Programmer:
 
********************************************************************************
Contents:
  1. Clock Configuration and System Timing
Functions Prototypes:
*******************************************************************************/
#ifndef  _KERNEL_H
#define  _KERNEL_H

#include "main.h"
#include "user_types.h"
#include "globdefs.h"
#include "tasks.h"

// Enumerate the number of slots possible 200usec/slot over 2 ms scheduling interval: 10
typedef enum
{
  SLOT_0=0,
  SLOT_1,
  SLOT_2,
  SLOT_3,
  SLOT_4,
  //SLOT_5,
  //SLOT_6,
  //SLOT_7,
  //SLOT_8,
  //SLOT_9,  
#ifdef REDUCE_KERNEL_SLOT_COUNT
 SLOT_MAX = SLOT_2,
#else
  SLOT_MAX = SLOT_4, 
#endif
  SLOT_COUNT,
  SLOT_UNASSIGNED=SLOT_COUNT
} KernelSlotIdE;

/* Define  */
/*********************************************************************** 
 * To do: 
 *  Decide to use systick 1 KHz granularity for kernel interrupt, or
 *      finer granularity provided by time14 interrupt at presently configured 
 * 100usec resolution
 ********************************************************************** */ 
/*Choose one or the other!! */
//#define DEPLOY_KERNEL_FROM_10KHZ_TIM14_INTERRUPT
//#define DEPLOY_KERNEL_FROM_1KHZ_TIM14_INTERRUPT
#ifndef REPLACE_KERNEL_WITH_LOOPING
#define DEPLOY_KERNEL_FROM_1KHZ_SYSTICK_INTERRUPT
#endif
/* Deploying coarse-grained millisecond resolution task scheduling */

#ifdef DEPLOY_KERNEL_FROM_1KHZ_SYSTICK_INTERRUPT
/* Define interrupt frequency: max is the parent interrupt frequency 1KHz if from Systick interrupt */
#define KERNEL_1KHZ_SYSTICK_PERIOD_MS           1//1 ms period is 1 kHz 
#define KERNEL_INTERRUPT_SKIP_COUNT             1//skip every other systick interrupt event, to get 1/2 frequency
#define KERNEL_INTERRUPT_MIN_PERIOD_MS          (KERNEL_INTERRUPT_SKIP_COUNT+KERNEL_1KHZ_SYSTICK_PERIOD_MS)
#define KERNEL_INTERRUPT_PERIOD_MS              KERNEL_INTERRUPT_MIN_PERIOD_MS
#define KERNEL_SLOT_DURATION_MS                 KERNEL_INTERRUPT_PERIOD_MS//directly matches to interrupt period
#define KERNEL_SLOT_DURATION                    KERNEL_SLOT_DURATION_MS

#else /* Deploying the finer grained , tim14 usec resolution interrupt */
#define KERNEL_SLOT_DURATION_USECS 200
#define KERNEL_SLOT_DURATION_x100_USECS 2//20
#define KERNEL_SLOT_DURATION_x1000_USECS 0//20
#define KERNEL_SLOT_DURATION                        KERNEL_SLOT_DURATION_USECS
#endif

/* Define granularity of Kernel: 2 ms period x 5 Slots produces a minimal period of 10 ms == 100Hz  */

#define KERNEL_EXEC_PERIOD_MS                      (KERNEL_SLOT_DURATION * SLOT_COUNT)  
#define KERNEL_MIN_SCHEDULING_PERIOD_MS            KERNEL_EXEC_PERIOD_MS  

#define KERNEL_ONE_SCHEDULING_INCREMENT_MS         KERNEL_SLOT_DURATION 
#define KERNEL_NO_SLEEP_INTERVAL                   0 /* Scheduling has same period as one scheduling loop*/
#define KERNEL_ALWAYS_READY                        KERNEL_NO_SLEEP_INTERVAL
#define KERNEL_SCHED_MIN_MS_PERIOD                 KERNEL_ALWAYS_READY //This period is directly based on the number of slots and the duration of a single slot

/* Explicitly define kernel scheduling periods in terms of multiples of sum of slots in a full period */
#define KERNEL_SCHED_10_MS_PERIOD                  KERNEL_SCHED_MIN_MS_PERIOD
#define KERNEL_SCHED_20_MS_PERIOD                  KERNEL_SCHED_MIN_MS_PERIOD+1 
#define KERNEL_SCHED_30_MS_PERIOD                  KERNEL_SCHED_MIN_MS_PERIOD+2 
#define KERNEL_SCHED_40_MS_PERIOD                  KERNEL_SCHED_MIN_MS_PERIOD+3 
#define KERNEL_SCHED_50_MS_PERIOD                  KERNEL_SCHED_MIN_MS_PERIOD+4 
#define KERNEL_SCHED_60_MS_PERIOD                  KERNEL_SCHED_MIN_MS_PERIOD+5 
#define KERNEL_SCHED_70_MS_PERIOD                  KERNEL_SCHED_MIN_MS_PERIOD+6
#define KERNEL_SCHED_80_MS_PERIOD                  KERNEL_SCHED_MIN_MS_PERIOD+7
#define KERNEL_SCHED_90_MS_PERIOD                  KERNEL_SCHED_MIN_MS_PERIOD+8
#define KERNEL_SCHED_100_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+9
#define KERNEL_SCHED_110_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+10
#define KERNEL_SCHED_120_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+11
#define KERNEL_SCHED_130_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+12
#define KERNEL_SCHED_140_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+13
#define KERNEL_SCHED_150_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+14
#define KERNEL_SCHED_160_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+15
#define KERNEL_SCHED_170_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+16
#define KERNEL_SCHED_180_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+17
#define KERNEL_SCHED_190_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+18
#define KERNEL_SCHED_200_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+19
#define KERNEL_SCHED_210_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+20
#define KERNEL_SCHED_220_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+21
#define KERNEL_SCHED_230_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+22
#define KERNEL_SCHED_240_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+23
#define KERNEL_SCHED_250_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+24
#define KERNEL_SCHED_260_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+25
#define KERNEL_SCHED_270_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+26
#define KERNEL_SCHED_280_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+27
#define KERNEL_SCHED_290_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+28
#define KERNEL_SCHED_300_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+29
#define KERNEL_SCHED_310_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+30
#define KERNEL_SCHED_320_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+31
#define KERNEL_SCHED_330_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+32
#define KERNEL_SCHED_340_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+33
#define KERNEL_SCHED_350_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+34
#define KERNEL_SCHED_360_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+35
#define KERNEL_SCHED_370_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+36
#define KERNEL_SCHED_380_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+37
#define KERNEL_SCHED_390_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+38
#define KERNEL_SCHED_400_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+39
#define KERNEL_SCHED_500_MS_PERIOD                 KERNEL_SCHED_MIN_MS_PERIOD+49
#define KERNEL_SHED_MS(sleep_interval) (SLOT_COUNT*)
//------------------------------------------------------------------------------
void kernel_init(void);
void kernel_init_task_control_block(void);
void kernel_interrupt(void);
void kernel_scheduler(void);
void kernel_ready_task(TaskIdE id);
void kernel_suspend_task(TaskIdE id);
BOOL kernel_task_is_suspended(TaskIdE id);

#ifdef KERNEL_TASK_CHECKIN_CHECKOUT
void kernel_task_check_in(void);
void kernel_task_check_out(void);
#endif

#ifdef DEPLOY_KERNEL_LOG_REPORTING
int kernel_serialize_logfile( char *out_buf, uint16_t size);
stat_t kernel_get_kxt(nvObj_t *nv);
void kernel_print_kxt(nvObj_t *nv);
void kernel_print_kfl(nvObj_t *nv);
stat_t kernel_get_kfl(nvObj_t *nv);
int kernel_get_log_entry_count(void);
uint16_t kernel_get_task_period_ms(TaskIdE taskid);
#endif
#endif



