/*******************************************************************************
 * File:   system_bantam.h
 * Description:
 *
 ******************************************************************************/

#ifndef SYSTEM_BANTAM_H
#define	SYSTEM_BANTAM_H
 
#include "main.h"
#include "user_types.h"
#include "globdefs.h"
#ifdef __cplusplus
extern "C" {
#endif
    
#define SYSTEM_TIMER_INTERRUPT_PERIOD_USEC 1000//100=10kHz
void system_timer_interrupt(void);
void system_init(void);

Boolean_t sys_time_ms_limit_reached(uint32_t start_time, int32_t time_limit);
int sys_time_ms_time_remaining(uint32_t start_time, int32_t time_limit);
void sys_blocking_delay_ms(uint32_t delay_ms);
#if 1//def  DEPLOY_KERNEL_TASK_TIMING
short sys_get_delta_time_ms(uint32_t start_time_arg);
extern unsigned long sys_get_time_ms(void);
extern void start_usec_timer(void);
extern uint16_t get_usec_elapsed_time(void);
extern void systick_interrupt_callback_helper(void);
#endif

#ifdef __cplusplus
}
#endif
#endif	/* SYSTEM_BANTAM_H */

