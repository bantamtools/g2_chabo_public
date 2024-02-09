/*******************************************************************************
* File:    main_controller.c
*
* Description: 
*
*     
*
*******************************************************************************/
#include "main.h"
#include "user_types.h"
#include "globdefs.h"
#include "g2core.h"	 
#include "main_controller.h"
#include "exception_mgr.h" 
#include "g2_controller.h"
#include "kernel.h"
#include "app.h"
#include "tasks.h"

extern void SYS_Tasks_restricted ( void );

 /***************************************************************************
 * 
 * Function:  main_controller_init 
 * 
 * Description:   
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
void main_controller_init(void)
{
  
}

 /****************************************************************************
 * 
 * Function:  main_controller_task 
 * 
 * Description:   

 * Assumptions/Requirement: 

 *****************************************************************************/
void main_controller_task(void)
{
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_in();
#endif  
   
     static volatile bool enable_sys_tasks_restricted_flag=true;
     if(enable_sys_tasks_restricted_flag==true)
     {
        SYS_Tasks_restricted();
     }
     controller_run();
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_out();
#endif  
}/* End Task */