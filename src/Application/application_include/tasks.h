/****************************************************************************** 
 * File:   tasks.h
 * Description: 
 *
 ******************************************************************************/

#ifndef TASKS_H
#define	TASKS_H

#include "user_types.h"
#include "globdefs.h"
#include "main.h"
/* Include the header files for each module that defines a task:*/
#include "motion_ctrl.h"
#include "main_controller.h"
#include "leds_mgr.h"
#include "sd_card_file_mgr.h"
#include "spi_comm.h"
#include "spindle_ctrl.h"
#include "comms_mgr.h"


/* Enumerate the Tasks */
typedef enum
{
  TASK_MAIN_CONTROLLER,

  TASK_COMMS_MGR, 
#ifndef CHABO_USE_MATTS_TMC2660_INIT
  TASK_MOTION_CTRL,//stepper motorS
#endif
  TASK_SPINDLE_CTRL,
  TASK_LEDS_MGR,
#ifdef DEPLOY_SD_TASK          
  TASK_SDMMC_MGR,
#endif
#ifdef DEPLOY_FUTURE_EXPANSION_BRD  
  TASK_SPI_MGR,  
#endif
#ifndef ROSECOMB_MVP
  TASK_UART_MGR,
#endif
  //TASK_WATCHDOG,          
  TASK_ID_COUNT, 
//    TASK_ID_FIRST= TASK_MAIN_CONTROLLER 
}TaskIdE;
extern void usb_middleware_task(void);//sme
extern void usb_hsdrv_task(void);//sme
extern void sdmmmc_task(void);//sme
extern bool SD_Detect ( void );//sme
#endif	/* TASKS_H */



 