/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/**********************************************************************************/    
/* Begin: Hardware/Board Identification Definitions                               */
/**********************************************************************************/ 
//Warning: this HW board definition must preceed the #include "g2core.h"!
/**********************************************************************************/    
/* Begin: Definitions to identify the hardware: mill or plotter, etc  */
/* each unique hw designation will pull in the correct settings file  */
/**********************************************************************************/    
/* Select only one: */
#define CHABO_MINIMILL      //uses g2_settings_bt_minimill.h
//#define CHABO_MINIMILL_ESC  //uses g2_settings_bt_minimill_esc.h
//#define CHABO_PLOTTER       //uses g2_settings_bt_plotter.h
//#define CHABO_LFP           //uses g2_settings_bt_lfp.h
//#define CHABO_DCNC          //uses g2_settings_bt_cncmm.h

/* Product IDs */
#ifdef CHABO_DCNC
#define CHABO_PID 0x02
#elif defined CHABO_MINIMILL_ESC
#define CHABO_PID 0x01
#else // Default, CHABO_MINIMILL (old ESC)
#define CHABO_PID 0xFFFFFFFF // Default memory value, all Fs
#endif
#define CHABO_PID_ADDRESS 0x450000

/**********************************************************************************/    
/* End: Definitions to identify the hardware: mill or plotter, etc */
/**********************************************************************************/     
//#define DEPLOY_EXPANSION_BOARD

/**********************************************************************************/    
/* End: Hardware/Board Identification Definitions                                 */
/**********************************************************************************/

/* Includes ------------------------------------------------------------------*/
/* USER CODE END Includes */  
#include "g2core.h"//Warning: g2core.h depends on above #define for HW board Id: 
                   
#ifdef __cplusplus //Warning: g2core.h also must be included before the #ifdef __cplusplus ...
extern "C" {
#endif 
    
 /* USER CODE END Includes */
/********************************************************************************/
/* Begin: Definitions Generated by Harmony3 Code generator for Chabo Board      */
/********************************************************************************/
/** 
  * @brief  HAL Status structures definition  
  */  
typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;
typedef enum
{ 
  USBD_STATUS_UNDEFINED,
  USBD_STATUS_CONNECTED,
  USBD_STATUS_DISCONNECTED,
  USBD_STATUS_COUNT  
}UsbdConnectStatusE;
 
/* Following USB Device status */
typedef enum {
  USBD_OK   = 0,
  USBD_BUSY,
  USBD_FAIL,
}USBD_StatusTypeDef;
 
/********************************************************************************/
/* End: Definitions Generated by Harmony3 Code generator for Chabo Board        */
/********************************************************************************/
/* Private define ------------------------------------------------------------*/
#ifdef CHABO_PLOTTER
#define DEPLOY_FOURTH_AXIS 
#define ROSECOMB_MVP
#elif defined CHABO_LFP
#define ROSECOMB_MVP
#elif defined CHABO_DCNC
//uncomment when deploying 4th axis on a DCNC:
//#define DEPLOY_FOURTH_AXIS 
#endif 

/* Define mnemonics for nc, no switch properties  
 *  Both sensors are pulled high. When switch is released, NO is pulled to GND, NO remains pulled HI 
 *    Sw sensor:                   SW PRESSED     SW RELEASED
 *  --------------------------     --------         ---------
 *  Normally Open (NO)               0 (closes)     1 (opens)
 *  Normally Closed (NC)             1 (opens)      0 (closes)
*/
typedef enum
{
  SWITCH_NO_ACTIVE_STATE=0,
  SWITCH_NC_ACTIVE_STATE=1,       
  SWITCH_ACTIVE_STATE_COUNT, 
  SWITCH_NO_INACTIVE_STATE = 1,
  SWITCH_NC_INACTIVE_STATE = 0 
}SwitchActiveStateE;

/********************************************************************************/
/* Begin: Developmental/Debug  conditional Compile Definitions                  */
/********************************************************************************/
#ifdef CHABO_LFP
#define DEBUG_ZAXIS_HOMING_ANOMALY//for plotter, first time homing z axis is forced to have prev_direction==1
#elif defined  CHABO_PLOTTER
#define DEBUG_ZAXIS_HOMING_ANOMALY//to do prove this is needed
#endif

#define REFACTOR_LIMIT_SWITCHES //3-29-2023: Eliminate dual redundant, accommodate either polarity for Active state NO, NC
#define DEPLOY_PROBE_FEEDHOLD_TIMEOUT //12-19-2022
#define DEBUG_SDCARD_BRICK_EFFECT
//#define EXIT_HARD_FAULT_EXCEPTION_BY_HW_RESET
#ifdef EXIT_HARD_FAULT_EXCEPTION_BY_HW_RESET
#define DEPLOY_HARD_FAULT_REPORTING
#endif

//#define DEPLOY_MCU_WD_TIMER
#define DEPLOY_SD_TASK//11-17-2022  NOTE: ifndef:  g2core's main controller loop handles persistence instead of the separate task  

#ifdef DEPLOY_MCU_WD_TIMER
//#define TEST_MCU_WD_TIMER
#endif

//#define REPLACE_KERNEL_WITH_LOOPING
#ifndef REPLACE_KERNEL_WITH_LOOPING
#define KERNEL_ALLOW_MULTISLOTS_PER_TASK
#define REDUCE_KERNEL_SLOT_COUNT //11-15-2022
#endif
#define DEPLOY_KERNEL_LOG_REPORTING
//#define DEPLOY_KERNEL_TASK_TIMING
//#define DEPLOY_KERNEL_TASK_FAULT_STATUS    
//#define DEPLOY_KERNEL_TASK_OVERRUN_CHECKING
//#define KERNEL_TASK_CHECKIN_CHECKOUT
#ifdef DEPLOY_HARD_FAULT_REPORTING
typedef struct
{
  bool fault_flag;
  uint32_t cfsr;
  uint32_t hfsr;
  char msg_text[80]; 
}HardFaultInfoS;
extern volatile HardFaultInfoS hard_fault_info;// __attribute__ ((persistent));
#endif

#define DEPLOY_SECOND_YAXIS_STEPPER
#define DEPLOY_GET_CAUSE_OF_RESET      
#define FLUSH_USB_RX_QUEUE_ON_ESTOP
#define DEPLOY_STEPPER_REINIT_AFTER_ESTOP_RELEASED
#define FILTER_SPURIOUS_PROBE_SWITCH_EVENTS
#define REPEAT_PERSISTENT_SWITCH_ANOMALIES
#define FILTER_SPURIOUS_SD_CARD_FAULTS
#define RECONCILE_TUID_NO_STRIP//persistence needs no_strip. json get/set report must provide stripped version for desktop app to work with tool offsets and uuid strings
#define APPLY_NO_STRIP_ON_TUIDS//needed for nv lookup during persistence but not simple retrieval of tuids from console
#define DEPLOY_GET_CAUSE_OF_RESET
#define REMOVE_MICROSTEP_SET_DEFECT//identifies where the defect was
#define DEPLOY_LIMIT_SWITCH_FAULT_CHECKING

/********************************************************************************/
/* End: Developmental/Debug  conditional Compile Definitions                    */
/********************************************************************************/

/********************************************************************************/
/* Begin: Stable #definitional subject to adjustments                           */
/********************************************************************************/
#define CHABO_USE_MATTS_TMC2660_INIT//11-16-2022 FYI: This replaces the motion_control_task--until we need to continuously monitor triamics over spi for motor states, stall etc
#define PRIORITY_HIGHEST 0
#define PRIORITY_HIGH    1
#define PRIORITY_MEDIUM_HIGH 2
#define PRIORITY_MEDIUM  4 
#define PRIORITY_LOW     6
#define PRIORITY_LOWEST  7

/* Assign interrupt priority hierarchy */
#define IRQ_PRIORITY_SYSTICK PRIORITY_HIGH
#define IRQ_PRIORITY_DDA PRIORITY_HIGH //11-11-2022 PRIORITY_HIGHEST
#define IRQ_PRIORITY_EXEC_MOVE PRIORITY_HIGHEST//11-11-2022 HIGH
#define IRQ_PRIORITY_FORWARD_PLAN PRIORITY_HIGH//11-11-2022 PRIORITY_MEDIUM 
#define IRQ_PRIORITY_USB PRIORITY_MEDIUM //11-11-2022 PRIORITY_HIGH
#define IRQ_PRIORITY_DMA PRIORITY_LOW 
#define IRQ_PRIORITY_UART3 PRIORITY_LOW 
#define IRQ_PRIORITY_SDCARD PRIORITY_LOW 
#define IRQ_PRIORITY_USART PRIORITY_LOWEST

/* Mnemonics for TMC2660 SPI Chip Select lines*/
#define X_MTRDRV_SPI_nCS_GPIO_PortId 0
#define Y_MTRDRV_SPI_nCS_GPIO_PortId 0
#define Z_MTRDRV_SPI_nCS_GPIO_PortId 0
#define Y2_MTRDRV_SPI_nCS_GPIO_PortId 0
#define X_MTRDRV_SPI_CSn_Pin X_MTRDRV_SPI_CSn_PIN
#define Y_MTRDRV_SPI_CSn_Pin Y1_MTRDRV_SPI_CSn_PIN
#define Z_MTRDRV_SPI_CSn_Pin Z_MTRDRV_SPI_CSn_PIN
#define Y2_MTRDRV_SPI_CSn_Pin Y2_MTRDRV_SPI_CSn_PIN

/********************************************************************************/
/* End: Stable #definitional subject to adjustments                           */
/********************************************************************************/

//---BEGIN: REFACTORED G2CORE-----------   
/* Below identify g2core updates July/Aug 2021, since original porting of "edge" version of g2core in 2020 */
//#define REFACTOR_REPORT //sme: a11-16-2022 Prior to this date, this was not defined-- this used double replaces float and, report requests are no longer rejected if one is already in the works
                           //It is still an unproven method. retaining original. 
//---END: REFACTORED G2CORE-----------


//#define DISABLE_STEP_CORRECTION 
#ifndef DISABLE_STEP_CORRECTION
#define STEP_CORRECTION_PARAM_ADJUSTMENTS //deploy run time-adjustable variables 
#endif
//#define DEBUG_SKIPPED_ALINE_DURING_PROBE//01-25-2023
//#define DEBUG_JOGGING_RUN_AWAY_DEFECT 
//#define MAIN_DEPLOY_UART7_STALL_EVENT_PRINTS
//#define STALLGUARD_TESTING// for printing status to terminal
//#define TMC2660_POLL_SG_ONLY
//#define TMC2660_ENABLE_STATUS_PRINTING
//#define TMC2660_ENABLE_PERIODIC_PRINTING_READBACKS
//#define DEPLOY_HI_RESOLUTION_SYSTIMER
//#define REPORT_TMC2660_OVER_TEMP
/*************************************************************************/
/* Begin: Special individual machine section */
/*************************************************************************/

/*************************************************************************/
/* End: Special , individual machine section   */
/*************************************************************************/
//#define ENABLE_PRINTF_P_CALLS
#define REDIRECT_FPRINTF_P_CALLS

/* The extern C declarations are needed by the compiler due to the C++ usages by G2core's latest edge version*/

extern  void main_refresh_adc1(void);
extern  void main_monitor_stall_events(void); 
extern  void blocking_delay_usec(unsigned short usecs);
extern  void start_usec_timer(void);
extern  unsigned short get_usec_elapsed_time(void);
extern  uint32_t main_get_x_stall_event_count(void);
extern  uint32_t main_get_y1_stall_event_count(void);
extern  uint32_t main_get_y2_stall_event_count(void);
extern  uint32_t main_get_z_stall_event_count(void);
extern  void _Error_Handler(char const *, int);

/*************************************************************************/
/*Begin: Globally visible Function prototypes                            */
/*************************************************************************/
int uart_debug_print(char * msg);
void main_set_dfu_bootload_mode(int state);
void MX_SDMMC1_MMC_Init(void);
void MX_SDMMC1_MMC_DeInit(void);
void MX_USART3_UART_Init(void);

#ifdef DEPLOY_GET_CAUSE_OF_RESET 
extern uint8_t rcc_csr_value;
#endif
void get_out_jail_free_reset(void);
int user_main(void);
//moved to where dependencies are already defined stat_t main_set_dfu_bootloader_mode(nvObj_t *nv);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
/*************************************************************************/
/*End: Globally visible Function prototypes                            */
/*************************************************************************/

#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/