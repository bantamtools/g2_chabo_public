/*******************************************************************************
 System Tasks File

  File Name:
    tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled tasks.

  Description:
    This file contains source code necessary to maintain system's polled tasks.
    It implements the "SYS_Tasks" function that calls the individual "Tasks"
    functions for all polled MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "configuration.h"
#include "definitions.h"
#include "system_bantam.h"
extern "C" void DRV_USBHSV1_Tasks(SYS_MODULE_OBJ object); 

// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************
void sdmmmc_task(void)
{
    DRV_SDMMC_Tasks(sysObj.drvSDMMC0);
}

void usb_middleware_task(void)
{
    USB_DEVICE_Tasks(sysObj.usbDevObject0);
}

void usb_hsdrv_task(void)
{ 
    DRV_USBHSV1_Tasks(sysObj.drvUSBHSV1Object);
}
 
/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/
void SYS_Tasks ( void )
{
    /* Maintain system services */   
    SYS_FS_Tasks();

    DRV_SDMMC_Tasks(sysObj.drvSDMMC0);

    /* Maintain Device Drivers */   
    /* Maintain Middleware & Other Libraries */
    	/* USB Device layer tasks routine */ 
    USB_DEVICE_Tasks(sysObj.usbDevObject0);

	/* USB HS Driver Task Routine */ 
    DRV_USBHSV1_Tasks(sysObj.drvUSBHSV1Object);
    /* Maintain the application's state machine. */
        /* Call Application task APP. */
    APP_Tasks();
}
void SYS_Tasks_restricted ( void )
{
    /* Maintain system services */   
    SYS_FS_Tasks();
    DRV_SDMMC_Tasks(sysObj.drvSDMMC0);
    //sd_card_task();
}
bool SD_Detect ( void )
{
    bool card_detected_flag=false;
    uint32_t start_time_ms=sys_get_time_ms();
    bool timeout_flag=false;
#define CARD_DETECT_TIMEOUT_MS 500
    while((!card_detected_flag)&&(!timeout_flag))
    {
       /* Maintain system services */   
       SYS_FS_Tasks();
       DRV_SDMMC_Tasks(sysObj.drvSDMMC0);
       card_detected_flag=SYS_FS_MediaIsAttached();
       if (card_detected_flag==true)
       {
           __NOP();
       }
       else
       {
           __NOP();
       }
       timeout_flag=sys_time_ms_limit_reached(start_time_ms,CARD_DETECT_TIMEOUT_MS);
    }
    return(card_detected_flag);
}


/*******************************************************************************
 End of File
 */

