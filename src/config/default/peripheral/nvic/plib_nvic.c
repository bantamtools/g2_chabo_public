/*******************************************************************************
  NVIC PLIB Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    plib_nvic.c

  Summary:
    NVIC PLIB Source File

  Description:
    None

*******************************************************************************/

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

#include "device.h"
#include "plib_nvic.h"
#include "main.h"

// *****************************************************************************
// *****************************************************************************
// Section: NVIC Implementation
// *****************************************************************************
// *****************************************************************************

void NVIC_Initialize( void )
{
    /* Priority 0 to 7 and no sub-priority. 0 is the highest priority */
    NVIC_SetPriorityGrouping( 0x00 );

    /* Enable NVIC Controller */
    __DMB();
    __enable_irq();

    /* Enable the interrupt sources and configure the priorities as configured
     * from within the "Interrupt Manager" of MHC. */
    NVIC_SetPriority(TC1_CH0_IRQn, IRQ_PRIORITY_SYSTICK);
    NVIC_EnableIRQ(TC1_CH0_IRQn);
    
    NVIC_SetPriority(TC0_CH1_IRQn, IRQ_PRIORITY_DDA);
    NVIC_EnableIRQ(TC0_CH1_IRQn);
       
    NVIC_SetPriority(TC3_CH1_IRQn, IRQ_PRIORITY_EXEC_MOVE);
    NVIC_EnableIRQ(TC3_CH1_IRQn);
    
    NVIC_SetPriority(TC3_CH2_IRQn, IRQ_PRIORITY_FORWARD_PLAN);
    NVIC_EnableIRQ(TC3_CH2_IRQn);
    
    NVIC_SetPriority(USBHS_IRQn, IRQ_PRIORITY_USB);
    NVIC_EnableIRQ(USBHS_IRQn);
    
    NVIC_SetPriority(XDMAC_IRQn, IRQ_PRIORITY_DMA);
    NVIC_EnableIRQ(XDMAC_IRQn);

    NVIC_SetPriority(UART0_IRQn, 7);
    NVIC_EnableIRQ(UART0_IRQn);

    NVIC_SetPriority(UART3_IRQn, IRQ_PRIORITY_UART3);
    NVIC_EnableIRQ(UART3_IRQn);
    
    NVIC_SetPriority(HSMCI_IRQn, IRQ_PRIORITY_SDCARD);
    NVIC_EnableIRQ(HSMCI_IRQn);
    
    NVIC_SetPriority(USART2_IRQn, IRQ_PRIORITY_USART);
    NVIC_EnableIRQ(USART2_IRQn);   
#if 1//not used    
    NVIC_SetPriority(TC3_CH0_IRQn, 7);//sme: this is not used but it is the offical systick which is instead performed by tc1_ch1
    NVIC_EnableIRQ(TC3_CH0_IRQn);
#endif
}

void NVIC_INT_Enable( void )
{
    __DMB();
    __enable_irq();
}

bool NVIC_INT_Disable( void )
{
    bool processorStatus;

    processorStatus = (bool) (__get_PRIMASK() == 0);

    __disable_irq();
    __DMB();

    return processorStatus;
}

void NVIC_INT_Restore( bool state )
{
    if( state == true )
    {
        __DMB();
        __enable_irq();
    }
    else
    {
        __disable_irq();
        __DMB();
    }
}
