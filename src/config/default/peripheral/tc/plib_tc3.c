/*******************************************************************************
  TC Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_tc3.c

  Summary
    TC peripheral library source file.

  Description
    This file implements the interface to the TC peripheral library.  This
    library provides access to and control of the associated peripheral
    instance.

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

/*  This section lists the other files that are included in this file.
*/
#include "device.h"
#include "plib_tc3.h"
#include "interrupts.h"
 
/* Callback object for channel 0 */
TC_TIMER_CALLBACK_OBJECT TC3_CH0_CallbackObj;

/* Initialize channel in timer mode */
void TC3_CH0_TimerInitialize (void)
{
    /* clock selection and waveform selection */
    TC3_REGS->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVEFORM_WAVSEL_UP_RC | \
                                                        TC_CMR_WAVE_Msk ;

    /* write period */
    TC3_REGS->TC_CHANNEL[0].TC_RC = 0U;


    /* enable interrupt */
    TC3_REGS->TC_CHANNEL[0].TC_IER = TC_IER_CPAS_Msk;
    TC3_CH0_CallbackObj.callback_fn = NULL;
}

/* Start the timer */
void TC3_CH0_TimerStart (void)
{
    TC3_REGS->TC_CHANNEL[0].TC_CCR = (TC_CCR_CLKEN_Msk | TC_CCR_SWTRG_Msk);
}

/* Stop the timer */
void TC3_CH0_TimerStop (void)
{
    TC3_REGS->TC_CHANNEL[0].TC_CCR = (TC_CCR_CLKDIS_Msk);
}

uint32_t TC3_CH0_TimerFrequencyGet( void )
{
    return (uint32_t)(1171875UL);
}

/* Configure timer period */
void TC3_CH0_TimerPeriodSet (uint16_t period)
{
    TC3_REGS->TC_CHANNEL[0].TC_RC = period;
}

/* Configure timer compare */
void TC3_CH0_TimerCompareSet (uint16_t compare)
{
    TC3_REGS->TC_CHANNEL[0].TC_RA = compare;
}

/* Read timer period */
uint16_t TC3_CH0_TimerPeriodGet (void)
{
    return TC3_REGS->TC_CHANNEL[0].TC_RC;
}

/* Read timer counter value */
uint16_t TC3_CH0_TimerCounterGet (void)
{
    return TC3_REGS->TC_CHANNEL[0].TC_CV;
}

/* Register callback for period interrupt */
void TC3_CH0_TimerCallbackRegister(TC_TIMER_CALLBACK callback, uintptr_t context)
{
    TC3_CH0_CallbackObj.callback_fn = callback;
    TC3_CH0_CallbackObj.context = context;
}

/* Interrupt handler for Channel 0 */
void TC3_CH0_InterruptHandler(void)
{
    TC_TIMER_STATUS timer_status = (TC_TIMER_STATUS)(TC3_REGS->TC_CHANNEL[0].TC_SR & TC_TIMER_STATUS_MSK);
    /* Call registered callback function */
    if ((TC_TIMER_NONE != timer_status) && TC3_CH0_CallbackObj.callback_fn != NULL)
    {
        TC3_CH0_CallbackObj.callback_fn(timer_status, TC3_CH0_CallbackObj.context);
    }
}

/* Callback object for channel 1 */
TC_TIMER_CALLBACK_OBJECT TC3_CH1_CallbackObj;

/* Initialize channel in timer mode */
void TC3_CH1_TimerInitialize (void)
{
    /* Use peripheral clock */
    TC3_REGS->TC_CHANNEL[1].TC_EMR = TC_EMR_NODIVCLK_Msk;
    /* clock selection and waveform selection */
    TC3_REGS->TC_CHANNEL[1].TC_CMR =  TC_CMR_WAVEFORM_WAVSEL_UP_RC | TC_CMR_WAVE_Msk | (TC_CMR_WAVEFORM_CPCSTOP_Msk);

    /* write period */
    TC3_REGS->TC_CHANNEL[1].TC_RC = 1U;


    /* enable interrupt */
    TC3_REGS->TC_CHANNEL[1].TC_IER = TC_IER_CPCS_Msk;
    TC3_CH1_CallbackObj.callback_fn = NULL;
}

/* Start the timer */
void TC3_CH1_TimerStart (void)
{
    TC3_REGS->TC_CHANNEL[1].TC_CCR = (TC_CCR_CLKEN_Msk | TC_CCR_SWTRG_Msk);
}

/* Stop the timer */
void TC3_CH1_TimerStop (void)
{
    TC3_REGS->TC_CHANNEL[1].TC_CCR = (TC_CCR_CLKDIS_Msk);
}

uint32_t TC3_CH1_TimerFrequencyGet( void )
{
    return (uint32_t)(150000000UL);
}

/* Configure timer period */
void TC3_CH1_TimerPeriodSet (uint16_t period)
{
    TC3_REGS->TC_CHANNEL[1].TC_RC = period;
}


/* Read timer period */
uint16_t TC3_CH1_TimerPeriodGet (void)
{
    return TC3_REGS->TC_CHANNEL[1].TC_RC;
}

/* Read timer counter value */
uint16_t TC3_CH1_TimerCounterGet (void)
{
    return TC3_REGS->TC_CHANNEL[1].TC_CV;
}

/* Register callback for period interrupt */
void TC3_CH1_TimerCallbackRegister(TC_TIMER_CALLBACK callback, uintptr_t context)
{
    TC3_CH1_CallbackObj.callback_fn = callback;
    TC3_CH1_CallbackObj.context = context;
}
extern  void st_forward_plan_interrupt_callback(void);
extern   void st_exec_move_interrupt_callback(void);
/* Interrupt handler for Channel 1 */
void TC3_CH1_InterruptHandler(void)
{
    TC_TIMER_STATUS timer_status = (TC_TIMER_STATUS)(TC3_REGS->TC_CHANNEL[1].TC_SR & TC_TIMER_STATUS_MSK);
#if 1
    if (TC_TIMER_NONE != timer_status)
    {
       st_exec_move_interrupt_callback();
    }
    else
    {
        __NOP();
    }
#else
    /* Call registered callback function */
    if ((TC_TIMER_NONE != timer_status) && TC3_CH1_CallbackObj.callback_fn != NULL)
    {
        TC3_CH1_CallbackObj.callback_fn(timer_status, TC3_CH1_CallbackObj.context);
    }
#endif
}

/* Callback object for channel 2 */
TC_TIMER_CALLBACK_OBJECT TC3_CH2_CallbackObj;

/* Initialize channel in timer mode */
void TC3_CH2_TimerInitialize (void)
{
    /* Use peripheral clock */
    TC3_REGS->TC_CHANNEL[2].TC_EMR = TC_EMR_NODIVCLK_Msk;
    /* clock selection and waveform selection */
    TC3_REGS->TC_CHANNEL[2].TC_CMR =  TC_CMR_WAVEFORM_WAVSEL_UP_RC | TC_CMR_WAVE_Msk | (TC_CMR_WAVEFORM_CPCSTOP_Msk);

    /* write period */
    TC3_REGS->TC_CHANNEL[2].TC_RC = 1U;


    /* enable interrupt */
    TC3_REGS->TC_CHANNEL[2].TC_IER = TC_IER_CPCS_Msk;
    TC3_CH2_CallbackObj.callback_fn = NULL;
}

/* Start the timer */
void TC3_CH2_TimerStart (void)
{
    TC3_REGS->TC_CHANNEL[2].TC_CCR = (TC_CCR_CLKEN_Msk | TC_CCR_SWTRG_Msk);
}

/* Stop the timer */
void TC3_CH2_TimerStop (void)
{
    TC3_REGS->TC_CHANNEL[2].TC_CCR = (TC_CCR_CLKDIS_Msk);
}

uint32_t TC3_CH2_TimerFrequencyGet( void )
{
    return (uint32_t)(150000000UL);
}

/* Configure timer period */
void TC3_CH2_TimerPeriodSet (uint16_t period)
{
    TC3_REGS->TC_CHANNEL[2].TC_RC = period;
}


/* Read timer period */
uint16_t TC3_CH2_TimerPeriodGet (void)
{
    return TC3_REGS->TC_CHANNEL[2].TC_RC;
}

/* Read timer counter value */
uint16_t TC3_CH2_TimerCounterGet (void)
{
    return TC3_REGS->TC_CHANNEL[2].TC_CV;
}

/* Register callback for period interrupt */
void TC3_CH2_TimerCallbackRegister(TC_TIMER_CALLBACK callback, uintptr_t context)
{
    TC3_CH2_CallbackObj.callback_fn = callback;
    TC3_CH2_CallbackObj.context = context;
}

/* Interrupt handler for Channel 2 */
void TC3_CH2_InterruptHandler(void)
{
    TC_TIMER_STATUS timer_status = (TC_TIMER_STATUS)(TC3_REGS->TC_CHANNEL[2].TC_SR & TC_TIMER_STATUS_MSK);
#if 1
    if (TC_TIMER_NONE != timer_status)
    {
        st_forward_plan_interrupt_callback();
    }
#else 
    /* Call registered callback function */
    if ((TC_TIMER_NONE != timer_status) && TC3_CH2_CallbackObj.callback_fn != NULL)
    {
        TC3_CH2_CallbackObj.callback_fn(timer_status, TC3_CH2_CallbackObj.context);
    }
#endif
}

 
/**
 End of File
*/
