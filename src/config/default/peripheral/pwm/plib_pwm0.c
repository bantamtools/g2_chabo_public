/*******************************************************************************
  PWM Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_PWM0.c

  Summary
    PWM0 peripheral library source file.

  Description
    This file implements the interface to the PWM peripheral library.  This
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
#include "plib_pwm0.h"
#include "interrupts.h"

#define SPINDLE_CHANNEL_NDX 3 

void PWM0_increment_spindle_speed(void)
{
    //min duty cycle 13.8%
    //max duty_cycle % 19.3
#define SPINDLE_DUTY_CYCLE_MIN  790//850// add a bit above min 808
#define SPINDLE_DUTY_CYCLE_MAX  1225
#define SPINDLE_DUTY_CYCLE_DELTA_RANGE 316
    

#define DUTY_CYCLE_INCREMENT (SPINDLE_DUTY_CYCLE_DELTA_RANGE/10)
    static uint16_t  duty_cycle_counts = PWM0_SPINDLE_IDLE_DUTY_CYCLE;
    static int16_t duty_cycle_increment = DUTY_CYCLE_INCREMENT;

   PWM0_REGS->PWM_CH_NUM[3].PWM_CDTY =duty_cycle_counts;
   if (duty_cycle_increment>0)
   {
      if(duty_cycle_counts+duty_cycle_increment>SPINDLE_DUTY_CYCLE_MAX)
      {
       duty_cycle_increment = -DUTY_CYCLE_INCREMENT;//decrement instead of increment 
      }
   }  
   else 
   {
      if (duty_cycle_counts+duty_cycle_increment<SPINDLE_DUTY_CYCLE_MIN)
      {
         duty_cycle_increment =DUTY_CYCLE_INCREMENT;// increment
      }
       
    }
    duty_cycle_counts+=duty_cycle_increment;
   if  (duty_cycle_counts<SPINDLE_DUTY_CYCLE_MIN)
   {
       duty_cycle_counts=SPINDLE_DUTY_CYCLE_MIN;
   }
   else if  (duty_cycle_counts>SPINDLE_DUTY_CYCLE_MAX)
   {
       duty_cycle_counts=SPINDLE_DUTY_CYCLE_MAX;
        duty_cycle_increment = -DUTY_CYCLE_INCREMENT;//decrement instead of increment 
   }
}


void PWM0_SetSpindleDutyCycleCounts(uint16_t counts)
{
    PWM0_REGS->PWM_CH_NUM[SPINDLE_CHANNEL_NDX].PWM_CDTY=(int)counts;
}
uint16_t PWM0_GetSpindlePwmIdleCounts(void){return  PWM0_SPINDLE_IDLE_DUTY_CYCLE;}

uint16_t PWM0_GetSpindlePwmPeriodCounts(void){return PWM0_REGS->PWM_CH_NUM[SPINDLE_CHANNEL_NDX].PWM_CPRD;}
void PWM0_SetSpindleDutyCycle(float duty_cycle_fraction)
{
   volatile float pwm_period_counts = PWM0_REGS->PWM_CH_NUM[SPINDLE_CHANNEL_NDX].PWM_CPRD;
   volatile float duty_cycle_counts = (duty_cycle_fraction*pwm_period_counts); 
   
   //need to convert the result--observed on scope e.g.: 10% duty was appearing as 90% duty:
  // duty_cycle_counts=pwm_period_counts-duty_cycle_counts;
    PWM0_REGS->PWM_CH_NUM[SPINDLE_CHANNEL_NDX].PWM_CDTY=(int)duty_cycle_counts;
}
/* Initialize enabled PWM channels */
void PWM0_Initialize (void)
{

    /* Synchronous channels configuration */
    PWM0_REGS->PWM_SCM = PWM_SCM_SYNC0_Msk | PWM_SCM_SYNC1_Msk | PWM_SCM_UPDM_MODE2;
    PWM0_REGS->PWM_SCUP = PWM_SCUP_UPR(0U);

    /************** Channel 0 *************************/
    /* PWM channel mode configurations */
    PWM0_REGS->PWM_CH_NUM[0].PWM_CMR = PWM_CMR_CPRE_MCK | PWM_CMR_CALG_CENTER_ALIGNED
                    | PWM_CMR_CPOL_LOW_POLARITY | PWM_CMR_UPDS_UPDATE_AT_PERIOD \
                    | PWM_CMR_CES_SINGLE_EVENT ;

    /* PWM period */
    PWM0_REGS->PWM_CH_NUM[0].PWM_CPRD = 90U;

    /* PWM duty cycle */
    PWM0_REGS->PWM_CH_NUM[0].PWM_CDTY = 90U;
 
     

    /************** Channel 1 *************************/

    /* PWM duty cycle */
    PWM0_REGS->PWM_CH_NUM[1].PWM_CDTY = 90U;
 
     
 

    /************** Channel 3 *************************/
    /* PWM channel mode configurations */
    PWM0_REGS->PWM_CH_NUM[3].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_256 | PWM_CMR_CALG_LEFT_ALIGNED
                    | PWM_CMR_CPOL_HIGH_POLARITY | PWM_CMR_UPDS_UPDATE_AT_PERIOD \
                    | PWM_CMR_CES_SINGLE_EVENT|PWM_CMR_DPOLI(1) ;

    /* PWM period */
    PWM0_REGS->PWM_CH_NUM[3].PWM_CPRD = PWM0_SPINDLE_PERIOD_COUNTS;// 5858U;

    /* PWM duty cycle */
    PWM0_REGS->PWM_CH_NUM[3].PWM_CDTY = PWM0_SPINDLE_IDLE_DUTY_CYCLE;//10% DUTY CYCLE IS IDLE FOR ESC
 
     
}

/* Start the PWM generation */
void PWM0_ChannelsStart (PWM_CHANNEL_MASK channelMask)
{
    PWM0_REGS->PWM_ENA = channelMask;
}

/* Stop the PWM generation */
void PWM0_ChannelsStop (PWM_CHANNEL_MASK channelMask)
{
    PWM0_REGS->PWM_DIS = channelMask;
}

/* configure PWM period */
void PWM0_ChannelPeriodSet (PWM_CHANNEL_NUM channel, uint16_t period)
{
    PWM0_REGS->PWM_CH_NUM[channel].PWM_CPRDUPD = period;
}

/* Read PWM period */
uint16_t PWM0_ChannelPeriodGet (PWM_CHANNEL_NUM channel)
{
    return (uint16_t)PWM0_REGS->PWM_CH_NUM[channel].PWM_CPRD;
}

/* Configure dead time */
void PWM0_ChannelDeadTimeSet (PWM_CHANNEL_NUM channel, uint16_t deadtime_high, uint16_t deadtime_low)
{
    PWM0_REGS->PWM_CH_NUM[channel].PWM_DTUPD = ((deadtime_low << PWM_DT_DTL_Pos) | deadtime_high);
}

/* Configure compare unit value */
void PWM0_CompareValueSet (PWM_COMPARE cmp_unit, uint16_t cmp_value)
{
    PWM0_REGS->PWM_CMP[cmp_unit].PWM_CMPVUPD = cmp_value;
}

/* Enable counter event */
void PWM0_ChannelCounterEventEnable (PWM_CHANNEL_MASK channelMask)
{
    PWM0_REGS->PWM_IER1 = channelMask;
}

/* Disable counter event */
void PWM0_ChannelCounterEventDisable (PWM_CHANNEL_MASK channelMask)
{
    PWM0_REGS->PWM_IDR1 = channelMask;
}


/* Enable synchronous update */
void PWM0_SyncUpdateEnable (void)
{
    PWM0_REGS->PWM_SCUC = PWM_SCUC_UPDULOCK_Msk;
}

/* Clear the fault status */
void PWM0_FaultStatusClear(PWM_FAULT_ID fault_id)
{
    PWM0_REGS->PWM_FCR = 0x1U << fault_id;
}

/* Override PWM outputs */
void PWM0_ChannelOverrideEnable(PWM_CHANNEL_NUM channel)
{
    PWM0_REGS->PWM_OS &= ~((1 << channel) | (1 << (channel + 16)));
    PWM0_REGS->PWM_OS |= ((0 << channel) | (0 << (channel + 16)));
}

/* Disable override of PWM outputs */
void PWM0_ChannelOverrideDisable(PWM_CHANNEL_NUM channel)
{
    PWM0_REGS->PWM_OS |= ((1 << channel) | (1 << (channel + 16)));
}


/* Check the status of counter event */
bool PWM0_ChannelCounterEventStatusGet (PWM_CHANNEL_NUM channel)
{
    bool status;
    status = (PWM0_REGS->PWM_ISR1 >> channel) & 0x1U;
    return status;
}

/**
 End of File
*/
