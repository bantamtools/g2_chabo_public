/*******************************************************************************
  USART2 SPI PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_usart2_spi.c

  Summary:
    USART2 SPI PLIB Implementation File

  Description:
    None

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2020 Microchip Technology Inc. and its subsidiaries.
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
#include "plib_usart2_spi.h"
#include "interrupts.h"

// *****************************************************************************
// *****************************************************************************
// Section: USART2 SPI Implementation
// *****************************************************************************
// *****************************************************************************
static USART_SPI_OBJECT usart2SPIObj;


void USART2_SPI_Initialize( void )
{
    /* Configure USART2 mode to SPI Master (0x0E) */
    USART2_REGS->US_MR = US_MR_SPI_USART_MODE(US_MR_SPI_USART_MODE_SPI_MASTER_Val);

    /* Reset SPI RX, SPI TX and SPI status */
    USART2_REGS->US_CR = (US_CR_SPI_RSTRX_Msk | US_CR_SPI_RSTTX_Msk | US_CR_SPI_RSTSTA_Msk);

    /* Configure clock source, clock phase, clock polarity and CKO = 1 */
    USART2_REGS->US_MR |= (US_MR_USART_USCLKS_MCK | US_MR_SPI_CHRL(US_MR_SPI_CHRL_8_BIT_Val) | US_MR_SPI_CPHA(0x0) | US_MR_SPI_CPOL(0x1) | US_MR_SPI_CLKO(1));

    /* Enable TX and RX */
    USART2_REGS->US_CR = (US_CR_SPI_RXEN_Msk | US_CR_SPI_TXEN_Msk);

    /* Configure USART2 Baud Rate */
    USART2_REGS->US_BRGR = US_BRGR_CD(150);

    /* Initialize instance object */
    usart2SPIObj.callback = NULL;
    usart2SPIObj.context = (uintptr_t)0;
    usart2SPIObj.transferIsBusy = false;
}

bool USART2_SPI_TransferSetup( USART_SPI_TRANSFER_SETUP * setup, uint32_t spiSourceClock )
{
    uint32_t clockDivider = 0;

    if ((setup == NULL) || (setup->clockFrequency == 0))
    {
        return false;
    }

    if(spiSourceClock == 0)
    {
        // Fetch Master Clock Frequency directly
        spiSourceClock = 150000000UL;
    }

    clockDivider = spiSourceClock/setup->clockFrequency;

    if(clockDivider < 6)
    {
        clockDivider = 6;
    }
    else if(clockDivider > 65535)
    {
        clockDivider = 65535;
    }

    USART2_REGS->US_MR = ((USART2_REGS->US_MR & ~US_MR_SPI_CPOL_Msk) | (uint32_t)setup->clockPolarity);
    USART2_REGS->US_MR = ((USART2_REGS->US_MR & ~US_MR_SPI_CPHA_Msk) | (uint32_t)setup->clockPhase);
    USART2_REGS->US_MR = ((USART2_REGS->US_MR & ~US_MR_SPI_CHRL_Msk) | (uint32_t)setup->dataBits);

    USART2_REGS->US_BRGR = ((USART2_REGS->US_BRGR & ~US_BRGR_CD_Msk) | US_BRGR_CD(clockDivider));

    return true;
}

bool USART2_SPI_WriteRead( void* pTransmitData, size_t txSize, void* pReceiveData, size_t rxSize )
{
    bool isRequestAccepted = false;
    uint32_t dummyData;

    /* Verify the request */
    if((((txSize > 0) && (pTransmitData != NULL)) || ((rxSize > 0) && (pReceiveData != NULL))) && (usart2SPIObj.transferIsBusy == false))
    {
        isRequestAccepted = true;

        usart2SPIObj.transferIsBusy = true;
        usart2SPIObj.txBuffer = pTransmitData;
        usart2SPIObj.rxBuffer = pReceiveData;
        usart2SPIObj.rxCount = 0;
        usart2SPIObj.txCount = 0;
        usart2SPIObj.dummySize = 0;

        if (pTransmitData != NULL)
        {
            usart2SPIObj.txSize = txSize;
        }
        else
        {
            usart2SPIObj.txSize = 0;
        }

        if (pReceiveData != NULL)
        {
            usart2SPIObj.rxSize = rxSize;
        }
        else
        {
            usart2SPIObj.rxSize = 0;
        }

        /* Reset over-run error if any */
        USART2_REGS->US_CR = US_CR_SPI_RSTSTA_Msk;

        /* Flush out any unread data in SPI read buffer */
        if (USART2_REGS->US_CSR & US_CSR_SPI_RXRDY_Msk)
        {
            dummyData = USART2_REGS->US_RHR;
            (void)dummyData;
        }

        if (usart2SPIObj.rxSize > usart2SPIObj.txSize)
        {
            usart2SPIObj.dummySize = usart2SPIObj.rxSize - usart2SPIObj.txSize;
        }

        /* Start the first write here itself, rest will happen in ISR context */
        if ((USART2_REGS->US_MR & US_MR_SPI_CHRL_Msk) == US_MR_SPI_CHRL_8_BIT)
        {

            if (usart2SPIObj.txCount < usart2SPIObj.txSize)
            {
                USART2_REGS->US_THR = *((uint8_t*)usart2SPIObj.txBuffer);
                usart2SPIObj.txCount++;
            }
            else if (usart2SPIObj.dummySize > 0)
            {
                USART2_REGS->US_THR = (uint8_t)(0xff);
                usart2SPIObj.dummySize--;
            }
        }

        if (rxSize > 0)
        {
            /* Enable receive interrupt to complete the transfer in ISR context */
            USART2_REGS->US_IER = US_IER_SPI_RXRDY_Msk;
        }
        else
        {
            /* Enable transmit interrupt to complete the transfer in ISR context */
            USART2_REGS->US_IER = US_IER_SPI_TXRDY_Msk;
        }
    }

    return isRequestAccepted;
}

bool USART2_SPI_Write( void* pTransmitData, size_t txSize )
{
    return(USART2_SPI_WriteRead(pTransmitData, txSize, NULL, 0));
}

bool USART2_SPI_Read( void* pReceiveData, size_t rxSize )
{
    return(USART2_SPI_WriteRead(NULL, 0, pReceiveData, rxSize));
}

bool USART2_SPI_IsBusy( void )
{
    return ((usart2SPIObj.transferIsBusy) || ((USART2_REGS->US_CSR & US_CSR_SPI_TXEMPTY_Msk) == 0));
}

void USART2_SPI_CallbackRegister( USART_SPI_CALLBACK callback, uintptr_t context )
{
    usart2SPIObj.callback = callback;
    usart2SPIObj.context = context;
}

void USART2_InterruptHandler( void )
{
    uint32_t receivedData;


    if (USART2_REGS->US_CSR & US_CSR_SPI_RXRDY_Msk)
    {
        receivedData = USART2_REGS->US_RHR;

        if (usart2SPIObj.rxCount < usart2SPIObj.rxSize)
        {
            ((uint8_t*)usart2SPIObj.rxBuffer)[usart2SPIObj.rxCount++] = receivedData;
        }
    }

    if(USART2_REGS->US_CSR & US_CSR_SPI_TXRDY_Msk)
    {
        /* Disable the TXRDY interrupt. This will be enabled back if one or more
         * bytes are pending transmission */

        USART2_REGS->US_IDR = US_IDR_SPI_TXRDY_Msk;

        if (usart2SPIObj.txCount < usart2SPIObj.txSize)
        {
            USART2_REGS->US_THR = ((uint8_t*)usart2SPIObj.txBuffer)[usart2SPIObj.txCount++];
        }
        else if (usart2SPIObj.dummySize > 0)
        {
            USART2_REGS->US_THR = (uint8_t)(0xff);
            usart2SPIObj.dummySize--;
        }

        if ((usart2SPIObj.txCount == usart2SPIObj.txSize) && (usart2SPIObj.dummySize == 0))
        {
            if (USART2_REGS->US_CSR & US_CSR_SPI_TXEMPTY_Msk)
            {
                /* Disable all interrupt sources - RXRDY, TXRDY and TXEMPTY */
                USART2_REGS->US_IDR = (US_IDR_SPI_RXRDY_Msk | US_IDR_SPI_TXRDY_Msk | US_IDR_SPI_TXEMPTY_Msk);

                usart2SPIObj.transferIsBusy = false;


                /* All characters are transmitted out and TX shift register is empty */
                if(usart2SPIObj.callback != NULL)
                {
                    usart2SPIObj.callback(usart2SPIObj.context);
                }
            }
            else
            {
                /* Enable TXEMPTY interrupt */
                USART2_REGS->US_IER = US_IER_SPI_TXEMPTY_Msk;
            }
        }
        else if (usart2SPIObj.rxCount == usart2SPIObj.rxSize)
        {
            /* Enable TXRDY interrupt as all the requested bytes are received
             * and can now make use of the internal transmit shift register.
             */
            USART2_REGS->US_IDR = US_IDR_SPI_RXRDY_Msk;
            USART2_REGS->US_IER = US_IER_SPI_TXRDY_Msk;
        }
    }
}
