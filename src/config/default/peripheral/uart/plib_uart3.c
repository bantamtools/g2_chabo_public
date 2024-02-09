/*******************************************************************************
  UART3 PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_uart3.c

  Summary:
    UART3 PLIB Implementation File

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
#include "plib_uart3.h"
#include "interrupts.h"

// *****************************************************************************
// *****************************************************************************
// Section: UART3 Implementation
// *****************************************************************************
// *****************************************************************************

UART_RING_BUFFER_OBJECT uart3Obj;

#define UART3_READ_BUFFER_SIZE      2000
/* Disable Read, Overrun, Parity and Framing error interrupts */
#define UART3_RX_INT_DISABLE()      UART3_REGS->UART_IDR = (UART_IDR_RXRDY_Msk | UART_IDR_FRAME_Msk | UART_IDR_PARE_Msk | UART_IDR_OVRE_Msk);
/* Enable Read, Overrun, Parity and Framing error interrupts */
#define UART3_RX_INT_ENABLE()       UART3_REGS->UART_IER = (UART_IER_RXRDY_Msk | UART_IER_FRAME_Msk | UART_IER_PARE_Msk | UART_IER_OVRE_Msk);

static uint8_t UART3_ReadBuffer[UART3_READ_BUFFER_SIZE];

#define UART3_WRITE_BUFFER_SIZE     2000
#define UART3_TX_INT_DISABLE()      UART3_REGS->UART_IDR = UART_IDR_TXEMPTY_Msk;
#define UART3_TX_INT_ENABLE()       UART3_REGS->UART_IER = UART_IER_TXEMPTY_Msk;

static uint8_t UART3_WriteBuffer[UART3_WRITE_BUFFER_SIZE];

void UART3_Initialize( void )
{
    /* Reset UART3 */
    UART3_REGS->UART_CR = (UART_CR_RSTRX_Msk | UART_CR_RSTTX_Msk | UART_CR_RSTSTA_Msk);

    /* Enable UART3 */
    UART3_REGS->UART_CR = (UART_CR_TXEN_Msk | UART_CR_RXEN_Msk);

    /* Configure UART3 mode */
    UART3_REGS->UART_MR = ((UART_MR_BRSRCCK_PERIPH_CLK) | (UART_MR_PAR_NO) | (0 << UART_MR_FILTER_Pos));

    /* Configure UART3 Baud Rate */
    UART3_REGS->UART_BRGR = UART_BRGR_CD(81);

    /* Initialize instance object */
    uart3Obj.rdCallback = NULL;
    uart3Obj.rdInIndex = 0;
	uart3Obj.rdOutIndex = 0;
    uart3Obj.isRdNotificationEnabled = false;
    uart3Obj.isRdNotifyPersistently = false;
    uart3Obj.rdThreshold = 0;
    uart3Obj.wrCallback = NULL;
    uart3Obj.wrInIndex = 0;
	uart3Obj.wrOutIndex = 0;
    uart3Obj.isWrNotificationEnabled = false;
    uart3Obj.isWrNotifyPersistently = false;
    uart3Obj.wrThreshold = 0;

    /* Enable receive interrupt */
    UART3_RX_INT_ENABLE()
}

bool UART3_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud = setup->baudRate;
    uint32_t brgVal = 0;
    uint32_t uartMode;

    if (setup != NULL)
    {
        if(srcClkFreq == 0)
        {
            srcClkFreq = UART3_FrequencyGet();
        }

        /* Calculate BRG value */
        brgVal = srcClkFreq / (16 * baud);

        /* If the target baud rate is acheivable using this clock */
        if (brgVal <= 65535)
        {
            /* Configure UART3 mode */
            uartMode = UART3_REGS->UART_MR;
            uartMode &= ~UART_MR_PAR_Msk;
            UART3_REGS->UART_MR = uartMode | setup->parity ;

            /* Configure UART3 Baud Rate */
            UART3_REGS->UART_BRGR = UART_BRGR_CD(brgVal);

            status = true;
        }
    }

    return status;
}

static void UART3_ErrorClear( void )
{
    uint8_t dummyData = 0u;

    UART3_REGS->UART_CR = UART_CR_RSTSTA_Msk;

    /* Flush existing error bytes from the RX FIFO */
    while( UART_SR_RXRDY_Msk == (UART3_REGS->UART_SR & UART_SR_RXRDY_Msk) )
    {
        dummyData = (UART3_REGS->UART_RHR & UART_RHR_RXCHR_Msk);
    }

    /* Ignore the warning */
    (void)dummyData;
}

UART_ERROR UART3_ErrorGet( void )
{
    UART_ERROR errors = UART_ERROR_NONE;
    uint32_t status = UART3_REGS->UART_SR;

    errors = (UART_ERROR)(status & (UART_SR_OVRE_Msk | UART_SR_PARE_Msk | UART_SR_FRAME_Msk));

    if(errors != UART_ERROR_NONE)
    {
        UART3_ErrorClear();
    }

    /* All errors are cleared, but send the previous error state */
    return errors;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static inline bool UART3_RxPushByte(uint8_t rdByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    tempInIndex = uart3Obj.rdInIndex + 1;

    if (tempInIndex >= UART3_READ_BUFFER_SIZE)
    {
        tempInIndex = 0;
    }

    if (tempInIndex == uart3Obj.rdOutIndex)
    {
        /* Queue is full - Report it to the application. Application gets a chance to free up space by reading data out from the RX ring buffer */
        if(uart3Obj.rdCallback != NULL)
        {
            uart3Obj.rdCallback(UART_EVENT_READ_BUFFER_FULL, uart3Obj.rdContext);
        }

        /* Read the indices again in case application has freed up space in RX ring buffer */
        tempInIndex = uart3Obj.rdInIndex + 1;

        if (tempInIndex >= UART3_READ_BUFFER_SIZE)
        {
            tempInIndex = 0;
        }

    }

    if (tempInIndex != uart3Obj.rdOutIndex)
    {
        UART3_ReadBuffer[uart3Obj.rdInIndex] = rdByte;
        uart3Obj.rdInIndex = tempInIndex;
        isSuccess = true;
    }
    else
    {
        /* Queue is full. Data will be lost. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART3_ReadNotificationSend(void)
{
    uint32_t nUnreadBytesAvailable;

    if (uart3Obj.isRdNotificationEnabled == true)
    {
        nUnreadBytesAvailable = UART3_ReadCountGet();

        if(uart3Obj.rdCallback != NULL)
        {
            if (uart3Obj.isRdNotifyPersistently == true)
            {
                if (nUnreadBytesAvailable >= uart3Obj.rdThreshold)
                {
                    uart3Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart3Obj.rdContext);
                }
            }
            else
            {
                if (nUnreadBytesAvailable == uart3Obj.rdThreshold)
                {
                    uart3Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart3Obj.rdContext);
                }
            }
        }
    }
}

size_t UART3_Read(uint8_t* pRdBuffer, const size_t size)
{
    size_t nBytesRead = 0;
	uint32_t rdOutIndex;
	uint32_t rdInIndex;

    while (nBytesRead < size)
    {
        UART3_RX_INT_DISABLE();
		
		rdOutIndex = uart3Obj.rdOutIndex;
		rdInIndex = uart3Obj.rdInIndex;

        if (rdOutIndex != rdInIndex)
        {
            pRdBuffer[nBytesRead++] = UART3_ReadBuffer[uart3Obj.rdOutIndex++];

            if (uart3Obj.rdOutIndex >= UART3_READ_BUFFER_SIZE)
            {
                uart3Obj.rdOutIndex = 0;
            }
            UART3_RX_INT_ENABLE();
        }
        else
        {
            UART3_RX_INT_ENABLE();
            break;
        }
    }

    return nBytesRead;
}

size_t UART3_ReadCountGet(void)
{
    size_t nUnreadBytesAvailable;
	uint32_t rdInIndex;
	uint32_t rdOutIndex;
    
	/* Take  snapshot of indices to avoid creation of critical section */
	rdInIndex = uart3Obj.rdInIndex;
	rdOutIndex = uart3Obj.rdOutIndex;

    if ( rdInIndex >=  rdOutIndex)
    {
        nUnreadBytesAvailable =  rdInIndex - rdOutIndex;
    }
    else
    {
        nUnreadBytesAvailable =  (UART3_READ_BUFFER_SIZE -  rdOutIndex) + rdInIndex;
    }
    
    return nUnreadBytesAvailable;
}

size_t UART3_ReadFreeBufferCountGet(void)
{
    return (UART3_READ_BUFFER_SIZE - 1) - UART3_ReadCountGet();
}

size_t UART3_ReadBufferSizeGet(void)
{
    return (UART3_READ_BUFFER_SIZE - 1);
}

bool UART3_ReadNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart3Obj.isRdNotificationEnabled;

    uart3Obj.isRdNotificationEnabled = isEnabled;

    uart3Obj.isRdNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART3_ReadThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0)
    {
        uart3Obj.rdThreshold = nBytesThreshold;
    }
}

void UART3_ReadCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart3Obj.rdCallback = callback;

    uart3Obj.rdContext = context;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static bool UART3_TxPullByte(uint8_t* pWrByte)
{
    bool isSuccess = false;
	uint32_t wrOutIndex = uart3Obj.wrOutIndex;
	uint32_t wrInIndex = uart3Obj.wrInIndex;

    if (wrOutIndex != wrInIndex)
    {
        *pWrByte = UART3_WriteBuffer[uart3Obj.wrOutIndex++];

        if (uart3Obj.wrOutIndex >= UART3_WRITE_BUFFER_SIZE)
        {
            uart3Obj.wrOutIndex = 0;
        }
        isSuccess = true;
    }

    return isSuccess;
}

static inline bool UART3_TxPushByte(uint8_t wrByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    tempInIndex = uart3Obj.wrInIndex + 1;

    if (tempInIndex >= UART3_WRITE_BUFFER_SIZE)
    {
        tempInIndex = 0;
    }
    if (tempInIndex != uart3Obj.wrOutIndex)
    {
        UART3_WriteBuffer[uart3Obj.wrInIndex] = wrByte;
        uart3Obj.wrInIndex = tempInIndex;
        isSuccess = true;
    }
    else
    {
        /* Queue is full. Report Error. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART3_WriteNotificationSend(void)
{
    uint32_t nFreeWrBufferCount;

    if (uart3Obj.isWrNotificationEnabled == true)
    {
        nFreeWrBufferCount = UART3_WriteFreeBufferCountGet();

        if(uart3Obj.wrCallback != NULL)
        {
            if (uart3Obj.isWrNotifyPersistently == true)
            {
                if (nFreeWrBufferCount >= uart3Obj.wrThreshold)
                {
                    uart3Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart3Obj.wrContext);
                }
            }
            else
            {
                if (nFreeWrBufferCount == uart3Obj.wrThreshold)
                {
                    uart3Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart3Obj.wrContext);
                }
            }
        }
    }
}

static size_t UART3_WritePendingBytesGet(void)
{
    size_t nPendingTxBytes;
	
	/* Take a snapshot of indices to avoid creation of critical section */
	uint32_t wrOutIndex = uart3Obj.wrOutIndex;
	uint32_t wrInIndex = uart3Obj.wrInIndex;

    if ( wrInIndex >=  wrOutIndex)
    {
        nPendingTxBytes =  wrInIndex -  wrOutIndex;
    }
    else
    {
        nPendingTxBytes =  (UART3_WRITE_BUFFER_SIZE -  wrOutIndex) + wrInIndex;
    }

    return nPendingTxBytes;
}

size_t UART3_WriteCountGet(void)
{
    size_t nPendingTxBytes;    

    nPendingTxBytes = UART3_WritePendingBytesGet();

    return nPendingTxBytes;
}

size_t UART3_Write(uint8_t* pWrBuffer, const size_t size )
{
    size_t nBytesWritten  = 0;

    UART3_TX_INT_DISABLE();

    while (nBytesWritten < size)
    {
        if (UART3_TxPushByte(pWrBuffer[nBytesWritten]) == true)
        {
            nBytesWritten++;
        }
        else
        {
            /* Queue is full, exit the loop */
            break;
        }
    }

    /* Check if any data is pending for transmission */
    if (UART3_WritePendingBytesGet() > 0)
    {
        /* Enable TX interrupt as data is pending for transmission */
        UART3_TX_INT_ENABLE();
    }

    return nBytesWritten;
}

size_t UART3_WriteFreeBufferCountGet(void)
{
    return (UART3_WRITE_BUFFER_SIZE - 1) - UART3_WriteCountGet();
}

size_t UART3_WriteBufferSizeGet(void)
{
    return (UART3_WRITE_BUFFER_SIZE - 1);
}

bool UART3_WriteNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart3Obj.isWrNotificationEnabled;

    uart3Obj.isWrNotificationEnabled = isEnabled;

    uart3Obj.isWrNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART3_WriteThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0)
    {
        uart3Obj.wrThreshold = nBytesThreshold;
    }
}

void UART3_WriteCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart3Obj.wrCallback = callback;

    uart3Obj.wrContext = context;
}

static void UART3_ISR_RX_Handler( void )
{
    /* Keep reading until there is a character availabe in the RX FIFO */
    while(UART_SR_RXRDY_Msk == (UART3_REGS->UART_SR& UART_SR_RXRDY_Msk))
    {
        if (UART3_RxPushByte( (uint8_t )(UART3_REGS->UART_RHR& UART_RHR_RXCHR_Msk) ) == true)
        {
            UART3_ReadNotificationSend();
        }
        else
        {
            /* UART RX buffer is full */
        }
    }
}

static void UART3_ISR_TX_Handler( void )
{
    uint8_t wrByte;

    /* Keep writing to the TX FIFO as long as there is space */
    while(UART_SR_TXRDY_Msk == (UART3_REGS->UART_SR & UART_SR_TXRDY_Msk))
    {
        if (UART3_TxPullByte(&wrByte) == true)
        {
            UART3_REGS->UART_THR |= wrByte;

            /* Send notification */
            UART3_WriteNotificationSend();
        }
        else
        {
            /* Nothing to transmit. Disable the data register empty interrupt. */
            UART3_TX_INT_DISABLE();
            break;
        }
    }
}

void UART3_InterruptHandler( void )
{
    /* Error status */
    uint32_t errorStatus = (UART3_REGS->UART_SR & (UART_SR_OVRE_Msk | UART_SR_FRAME_Msk | UART_SR_PARE_Msk));

    if(errorStatus != 0)
    {
        /* Client must call UARTx_ErrorGet() function to clear the errors */

        /* Disable Read, Overrun, Parity and Framing error interrupts */

        UART3_REGS->UART_IDR = (UART_IDR_RXRDY_Msk | UART_IDR_FRAME_Msk | UART_IDR_PARE_Msk | UART_IDR_OVRE_Msk);

        /* UART errors are normally associated with the receiver, hence calling
         * receiver callback */
        if( uart3Obj.rdCallback != NULL )
        {
            uart3Obj.rdCallback(UART_EVENT_READ_ERROR, uart3Obj.rdContext);
        }
    }

    /* Receiver status */
    if(UART_SR_RXRDY_Msk == (UART3_REGS->UART_SR & UART_SR_RXRDY_Msk))
    {
        UART3_ISR_RX_Handler();
    }

    /* Transmitter status */
    if(UART_SR_TXRDY_Msk == (UART3_REGS->UART_SR & UART_SR_TXRDY_Msk))
    {
        UART3_ISR_TX_Handler();
    }

}