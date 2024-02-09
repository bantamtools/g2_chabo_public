/*******************************************************************************
  PIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_pio.h

  Summary:
    PIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (PIO) module.

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

#ifndef PLIB_PIO_H
#define PLIB_PIO_H

#include "device.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************

/*** SME: forward references ***/
 
/*** Macros for X_STPR_ENn pin ***/
#define X_STPR_ENn_Set()               (PIOD_REGS->PIO_SODR = (1<<0))
#define X_STPR_ENn_Clear()             (PIOD_REGS->PIO_CODR = (1<<0))
#define X_STPR_ENn_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<0))
#define X_STPR_ENn_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<0))
#define X_STPR_ENn_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<0))
#define X_STPR_ENn_Get()               ((PIOD_REGS->PIO_PDSR >> 0) & 0x1)
#define X_STPR_ENn_PIN                  PIO_PIN_PD0

/*** Macros for X_MTRDRV_SPI_CSn pin ***/
#define X_MTRDRV_SPI_CSn_Set()               (PIOD_REGS->PIO_SODR = (1<<31))
#define X_MTRDRV_SPI_CSn_Clear()             (PIOD_REGS->PIO_CODR = (1<<31))
#define X_MTRDRV_SPI_CSn_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<31))
#define X_MTRDRV_SPI_CSn_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<31))
#define X_MTRDRV_SPI_CSn_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<31))
#define X_MTRDRV_SPI_CSn_Get()               ((PIOD_REGS->PIO_PDSR >> 31) & 0x1)
#define X_MTRDRV_SPI_CSn_PIN                  PIO_PIN_PD31

/*** Macros for X_MTRDRV_STEP pin ***/
#define X_MTRDRV_STEP_Set()               (PIOB_REGS->PIO_SODR = (1<<1))
#define X_MTRDRV_STEP_Clear()             (PIOB_REGS->PIO_CODR = (1<<1))
#define X_MTRDRV_STEP_Toggle()            (PIOB_REGS->PIO_ODSR ^= (1<<1))
#define X_MTRDRV_STEP_OutputEnable()      (PIOB_REGS->PIO_OER = (1<<1))
#define X_MTRDRV_STEP_InputEnable()       (PIOB_REGS->PIO_ODR = (1<<1))
#define X_MTRDRV_STEP_Get()               ((PIOB_REGS->PIO_PDSR >> 1) & 0x1)
#define X_MTRDRV_STEP_PIN                  PIO_PIN_PB1

/*** Macros for X_STPR_DIR pin ***/
#define X_STPR_DIR_Set()               (PIOB_REGS->PIO_SODR = (1<<0))
#define X_STPR_DIR_Clear()             (PIOB_REGS->PIO_CODR = (1<<0))
#define X_STPR_DIR_Toggle()            (PIOB_REGS->PIO_ODSR ^= (1<<0))
#define X_STPR_DIR_OutputEnable()      (PIOB_REGS->PIO_OER = (1<<0))
#define X_STPR_DIR_InputEnable()       (PIOB_REGS->PIO_ODR = (1<<0))
#define X_STPR_DIR_Get()               ((PIOB_REGS->PIO_PDSR >> 0) & 0x1)
#define X_STPR_DIR_PIN                  PIO_PIN_PB0

/*** Macros for Y1_MTRDRV_SPI_CSn pin ***/
#define Y1_MTRDRV_SPI_CSn_Set()               (PIOA_REGS->PIO_SODR = (1<<19))
#define Y1_MTRDRV_SPI_CSn_Clear()             (PIOA_REGS->PIO_CODR = (1<<19))
#define Y1_MTRDRV_SPI_CSn_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<19))
#define Y1_MTRDRV_SPI_CSn_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<19))
#define Y1_MTRDRV_SPI_CSn_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<19))
#define Y1_MTRDRV_SPI_CSn_Get()               ((PIOA_REGS->PIO_PDSR >> 19) & 0x1)
#define Y1_MTRDRV_SPI_CSn_PIN                  PIO_PIN_PA19

/*** Macros for Y1_STPR_ENn pin ***/
#define Y1_STPR_ENn_Set()               (PIOA_REGS->PIO_SODR = (1<<18))
#define Y1_STPR_ENn_Clear()             (PIOA_REGS->PIO_CODR = (1<<18))
#define Y1_STPR_ENn_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<18))
#define Y1_STPR_ENn_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<18))
#define Y1_STPR_ENn_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<18))
#define Y1_STPR_ENn_Get()               ((PIOA_REGS->PIO_PDSR >> 18) & 0x1)
#define Y1_STPR_ENn_PIN                  PIO_PIN_PA18

/*** Macros for Y1_MTRDRV_STEP pin ***/
#define Y1_MTRDRV_STEP_Set()               (PIOA_REGS->PIO_SODR = (1<<17))
#define Y1_MTRDRV_STEP_Clear()             (PIOA_REGS->PIO_CODR = (1<<17))
#define Y1_MTRDRV_STEP_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<17))
#define Y1_MTRDRV_STEP_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<17))
#define Y1_MTRDRV_STEP_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<17))
#define Y1_MTRDRV_STEP_Get()               ((PIOA_REGS->PIO_PDSR >> 17) & 0x1)
#define Y1_MTRDRV_STEP_PIN                  PIO_PIN_PA17

/*** Macros for Y1_STPR_DIR pin ***/
#define Y1_STPR_DIR_Set()               (PIOB_REGS->PIO_SODR = (1<<2))
#define Y1_STPR_DIR_Clear()             (PIOB_REGS->PIO_CODR = (1<<2))
#define Y1_STPR_DIR_Toggle()            (PIOB_REGS->PIO_ODSR ^= (1<<2))
#define Y1_STPR_DIR_OutputEnable()      (PIOB_REGS->PIO_OER = (1<<2))
#define Y1_STPR_DIR_InputEnable()       (PIOB_REGS->PIO_ODR = (1<<2))
#define Y1_STPR_DIR_Get()               ((PIOB_REGS->PIO_PDSR >> 2) & 0x1)
#define Y1_STPR_DIR_PIN                  PIO_PIN_PB2

/*** Macros for CHABO_MODE pin ***/
#define CHABO_MODE_Set()               (PIOA_REGS->PIO_SODR = (1<<21))
#define CHABO_MODE_Clear()             (PIOA_REGS->PIO_CODR = (1<<21))
#define CHABO_MODE_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<21))
#define CHABO_MODE_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<21))
#define CHABO_MODE_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<21))
#define CHABO_MODE_Get()               ((PIOA_REGS->PIO_PDSR >> 21) & 0x1)
#define CHABO_MODE_PIN                  PIO_PIN_PA21

/*** Macros for DEBUG_UART_TX pin ***/
#define DEBUG_UART_TX_Get()               ((PIOD_REGS->PIO_PDSR >> 30) & 0x1)
#define DEBUG_UART_TX_PIN                  PIO_PIN_PD30

/*** Macros for SPINDLE_PWM_OUT pin ***/
#define SPINDLE_PWM_OUT_Get()               ((PIOA_REGS->PIO_PDSR >> 7) & 0x1)
#define SPINDLE_PWM_OUT_PIN                  PIO_PIN_PA7

/*** Macros for ESTOP pin ***/
#define ESTOP_Set()               (PIOA_REGS->PIO_SODR = (1<<22))
#define ESTOP_Clear()             (PIOA_REGS->PIO_CODR = (1<<22))
#define ESTOP_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<22))
#define ESTOP_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<22))
#define ESTOP_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<22))
#define ESTOP_Get()               ((PIOA_REGS->PIO_PDSR >> 22) & 0x1)
#define ESTOP_PIN                  PIO_PIN_PA22
#define ESTOP_FLAG_Pin              ESTOP_PIN 
/*** Macros for ESC_EN pin ***/
#define ESC_EN_Set()               (PIOA_REGS->PIO_SODR = (1<<13))
#define ESC_EN_Clear()             (PIOA_REGS->PIO_CODR = (1<<13))
#define ESC_EN_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<13))
#define ESC_EN_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<13))
#define ESC_EN_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<13))
#define ESC_EN_Get()               ((PIOA_REGS->PIO_PDSR >> 13) & 0x1)
#define ESC_EN_PIN                  PIO_PIN_PA13

/*** Macros for ROSECOMB_DETn pin ***/
#define ROSECOMB_DETn_Set()               (PIOA_REGS->PIO_SODR = (1<<23))
#define ROSECOMB_DETn_Clear()             (PIOA_REGS->PIO_CODR = (1<<23))
#define ROSECOMB_DETn_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<23))
#define ROSECOMB_DETn_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<23))
#define ROSECOMB_DETn_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<23))
#define ROSECOMB_DETn_Get()               ((PIOA_REGS->PIO_PDSR >> 23) & 0x1)
#define ROSECOMB_DETn_PIN                  PIO_PIN_PA23

/*** Macros for MICROSD_CD pin ***/
#define MICROSD_CD_Set()               (PIOA_REGS->PIO_SODR = (1<<15))
#define MICROSD_CD_Clear()             (PIOA_REGS->PIO_CODR = (1<<15))
#define MICROSD_CD_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<15))
#define MICROSD_CD_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<15))
#define MICROSD_CD_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<15))
#define MICROSD_CD_Get()               ((PIOA_REGS->PIO_PDSR >> 15) & 0x1)
#define MICROSD_CD_PIN                  PIO_PIN_PA15

/*** Macros for EXP_UART_RTS pin ***/
#define EXP_UART_RTS_Set()               (PIOA_REGS->PIO_SODR = (1<<14))
#define EXP_UART_RTS_Clear()             (PIOA_REGS->PIO_CODR = (1<<14))
#define EXP_UART_RTS_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<14))
#define EXP_UART_RTS_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<14))
#define EXP_UART_RTS_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<14))
#define EXP_UART_RTS_Get()               ((PIOA_REGS->PIO_PDSR >> 14) & 0x1)
#define EXP_UART_RTS_PIN                  PIO_PIN_PA14

/*** Macros for GPIO_DBG1 pin ***/
#define GPIO_DBG1_Set()               (PIOD_REGS->PIO_SODR = (1<<25))
#define GPIO_DBG1_Clear()             (PIOD_REGS->PIO_CODR = (1<<25))
#define GPIO_DBG1_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<25))
#define GPIO_DBG1_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<25))
#define GPIO_DBG1_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<25))
#define GPIO_DBG1_Get()               ((PIOD_REGS->PIO_PDSR >> 25) & 0x1)
#define GPIO_DBG1_PIN                  PIO_PIN_PD25

/*** Macros for GPIO_DBG0 pin ***/
#define GPIO_DBG0_Set()               (PIOD_REGS->PIO_SODR = (1<<24))
#define GPIO_DBG0_Clear()             (PIOD_REGS->PIO_CODR = (1<<24))
#define GPIO_DBG0_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<24))
#define GPIO_DBG0_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<24))
#define GPIO_DBG0_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<24))
#define GPIO_DBG0_Get()               ((PIOD_REGS->PIO_PDSR >> 24) & 0x1)
#define GPIO_DBG0_PIN                  PIO_PIN_PD24

/*** Macros for SPOT_LED_DOUT pin ***/
#define SPOT_LED_DOUT_Get()               ((PIOA_REGS->PIO_PDSR >> 24) & 0x1)
#define SPOT_LED_DOUT_PIN                  PIO_PIN_PA24

/*** Macros for MICROSD_SDIO_CK pin ***/
#define MICROSD_SDIO_CK_Get()               ((PIOA_REGS->PIO_PDSR >> 25) & 0x1)
#define MICROSD_SDIO_CK_PIN                  PIO_PIN_PA25

/*** Macros for MICROSD_SDIO_D2 pin ***/
#define MICROSD_SDIO_D2_Get()               ((PIOA_REGS->PIO_PDSR >> 26) & 0x1)
#define MICROSD_SDIO_D2_PIN                  PIO_PIN_PA26

/*** Macros for STAT_LED_DOUT pin ***/
#define STAT_LED_DOUT_Get()               ((PIOA_REGS->PIO_PDSR >> 11) & 0x1)
#define STAT_LED_DOUT_PIN                  PIO_PIN_PA11

/*** Macros for EXP_UART_TX pin ***/
#define EXP_UART_TX_Get()               ((PIOA_REGS->PIO_PDSR >> 10) & 0x1)
#define EXP_UART_TX_PIN                  PIO_PIN_PA10

/*** Macros for MICROSD_SDIO_D3 pin ***/
#define MICROSD_SDIO_D3_Get()               ((PIOA_REGS->PIO_PDSR >> 27) & 0x1)
#define MICROSD_SDIO_D3_PIN                  PIO_PIN_PA27

/*** Macros for DEBUG_UART_RX pin ***/
#define DEBUG_UART_RX_Get()               ((PIOD_REGS->PIO_PDSR >> 28) & 0x1)
#define DEBUG_UART_RX_PIN                  PIO_PIN_PD28

/*** Macros for MTRDRV_SPI_SCK pin ***/
#define MTRDRV_SPI_SCK_Get()               ((PIOD_REGS->PIO_PDSR >> 17) & 0x1)
#define MTRDRV_SPI_SCK_PIN                  PIO_PIN_PD17

/*** Macros for EXP_UART_RX pin ***/
#define EXP_UART_RX_Get()               ((PIOA_REGS->PIO_PDSR >> 9) & 0x1)
#define EXP_UART_RX_PIN                  PIO_PIN_PA9

/*** Macros for CHABO_DETn pin ***/
#define CHABO_DETn_Set()               (PIOA_REGS->PIO_SODR = (1<<4))
#define CHABO_DETn_Clear()             (PIOA_REGS->PIO_CODR = (1<<4))
#define CHABO_DETn_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<4))
#define CHABO_DETn_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<4))
#define CHABO_DETn_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<4))
#define CHABO_DETn_Get()               ((PIOA_REGS->PIO_PDSR >> 4) & 0x1)
#define CHABO_DETn_PIN                  PIO_PIN_PA4

/*** Macros for MTRDRV_SPI_MOSI pin ***/
#define MTRDRV_SPI_MOSI_Get()               ((PIOD_REGS->PIO_PDSR >> 16) & 0x1)
#define MTRDRV_SPI_MOSI_PIN                  PIO_PIN_PD16

/*** Macros for Z_STPR_DIR pin ***/
#define Z_STPR_DIR_Set()               (PIOD_REGS->PIO_SODR = (1<<14))
#define Z_STPR_DIR_Clear()             (PIOD_REGS->PIO_CODR = (1<<14))
#define Z_STPR_DIR_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<14))
#define Z_STPR_DIR_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<14))
#define Z_STPR_DIR_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<14))
#define Z_STPR_DIR_Get()               ((PIOD_REGS->PIO_PDSR >> 14) & 0x1)
#define Z_STPR_DIR_PIN                  PIO_PIN_PD14

/*** Macros for Z_MTRDRV_STEP pin ***/
#define Z_MTRDRV_STEP_Set()               (PIOB_REGS->PIO_SODR = (1<<12))
#define Z_MTRDRV_STEP_Clear()             (PIOB_REGS->PIO_CODR = (1<<12))
#define Z_MTRDRV_STEP_Toggle()            (PIOB_REGS->PIO_ODSR ^= (1<<12))
#define Z_MTRDRV_STEP_OutputEnable()      (PIOB_REGS->PIO_OER = (1<<12))
#define Z_MTRDRV_STEP_InputEnable()       (PIOB_REGS->PIO_ODR = (1<<12))
#define Z_MTRDRV_STEP_Get()               ((PIOB_REGS->PIO_PDSR >> 12) & 0x1)
#define Z_MTRDRV_STEP_PIN                  PIO_PIN_PB12

/*** Macros for Z_MTRDRV_SPI_CSn pin ***/
#define Z_MTRDRV_SPI_CSn_Set()               (PIOD_REGS->PIO_SODR = (1<<13))
#define Z_MTRDRV_SPI_CSn_Clear()             (PIOD_REGS->PIO_CODR = (1<<13))
#define Z_MTRDRV_SPI_CSn_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<13))
#define Z_MTRDRV_SPI_CSn_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<13))
#define Z_MTRDRV_SPI_CSn_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<13))
#define Z_MTRDRV_SPI_CSn_Get()               ((PIOD_REGS->PIO_PDSR >> 13) & 0x1)
#define Z_MTRDRV_SPI_CSn_PIN                  PIO_PIN_PD13

/*** Macros for Z_STPR_ENn pin ***/
#define Z_STPR_ENn_Set()               (PIOD_REGS->PIO_SODR = (1<<12))
#define Z_STPR_ENn_Clear()             (PIOD_REGS->PIO_CODR = (1<<12))
#define Z_STPR_ENn_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<12))
#define Z_STPR_ENn_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<12))
#define Z_STPR_ENn_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<12))
#define Z_STPR_ENn_Get()               ((PIOD_REGS->PIO_PDSR >> 12) & 0x1)
#define Z_STPR_ENn_PIN                  PIO_PIN_PD12

/*** Macros for Y2_STPR_DIR pin ***/
#define Y2_STPR_DIR_Set()               (PIOD_REGS->PIO_SODR = (1<<11))
#define Y2_STPR_DIR_Clear()             (PIOD_REGS->PIO_CODR = (1<<11))
#define Y2_STPR_DIR_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<11))
#define Y2_STPR_DIR_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<11))
#define Y2_STPR_DIR_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<11))
#define Y2_STPR_DIR_Get()               ((PIOD_REGS->PIO_PDSR >> 11) & 0x1)
#define Y2_STPR_DIR_PIN                  PIO_PIN_PD11

/*** Macros for Y2_MTRDRV_STEP pin ***/
#define Y2_MTRDRV_STEP_Set()               (PIOA_REGS->PIO_SODR = (1<<1))
#define Y2_MTRDRV_STEP_Clear()             (PIOA_REGS->PIO_CODR = (1<<1))
#define Y2_MTRDRV_STEP_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<1))
#define Y2_MTRDRV_STEP_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<1))
#define Y2_MTRDRV_STEP_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<1))
#define Y2_MTRDRV_STEP_Get()               ((PIOA_REGS->PIO_PDSR >> 1) & 0x1)
#define Y2_MTRDRV_STEP_PIN                  PIO_PIN_PA1

/*** Macros for Y2_STPR_ENn pin ***/
#define Y2_STPR_ENn_Set()               (PIOD_REGS->PIO_SODR = (1<<10))
#define Y2_STPR_ENn_Clear()             (PIOD_REGS->PIO_CODR = (1<<10))
#define Y2_STPR_ENn_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<10))
#define Y2_STPR_ENn_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<10))
#define Y2_STPR_ENn_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<10))
#define Y2_STPR_ENn_Get()               ((PIOD_REGS->PIO_PDSR >> 10) & 0x1)
#define Y2_STPR_ENn_PIN                  PIO_PIN_PD10

/*** Macros for Y2_MTRDRV_SPI_CSn pin ***/
#define Y2_MTRDRV_SPI_CSn_Set()               (PIOA_REGS->PIO_SODR = (1<<0))
#define Y2_MTRDRV_SPI_CSn_Clear()             (PIOA_REGS->PIO_CODR = (1<<0))
#define Y2_MTRDRV_SPI_CSn_Toggle()            (PIOA_REGS->PIO_ODSR ^= (1<<0))
#define Y2_MTRDRV_SPI_CSn_OutputEnable()      (PIOA_REGS->PIO_OER = (1<<0))
#define Y2_MTRDRV_SPI_CSn_InputEnable()       (PIOA_REGS->PIO_ODR = (1<<0))
#define Y2_MTRDRV_SPI_CSn_Get()               ((PIOA_REGS->PIO_PDSR >> 0) & 0x1)
#define Y2_MTRDRV_SPI_CSn_PIN                  PIO_PIN_PA0

/*** Macros for MTRDRV_SPI_MISO pin ***/
#define MTRDRV_SPI_MISO_Get()               ((PIOD_REGS->PIO_PDSR >> 15) & 0x1)
#define MTRDRV_SPI_MISO_PIN                  PIO_PIN_PD15

/*** Macros for PROBE_IN pin ***/
#define PROBE_IN_Set()               (PIOD_REGS->PIO_SODR = (1<<9))
#define PROBE_IN_Clear()             (PIOD_REGS->PIO_CODR = (1<<9))
#define PROBE_IN_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<9))
#define PROBE_IN_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<9))
#define PROBE_IN_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<9))
#define PROBE_IN_Get()               ((PIOD_REGS->PIO_PDSR >> 9) & 0x1)
#define PROBE_IN_PIN                  PIO_PIN_PD9
#define PROBE_SW_NO_Pin  PROBE_IN_PIN
/*** Macros for MICROSD_SDIO_CMD pin ***/
#define MICROSD_SDIO_CMD_Get()               ((PIOA_REGS->PIO_PDSR >> 28) & 0x1)
#define MICROSD_SDIO_CMD_PIN                  PIO_PIN_PA28

/*** Macros for Z_ENDSTOP_SW_NC pin ***/
#define Z_ENDSTOP_SW_NC_Set()               (PIOD_REGS->PIO_SODR = (1<<8))
#define Z_ENDSTOP_SW_NC_Clear()             (PIOD_REGS->PIO_CODR = (1<<8))
#define Z_ENDSTOP_SW_NC_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<8))
#define Z_ENDSTOP_SW_NC_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<8))
#define Z_ENDSTOP_SW_NC_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<8))
#define Z_ENDSTOP_SW_NC_Get()               ((PIOD_REGS->PIO_PDSR >> 8) & 0x1)
#define Z_ENDSTOP_SW_NC_PIN                  PIO_PIN_PD8

/*** Macros for MICROSD_SDIO_D0 pin ***/
#define MICROSD_SDIO_D0_Get()               ((PIOA_REGS->PIO_PDSR >> 30) & 0x1)
#define MICROSD_SDIO_D0_PIN                  PIO_PIN_PA30

/*** Macros for MICROSD_SDIO_D1 pin ***/
#define MICROSD_SDIO_D1_Get()               ((PIOA_REGS->PIO_PDSR >> 31) & 0x1)
#define MICROSD_SDIO_D1_PIN                  PIO_PIN_PA31

/*** Macros for ITRLK_LOOP0 pin ***/
#define ITRLK_LOOP0_Set()               (PIOD_REGS->PIO_SODR = (1<<7))
#define ITRLK_LOOP0_Clear()             (PIOD_REGS->PIO_CODR = (1<<7))
#define ITRLK_LOOP0_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<7))
#define ITRLK_LOOP0_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<7))
#define ITRLK_LOOP0_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<7))
#define ITRLK_LOOP0_Get()               ((PIOD_REGS->PIO_PDSR >> 7) & 0x1)
#define ITRLK_LOOP0_PIN                  PIO_PIN_PD7
#define ITRLK_LOOP_FLAG0_Pin            ITRLK_LOOP0_PIN 
/*** Macros for X_ENDSTOP_SW_NC pin ***/
#define X_ENDSTOP_SW_NC_Set()               (PIOD_REGS->PIO_SODR = (1<<6))
#define X_ENDSTOP_SW_NC_Clear()             (PIOD_REGS->PIO_CODR = (1<<6))
#define X_ENDSTOP_SW_NC_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<6))
#define X_ENDSTOP_SW_NC_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<6))
#define X_ENDSTOP_SW_NC_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<6))
#define X_ENDSTOP_SW_NC_Get()               ((PIOD_REGS->PIO_PDSR >> 6) & 0x1)
#define X_ENDSTOP_SW_NC_PIN                  PIO_PIN_PD6

/*** Macros for Y_ENDSTOP_SW_NC pin ***/
#define Y_ENDSTOP_SW_NC_Set()               (PIOD_REGS->PIO_SODR = (1<<5))
#define Y_ENDSTOP_SW_NC_Clear()             (PIOD_REGS->PIO_CODR = (1<<5))
#define Y_ENDSTOP_SW_NC_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<5))
#define Y_ENDSTOP_SW_NC_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<5))
#define Y_ENDSTOP_SW_NC_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<5))
#define Y_ENDSTOP_SW_NC_Get()               ((PIOD_REGS->PIO_PDSR >> 5) & 0x1)
#define Y_ENDSTOP_SW_NC_PIN                  PIO_PIN_PD5

/*** Macros for ITRLK_LOOP1 pin ***/
#define ITRLK_LOOP1_Set()               (PIOD_REGS->PIO_SODR = (1<<4))
#define ITRLK_LOOP1_Clear()             (PIOD_REGS->PIO_CODR = (1<<4))
#define ITRLK_LOOP1_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<4))
#define ITRLK_LOOP1_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<4))
#define ITRLK_LOOP1_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<4))
#define ITRLK_LOOP1_Get()               ((PIOD_REGS->PIO_PDSR >> 4) & 0x1)
#define ITRLK_LOOP1_PIN                  PIO_PIN_PD4
#define ITRLK_LOOP_FLAG1_Pin        ITRLK_LOOP1_PIN
/*** Macros for FAN1_PWM pin ***/
#define FAN1_PWM_Get()               ((PIOD_REGS->PIO_PDSR >> 3) & 0x1)
#define FAN1_PWM_PIN                  PIO_PIN_PD3

/*** Macros for B_ENDSTOP_SW_NC pin ***/
#define B_ENDSTOP_SW_NC_Set()               (PIOD_REGS->PIO_SODR = (1<<2))
#define B_ENDSTOP_SW_NC_Clear()             (PIOD_REGS->PIO_CODR = (1<<2))
#define B_ENDSTOP_SW_NC_Toggle()            (PIOD_REGS->PIO_ODSR ^= (1<<2))
#define B_ENDSTOP_SW_NC_OutputEnable()      (PIOD_REGS->PIO_OER = (1<<2))
#define B_ENDSTOP_SW_NC_InputEnable()       (PIOD_REGS->PIO_ODR = (1<<2))
#define B_ENDSTOP_SW_NC_Get()               ((PIOD_REGS->PIO_PDSR >> 2) & 0x1)
#define B_ENDSTOP_SW_NC_PIN                  PIO_PIN_PD2

/*** Macros for FAN0_PWM pin ***/
#define FAN0_PWM_Get()               ((PIOD_REGS->PIO_PDSR >> 1) & 0x1)
#define FAN0_PWM_PIN                  PIO_PIN_PD1

/*** Macros for A_ENDSTOP_SW_NC pin ***/
#define A_ENDSTOP_SW_NC_Set()               (PIOB_REGS->PIO_SODR = (1<<8))
#define A_ENDSTOP_SW_NC_Clear()             (PIOB_REGS->PIO_CODR = (1<<8))
#define A_ENDSTOP_SW_NC_Toggle()            (PIOB_REGS->PIO_ODSR ^= (1<<8))
#define A_ENDSTOP_SW_NC_OutputEnable()      (PIOB_REGS->PIO_OER = (1<<8))
#define A_ENDSTOP_SW_NC_InputEnable()       (PIOB_REGS->PIO_ODR = (1<<8))
#define A_ENDSTOP_SW_NC_Get()               ((PIOB_REGS->PIO_PDSR >> 8) & 0x1)
#define A_ENDSTOP_SW_NC_PIN                  PIO_PIN_PB8

/*** Macros for HOST_USB_HS_VBUS pin ***/
#define HOST_USB_HS_VBUS_Set()               (PIOB_REGS->PIO_SODR = (1<<13))
#define HOST_USB_HS_VBUS_Clear()             (PIOB_REGS->PIO_CODR = (1<<13))
#define HOST_USB_HS_VBUS_Toggle()            (PIOB_REGS->PIO_ODSR ^= (1<<13))
#define HOST_USB_HS_VBUS_OutputEnable()      (PIOB_REGS->PIO_OER = (1<<13))
#define HOST_USB_HS_VBUS_InputEnable()       (PIOB_REGS->PIO_ODR = (1<<13))
#define HOST_USB_HS_VBUS_Get()               ((PIOB_REGS->PIO_PDSR >> 13) & 0x1)
#define HOST_USB_HS_VBUS_PIN                  PIO_PIN_PB13


// *****************************************************************************
/* PIO Port

  Summary:
    Identifies the available PIO Ports.

  Description:
    This enumeration identifies the available PIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/

typedef enum
{
    PIO_PORT_A = PIOA_BASE_ADDRESS,
    PIO_PORT_B = PIOB_BASE_ADDRESS,
    PIO_PORT_D = PIOD_BASE_ADDRESS,
} PIO_PORT;

// *****************************************************************************
/* PIO Port Pins

  Summary:
    Identifies the available PIO port pins.

  Description:
    This enumeration identifies the available PIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/

typedef enum
{
    PIO_PIN_PA0 = 0,
    PIO_PIN_PA1 = 1,
    PIO_PIN_PA2 = 2,
    PIO_PIN_PA3 = 3,
    PIO_PIN_PA4 = 4,
    PIO_PIN_PA5 = 5,
    PIO_PIN_PA7 = 7,
    PIO_PIN_PA8 = 8,
    PIO_PIN_PA9 = 9,
    PIO_PIN_PA10 = 10,
    PIO_PIN_PA11 = 11,
    PIO_PIN_PA12 = 12,
    PIO_PIN_PA13 = 13,
    PIO_PIN_PA14 = 14,
    PIO_PIN_PA15 = 15,
    PIO_PIN_PA16 = 16,
    PIO_PIN_PA17 = 17,
    PIO_PIN_PA18 = 18,
    PIO_PIN_PA19 = 19,
    PIO_PIN_PA20 = 20,
    PIO_PIN_PA21 = 21,
    PIO_PIN_PA22 = 22,
    PIO_PIN_PA23 = 23,
    PIO_PIN_PA24 = 24,
    PIO_PIN_PA25 = 25,
    PIO_PIN_PA26 = 26,
    PIO_PIN_PA27 = 27,
    PIO_PIN_PA28 = 28,
    PIO_PIN_PA30 = 30,
    PIO_PIN_PA31 = 31,
    PIO_PIN_PB0 = 32,
    PIO_PIN_PB1 = 33,
    PIO_PIN_PB2 = 34,
    PIO_PIN_PB3 = 35,
    PIO_PIN_PB4 = 36,
    PIO_PIN_PB5 = 37,
    PIO_PIN_PB6 = 38,
    PIO_PIN_PB7 = 39,
    PIO_PIN_PB8 = 40,
    PIO_PIN_PB9 = 41,
    PIO_PIN_PB12 = 44,
    PIO_PIN_PB13 = 45,
    PIO_PIN_PD0 = 96,
    PIO_PIN_PD1 = 97,
    PIO_PIN_PD2 = 98,
    PIO_PIN_PD3 = 99,
    PIO_PIN_PD4 = 100,
    PIO_PIN_PD5 = 101,
    PIO_PIN_PD6 = 102,
    PIO_PIN_PD7 = 103,
    PIO_PIN_PD8 = 104,
    PIO_PIN_PD9 = 105,
    PIO_PIN_PD10 = 106,
    PIO_PIN_PD11 = 107,
    PIO_PIN_PD12 = 108,
    PIO_PIN_PD13 = 109,
    PIO_PIN_PD14 = 110,
    PIO_PIN_PD15 = 111,
    PIO_PIN_PD16 = 112,
    PIO_PIN_PD17 = 113,
    PIO_PIN_PD18 = 114,
    PIO_PIN_PD19 = 115,
    PIO_PIN_PD20 = 116,
    PIO_PIN_PD21 = 117,
    PIO_PIN_PD22 = 118,
    PIO_PIN_PD24 = 120,
    PIO_PIN_PD25 = 121,
    PIO_PIN_PD26 = 122,
    PIO_PIN_PD27 = 123,
    PIO_PIN_PD28 = 124,
    PIO_PIN_PD30 = 126,
    PIO_PIN_PD31 = 127,

    /* This element should not be used in any of the PIO APIs.
       It will be used by other modules or application to denote that none of the PIO Pin is used */
    PIO_PIN_NONE = -1

} PIO_PIN;
//SME: add functions:
#define X_ENDSTOP_SW_NC_Get2()               PIO_PinRead(X_ENDSTOP_SW_NC_PIN)//((PIOD_REGS->PIO_PDSR >> 6) & 0x1)
#define X_ENDSTOP_SW_NC_Get3()                hal_read_pin(0,X_ENDSTOP_SW_NC_PIN)
#define GPIO_DBG0_Toggle2()                   hal_toggle_pin(0,GPIO_DBG0_PIN)
#define GPIO_DBG1_Toggle2()                   hal_toggle_pin(0,GPIO_DBG1_PIN)
void PIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: PIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t PIO_PortRead(PIO_PORT port);

void PIO_PortWrite(PIO_PORT port, uint32_t mask, uint32_t value);

uint32_t PIO_PortLatchRead ( PIO_PORT port );

void PIO_PortSet(PIO_PORT port, uint32_t mask);

void PIO_PortClear(PIO_PORT port, uint32_t mask);

void PIO_PortToggle(PIO_PORT port, uint32_t mask);

void PIO_PortInputEnable(PIO_PORT port, uint32_t mask);

void PIO_PortOutputEnable(PIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: PIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void PIO_PinWrite(PIO_PIN pin, bool value)
{
    PIO_PortWrite((PIO_PORT)(PIOA_BASE_ADDRESS + (0x200 * (pin>>5))), (uint32_t)(0x1) << (pin & 0x1f), (uint32_t)(value) << (pin & 0x1f));
}

static inline bool PIO_PinRead(PIO_PIN pin)
{
    return (bool)((PIO_PortRead((PIO_PORT)(PIOA_BASE_ADDRESS + (0x200 * (pin>>5)))) >> (pin & 0x1F)) & 0x1);
}

static inline bool PIO_PinLatchRead(PIO_PIN pin)
{
    return (bool)((PIO_PortLatchRead((PIO_PORT)(PIOA_BASE_ADDRESS + (0x200 * (pin>>5)))) >> (pin & 0x1F)) & 0x1);
}

static inline void PIO_PinToggle(PIO_PIN pin)
{
    PIO_PortToggle((PIO_PORT)(PIOA_BASE_ADDRESS + (0x200 * (pin>>5))), 0x1 << (pin & 0x1F));
}

static inline void PIO_PinSet(PIO_PIN pin)
{
    PIO_PortSet((PIO_PORT)(PIOA_BASE_ADDRESS + (0x200 * (pin>>5))), 0x1 << (pin & 0x1F));
}

static inline void PIO_PinClear(PIO_PIN pin)
{
    PIO_PortClear((PIO_PORT)(PIOA_BASE_ADDRESS + (0x200 * (pin>>5))), 0x1 << (pin & 0x1F));
}

static inline void PIO_PinInputEnable(PIO_PIN pin)
{
    PIO_PortInputEnable((PIO_PORT)(PIOA_BASE_ADDRESS + (0x200 * (pin>>5))), 0x1 << (pin & 0x1F));
}

static inline void PIO_PinOutputEnable(PIO_PIN pin)
{
    PIO_PortOutputEnable((PIO_PORT)(PIOA_BASE_ADDRESS + (0x200 * (pin>>5))), 0x1 << (pin & 0x1F));
}


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_PIO_H
