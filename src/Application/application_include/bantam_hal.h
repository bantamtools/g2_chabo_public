/*******************************************************************************
* 
* File:  bantam_hal.h
*
* Description: designate which mcu h/w is active in the project:
*
*     
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BANTAM_HAL_H
#define __BANTAM_HAL_H

#include "main.h"
#include "app.h"

#ifndef GPIO_PinState
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;
#endif

/* Declare hal interface functions */
void hal_write_pin(uint8_t port, uint16_t pin, GPIO_PinState state);
void hal_toggle_pin(uint8_t port, uint16_t pin);
void hal_set_pin(uint8_t port, uint16_t pin);
void hal_reset_pin(uint8_t port, uint16_t pin);
GPIO_PinState hal_read_pin(uint8_t port, uint16_t pin);
#endif