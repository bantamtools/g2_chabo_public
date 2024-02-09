/*******************************************************************************
* File:  bantam_hal.c
*
* Description: designate which mcu h/w is active in the project:   
*    Provides an mcu-independent interface to mcu gpio and devices
*     
*******************************************************************************/
 
#include "main.h"
#include "bantam_hal.h"

/* Write Pin: */
/***************************************************************************
 * 
 * Function: hal_write_pin 
 *
 * Description:  
 * 
 **************************************************************************/ 
void hal_write_pin(uint8_t port_id, uint16_t pin, GPIO_PinState state)
{
  PIO_PinWrite((PIO_PIN) pin, (bool) state);
}
/***************************************************************************
 * 
 * Function: hal_set_pin 
 *
 * Description:  
 * 
 **************************************************************************/ 
void hal_set_pin(uint8_t port_id, uint16_t pin)
{
 PIO_PinWrite((PIO_PIN) pin, true);
}
/***************************************************************************
 * 
 * Function: hal_reset_pin 
 *
 * Description:  
 * 
 **************************************************************************/
void hal_reset_pin(uint8_t port_id, uint16_t pin) 
{
  PIO_PinWrite((PIO_PIN) pin, false); 
}

/***************************************************************************
 * 
 * Function: hal_toggle_pin 
 *
 * Description:  
 * 
 **************************************************************************/
void hal_toggle_pin(uint8_t port_id, uint16_t pin)
{
  PIO_PinToggle((PIO_PIN) pin);
}
/***************************************************************************
 * 
 * Function: hal_read_pin 
 *
 * Description:  
 * 
 **************************************************************************/
GPIO_PinState hal_read_pin(uint8_t port_id, uint16_t pin)
{
   GPIO_PinState state = GPIO_PIN_RESET;
   state = (GPIO_PinState)PIO_PinRead((PIO_PIN) pin);
   return state;
}

