/*
 * util.c
 *
 *  Created on: May 19, 2022
 *      Author: matt
 */

#include "util.h"

SYS_TIME_HANDLE delay_timer = SYS_TIME_HANDLE_INVALID;

// Delays for the given number of milliseconds
void delay_ms(uint32_t delay) {
  if (SYS_TIME_DelayMS(delay, &delay_timer) != SYS_TIME_SUCCESS) {
    // Handle error
  } else if(SYS_TIME_DelayIsComplete(delay_timer) != true) {
    // Wait till the delay has not expired
    while (SYS_TIME_DelayIsComplete(delay_timer) == false);
  }  
}
