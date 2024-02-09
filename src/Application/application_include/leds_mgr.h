/*******************************************************************************
 * File:   leds_mgr.h
 * Description:
 *
 ******************************************************************************/

#ifndef LEDSMGR_H
#define	LEDSMGR_H

#include "user_types.h"
#include "globdefs.h"
#include "g2core.h"  // #1
#include "g2_config.h" 
typedef enum
{
   PWM_LEDS_OFF,
   PWM_LEDS_WHITE,
   PWM_LEDS_RED,
   //PWM_LEDS_YELLOW,

   PWM_LEDS_COLOR_COUNT,
   PWM_LEDS_MAX_RANGE=PWM_LEDS_RED,
   PWM_LEDS_MIN_RANGE=PWM_LEDS_OFF,
   PWM_LEDS_ESTOP_PRESSED=PWM_LEDS_RED,
   PWM_LEDS_ESTOP_RELEASED=PWM_LEDS_WHITE,   
        
}PwmLedsColorSettingE;

void leds_mgr_set_rgbw_id(int color_id);
void leds_mgr_set_rgbw(uint8_t red, uint8_t green, uint8_t blue,uint8_t white);
 
void pwm_rgbw_print(nvObj_t *nv);
void pwm_rgbdemo_print(nvObj_t *nv);
void leds_mgr_init(void);
void leds_mgr_task(void);
#endif	/* LEDSMGR_H */

