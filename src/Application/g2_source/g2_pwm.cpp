

/*
 * pwm.cpp - pulse width modulation drivers
 * This file is part of the g2core project
 *
 * Copyright (c) 2012 - 2019 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "g2core.h"  // #1
#include "g2_config.h"  // #2
#include "g2_spindle.h"
#include "g2_text_parser.h"
#include "g2_pwm.h"
#include "g2_canonical_machine.h"
#include "g2_settings.h"
#include "app.h"
/***** PWM defines, structures and memory allocation *****/

pwmControl_t pwm;

/***** PWM code *****/
/*
 * pwm_init() - initialize pwm channels
 *
 *  Notes:
 *    - Whatever level interrupts you use must be enabled in main()
 *    - init assumes PWM1 output bit (D5) has been set to output previously (stepper.c)
 *    - See system.h for timer and port assignments
 *  - Don't do this: memset(&TIMER_PWM1, 0, sizeof(PWM_TIMER_t)); // zero out the timer registers
 */
void pwm_init() 
{

#if 0//to _integrate pwm_init()   spindle_pwm_output->setFrequency   
     spindle_pwm_output = d_out[SPINDLE_PWM_NUMBER-1];        
     spindle_pwm_output->setFrequency(P1_PWM_FREQUENCY);
#endif
}

/*
 * pwm_set_freq() - set PWM channel frequency
 *
 *  channel - PWM channel
 *  freq    - PWM frequency in Khz as a float
 *
 *  Assumes 32MHz clock.
 *  Doesn't turn time on until duty cycle is set
 */

stat_t pwm_set_freq(uint8_t chan, float freq)
{  
#if 0//to _integrate pwm_set_freq
  if (spindle_pwm_output != nullptr) 
  {
      spindle_pwm_output->setFrequency(freq);
  }
#endif
    return (STAT_OK);
}

/*
 * pwm_set_duty() - set PWM channel duty cycle
 *
 *  channel - PWM channel
 *  duty    - PWM duty cycle from 0% to 100%
 *
 *  Setting duty cycle to 0 disables the PWM channel with output low
 *  Setting duty cycle to 100 disables the PWM channel with output high
 *  Setting duty cycle between 0 and 100 enables PWM channel
 *
 *  The frequency must have been set previously
 */
#if 1
//cosmetic, does not result in change in ESC, that occurs in pwm_set_duty
void pwm_refresh_duty_cycle_pct(float value)
{
     if (value<pwm.c[0].phase_off)
    {
       value= pwm.c[0].phase_off;
    }
    else if(value>pwm.c[0].phase_hi)
    {
       value= pwm.c[0].phase_hi;
    }
    pwm.c[0].duty_cycle=value;
}
//sme;8-17-2022 ignore channel for now, refactor later
stat_t pwm_set_duty(uint8_t chan, float value)
{
    if (value<pwm.c[0].phase_off)
    {
       value= pwm.c[0].phase_off;
    }
    else if(value>pwm.c[0].phase_hi)
    {
       value= pwm.c[0].phase_hi;
    }
     
    pwm.c[0].duty_cycle=value;
    PWM0_SetSpindleDutyCycle(value);

    return (STAT_OK);
}

#else //original
stat_t pwm_set_duty(uint8_t chan, float duty)
{
    if (duty < P1_PWM_PHASE_OFF) 
    { 
       duty = P1_PWM_PHASE_OFF; 
      //return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (duty > P1_PHASE_HI)//1.0) 
    { 
        duty = P1_PHASE_HI;
       //return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    PWM0_SetSpindleDutyCycle(duty);
    return (STAT_OK);
}
#endif

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/
/*
 * pwm_set_pwm() - set generic PWM parameter and reset PWM channels
 *
 * See config_app.cpp PWM settings for details of what paramters call this function
 */
stat_t pwm_set_pwm(nvObj_t *nv)
{
    set_flt(nv);
    spindle_init();
    return(STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_p1frq[] = "[p1frq] pwm frequency%18.0f Hz\n";
static const char fmt_p1csl[] = "[p1csl] pwm cw speed lo%16.0f RPM\n";
static const char fmt_p1csh[] = "[p1csh] pwm cw speed hi%16.0f RPM\n";
static const char fmt_p1cpl[] = "[p1cpl] pwm cw phase lo%16.3f [0..1]\n";
static const char fmt_p1cph[] = "[p1cph] pwm cw phase hi%16.3f [0..1]\n";
static const char fmt_p1wsl[] = "[p1wsl] pwm ccw speed lo%15.0f RPM\n";
static const char fmt_p1wsh[] = "[p1wsh] pwm ccw speed hi%15.0f RPM\n";
static const char fmt_p1wpl[] = "[p1wpl] pwm ccw phase lo%15.3f [0..1]\n";
static const char fmt_p1wph[] = "[p1wph] pwm ccw phase hi%15.3f [0..1]\n";
static const char fmt_p1pof[] = "[p1pof] pwm phase off%18.3f [0..1]\n";
static const char fmt_p1k[]   = "[p1k] pwm K value%18.3f\n";
static const char fmt_p1dc[]  = "[p1dc] pwm duty cycle  %15.3f [0..1]\n";//sme added 10-15-2019
static const char fmt_p1dco[] = "[p1dc] pwm duty cycle off  %15.3f \n";//sme added 7-12-22
static const char fmt_p2dc[]  = "[p2dc] sol4 pwm min duty cycle  %15.3f [0.30..0.90]\n";//sme added 4-20-2021
void pwm_print_p1frq(nvObj_t *nv) { text_print(nv, fmt_p1frq);}     // all TYPE_FLOAT
void pwm_print_p1dc(nvObj_t *nv)  { text_print_flt(nv, fmt_p1dc);}//sme added 10-15-2019
void pwm_print_p1dco(nvObj_t *nv)  { text_print_flt(nv, fmt_p1dc);}//sme added 10-15-2019
void pwm_print_p2dc(nvObj_t *nv)  { text_print_flt(nv, fmt_p2dc);}//sme added 4-20-2021
void pwm_print_p1sl(nvObj_t *nv) { text_print(nv, fmt_p1csl);}
void pwm_print_p1sh(nvObj_t *nv) { text_print(nv, fmt_p1csh);}

void pwm_print_p1pl(nvObj_t *nv) { text_print(nv, fmt_p1cpl);}
void pwm_print_p1ph(nvObj_t *nv) { text_print(nv, fmt_p1cph);}
void pwm_print_p1wsl(nvObj_t *nv) { text_print(nv, fmt_p1wsl);}
void pwm_print_p1wsh(nvObj_t *nv) { text_print(nv, fmt_p1wsh);}
void pwm_print_p1wpl(nvObj_t *nv) { text_print(nv, fmt_p1wpl);}
void pwm_print_p1wph(nvObj_t *nv) { text_print(nv, fmt_p1wph);}
void pwm_print_p1pof(nvObj_t *nv) { text_print(nv, fmt_p1pof);}
void pwm_print_p1k(nvObj_t *nv) { text_print(nv, fmt_p1k);}

#endif //__TEXT_MODE
#if 1//sme 10-15-2019 provide a separate command for setting raw duty cycle:
float pwm_get_duty_cycle(void)
{
  return pwm.c[0].duty_cycle;
}
#else //commandeered "set frequency" command to set duty cycle 
float pwm_get_freq_as_duty_cycle(void){return pwm.c[0].frequency;}
void pwm_restore_freq_as_freq(void){pwm.c[0].frequency =P1_PWM_FREQUENCY;}
#endif

/* sme: provide simple accessors to individual fields, with Bantam app caring only for CW, and  ndx==0 */

float pwm_calc_get_min_rpm(void)
{
  return pwm.c[0].speed_lo;
}
float pwm_calc_get_max_rpm(void)
{
  return pwm.c[0].speed_hi;
} 
float pwm_calc_get_min_duty_cycle(void)
{
  return pwm.c[0].phase_lo;
}
float pwm_calc_get_max_duty_cycle(void)
{
  return pwm.c[0].phase_hi;
} 

float pwm_get_frequency(void)
{
  return pwm.c[PWM_1].frequency;
}

float pwm_get_min_rpm(void)
{
  return pwm.c[PWM_1].speed_lo;
}
float pwm_get_max_rpm(void)
{
  return pwm.c[PWM_1].speed_hi;
} 
float pwm_get_min_duty_cycle(void)
{
  return pwm.c[PWM_1].phase_lo;
}
float pwm_get_max_duty_cycle(void)
{
  return pwm.c[PWM_1].phase_hi;
} 
float pwm_get_phase_off(void)
{
  return pwm.c[PWM_1].phase_off;
}

float pwm_get_k_value(void) {
  return pwm.c[PWM_1].k_value;
}