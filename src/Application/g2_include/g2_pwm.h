
/*
 * pwm.h - pulse width modulation drivers
 * This file is part of the g2core project
 *
 * Copyright (c) 2012 - 2018 Alden S. Hart, Jr.
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

#ifndef PWM_H_ONCE
#define PWM_H_ONCE

typedef struct pwmConfigChannel {
    float frequency;           // base frequency for PWM driver, in Hz
    float duty_cycle;          // SME 1015-2019 added: duty cycle for directly driving the spindle without translating from RPM 
    float speed_lo;            // minimum clockwise spindle speed [0..N]
    float speed_hi;            // maximum clockwise spindle speed
    float phase_lo;            // pwm phase at minimum spindle speed, clamped [0..1]
    float phase_hi;            // pwm phase at maximum spindle speed, clamped [0..1]
    float phase_off;           // pwm phase when spindle is disabled
    float k_value;             // pwm factor to change rate of increase (curve fit) [1..N]
} pwmConfigChannel_t;

typedef struct pwmChannel {
    // no data required in ARM
} pwmChannel_t;

typedef struct pwmControl {
    pwmConfigChannel_t  c[PWMS];    // array of channel configs
    pwmChannel_t        p[PWMS];    // array of PWM channels
} pwmControl_t;

extern pwmControl_t pwm;

/*** function prototypes ***/

void pwm_init(void);
stat_t pwm_set_freq(uint8_t channel, float freq);
stat_t pwm_set_duty(uint8_t channel, float duty); 
stat_t pwm_set_pwm(nvObj_t *nv);

float pwm_calc_get_min_rpm(void);
float pwm_calc_get_max_rpm(void);
float pwm_calc_get_min_duty_cycle(void);
float pwm_calc_get_max_duty_cycle(void);
 
/*** function prototypes ***/
float pwm_get_min_rpm(void);
float pwm_get_max_rpm(void);
float pwm_get_min_duty_cycle(void);
float pwm_get_max_duty_cycle(void);
float pwm_get_frequency(void);
float pwm_get_duty_cycle(void);
float pwm_get_phase_off(void);
float pwm_get_k_value(void);

 void pwm_refresh_duty_cycle_pct(float value);//sme 8-17-2022
#ifdef __TEXT_MODE

    void pwm_print_p1frq(nvObj_t *nv);
    void pwm_print_p1sl(nvObj_t *nv);
    void pwm_print_p1dc(nvObj_t *nv);//sme: added 10-15-2019
        void pwm_print_p1dco(nvObj_t *nv);//sme: added 10-15-2019
    void pwm_print_p2dc(nvObj_t *nv);//sme: added 4-20-2021
    void pwm_print_p1sh(nvObj_t *nv);
    void pwm_print_p1pl(nvObj_t *nv);
    void pwm_print_p1ph(nvObj_t *nv);
    void pwm_print_p1wsl(nvObj_t *nv);
    void pwm_print_p1wsh(nvObj_t *nv);
    void pwm_print_p1wpl(nvObj_t *nv);
    void pwm_print_p1wph(nvObj_t *nv);
    void pwm_print_p1pof(nvObj_t *nv);
    void pwm_print_p1k(nvObj_t *nv);

#else

    #define pwm_print_p1frq tx_print_stub
    #define pwm_print_p1csl tx_print_stub
    #define pwm_print_p1csh tx_print_stub
    #define pwm_print_p1cpl tx_print_stub
    #define pwm_print_p1cph tx_print_stub
    #define pwm_print_p1wsl tx_print_stub
    #define pwm_print_p1wsh tx_print_stub
    #define pwm_print_p1wpl tx_print_stub
    #define pwm_print_p1wph tx_print_stub
    #define pwm_print_p1pof tx_print_stub
    #define pwm_print_p1k tx_print_stub

#endif // __TEXT_MODE
    
#if 1//legacy/V3
#define P1_PWM_MIN_DUTY_CYCLE 0.0
#define P1_PWM_MAX_DUTY_CYCLE 1.0
#endif
    
#endif    // End of include guard: PWM_H_ONCE

