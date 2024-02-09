/*******************************************************************************
 * File:spindle_ctrl.h
 * Description:
 *
 ******************************************************************************/

#ifndef SPINDLE_CTRL_H
#define	SPINDLE_CTRL_H
#include "main.h"  
#include "bantam_hal.h"
#include "user_types.h"
#include "globdefs.h"

#ifdef CHABO_DCNC
#include "g2_settings_bt_cncmm.h"
#elif defined CHABO_MINIMILL
#include "g2_settings_bt_minimill.h"
#elif defined CHABO_MINIMILL_ESC
#include "g2_settings_bt_minimill_esc.h"
#elif defined CHABO_LFP
#include "g2_settings_bt_lfp.h"
#elif defined CHABO_PLOTTER
#include "g2_settings_bt_plotter.h"
#endif
 
#define SPINDLE_PWM_FREQUENCY_HZ 100
#define SPINDLE_PWM_PERIOD_SEC 0.0100000
#define SPINDLE_PWM_TMR_PRESCALAR 108 //SME: 9-27-2019: TO DO, verify!!!!!!!-->108 APB2 TIMER CLOCK MHz/108==1Mhz
#define SPINDLE_PWM_50HZ_PERIOD_RELOAD_VALUE 20000//for 50Hz
#define SPINDLE_PWM_500HZ_PERIOD_RELOAD_VALUE 2000 //rev B
#define SPINDLE_PWM_443HZ_PERIOD_RELOAD_VALUE 2228//2230-2229==442// 2220-2228==444Hz //should actually  be 2257 fwith 1 usec per timer tic rev B
#define SPINDLE_PWM_100HZ_PERIOD_RELOAD_VALUE 10000//for 100Hz

#define DEPLOY_SP_DLY_PER_RAMP_CFG_PARAM //8-12-2022
 // tiny legacy, "PWM" soft-start: original estimate 13ms/100RPM, but tweaked for slipping
 // RPM ramp up/down step increment
#define SP_MIN_DLY_PER_RPM_STEP  75
#define SP_MAX_DLY_PER_RPM_STEP  150
#define SP_MIN_RPM_STEP_INCR  500.0 
#define SP_MAX_RPM_STEP_INCR  2000.0 
#define SP_RPM_STEP_INCR     SP_MIN_RPM_STEP_INCR 
#define SP_DLY_PER_RPM_STEP_MS  75     // ms per RPM increment (above)
#define SPINDLE_RPM_STEP_INCREMENT   SP_RPM_STEP_INCR  // RPM ramp up/down step increment
#define SPINDLE_MS_DLY_PER_RPM_INCR  SP_DLY_PER_RPM_STEP_MS    // ms per RPM increment (above)

#define SPINDLE_PWM_PERIOD_RELOAD_VALUE SPINDLE_PWM_50HZ_PERIOD_RELOAD_VALUE
#define SPINDLE_PWM_STARTUP_COUNT PWM0_SPINDLE_IDLE_DUTY_CYCLE 
#define SPINDLE_PWM_MID_COUNT (SPINDLE_PWM_PERIOD_RELOAD_VALUE/7)//14%duty cycle
#define SPINDLE_PWM_MAX_COUNT (SPINDLE_PWM_PERIOD_RELOAD_VALUE/5) //20% duty cycle=="max throttle"

#define SPINDLE_PWM_50PCT_DUTY 5000
#define SPINDLE_PWM_80PCT_DUTY 8000
#define SPINDLE_PWM_100PCT_DUTY SPINDLE_PWM_PERIOD_RELOAD_VALUE
#define SPINDLE_PWM_0PCT_DUTY 0
  
#define SPINDLE_FEEDBACK_TMR_PRESCALAR 10800//want 10 KHz --10 counts per ms
#define SPINDLE_FEEDBACK_TMR_TICS_PER_SEC 10000//per 10 kHz timer
#define SPINDLE_FEEDBACK_TMR_PERIOD 10000000//SME ADDED HERE
#define SPINDLE_MIN_RPM_SPEED_CHANGE 50.000 //arbitrary
#define SPINDLE_MIN_SPEED_RPM_TINYG_SPINDLE 0//9-27-2019 do experiment 10770 //9-27-2019 Observed rpm by tachometer, when applying S8500 gcode command
#define SPINDLE_MAX_SPEED_RPM_TINYG_SPINDLE 21500//applying "S15000"

#define SPINDLE_MIN_SPEED_RPM (float) P1_SPEED_LO//vfd==7000 SPINDLE_MIN_SPEED_RPM_TINYG_SPINDLE//11500//per 7-26-2019 observation spindle won't respond below that8000.00///per tinyG

#define SPINDLE_MAX_SPEED_RPM (float)P1_SPEED_HI//vfd==24000.00
#define SPINDLE_LOW_SPEED_RPM SPINDLE_MIN_SPEED_RPM 
#define SPINDLE_MID_SPEED_RPM 16000.00
#define SPINDLE_HIGH_ALLOWED_SPEED_RPM SPINDLE_MAX_SPEED_RPM
#define SPINDLE_MAX_ALLOWED_SPEED_RPM SPINDLE_HIGH_ALLOWED_SPEED_RPM
#define SPINDLE_MIN_DUTY_CYCLE_PCT (float)0.00
#define SPINDLE_IDLE_DUTY_CYCLE_PCT (float)0.10
#define SPINDLE_MAX_DUTY_CYCLE_PCT (float)100.00

#define SPINDLE_MIN_DUTY_CYCLE_DECIMAL (float)P1_PHASE_LO
#define SPINDLE_MAX_DUTY_CYCLE_DECIMAL (float)P1_PHASE_HI

/* Define safe default init values */
#define SPNDL_DEFAULT_MIN_DUTY_CYCLE SPINDLE_MIN_DUTY_CYCLE_DECIMAL
#define SPNDL_DEFAULT_MAX_DUTY_CYCLE SPINDLE_MAX_DUTY_CYCLE_DECIMAL

#define SPNDL_DEFAULT_MIN_SPEED_RPM SPINDLE_MIN_SPEED_RPM
#define SPNDL_DEFAULT_MAX_SPEED_RPM SPINDLE_MAX_SPEED_RPM
#define THE_TINYG_MILL_SPINDLE_MOTOR_NUM_POLES 8//?

/* 
 * Define scale factor ESC Feedback signal Hz to RPM: N samples per second x 60 seconds per minute
 *  use 60  for 2 pole motor, 30  for 4-pole motor
 *   RPM= (feedback samples/sec/pole pairs)
 *   two-pole motor has 1 pole pair
 *   four pole motor has 2 pole pairs
 *   ten pole motor has 5 pole pairs
 */
#define SPINDLE_ESC_SCALE_FACTOR_TWO_POLE_MOTOR 60
#define SPINDLE_ESC_SCALE_FACTOR_FOUR_POLE_MOTOR 30
#define SPINDLE_ESC_SCALE_FACTOR_8_POLE_MOTOR  15
#define SPINDLE_ESC_RPM_PER_100HZ  SPINDLE_ESC_SCALE_FACTOR_FOUR_POLE_MOTOR//SPINDLE_ESC_SCALE_FACTOR_FOUR_POLE_MOTOR 
#define SPINDLE_SPEED_16K_RPM_PWM_COUNTS  1425//1425==~11500 rpm observed as of 7-26-2019 is the minimum observed pwm to get spindle to move 1450//1350
#define SPINDLE_SPEED_12K_RPM_PWM_COUNTS  1425//1425==~11500 rpm observed as of 7-26-2019 is the minimum observed pwm to get spindle to move 1450//1350

#if 1//target spindle, 2-pole pairs
#define SPINDLE_LOW_SPEED_DUTY_CYCLE_SETTING  1425 
#define SPINDLE_MID_SPEED_DUTY_CYCLE_SETTING  1765
#define SPINDLE_HIGH_SPEED_DUTY_CYCLE_SETTING 1935 
#define SPINDLE_MIN_SPEED_DUTY_CYCLE_SETTING  SPINDLE_LOW_SPEED_DUTY_CYCLE_SETTING
#define SPINDLE_MAX_SPEED_DUTY_CYCLE_SETTING  SPINDLE_HIGH_SPEED_DUTY_CYCLE_SETTING
#else //4-pole pair spindle
#define SPINDLE_SPEED_20K_RPM_PWM_COUNTS  1650 
#define SPINDLE_SPEED_26K_RPM_PWM_COUNTS  1800  
#define SPINDLE_SPEED_30K_RPM_PWM_COUNTS 2000   
#define SPINDLE_LOW_SPEED_DUTY_CYCLE_SETTING  SPINDLE_SPEED_12K_RPM_PWM_COUNTS//SPINDLE_SPEED_16K_RPM_PWM_COUNTS
#define SPINDLE_MID_SPEED_DUTY_CYCLE_SETTING  SPINDLE_SPEED_20K_RPM_PWM_COUNTS//SPINDLE_PWM_INIT_75PCT_COUNT   
#define SPINDLE_HIGH_SPEED_DUTY_CYCLE_SETTING SPINDLE_SPEED_26K_RPM_PWM_COUNTS 
#endif

int spindle_set_speed_rpm(float value);
int spndl_rpm_from_duty_cycle(float duty_cycle);
//int spindle_set_pwm_duty_cycle(float value);
void spindle_pwm_startup_task(void);
Boolean_t spindle_is_ramping(void);
void spindle_ctrl_init(void);
void spindle_ctrl_task(void);
void spindle_enable_power(void);
void spindle_disable_power(void);
void spindle_startup_pwm(void);
bool spindle_is_running(void);
bool spindle_is_running_or_paused(void);

#endif	/* SPINDLE_CTRL_H */

