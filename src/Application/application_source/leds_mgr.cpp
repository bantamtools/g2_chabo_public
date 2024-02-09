/*******************************************************************************
* File:   leds_mgr.c
*
* Description: 
*
*******************************************************************************/
#include "main.h"  
#include "bantam_hal.h"
#include "user_types.h"
#include "globdefs.h"
#include "leds_mgr.h"
#include "sk6812.h"//compiler error!w/o explicit path. Why?
#include "g2_controller.h" //blink rates
#include "system_bantam.h"
#include "kernel.h"

typedef enum 
{
  CONTROL_STATE_OFF,
  CONTROL_STATE_STARTUP,
  CONTROL_STATE_NORMAL,
  CONTROL_STATE_INFO_WARNING,//e.g. unhomed        
  CONTROL_STATE_ALARM,
  CONTROL_STATE_SHUTDOWN,//e-stop? 
  CONTROL_STATE_PANIC,
  CONTROL_STATE_COUNT
}MainControlStateE;


volatile static MainControlStateE main_control_state=CONTROL_STATE_STARTUP;  
static volatile int leds_mgr_sched_freq_hz=0;
static volatile int leds_mgr_sched_period_ms=0;
static bool leds_mgr_update_rgbw_flag= false;
static volatile PwmLedsColorSettingE leds_mgr_color_setting_id=PWM_LEDS_WHITE;
extern void led_color_state_machine(void);

int leds_mgr_set_main_state(MainControlStateE state)
{
  if (state<CONTROL_STATE_COUNT)
  {
    main_control_state=state;
  }
  return 0;
}


void leds_mgr_init(void)
{
    /* 8-30-2022 leds_mgr task scheduling parameters are applied in performing blink rates for spot leds*/
  leds_mgr_sched_period_ms = kernel_get_task_period_ms(TASK_LEDS_MGR);
  leds_mgr_sched_freq_hz =1000/leds_mgr_sched_period_ms;
}

PwmLedsColorSettingE leds_mgr_rgbw_setting_id = PWM_LEDS_WHITE;
PwmLedsColorSettingE leds_mgr_saved_rgbw_setting_id = PWM_LEDS_OFF;

void leds_mgr_rgbw_id(int color_id)
{
    if (color_id<PWM_LEDS_MIN_RANGE)
    {
       color_id=PWM_LEDS_MIN_RANGE;
    }
    if (color_id>PWM_LEDS_MAX_RANGE)
    {
       color_id=PWM_LEDS_MAX_RANGE;
    }
    
    leds_mgr_rgbw_setting_id=(PwmLedsColorSettingE)color_id;
}

void leds_mgr_set_rgbw_id(int color_id)
{
    static const PwmRgbwPixelS RGBW[ PWM_LEDS_COLOR_COUNT]=
    {
        //red             green            blue             white
        {LED_OFF,         LED_OFF,         LED_OFF,         LED_OFF},  
        {LED_MAX_SETTING, LED_MAX_SETTING, LED_MAX_SETTING, LED_MAX_SETTING,},
        {LED_MAX_SETTING, LED_OFF,         LED_OFF,         LED_OFF}      
    };

    leds_mgr_rgbw_setting_id=(PwmLedsColorSettingE)color_id;
    pwm_rgbw_setting.red=RGBW[color_id].red;
    pwm_rgbw_setting.green=RGBW[color_id].green;
    pwm_rgbw_setting.blue=RGBW[color_id].blue;
    pwm_rgbw_setting.white=RGBW[color_id].white;  
    pwm_rgbw_setting.blink_hz=LED_OFF;//for now
}
void leds_mgr_set_rgbw(uint8_t red, uint8_t green, uint8_t blue,uint8_t white)
{
    pwm_led_set_all_RGBW(pwm_led_device_info[SPOT_LEDS],red,green, blue, white);
    pwm_led_render(pwm_led_device_info[SPOT_LEDS]);
}
 
 /*******************************************************
 * 
 * Function:  leds_mgr_task  
 * 
 * Description:   
 * Controls the LED function based on the system state:
 * 
 * Assumptions/Requirement: This task is scheduled at an interval
 *     that will naturally produce blink rate, with or without
 *     an additional divider counter. For example, A blink (toggle) 
 *    rate of 4 Hz at 50% duty cycle will be accomplished by scheduling
 *     this task every  250 ms
 ********************************************************/
void leds_mgr_task(void)
{ 
    static enum {LEDS_MGR_INIT, LEDS_MGR_RUN} leds_mgr_state=LEDS_MGR_INIT;
    
    static PwmRgbwPixelS saved_rgbw_setting={LED_OFF,
                                             LED_OFF,
                                             LED_OFF,
                                             LED_OFF,
                                             RGBW_DEFAULT_BLINK_HZ};
    
    static PwmRgbwPixelS toggled_rgbw_setting={RGBW_DEFAULT_SETTING,
                                             RGBW_DEFAULT_SETTING,
                                             RGBW_DEFAULT_SETTING,
                                             RGBW_DEFAULT_SETTING,
                                             RGBW_DEFAULT_BLINK_HZ};
    static volatile int led_blink_skip_interval= 0;
    static volatile int led_blink_interval_counter = 0;
    static volatile enum{TOGGLE_STATE_OFF, TOGGLE_STATE_ON}blink_toggle_state=TOGGLE_STATE_ON;
    /* Task state:*/
    switch (leds_mgr_state)
    {
        case LEDS_MGR_INIT:
            leds_mgr_init(); 
            leds_mgr_state=LEDS_MGR_RUN;
            break;
            
        case LEDS_MGR_RUN:
      
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
          kernel_task_check_in();
#endif 
#ifdef REPAIR_SPOT_LED_ESTOP_WHITE_ON_STARTUP
       if ((leds_mgr_rgbw_setting_id!= PWM_LEDS_ESTOP_PRESSED)          
#else //original
         if ((leds_mgr_color_setting_id!= PWM_LEDS_ESTOP_PRESSED)//needed to avoid overwriting estop color at tail end of turning blink off
#endif
         &&  ((pwm_rgbw_setting.blink_hz>0)
         ||  (blink_toggle_state==TOGGLE_STATE_OFF)))//this is needed if blink_hz 
                                                    //just got turned off and toggle state was OFF
                                                    //for one last entry here, leds need to be turned back on
         {
             /* Calculate the blink rate based on the leds_mgr's scheduling frequency--blink rate is limited to that */
             led_blink_skip_interval=(leds_mgr_sched_freq_hz/pwm_rgbw_setting.blink_hz)-1;

             /* Toggle the rgbw state off/on to match the blink rate (hz)*/
             if ((++led_blink_interval_counter)>led_blink_skip_interval)
             {
                led_blink_interval_counter=0;
             
                if (blink_toggle_state==TOGGLE_STATE_ON)
                {               
                    blink_toggle_state=TOGGLE_STATE_OFF;
                    /*Save the leds settings that are about to be turned OFF */
                    toggled_rgbw_setting.red      = pwm_rgbw_setting.red;
                    toggled_rgbw_setting.green    = pwm_rgbw_setting.green;
                    toggled_rgbw_setting.blue     = pwm_rgbw_setting.blue;
                    toggled_rgbw_setting.white    = pwm_rgbw_setting.white;                          
                    
                    /* Blink the leds--turn them off */ 
                    pwm_rgbw_setting.red          = LED_OFF;
                    pwm_rgbw_setting.green        = LED_OFF;
                    pwm_rgbw_setting.blue         = LED_OFF;
                    pwm_rgbw_setting.white        = LED_OFF;
                }
                else
                {
                    blink_toggle_state=TOGGLE_STATE_ON;
                    /* Unblink the leds--turn them back on with the saved settings */
                    pwm_rgbw_setting.red          = toggled_rgbw_setting.red;
                    pwm_rgbw_setting.green        = toggled_rgbw_setting.green;
                    pwm_rgbw_setting.blue         = toggled_rgbw_setting.blue;
                    pwm_rgbw_setting.white        = toggled_rgbw_setting.white;
                }
             }
         }
        /* two ways to set rgbw spot leds: JSON command, or internal fw via color enum code*/
        if( leds_mgr_rgbw_setting_id != leds_mgr_saved_rgbw_setting_id )
        {
           leds_mgr_saved_rgbw_setting_id = leds_mgr_rgbw_setting_id; 
           leds_mgr_set_rgbw_id(leds_mgr_rgbw_setting_id);
        }   
        /* update from JSON command from desktop app:*/
        if ( (saved_rgbw_setting.red != pwm_rgbw_setting.red)
        || (saved_rgbw_setting.green != pwm_rgbw_setting.green)
        || (saved_rgbw_setting.blue != pwm_rgbw_setting.blue)
        || (saved_rgbw_setting.white != pwm_rgbw_setting.white)
        || (saved_rgbw_setting.blink_hz != pwm_rgbw_setting.blink_hz) 
        )
        {
          /* Update the leds setting saved state: */
          saved_rgbw_setting.red      = pwm_rgbw_setting.red;
          saved_rgbw_setting.green    = pwm_rgbw_setting.green;
          saved_rgbw_setting.blue     = pwm_rgbw_setting.blue;
          saved_rgbw_setting.white    = pwm_rgbw_setting.white;
          saved_rgbw_setting.blink_hz = pwm_rgbw_setting.blink_hz; 
       
          /* Apply the changes: */
          leds_mgr_set_rgbw(pwm_rgbw_setting.red,
                         pwm_rgbw_setting.blue,
                         pwm_rgbw_setting.green,
                         pwm_rgbw_setting.white 
                         );
          //to do: implement blink rate
        }
        hal_toggle_pin(0, GPIO_DBG0_PIN );
        hal_toggle_pin(0, GPIO_DBG1_PIN ); 
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
        kernel_task_check_out();
#endif  
          break;
        default: 
            break;
    }/* End switch */
    
}/* End Task */
