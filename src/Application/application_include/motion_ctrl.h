/*******************************************************************************
 * File:   motion_ctrl.h
 * Description:
 *
 ******************************************************************************/
#ifndef MOTION_CTRL_H
#define	MOTION_CTRL_H
#include "main.h" 
#include "bantam_hal.h"
#include "user_types.h"
#include "globdefs.h"
#include "g2_config.h"

/* Define mnemonics for nc, no switch properties  
 *  Both sensors are pulled high. When switch is released, NO is pulled to GND, NO remains pulled HI 
 *    Sw sensor:                   SW PRESSED     SW RELEASED
 *  --------------------------     --------         ---------
 *  Normally Closed (NC)             1 (opens)      0 (closes)
 *  Normally Open (NO)               0 (closes)     1 (opens)
*/

/* Enumerate limit switches: Mill 3 axes homing */
typedef enum
{
  LIMIT_SW_AXIS_X,
  LIMIT_SW_AXIS_Y,
  LIMIT_SW_AXIS_Z,
#ifdef DEPLOY_FOURTH_AXIS
  LIMIT_SW_AXIS_A,          
#endif  
  LIMIT_SW_COUNT
}LimitSwitchIdE;

/* Enumerate previous, present polled states for transition detection: */
typedef enum
{
  PRESENT_POLLED_STATE,
  PREVIOUS_POLLED_STATE,
  POLLED_STATE_COUNT
}SwitchPolledStateTrackerE;

#ifdef REFACTOR_LIMIT_SWITCHES
/* 
 * Accommodate NO==Active Low or NC=Active High limit switch hardware, 
 * by initializing active_state field to what applies to given limit switch 
 */
typedef struct
{
     GPIO_PinState state; /* what is read: Logic High==1 or or Logic Low==0 */
     GPIO_PinState active_state;/* AKA Polarity.  Defines what state switch is active: NC==HIGH, NO==LOW */
     Boolean_t active_flg;/* active == "pressed" == "closed" == "falling edge" */
     Boolean_t new_transition_flg; /* if state has just transitioned between active, inactive */
}LimitSwitchInfoS;
extern LimitSwitchInfoS motion_limit_sw[POLLED_STATE_COUNT][LIMIT_SW_COUNT];
void motion_set_limit_sw_active_state(uint8_t axis, uint8_t state );
#endif

void motion_ctrl_restart_tmc2660_setup(void);
bool motion_ctrl_running(void);
int motion_poll_limit_switches(void);
bool motion_probe_switch_active(void);
bool motion_probe_new_transition(void);                                      
bool motion_ctrl_tmc2660_setup_completed(void);
void motion_set_step_direction( MotorAxisIdE axis, DirectionE direction);
bool motion_switch_active(int8_t axis);
bool motion_xlimit_active(void);
bool motion_ylimit_active(void);
bool motion_zlimit_active(void);
bool motion_alimit_active(void);
bool motion_any_limit_active(void);
bool motion_xlimit_new_transition(void);
bool motion_ylimit_new_transition(void);
bool motion_zlimit_new_transition(void);
bool motion_alimit_new_transition(void);
void motion_request_forward_plan(void); 
void motion_request_exec_move(void); 
void motion_ctrl_start_dda_interrupt(void);
void motion_ctrl_stop_dda_interrupt(void);
void motion_ctrl_init(void);
void motion_ctrl_task(void);
extern bool _probing_handler( bool state);
#endif	/* MOTION_CTRL_H */