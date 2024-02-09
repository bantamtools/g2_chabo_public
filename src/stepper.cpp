/*
 * stepper.c
 *
 *  Created on: May 18, 2022
 *      Author: matt
 */

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "main.h"
#include "stepper.h"
#include "tmc2660_mtr_ctrlr.h"

// Stepper motor variables
stepper_type mot[STEPPER_NUM_AXES];

volatile bool isTransferDone = false;

// SPI transfer complete callback
void SPIEventHandler(uintptr_t context )
{
    isTransferDone = true;
   tmc2660_notify_spi_rx_complete(); 
}
extern "C" void TC0_CH1_TimerInterruptHandler(TC_TIMER_STATUS status, uintptr_t context);
// Step timer interrupt callback
void TC0_CH1_TimerInterruptHandler(TC_TIMER_STATUS status, uintptr_t context)
{
    #define STEP_COUNT_ONE_REVOLUTION 1600
    #define TARGET_STEP_COUNT STEP_COUNT_ONE_REVOLUTION
    static uint16_t target_step_count = TARGET_STEP_COUNT;
#if 1//debug only
    static volatile  bool debug_toggle_pin=true;
if (debug_toggle_pin==true) 
{
      hal_toggle_pin(0, GPIO_DBG1_PIN );
}
#endif   
 //matt's
  // Toggle the active STEP outputs @ 100Hz
  for (int i = X_AXIS; i < STEPPER_NUM_AXES; i++) 
  {
    if (mot[i].enabled) 
    {
      mot[i].step_accum_count++;  
      PIO_PinToggle(mot[i].step_pin);
      //stop after the designated steps and measure with a ruler.
      if (mot[i].step_accum_count>=target_step_count)
      {
       //   mot[i].enabled=false;
      }
      
    }
  }
 
}

// Writes a register value out to the specified stepper driver axis
void stepper_tmc2660_write(uint8_t axis, uint32_t reg_value) {

  uint8_t spi_buf[3] = {0xFF, 0xFF, 0xFF};

  // Shift bits into [19:0] data format for motor drivers
  spi_buf[0] = reg_value >> 16;
  spi_buf[1] = (reg_value >> 8) & 0xFF;
  spi_buf[2] = reg_value & 0xFF;
 
  // Write Register value out to SPI
  PIO_PinClear(mot[axis].cs_pin);
  USART2_SPI_Write(spi_buf, 3);

  // Wait for SPI transfer to complete
  while(!isTransferDone);

  // De-assert slave select and clear flag
  PIO_PinSet(mot[axis].cs_pin);
  isTransferDone = false;
}

// Initialize the steppers and drivers
void stepper_init_matt(void) {

  int i;
  static volatile bool start_tc0_ch1_here=false;
  // Initialize the stepper array
  memset((void*)&mot, 0, sizeof(mot));

   mot[X_AXIS].cs_pin       = X_MTRDRV_SPI_CSn_PIN;
   mot[X_AXIS].dir_pin      = X_STPR_DIR_PIN;
   mot[X_AXIS].en_pin       = X_STPR_ENn_PIN;
   mot[X_AXIS].step_pin     = X_MTRDRV_STEP_PIN;
   mot[X_AXIS].enabled      = false;

   mot[Y1_AXIS].cs_pin       = Y1_MTRDRV_SPI_CSn_PIN;
   mot[Y1_AXIS].dir_pin      = Y1_STPR_DIR_PIN;
   mot[Y1_AXIS].en_pin       = Y1_STPR_ENn_PIN;
   mot[Y1_AXIS].step_pin     = Y1_MTRDRV_STEP_PIN;
   mot[Y1_AXIS].enabled      = false;

   mot[Y2_AXIS].cs_pin       = Y2_MTRDRV_SPI_CSn_PIN;
   mot[Y2_AXIS].dir_pin      = Y2_STPR_DIR_PIN;
   mot[Y2_AXIS].en_pin       = Y2_STPR_ENn_PIN;
   mot[Y2_AXIS].step_pin     = Y2_MTRDRV_STEP_PIN;
   mot[Y2_AXIS].enabled      = false;
 
   mot[Z_AXIS].cs_pin       = Z_MTRDRV_SPI_CSn_PIN;
   mot[Z_AXIS].dir_pin      = Z_STPR_DIR_PIN;
   mot[Z_AXIS].en_pin       = Z_STPR_ENn_PIN;
   mot[Z_AXIS].step_pin     = Z_MTRDRV_STEP_PIN;
   mot[Z_AXIS].enabled      = false;

  // Register the SPI transfer complete callback
  USART2_SPI_CallbackRegister(SPIEventHandler, (uintptr_t) 0); 
#if 1//sme replace bring up usage
  //until figure how to context arg to pass arg to callback, call callback directly w/o registering it
   //not here--too soon 
  if (start_tc0_ch1_here==true)//debug only
  {
      TC0_CH1_TimerStart();
  }
#else//original demo usage
  // Start 100Hz step timer
    TC0_CH1_TimerCallbackRegister(TC0_CH1_TimerInterruptHandler, (uintptr_t)NULL);
    TC0_CH1_TimerStart();
#endif 
  // Drive CSn pins high to deselect all motor driver boards on SPI
  for (i = X_AXIS; i < STEPPER_NUM_AXES; i++) {
    PIO_PinSet(mot[i].cs_pin);
    PIO_PinSet(mot[i].dir_pin);
  }

#if 1//This is done in the  motion_ctrl startup
  for (i = X_AXIS; i < STEPPER_NUM_AXES; i++) {
#ifdef CHABO_USE_MATTS_TMC2660_INIT
       PIO_PinClear(mot[i].en_pin);//sme
#endif
    stepper_tmc2660_write(i, TMC2660_DRVCTRL_REG);
    stepper_tmc2660_write(i, TMC2660_CHOPCONF_REG);
    stepper_tmc2660_write(i, TMC2660_SMARTEN_REG);
    stepper_tmc2660_write(i, TMC2660_SGCSCONF_REG);
    stepper_tmc2660_write(i, TMC2660_DRVCONF_REG);
  }
#endif
}
void tmc2660_re_init_matt(void)
{
    int i;
    // Initialize the stepper array
   memset((void*)&mot, 0, sizeof(mot));
   mot[X_AXIS].cs_pin       = X_MTRDRV_SPI_CSn_PIN;
   mot[X_AXIS].dir_pin      = X_STPR_DIR_PIN;
   mot[X_AXIS].en_pin       = X_STPR_ENn_PIN;
   mot[X_AXIS].step_pin     = X_MTRDRV_STEP_PIN;
   mot[X_AXIS].enabled      = false;

   mot[Y1_AXIS].cs_pin       = Y1_MTRDRV_SPI_CSn_PIN;
   mot[Y1_AXIS].dir_pin      = Y1_STPR_DIR_PIN;
   mot[Y1_AXIS].en_pin       = Y1_STPR_ENn_PIN;
   mot[Y1_AXIS].step_pin     = Y1_MTRDRV_STEP_PIN;
   mot[Y1_AXIS].enabled      = false;

   mot[Y2_AXIS].cs_pin       = Y2_MTRDRV_SPI_CSn_PIN;
   mot[Y2_AXIS].dir_pin      = Y2_STPR_DIR_PIN;
   mot[Y2_AXIS].en_pin       = Y2_STPR_ENn_PIN;
   mot[Y2_AXIS].step_pin     = Y2_MTRDRV_STEP_PIN;
   mot[Y2_AXIS].enabled      = false;
 
   mot[Z_AXIS].cs_pin       = Z_MTRDRV_SPI_CSn_PIN;
   mot[Z_AXIS].dir_pin      = Z_STPR_DIR_PIN;
   mot[Z_AXIS].en_pin       = Z_STPR_ENn_PIN;
   mot[Z_AXIS].step_pin     = Z_MTRDRV_STEP_PIN;
   mot[Z_AXIS].enabled      = false;
  
   // Drive CSn pins high to deselect all motor driver boards on SPI
  for (i = X_AXIS; i < STEPPER_NUM_AXES; i++)
  {
    PIO_PinSet(mot[i].cs_pin);
    PIO_PinSet(mot[i].dir_pin);
  }
  // Write out Chabo defaults for TMC2660 drivers all axes
  for (i = X_AXIS; i < STEPPER_NUM_AXES; i++) 
  { 
    PIO_PinClear(mot[i].en_pin);//sme 
    stepper_tmc2660_write(i, TMC2660_DRVCTRL_REG);
    stepper_tmc2660_write(i, TMC2660_CHOPCONF_REG);
    stepper_tmc2660_write(i, TMC2660_SMARTEN_REG);
    stepper_tmc2660_write(i, TMC2660_SGCSCONF_REG);
    stepper_tmc2660_write(i, TMC2660_DRVCONF_REG);
  }
 
}
// Turn stepper motor on
void stepper_on(uint8_t axis) {

    static bool toggle_direction_pin=true;
  // Turn on 100Hz STEP signal
  mot[axis].enabled = true;
  mot[axis].step_accum_count=0;//change direction every move
  if( toggle_direction_pin==true)
  {
    if(mot[axis].direction==0)
    {
      mot[axis].direction=1;
      PIO_PinSet(mot[axis].dir_pin);
    }
    else
    {
       mot[axis].direction=0; 
       PIO_PinClear(mot[axis].dir_pin);
    }
  }
  // Set direction
  PIO_PinClear(mot[axis].dir_pin);

  // Enable motor
  PIO_PinClear(mot[axis].en_pin);
}

// Turn stepper motor off
void stepper_off(uint8_t axis) {

  // Disable motor
  PIO_PinSet(mot[axis].en_pin);

  // Turn off 100Hz STEP signal
  mot[axis].enabled = false;
}
