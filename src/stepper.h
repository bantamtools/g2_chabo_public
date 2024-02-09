/*
 * stepper.h
 *
 *  Created on: May 18, 2022
 *      Author: matt
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "definitions.h"

// Definitions
#define TMC2660_DRVCTRL_REG   0x00005 // DRVCTRL default register settings
#define TMC2660_CHOPCONF_REG  0x901B4 // CHOPCONF default register settings
#define TMC2660_SMARTEN_REG   0xA8202 // SMARTEN default register settings
#define TMC2660_SGCSCONF_REG  0xD000F // SGCSCONF default register settings
#define TMC2660_DRVCONF_REG   0xE0050 // DRVCONF default register settings

#define STEPPER_NUM_AXES      4

// Type Definitions
 enum axis {
  X_AXIS,
  Y1_AXIS,
  Z_AXIS,
  Y2_AXIS
};
typedef struct {

  PIO_PIN            cs_pin;
  PIO_PIN            dir_pin;
  PIO_PIN            en_pin;
  PIO_PIN            step_pin;
  uint16_t           step_accum_count;
  bool               direction;
  bool               enabled;             
} stepper_type;

// Function Prototypes
void stepper_tmc2660_write(uint8_t, uint32_t);
void stepper_init_matt(void);
void stepper_on(uint8_t);
void stepper_off(uint8_t);
void tmc2660_re_init_matt(void);//especially for re-init after e-stop
#endif /* INC_STEPPER_H_ */
