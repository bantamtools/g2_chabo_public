/**
  ******************************************************************************
  * @file    tmc2660_mtr_ctrlr.h
  * @author  SME
  * @version V1.0.0
  * @date    27-Feb-2018
  * @brief    Header for the Trinamic TMC2660 Motor Controller driver chip
  ******************************************************************************
  ******************************************************************************
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TMC2660_MTR_CTRLR_H
#define __TMC2660_MTR_CTRLR_H

//#ifdef __cplusplus
//extern "C" {
//#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32f7xx_hal_def.h"
#include "main.h"
#include "bantam_hal.h"
#include "user_types.h"
#include "g2_config.h" 
  
/* Defines */
//#define USE_TMC2660C_DRIVERS

#ifdef USE_TMC2660C_DRIVERS
#define DRVR_CFG_OTSENS     1  // 1=sensitive overtemp shutdown @ 136C
#define DRVR_CFG_SHRTSENS   1  // 1=short to GND high sensitivity
#define DRVR_CFG_EN_PFD     0  // 0=no additional motor dampening
#define DRVR_CFG_EN_S2VS    1  // 1=short to VS and clock failsafe protection enabled
#endif

#define  DEPLOY_TRINAMICS_TUNE_JSON_CMDS 
#define TMC2660_ENABLE_PERIODIC_PRINTING_READBACKS

/* Define STEPS PER RE AND uSteps per step: */
#define STEPPER_STEPS_PER_REVOLUTION 200 
#define STEPPER_USTEPS_PER_STEP 8 //DEBUG ONLY 9-6-2019   
#define STEPPER_USTEPS_PER_REVOL     (STEPPER_USTEPS_PER_STEP*STEPPER_STEPS_PER_REVOLUTION)

/* Define tmc2660 factory defaults provided by app note */
#define DRVCTRL_DEFAULT_VAL  0x00000 //256 usteps per whole step --change to app default */
#define DRVCONF_DEFAULT_VAL  0xEF010 //High gate drv strength, SDOff==0
#define SGSCONF_DEFAULT_VAL  0xD001F //Current setting: (maximum current)
#define SMARTEN_DEFAULT_VAL  0xA8202 //Enable coolStep with minimum current = ¼ of max. I
#define CHOPCONF_DEFAULT_VAL 0x901B4 //Use hysteresis mode chopper

/* Define register values for designated uStep per step:*/
#define SD_CFG_USTEP_256  0
#define SD_CFG_USTEP_128  1
#define SD_CFG_USTEP_64   2
#define SD_CFG_USTEP_32   3
#define SD_CFG_USTEP_16   4 
#define SD_CFG_USTEP_8    5
#define SD_CFG_USTEP_4    6
#define SD_CFG_USTEP_HALF_STEP 7
#define SD_CFG_USTEP_FULL_STEP 8
#define SD_USER_UNITS_USTEP_MIN 1
#define SD_USER_UNITS_USTEP_MAX 256// 12/3/2019 limit to 128 until change tinyG var from u8 to u16  
/* Define TMC2660 control/config bit-field assignments */
#define DRVR_CFG_TEST_MODE_OFF 0//OFF
  
#define DRVR_CFG_SD_MODE_STEP_DIR 0
#define DRVR_CFG_SD_MODE_SPI_INTERFACE 1
#define DRVR_CFG_SD_MODE DRVR_CFG_SD_MODE_STEP_DIR
  
#define DRVR_CFG_SLOPE_CONTROL_MIN 0 
#define DRVR_CFG_SLOPE_CONTROL_MAX 3
#define DRVR_CFG_SLOPE_CTRL_DEFAULT_HI 0 
#define DRVR_CFG_SLOPE_CTRL_DEFAULT_LS 0  
#define DRVR_CFG_SLOPE_CONTROL_HS DRVR_CFG_SLOPE_CTRL_DEFAULT_HI//MIN
#define DRVR_CFG_SLOPE_CONTROL_LS DRVR_CFG_SLOPE_CTRL_DEFAULT_LS//MIN
  
#define DRVR_CFG_SHORT_TO_GROUND_PROT_ENABLE 0
#define DRVR_CFG_SHORT_TO_GROUND_PROT_DISABLE 1
  
#define DRVR_CFG_SHORT_TO_GROUND_PROT_DIS DRVR_CFG_SHORT_TO_GROUND_PROT_ENABLE//0==PROTECTION ENABLED

#define TIME_3PT2_USEC  0
#define TIME_1PT6_USEC  1
#define TIME_1PT2_USEC  2
#define TIME_0PT8_USEC  3
#define SHRT_TO_GND_MIN_DLY 0//NOTE INVERSE PROPORTIONALITY 
#define SHRT_TO_GND_MAX_DLY 3

#define DRVR_CFG_SHORT_TO_GND_DETECT_TMR TIME_3PT2_USEC
#define DRVR_CFG_SD_ENABLE 0//enable StepDir mode
#define DRVR_CFG_SD_DISABLE 1//enable SPI mode

#define VAL_310_MV 0
#define VAL_165_MV 1

#if 1//12/22/2020
typedef enum
{
  TMC_READBACK_USTEPS,
  TMC_READBACK_SG,
  TMC_READBACK_SG_AND_COOLSTEP,
  TMC_READBACK_COUNT,
 TMC_DEFAULT_READBACK=TMC_READBACK_SG,
 DRVR_CFG_READOUT_SELECT=TMC_DEFAULT_READBACK  
}TmcReadBackIdE;
#else
#define USTEP_READBACK 0
#define SG_READBACK 1
#define DEFAULT_READBACK SG_READBACK
#define SG_AND_COOLSTEP_READBACK 2
#define DRVR_CFG_READOUT_SELECT DEFAULT_READBACK
#endif
#define SG_CFG_FILTER_ENABLE 1//0//STD, 1//FILTERED
#define SG_CFG_THRESHOLD_MINVAL -64
#define SG_CFG_THRSHLD_RECOMMENDED_MIN_LIMIT -10   
#define SG_CFG_THRESHOLD_MAXVAL 63
#define SG_CFG_THRESHOLD_START_VAL 0//0
#define SG_CFG_THRESHOLD_VAL SG_CFG_THRESHOLD_START_VAL  
#define SG_CFG_ISCALE_MIN 0
#define SG_CFG_ISCALE_MAX 31 
    
#ifdef FINTAN_SG_TUNE_CS
#define SG_CFG_ISCALE_STARTVAL 3//Note, on othermill this value causes a board reset!!!
#define DRVR_CFG_VSENSE_FSCALE VAL_165_MV  
#else
#define SG_CFG_ISCALE_STARTVAL 3// 26==harsh!//15, Fintan uses 31
#define DRVR_CFG_VSENSE_FSCALE VAL_165_MV// Fintan's is 310 VAL_310_MV 
#endif
#define SG_CFG_ISCALE  SG_CFG_ISCALE_STARTVAL 

#define SD_CFG_INTERPOL_ENABLE  1 
#define SD_CFG_INTERPOL_DISABLE 0 
#define SD_CFG_INTERPOL SD_CFG_INTERPOL_DISABLE
#define SD_CFG_RISING_EDGE_ONLY 0
#define SD_CFG_RISING_AND_FALLING_EDGES 1
#define SD_CFG_EDGES_PER_PULSE SD_CFG_RISING_EDGE_ONLY

//0xA8202 101X 1 00 X 0100 X 00 X 0010
#define CS_CTRL_SEUP_COIL_INCR_MIN 0
#define CS_CTRL_SEUP_COIL_INCR_MAX 1 //1.2 can't be a float range is integer 0..3//7-1-2022 dcnc-specific. tbd: minimill's max setting may be different
#define CS_CTRL_SEUP_COIL_INCR  CS_CTRL_SEUP_COIL_INCR_MIN
  
#define CS_CTRL_STANDSTILL_AMPS_MIN 0 
#define CS_CTRL_STANDSTILL_AMPS_MAX 1  
#define CS_CTRL_SE_IMIN  1//FACTORY =1/4
  
#define CS_CTRL_I_DECREM_SPEED 0//0=FACTORY==32
#define CS_CTRL_I_INCREM_SIZE 0//0==1 FACTORY
  
#define CS_CTRL_SE_THRESH_MIN 0  
#define CS_CTRL_SE_THRESH_MAX 15   
#define CS_CTRL_SE_DISABLED 0  

#define CS_CTRL_SE_MAX_THRESH 4//2//4//4==FACTORY
#define CS_CTRL_SE_MIN_THRESH CS_CTRL_SE_DISABLED//2==FACTORY
  
#define CS_READBACK_MIN 0 
#define CS_READBACK_MAX 32 
  
#define STALL_VAL_READBACK_MIN 0 
#define STALL_VAL_READBACK_MAX 1023
  
#define SPI_MODE_POLARITY_COIL_PINS_1_TO_2 0
#define SPI_MODE_POLARITY_COIL_PINS_2_TO_1 1
  
#define SPI_MODE_POLARITY_COIL_A SPI_MODE_POLARITY_COIL_PINS_1_TO_2
#define SPI_MODE_POLARITY_COIL_B SPI_MODE_POLARITY_COIL_PINS_1_TO_2
  
#define MIN_IFLOW 0
#define MAX_IFLOW 248
#define DEFAULT_IFLOW 64//ARBITRARY
#define SPI_MODE_IFLOW_COIL_A DEFAULT_IFLOW
#define SPI_MODE_IFLOW_COIL_B DEFAULT_IFLOW

#define BLANKING_TIME_16_CLOCK_PERIODS 0
#define BLANKING_TIME_24_CLOCK_PERIODS 1
#define BLANKING_TIME_36_CLOCK_PERIODS 2//FACTORY
#define BLANKING_TIME_54_CLOCK_PERIODS 3
#define CHOP_CFG_BLANKING_TIME BLANKING_TIME_36_CLOCK_PERIODS

#define CHOP_MODE_STD_SPREAD_CYCLE 0
#define CHOP_MODE_CONST_TIME_OFF 1
#define CHOP_CFG_MODE CHOP_MODE_STD_SPREAD_CYCLE// CHOP_MODE_CONST_TIME_OFF
  
#define CHOP_TOFF_FIXED 0 
#define CHOP_TOFF_RANDOM 1
#define CHOP_CFG_RANDOM_OFF_TIME CHOP_TOFF_FIXED//RANDOM
  
#define CHOP_HYST_DECR_INTRVL_16_CYCLES 0
#define CHOP_HYST_DECR_INTRVL_32_CYCLES 1
#define CHOP_HYST_DECR_INTRVL_48_CYCLES 2
#define CHOP_HYST_DECR_INTRVL_64_CYCLES 3
#define CHOP_HYST_DECR_INTRVL CHOP_HYST_DECR_INTRVL_16_CYCLES 
  
#define CHOP_FAST_DECAY_TIME_OFF_FREEWHEELS 0//off freewheels   
#define CHOP_FAST_DECAY_MIN_OFF_TIME 1// 
#define CHOP_FAST_DECAY_MAX_OFF_TIME 15// 
#define CHOP_FAST_DECAY_TIME CHOP_FAST_DECAY_TIME_OFF_FREEWHEELS
  
#define CHOP_FAST_DECAY_MODE_COMPARATOR_END 0
#define CHOP_FAST_DECAY_MODE_TIME_END 1
#define CHOP_FAST_DECAY_MODE CHOP_FAST_DECAY_MODE_TIME_END //CHOP_FAST_DECAY_COMPARATOR_END

#define CHOP_CFG_HYST_DECR_OR_FAST_DECAY_MODE CHOP_HYST_DECR_INTRVL_16_CYCLES 
#define CHOP_HYST_END_MIN -3
#define CHOP_HYST_END_DEFAULT 3//FACTORY
#define CHOP_HYST_END_MAX 12
  
#define CHOP_SINE_OFFSET_MIN 0//0 maps to-3, 1 to -2, 2 to -1
#define CHOP_SINE_NO_OFFSET 3//MAPS TO 0 
#define CHOP_SINE_OFFSET_MAX 15 //RANGE 4-15 IS POSITIVE 
#define CHOP_SINE_OFFSET CHOP_SINE_NO_OFFSET
 
#define CHOP_CFG_HYST_END_VAL_OR_SINE_OFFSET CHOP_HYST_END_DEFAULT
#define CHOP_HYST_START_MIN 0
#define CHOP_HYST_START_MAX 15

#define CHOP_HYST_START CHOP_HYST_START_MAX 
#define CHOP_HYST_END CHOP_HYST_END_MAX 
#define CHOP_HYST_START_DEFAULT 3//3==FACTORY  
#define CHOP_CFG_HYST_START_VAL_OR_FAST_DECAY CHOP_HYST_START_DEFAULT
  
#define CHOP_MOSFET_OFF  0 
#define CHOP_OFF_TIME_DEFAULT 4//FACTORY
#define CHOP_OFF_TIME_MAX 15
#define CHOP_CFG_OFF_TIME_OR_MOSFET_DISABLE CHOP_OFF_TIME_DEFAULT 
//0x901B4 100 10 0 0 00 0011 011 0100

/* Define application startup settings */
#define DRVCTRL_INIT_VAL 0
#define DRVCONF_INIT_VAL 0
#define SGSCONF_INIT_VAL 0
#define SMARTEN_INIT_VAL 0 
#define CHOPCONF_INIT_VAL 0
#define SG_MIN_STALL_VAL 0
  
/* Enumerate read selections*/
typedef enum
{
  READ_SELECT_USEPS,
  READ_SELECT_STALL_GUARD_VALUE,
  READ_SELECT_COOLSTEP,
  READ_SELECT_COUNT
}Tmc2660ReadSelIdE;

/* Enumerate the Microsteps designations */
typedef enum
{
  USTEPS_256=SD_CFG_USTEP_256,
  USTEPS_128,
  USTEPS_64,
  USTEPS_32,
  USTEPS_16,
  USTEPS_8,
  USTEPS_4,
  USTEPS_2,
  USTEPS_1,
  USTEPS_COUNT  
}Tmc2660MicrostepsE;
  
/* 
 * Define uStep setting for control register
 * which must correspond with: #define STEPPER_USTEPS_PER_STEP 
*/ 

#define SD_CFG_USTEP_RESOLUTION SD_CFG_USTEP_8 //SD_CFG_USTEP_FULL_STEP//SD_CFG_USTEP_8 DEBUG ONLY 9-6-2019 
#define SD_CFG_USTEP_RESOL_MIN USTEPS_256
#define SD_CFG_USTEP_RESOL_MAX USTEPS_1
/* Enumerate TMC2660 REGISTERS */
typedef enum
{
  TMC2660_REG_DRVCTRL,
  TMC2660_REG_CHOPPER_CFG,
  TMC2660_REG_COOLSTEP_CTRL,
  TMC2660_REG_SG_CFG,
  TMC2660_REG_DRV_CFG,
  TMC2660_MAX_WRITE_REG = TMC2660_REG_DRV_CFG,  
  TMC2660_REG_READ_STATUS,  
  TMC2660_REG_COUNT
}Tmc2660_command_idE;


typedef struct
{
  uint16_t spi_ssl;   
}SpiSlaveAccessInfoS;

/* Enumerate states needed to init all registers, using non-blocking SPI*/
typedef enum
{
 SETUP_STATE_IDLE, 
 SETUP_STATE_START,
 
 SETUP_CHOPCONF,
 POLL_CHOPCONF_COMPLETE,
 
 SETUP_SGCSCONF, 
 POLL_SGCSCONF_COMPLETE,
 
 SETUP_DRVCONF,
 POLL_DRVCONF_COMPLETE,
 
 SETUP_DRVCTRL_STEP_DIR,  
 POLL_DRVCTRL_STEP_DIR_COMPLETE,
 
 SETUP_SMARTEN, 
 POLL_SMARTEN_COMPLETE,
 
 TMC2660_SETUP_COMPLETE,
 TMC2660_START_READ_STATUS,
 TMC2660_POLL_READ_STATUS_COMPLETE,
 TMC2660_SETUP_COUNT 
}Tmc2660SetupRegStateE;

#ifdef DEPLOY_TRINAMICS_TUNE_JSON_CMDS 
#define M1_TN_COOLSTEP_THRESH_MIN CS_CTRL_SE_MIN_THRESH
#define M1_TN_COOLSTEP_THRESH_MAX CS_CTRL_SE_MAX_THRESH

#define M2_TN_COOLSTEP_THRESH_MIN CS_CTRL_SE_MIN_THRESH
#define M2_TN_COOLSTEP_THRESH_MAX CS_CTRL_SE_MAX_THRESH

#define M3_TN_COOLSTEP_THRESH_MIN CS_CTRL_SE_MIN_THRESH
#define M3_TN_COOLSTEP_THRESH_MAX CS_CTRL_SE_MAX_THRESH 

#define M4_TN_COOLSTEP_THRESH_MIN CS_CTRL_SE_MIN_THRESH
#define M4_TN_COOLSTEP_THRESH_MAX CS_CTRL_SE_MAX_THRESH 

#define M1_TN_COIL_I_INCR_UP CS_CTRL_SEUP_COIL_INCR
#define M1_TN_COIL_I_INCR_DN CS_CTRL_SEUP_COIL_INCR 

#define M2_TN_COIL_I_INCR_UP CS_CTRL_SEUP_COIL_INCR
#define M2_TN_COIL_I_INCR_DN CS_CTRL_SEUP_COIL_INCR 

#define M3_TN_COIL_I_INCR_UP CS_CTRL_SEUP_COIL_INCR
#define M3_TN_COIL_I_INCR_DN CS_CTRL_SEUP_COIL_INCR 

#define M4_TN_COIL_I_INCR_UP CS_CTRL_SEUP_COIL_INCR
#define M4_TN_COIL_I_INCR_DN CS_CTRL_SEUP_COIL_INCR 

#define M1_TN_SE_IMIN CS_CTRL_SE_IMIN
#define M2_TN_SE_IMIN CS_CTRL_SE_IMIN   
#define M3_TN_SE_IMIN CS_CTRL_SE_IMIN
#define M4_TN_SE_IMIN CS_CTRL_SE_IMIN

#define M1_TN_SG_THRESH_VAL 10 //63//emperically determined on prototype SG_CFG_THRESHOLD_VAL
#define M2_TN_SG_THRESH_VAL 10 //14//emperically determined on prototype SG_CFG_THRESHOLD_VAL   
#define M3_TN_SG_THRESH_VAL 10 // emperically determined on prototype SG_CFG_THRESHOLD_VALSG_CFG_THRESHOLD_VAL  
#define M4_TN_SG_THRESH_VAL 10 // 14//emperically determined on prototype SG_CFG_THRESHOLD_VAL   SG_CFG_THRESHOLD_VAL  

#define M1_TN_SG_FILT_EN SG_CFG_FILTER_ENABLE
#define M2_TN_SG_FILT_EN SG_CFG_FILTER_ENABLE 
#define M3_TN_SG_FILT_EN SG_CFG_FILTER_ENABLE
#define M4_TN_SG_FILT_EN SG_CFG_FILTER_ENABLE

#define M1_TN_CHOP_CFG_MODE CHOP_CFG_MODE
#define M2_TN_CHOP_CFG_MODE CHOP_CFG_MODE   
#define M3_TN_CHOP_CFG_MODE CHOP_CFG_MODE  
#define M4_TN_CHOP_CFG_MODE CHOP_CFG_MODE 

#define M1_TN_CHOP_CFG_BLNKNG_TIME CHOP_CFG_BLANKING_TIME
#define M2_TN_CHOP_CFG_BLNKNG_TIME CHOP_CFG_BLANKING_TIME  
#define M3_TN_CHOP_CFG_BLNKNG_TIME CHOP_CFG_BLANKING_TIME
#define M4_TN_CHOP_CFG_BLNKNG_TIME CHOP_CFG_BLANKING_TIME

#define M1_TN_CHOP_CFG_RNDM_OFF_TIME CHOP_CFG_RANDOM_OFF_TIME
#define M2_TN_CHOP_CFG_RNDM_OFF_TIME CHOP_CFG_RANDOM_OFF_TIME   
#define M3_TN_CHOP_CFG_RNDM_OFF_TIME CHOP_CFG_RANDOM_OFF_TIME  
#define M4_TN_CHOP_CFG_RNDM_OFF_TIME CHOP_CFG_RANDOM_OFF_TIME 

#define M1_TN_CHOP_FAST_DECAY_TIME CHOP_FAST_DECAY_TIME
#define M2_TN_CHOP_FAST_DECAY_TIME CHOP_FAST_DECAY_TIME  
#define M3_TN_CHOP_FAST_DECAY_TIME CHOP_FAST_DECAY_TIME 
#define M4_TN_CHOP_FAST_DECAY_TIME CHOP_FAST_DECAY_TIME

#define M1_TN_CHOP_FAST_DECAY_MODE CHOP_FAST_DECAY_MODE
#define M2_TN_CHOP_FAST_DECAY_MODE CHOP_FAST_DECAY_MODE   
#define M3_TN_CHOP_FAST_DECAY_MODE CHOP_FAST_DECAY_MODE  
#define M4_TN_CHOP_FAST_DECAY_MODE CHOP_FAST_DECAY_MODE

#define M1_TN_CHOP_SINE_OFFSET CHOP_SINE_OFFSET
#define M2_TN_CHOP_SINE_OFFSET CHOP_SINE_OFFSET   
#define M3_TN_CHOP_SINE_OFFSET CHOP_SINE_OFFSET
#define M4_TN_CHOP_SINE_OFFSET CHOP_SINE_OFFSET

#define M1_TN_CHOP_HYST_DECR_INTRVL CHOP_HYST_DECR_INTRVL
#define M2_TN_CHOP_HYST_DECR_INTRVL CHOP_HYST_DECR_INTRVL   
#define M3_TN_CHOP_HYST_DECR_INTRVL CHOP_HYST_DECR_INTRVL
#define M4_TN_CHOP_HYST_DECR_INTRVL CHOP_HYST_DECR_INTRVL

#define M1_TN_CHOP_HYST_START_DFLT CHOP_HYST_START_DEFAULT
#define M2_TN_CHOP_HYST_START_DFLT CHOP_HYST_START_DEFAULT   
#define M3_TN_CHOP_HYST_START_DFLT CHOP_HYST_START_DEFAULT
#define M4_TN_CHOP_HYST_START_DFLT CHOP_HYST_START_DEFAULT

#define M1_TN_CHOP_HYST_END_DFLT CHOP_HYST_END_DEFAULT
#define M2_TN_CHOP_HYST_END_DFLT CHOP_HYST_END_DEFAULT   
#define M3_TN_CHOP_HYST_END_DFLT CHOP_HYST_END_DEFAULT 
#define M4_TN_CHOP_HYST_END_DFLT CHOP_HYST_END_DEFAULT

#define M1_TN_CHOP_OFF_TIME_DFLT CHOP_OFF_TIME_DEFAULT
#define M2_TN_CHOP_OFF_TIME_DFLT CHOP_OFF_TIME_DEFAULT   
#define M3_TN_CHOP_OFF_TIME_DFLT CHOP_OFF_TIME_DEFAULT 
#define M4_TN_CHOP_OFF_TIME_DFLT CHOP_OFF_TIME_DEFAULT

#define M1_TN_DRVR_CFG_SLOPE_CNTRL_LS DRVR_CFG_SLOPE_CONTROL_LS
#define M2_TN_DRVR_CFG_SLOPE_CNTRL_LS DRVR_CFG_SLOPE_CONTROL_LS   
#define M3_TN_DRVR_CFG_SLOPE_CNTRL_LS DRVR_CFG_SLOPE_CONTROL_LS 
#define M4_TN_DRVR_CFG_SLOPE_CNTRL_LS DRVR_CFG_SLOPE_CONTROL_LS

#define M1_TN_DRVR_CFG_SLOPE_CNTRL_HS DRVR_CFG_SLOPE_CONTROL_HS
#define M2_TN_DRVR_CFG_SLOPE_CNTRL_HS DRVR_CFG_SLOPE_CONTROL_HS   
#define M3_TN_DRVR_CFG_SLOPE_CNTRL_HS DRVR_CFG_SLOPE_CONTROL_HS  
#define M4_TN_DRVR_CFG_SLOPE_CNTRL_HS DRVR_CFG_SLOPE_CONTROL_HS 

#define M1_TN_DRVR_CFG_S2G_PROT_DIS DRVR_CFG_SHORT_TO_GROUND_PROT_DIS
#define M2_TN_DRVR_CFG_S2G_PROT_DIS DRVR_CFG_SHORT_TO_GROUND_PROT_DIS   
#define M3_TN_DRVR_CFG_S2G_PROT_DIS DRVR_CFG_SHORT_TO_GROUND_PROT_DIS  
#define M4_TN_DRVR_CFG_S2G_PROT_DIS DRVR_CFG_SHORT_TO_GROUND_PROT_DIS

#define M1_TN_DRVR_CFG_S2G_DETCT_TMR DRVR_CFG_SHORT_TO_GND_DETECT_TMR
#define M2_TN_DRVR_CFG_S2G_DETCT_TMR DRVR_CFG_SHORT_TO_GND_DETECT_TMR   
#define M3_TN_DRVR_CFG_S2G_DETCT_TMR DRVR_CFG_SHORT_TO_GND_DETECT_TMR
#define M4_TN_DRVR_CFG_S2G_DETCT_TMR DRVR_CFG_SHORT_TO_GND_DETECT_TMR

#define M1_TN_DRVR_CFG_TEST_MODE_OFF DRVR_CFG_TEST_MODE_OFF
#define M2_TN_DRVR_CFG_TEST_MODE_OFF DRVR_CFG_TEST_MODE_OFF   
#define M3_TN_DRVR_CFG_TEST_MODE_OFF DRVR_CFG_TEST_MODE_OFF 
#define M4_TN_DRVR_CFG_TEST_MODE_OFF DRVR_CFG_TEST_MODE_OFF

#define M1_TN_SD_CFG_INTERPOL SD_CFG_INTERPOL
#define M2_TN_SD_CFG_INTERPOL SD_CFG_INTERPOL   
#define M3_TN_SD_CFG_INTERPOL SD_CFG_INTERPOL 
#define M4_TN_SD_CFG_INTERPOL SD_CFG_INTERPOL

#define M1_TN_EDGES_PER_PULSE SD_CFG_EDGES_PER_PULSE
#define M2_TN_EDGES_PER_PULSE SD_CFG_EDGES_PER_PULSE   
#define M3_TN_EDGES_PER_PULSE SD_CFG_EDGES_PER_PULSE
#define M4_TN_EDGES_PER_PULSE SD_CFG_EDGES_PER_PULSE

#define M1_TN_DRVR_CFG_SD_EN  DRVR_CFG_SD_ENABLE
#define M2_TN_DRVR_CFG_SD_EN  DRVR_CFG_SD_ENABLE   
#define M3_TN_DRVR_CFG_SD_EN  DRVR_CFG_SD_ENABLE  
#define M4_TN_DRVR_CFG_SD_EN  DRVR_CFG_SD_ENABLE

#define M1_TN_SG_CFG_ISCALE SG_CFG_ISCALE
#define M2_TN_SG_CFG_ISCALE SG_CFG_ISCALE   
#define M3_TN_SG_CFG_ISCALE SG_CFG_ISCALE
#define M4_TN_SG_CFG_ISCALE SG_CFG_ISCALE

#define M1_TN_DRVR_CFG_VSENSE_FSCALE DRVR_CFG_VSENSE_FSCALE
#define M2_TN_DRVR_CFG_VSENSE_FSCALE DRVR_CFG_VSENSE_FSCALE   
#define M3_TN_DRVR_CFG_VSENSE_FSCALE DRVR_CFG_VSENSE_FSCALE
#define M4_TN_DRVR_CFG_VSENSE_FSCALE DRVR_CFG_VSENSE_FSCALE

#ifdef USE_TMC2660C_DRIVERS
#define M1_TN_DRVR_CFG_OTSENS DRVR_CFG_OTSENS
#define M2_TN_DRVR_CFG_OTSENS DRVR_CFG_OTSENS
#define M3_TN_DRVR_CFG_OTSENS DRVR_CFG_OTSENS
#define M4_TN_DRVR_CFG_OTSENS DRVR_CFG_OTSENS

#define M1_TN_DRVR_CFG_SHRTSENS DRVR_CFG_SHRTSENS
#define M2_TN_DRVR_CFG_SHRTSENS DRVR_CFG_SHRTSENS
#define M3_TN_DRVR_CFG_SHRTSENS DRVR_CFG_SHRTSENS
#define M4_TN_DRVR_CFG_SHRTSENS DRVR_CFG_SHRTSENS

#define M1_TN_DRVR_CFG_EN_PFD DRVR_CFG_EN_PFD
#define M2_TN_DRVR_CFG_EN_PFD DRVR_CFG_EN_PFD
#define M3_TN_DRVR_CFG_EN_PFD DRVR_CFG_EN_PFD
#define M4_TN_DRVR_CFG_EN_PFD DRVR_CFG_EN_PFD

#define M1_TN_DRVR_CFG_EN_S2VS DRVR_CFG_EN_S2VS
#define M2_TN_DRVR_CFG_EN_S2VS DRVR_CFG_EN_S2VS
#define M3_TN_DRVR_CFG_EN_S2VS DRVR_CFG_EN_S2VS
#define M4_TN_DRVR_CFG_EN_S2VS DRVR_CFG_EN_S2VS
#endif

typedef struct
{
  uint16_t unused:7;
  uint16_t dir:1;
  uint16_t ustep_counts:8;
}Tmc2660_ustepInfoS;

typedef  union 
{
 uint16_t u16_data; 
 uint8_t u8_data[sizeof(uint16_t)];
 Tmc2660_ustepInfoS struct_data;
}Tmc2660_ustepInfoU;
#pragma pack(1)
/* Define a struct the contains all Trinamics motor controller parameters */
typedef struct
{
//Sets:
  /* Coolstep control reg: */
  uint8_t  cs_set_i_standstill;  //"1tnseimin" minimum standstill current limit,range:  0= CS/2, 1 = CS=1/4
  uint8_t  cs_set_i_decr_speed;  //"1tnsedn" coil current decrement speed range 0-3
  uint8_t  cs_set_se_max_thresh; //"1tnsethh" se=SmartEnergy akaCoolStep, range 0-15, [sme???"255 disables" ] 
  uint8_t  cs_set_i_incr_step_size;//"1tnseup" coil current increments range 0-3 
  uint8_t  cs_set_se_min_thresh;//"1tnsethl" se=SmartEnergy akaCoolStep, range 0-15, [sme???"255 disables" ]
  
  /* StallGuard2 control reg: */
  uint8_t  sg_set_stall_filter_enable;//"1tnsgf" enable=1, disable=0 stall guard filtering
  int8_t   sg_set_stall_threshold;  //"1tnsgt" SG==Stallguard, range -64-+63, advised to stay above -10
  uint8_t  sg_set_i_scale;//"1tncs"  Set Current Scale (CS)  Range 0-31, 0=1/32, 1=2/32,...31=32/32
  
 /* Chopper Ctrl reg: */
  uint8_t  chpr_set_mode;//"1tnchm" set chopper mode range  0=spreadcycle, 1= constant off-time 
  uint8_t  chpr_set_blanking_time;//"1tnchbt" set blanking time rnge 0-3: 0=16,1=24,2=36,3=54
  uint8_t  chpr_set_rndm_off_time;//"1tnchrt" set random TimeOff enable. range 0= disable, 1=enable needed when hering audible beat frequency
  uint8_t  chpr_set_chm1_fast_decay_mode;//"1tnchfd" CHM=1,set fast decay mode. range  0=enable comparator to terminate, 1= end by time only, 
  uint8_t  chpr_set_chm1_fast_decay_time;//"1tnchfd" CHM=1,set fast decay time. range 0= slow, 1-15 duration of fast decay, precondition: 
  uint8_t  chpr_set_chm1_sine_offset;//"1tnchswo" CH1=1,Set Sine wave offset, range 0-2 negative, 3 no offset, 4-15 positive offset  precondition   
  uint8_t  chpr_set_chm0_hyst_decr_intrvl;//"1tnchdi" CHM=0,Set Hysteresis decrement interval in system clocks range 0=16,1=32,2=48,3=64 precondition: 
  uint8_t  chpr_set_chm0_hyst_start;//"1tnchst" (CHM=0),Set (HSTRT)- Hysteresis start setting it's an offset from HEND 0=1,1=2...7=8 precondition: Spreadycycle 
  uint8_t  chpr_set_chm0_hyst_end;//"1tnchen" CHM=0, Set HEND - Hysteresting end setting  range 0-15, precondition: 
  uint8_t  chpr_set_off_time;//"1tnchot" Set TOFF - Off Time: range 0=chopper off; Freewheels the motor 1 to 15= slow decay time t=1/(f_clck)*((Toff*32)+12)

  uint8_t  pwr_set_slope_ls;//"1tnscl"  Set SLPL - Low-side slope control range 0=Min 3=Max
  uint8_t  pwr_set_slope_hs;//"1tnsch" Set SLPH - High-side slope control 0=Min, 1=Min+Temper compens, 2=Med+Temper compens,3=Max
  uint8_t  prtct_set_shrt2gnd;//"1tnlgp"  Set DISS2G range 1=Enable/0=Disable short to ground protection 
  uint8_t  prtct_set_shrt2gnd_dly;//"1tnlgd" Set TS2G - Short to ground detection delay range 0=3.2uS, 1=1.6uS, 10=1.2uS, 11=0.8uS
  uint8_t  diag_set_test_mode;//"tntst" TST: Range 0-3  0= Normal operation
  uint8_t  drv_set_step_interpl;//"1tndsi"  Enable Step interpolation range 1-Enable, 0-Disable
  uint8_t  drv_set_dbl_edge_step;//"1tnde"  Enable double edge Step pulses 1: Both rising and falling STEP pulse edges are active
  uint8_t  drv_set_stpdir_off;//"1tnsdo" SDOFF: 0=Enable 1=Disable Step / dir interface  

  uint16_t  drv_set_mstep_resol;//"1mi" [Reconcile tn with tinyG]Set microstep resolution 8=1, 7=2...5=8,..0=256
  uint8_t  drv_set_vsense;//"1tncv"  Set sense Vsense value Range 0-1,  0 = Vsense to 0.36V,1 = Vsense to 0.165V (for reduced power dissipation)  
#ifdef USE_TMC2660C_DRIVERS
  uint8_t  drv_set_otsens;//"tnotsens"  Overtemperature sensitivity:0-1,  0 = shutdown at 150C, 1 = sensitive shutdown at 136C
  uint8_t  drv_set_shrtsens;//"tnshrts"  Short to GND detection sensitivity Range 0-1,  0 = low sensitivity, 1 = high sensitivity 
  uint8_t  drv_set_en_pfd;//"tnenpfd"  Enable Passive fast decay / 5V undervoltage threshold Range 0-1,  0 = no additional dampening, 1 = dampening 
  uint8_t  drv_set_en_s2vs;//"tnens2vs"  Enable short to VS & CLK fail protection Range 0-1,  0 = disabled, 1 = enabled 
#endif

  //Gets:
  uint8_t  cs_get_coolstep_value;  //"1tnser" read coolstep se value
  uint8_t  diag_get_stndstill_indic;//"1tnlstr" Read Standstill indicator (STST)range 0=No standstill, 1=Stand still
  uint8_t  sg_get_stall_flag;   //"1tnsgdr"  range: 0=no stall, 1= stall happened
  uint8_t  prtct_get_a_b_shrt2gnd;//"1tnlgr" Read S2GA and S2GB range 0=No short 1=Short 
  uint8_t  prtct_get_ovrtmp_wrn;//"1tnlotr" Read OTPW - Overtemp warning
  uint8_t  prtct_get_ovrtmp_shtdn;//"1tnlotr" Read OT - Overtemp shutdown
  uint8_t  prtct_get_openld_a_b;//"1tnlolr" Read OLA and OLB - Open load detection range 0=No open load detected,1= Open load detected
  uint16_t drv_get_mstep_position;//"1tndmr" Read Microstep position for coil A in table range:
  uint16_t safety_fence;//this should remain the init value
  uint16_t sg_get_stall_val; //"1tnsgr" range 0-1023, high load is inversely proportional to value
  uint32_t safety_fence2;//this should remain the init value
}Tmc2660TuneParamS;

/* Define a struct that contains just the readback items , for tracking changes */
typedef struct
{
  uint16_t drv_get_mstep_position;//"1tndmr" Read Microstep position for coil A in table range:  
  uint16_t sg_get_stall_val;       //"1tnsgr" range 0-1023, high load is inversely proportional to value
  uint8_t  sg_get_stall_flag;   //"1tnsgdr"  range: 0=no stall, 1= stall happened
  uint8_t  cs_get_coolstep_value;  //"1tnser" read coolstep se value
  uint8_t  diag_get_stndstill_indic;//"1tnlstr" Read Standstill indicator (STST)range 0=No standstill, 1=Stand still
  uint8_t  prtct_get_a_b_shrt2gnd;//"1tnlgr" Read S2GA and S2GB range 0=No short 1=Short 
  uint8_t  prtct_get_openld_a_b;//"1tnlolr" Read OLA and OLB - Open load detection range 0=No open load detected,1= Open load detected
  uint8_t  prtct_get_ovrtmp_wrn;//"1tnlotr" Read OTPW - Overtemp warning
  uint8_t  prtct_get_ovrtmp_shtdn;//"1tnlotr" Read OT - Overtemp shutdown 
}Tmc2660ReadbackInfoS;

/* Enumerate the subset of tn readback items */
typedef enum
{
    READ_USTEPS,
    READ_STALL_GUARD_VAL,
    READ_COOLSTEP,
    READ_STNDSTILL,
    READ_SHRT2GND,
    READ_OT_WRN,
    READ_OT_SHTDN,
    READ_STALL_EVENT,
    READ_OPEN_LOAD,    
    READ_COUNT
}Tmc2660ReadbackItemE;

#endif
/*  4-30-20201 Provide a means to restrict response polling to a single motor*/
typedef enum
{ 
  SELECT_NONE=0,
  SELECT_MOTOR1,
  SELECT_MOTOR2,
  SELECT_MOTOR3,
  SELECT_MOTOR4,  
  SELECT_MOTOR_COUNT
}TMC26660_SelectMotorE;
int spiAssertSSL(SpiSlaveSelectIdE id, GPIO_PinState state);   

/* Define accessor functions for each command and readback response */
int tmc2660_mtr_select(MotorAxisIdE id, void *hspi,SpiSlaveSelectIdE spi_ssl);
int tmc2660_register_init(void);
int tmc2660_setup_registers_task(void *hspi,MotorAxisIdE motor_axis_id); 
int tmc2660_stallguard_tune_task(void);
void tmc2660_print_readout_info(void);
BOOL tmc2660_mtr_rngchk_microsteps(uint16_t value);
int tmc2660_spi_comm_task(void);

#ifdef DEPLOY_TRINAMICS_TUNE_JSON_CMDS 
//------------------------------------------
stat_t tmc2660_set_short_to_gnd_dly_time(nvObj_t *nv);
stat_t tmc2660_set_short_to_gnd_detect(nvObj_t *nv);
stat_t tmc2660_set_chpr_blanking_time(nvObj_t *nv);
stat_t tmc2660_set_chpr_rndm_off_time(nvObj_t *nv);
stat_t tmc2660_set_chpr_i_comparator_mode(nvObj_t *nv);
stat_t tmc2660_set_chpr_chm1_sine_offset(nvObj_t *nv);
stat_t tmc2660_set_chpr_off_time(nvObj_t *nv);
stat_t tmc2660_set_chpr_mode(nvObj_t *nv);
stat_t tmc2660_set_chpr_chm0_hyst_decr_intrvl(nvObj_t *nv);
stat_t tmc2660_set_chpr_chm0_hyst_start(nvObj_t *nv);
stat_t tmc2660_set_chpr_chm0_hyst_end(nvObj_t *nv);
stat_t tmc2660_set_chpr_chm1_fast_decay_mode(nvObj_t *nv);
stat_t tmc2660_set_chpr_chm1_fast_decay_time(nvObj_t *nv);
stat_t tmc2660_set_sg_filt_enable(nvObj_t *nv);
stat_t tmc2660_set_sg_thresh(nvObj_t *nv);
stat_t tmc2660_set_cs_standstill_imin(nvObj_t *nv);
stat_t tmc2660_set_cs_coil_incr_dn(nvObj_t *nv);
stat_t tmc2660_set_cs_coil_incr_up(nvObj_t *nv);
stat_t tmc2660_set_lower_thresh(nvObj_t *nv);
stat_t tmc2660_set_upper_thresh(nvObj_t *nv);
stat_t tmc2660_set_slope_hs(nvObj_t *nv);
stat_t tmc2660_set_slope_ls(nvObj_t *nv);
stat_t tmc2660_set_ustep_interpol(nvObj_t *nv);
stat_t tmc2660_set_edges_per_pulse(nvObj_t *nv);
stat_t tmc2660_set_test_mode(nvObj_t *nv);
stat_t tmc2660_set_step_dir_mode(nvObj_t *nv);
stat_t tmc2660_set_vsense_fscale(nvObj_t *nv);
#ifdef USE_TMC2660C_DRIVERS
stat_t tmc2660_set_otsens(nvObj_t *nv);
stat_t tmc2660_set_shrtsens(nvObj_t *nv);
stat_t tmc2660_set_en_pfd(nvObj_t *nv);
stat_t tmc2660_set_en_s2vs(nvObj_t *nv);
#endif
stat_t tmc2660_set_cs_lower_thrsh(nvObj_t *nv);
stat_t tmc2660_set_cs_upper_thrsh(nvObj_t *nv);

stat_t tmc2660_set_sg_iscale(nvObj_t *nv);
stat_t tmc2660_set_microsteps(nvObj_t *nv);
//------------------------------------------
void tn_print_cs_thrsl(nvObj_t *nv);
void tn_print_cs_thrsh(nvObj_t *nv);
void tn_print_cs_coil_i_up(nvObj_t *nv);
void tn_print_cs_coil_i_dn(nvObj_t *nv);
void tn_print_standstill_imin(nvObj_t *nv);
void tn_print_coolstep_val(nvObj_t *nv);
void tn_print_(nvObj_t *nv);
void tn_print_stall_val(nvObj_t *nv);
void tn_print_stall_flg(nvObj_t *nv);
void tn_print_shrt2gnd_ab(nvObj_t *nv);
void tn_print_ol_ab_flgs(nvObj_t *nv);
void tn_print_otp_ot_flgs(nvObj_t *nv);
void tn_print_stndstll_flg(nvObj_t *nv);
void tn_print_ustep_pos(nvObj_t *nv);
void tn_print_stall_val(nvObj_t *nv);
void tn_print_stall_flg(nvObj_t *nv);
void tn_print_shrt2gnd_ab(nvObj_t *nv);
void tn_print_ol_ab_flgs(nvObj_t *nv);
void tn_print_otp_ot_flgs(nvObj_t *nv);
void tn_print_stndstll_flg(nvObj_t *nv);
void tn_print_ustep_pos(nvObj_t *nv);
void tn_print_sg_thrsh(nvObj_t *nv);
void tn_print_filt_en(nvObj_t *nv);
void tn_print_chpr_mode(nvObj_t *nv);
void tn_print_blnkng_time(nvObj_t *nv);
void tn_print_rndm_off_time(nvObj_t *nv);
void tn_print_fast_decay_time(nvObj_t *nv);
void tn_print_chpr_chm1_fast_decay_mode(nvObj_t *nv);
void tn_print_chpr_chm1_sine_offset(nvObj_t *nv);
void tn_print_chpr_chm0_hyst_start(nvObj_t *nv);
void tn_print_chpr_chm0_hyst_end(nvObj_t *nv);
void tn_print_chpr_chm0_hyst_decr_intrvl(nvObj_t *nv);
void tn_print_chpr_off_time(nvObj_t *nv);
void tn_print_slope_ctrl_hs(nvObj_t *nv);
void tn_print_slope_ctrl_ls(nvObj_t *nv);
void tn_print_s2g_detct_tmr(nvObj_t *nv);
void tn_print_s2g_detct(nvObj_t *nv);
void tn_print_test_mode_off(nvObj_t *nv);
void tn_print_sd_interpol(nvObj_t *nv);
void tn_print_edges_per_puls(nvObj_t *nv);
void tn_print_sd_en(nvObj_t *nv);
void tn_print_i_scale(nvObj_t *nv);
void tn_print_vsense_fscale(nvObj_t *nv);
#ifdef USE_TMC2660C_DRIVERS
void tn_print_otsens(nvObj_t *nv);
void tn_print_shrtsens(nvObj_t *nv);
void tn_print_en_pfd(nvObj_t *nv);
void tn_print_en_s2vs(nvObj_t *nv);
#endif
void tn_print_select(nvObj_t *nv);
stat_t tmc2660_set_select(nvObj_t *nv);
extern volatile Tmc2660TuneParamS tn[];
extern volatile TMC26660_SelectMotorE tmc2660_select_motor_id;
#endif

//---------------------------------------------
//extern volatile uint8_t tmc2660_spi_tx_completed_flag;
extern volatile uint8_t tmc2660_spi_rx_completed_flag;
void tmc2660_notify_spi_rx_complete(void);
//#ifdef __cplusplus
//}
//#endif
#endif /* End of File */