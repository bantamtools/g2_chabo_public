/**
  ******************************************************************************
  * @file    tmc2660_mtr_ctrlr.c
  * @author  SME
  * @version V1.0.0
  * @date    27-Feb-2018
  * @brief   Driver for the Trinamics TMC2660 Motor Controller IC.
  ******************************************************************************
  ******************************************************************************
  *
  *  Digital Interface GPIO:
  *    Feature:      Mode:       Function:
  *     -------      -----       --------
  *     ENN         Input        Enable [power to] device, Active low.
  *     Step:        Input        Active Edge {rising|faling|both} per single 
  *                               [Full or micro 2/4:/8/16/32/64/128/256] step.
  *
  *     Dir:         Input        Step Direction: {Forward==LOW, reverse==High}.
  *
  *     SG_TST       Output       Falling Edge indicates stall condition
  *
  *     CSN          Input        SPI Slave Chip Select, Active Low.
  *
  *     TEST_ANA     Input        Analog, supply voltage at GND [NOT USED] 
  *     TST_MODE     Input        Test Mode, Active High  [Tied to GND, Not USED]
  *
  *  Serial Interface: SPI6         
  *   SPI setup: 8 bits, MSB first, Div by 8 for BaudRate: 6.75 MHz                              
  *   
  *   Device SPI protocol:
  *      Data Register size: 20 bits
  *      Communication is sending one 20 bit command word and receiving back one
  *      20 bit response.
  *      For each bit sent to device, 1 bit is sent back simultaneously.

  *                          
  * SPI Commands:    19,18,....                               ...1, 0                    
  * DRVCTRL(SDOFF=1) 00  PHA, CA7-CA0, PHB, CB7-CB0                     
  * DRVCTRL(SDOFF=0) 00  17-10=0, 9=INPOL, 8=DEDGE, 7-4=0, MRES3-0                      
  * CHOPCONF         100 TBL1,0,CHM, HDEC1,0,HEN3-0, HSRT2-0, TOFF3-0                          
  * SMARTEN          101 0, SEIMIN,SEDN1-0, SEMAX3-0, 0, SEUP1,0, 0, SEMIN                      
  * SGCSCONF         110 SFILT,0,SGT6-0, 7-5=0, CS4-0                        
  * DRVCONF  111 TST. SLPH1-0, SLPL1-0,0, DISS2G, TS2G1-0, SDOFF,VSENSE,RDSEL1-0                        
  
  ******************************************************************************
  */
#include "main.h"
#include "bantam_hal.h"
#include "user_types.h"
#include "globdefs.h"
#include "motion_ctrl.h" 
#include "g2core.h" 
#include "g2_gcode.h"//stallguard status streaming used gv.F_word ==feedrate
#include "g2_config.h"
#include "g2_stepper.h"
#include "tmc2660_mtr_ctrlr.h" 
#include "motion_ctrl.h"
#include "g2_util.h"
#include "comms_mgr.h"
#include <string.h>
#include "definitions.h"

//#include "peripheral/pio/plib_pio.h"
#undef SPI_HandleTypeDef
#define SPI_HandleTypeDef void

/* Define register addresses */  
#define ADDR_DRV_CTRL      (uint8_t)0
#define ADDR_SPI_MODE      (uint8_t)0//same as step_dir ctrl mode 
#define ADDR_CHOPPER_CFG   (uint8_t)0x04 
#define ADDR_COOLSTEP_CTRL (uint8_t)5 
#define ADDR_SG_CFG        (uint8_t)6     
#define ADDR_DRV_CFG       (uint8_t)7
#define TMC2660_SPI_TX_RX_BYTES  4 //

#define SHIFT_STEPS 4 //needed for alignment of tmc2660 register 20bit paylod withing 32bit data store

//const uint8_t MOTOR_AXIS_ID[MTR_AXIS_ID_COUNT]={'1','2','3','A','B'};
/* Define bitfield structs for each SPI command and response */
typedef struct
{
  uint32_t unused_12     :12;//unused (0nly lower 20 bits are significant) 
  uint32_t current_b     :8; //range 0..248 magnitude
  uint32_t polarity_b    :1; //current flows from 0B1 pins to 0B2 pins
  uint32_t current_a     :8; //range 0..248 magnitude 
  uint32_t polarity_a    :1; //current flows from 0A1 pins to 0A2 pins
  uint32_t addr          :2;//register address bits  
}Tmc2660_DrvCtrlSpiMode_t;// aka: SDOFF=1 Step/dir Mode Disabled

/* Define a union for SPI mode */
typedef union
{
  uint32_t u32_data;
  uint8_t  u8_data[sizeof(uint32_t)];
  Tmc2660_DrvCtrlSpiMode_t struct_data;
}Tmc2660_DrvCtrlSpiMode_u;

/* DRVCTRL(SDOFF=0) 00  17-10=0, 9=INPOL, 8=DEDGE, 7-4=0, MRES3-0 */ 
typedef struct
{
  uint32_t unused_12     :12;//unused (0nly lower 20 bits are significant)  
  uint32_t ustep_resol   :4; //usteps per step: 
  uint32_t reserved_4    :4;
  uint32_t dbl_edge_pulse:1; // double edged step pulse: 0=disabled, 1=enabled
  uint32_t step_intpol   :1; // step pulse interpolation 0=disabled, 1=enabled
  uint32_t reserved_8    :8;
  uint32_t addr          :2;//register address bits
}Tmc2660_DrvCtrlStepDirMode_t;//aka: SDOFF=0 Stpe Dir Not Disabled

typedef union
{
  uint32_t u32_data;
  uint8_t  u8_data[sizeof(uint32_t)];
  Tmc2660_DrvCtrlStepDirMode_t struct_data;
}Tmc2660_DrvCtrlStepDirMode_u;

/* CHOPCONF         100 TBL1,0,CHM, HDEC1,0,HEN3-0, HSRT2-0, TOFF3-0*/ 
typedef struct
{
  uint32_t unused_12      :12;//unused (0nly lower 20 bits are significant) 
  uint32_t msft_off_time  :4 ;//0==Mosfet disabled;1..15 use with table
  uint32_t hyst_start_val :3 ;//or, fast decay time (chm=1)
  uint32_t hyst_end_val   :4 ;//or, sine wave offset (chm=1)
  uint32_t hyst_decr_intv :2 ;//or, fast decay mode (chm=1)
  uint32_t randm_off_time :1 ;//0=chop off time fixed by set bits, 1= random  
  uint32_t chopper_mode   :1 ;//0=std mode (sprdcycle), 1= const toff, fast decay  
  uint32_t blanking_time  :2 ;//in sysclock_cycles: 0=16, 1=24,2=36, 3=54
  uint32_t addr           :3; //register address bits   
}Tmc2660_ChopConfig_t;
/* define a struct that has the Constant Toff chopper mode 1 bitfields*/
typedef struct
{
  uint32_t unused_12      :12;//unused (0nly lower 20 bits are significant) 
  uint32_t msft_off_time  :4 ;//0==Mosfet disabled;1..15 use with table
  uint32_t fast_decay_time_lsb3:3;//lower 3 bits of four bit, fast decay time setting
  uint32_t sinewave_offset   :4 ;//or, sine wave offset (chm=1)
  uint32_t fast_decay_time_msb1 :1 ;//msb of fast decay time setting
  uint32_t fast_decay_mode :1 ;//cfast decay mode (chm=1)
  uint32_t randm_off_time :1 ;//0=chop off time fixed by set bits, 1= random  
  uint32_t chopper_mode   :1 ;//0=std mode (sprdcycle), 1= const toff, fast decay  
  uint32_t blanking_time  :2 ;//in sysclock_cycles: 0=16, 1=24,2=36, 3=54
  uint32_t addr           :3; //register address bits   
}Tmc2660_ChopConfig_chm1_t;
typedef union
{
  uint32_t u32_data;
  uint8_t  u8_data[sizeof(uint32_t)];
  Tmc2660_ChopConfig_t struct_data;
  Tmc2660_ChopConfig_chm1_t chm1_struct_data;
}Tmc2660_ChopConfig_u;


/* SMARTEN          101 0, SEIMIN,SEDN1-0, SEMAX3-0, 0, SEUP1,0, 0, SEMIN */ 
typedef struct
{
  uint32_t unused_12     :12;//unused (0nly lower 20 bits are significant)
  uint32_t se_min        :4;//minx coolStep threshold  
  uint32_t reserved_4    :1;//
  uint32_t cur_incr_size :2 ;//
  uint32_t reserved_3    :1;//
  uint32_t se_max        :4;//max coolStep threshold
  uint32_t reserved_2    :1;//
  uint32_t cur_decr_speed:2 ;//current decremt speed for stallguard sampling
  uint32_t se_min_cur    :1;//min coolstep current
  uint32_t reserved_1    :1;//
  uint32_t addr          :3; //register address bits 
}Tmc2660_SmartEn_t;//aka: CoolStep Control Reg

typedef union
{
  uint32_t u32_data;
  uint8_t  u8_data[sizeof(uint32_t)];
  Tmc2660_SmartEn_t struct_data;
}Tmc2660_SmartEn_u;

/* SGCSCONF         110 SFILT,0,SGT6-0, 7-5=0, CS4-0 */
typedef struct
{
  uint32_t unused_12     :12;//unused (0nly lower 20 bits are significant)
  uint32_t curr_scale    :5 ;// 0=1/32, ..31/32 
  uint32_t reserved2     :3 ;//
  uint32_t sg_thresh_val :7;//-64 to +63 sensitivity inversely proprtnl to value
  uint32_t reserved      :1 ;//
  uint32_t sg_filt_en    :1 ;//0: std mod, 2: filtered
  uint32_t addr          :3;//register address bits    
}Tmc2660_SGSCConf_t;

typedef union
{
  uint32_t u32_data;
  uint8_t  u8_data[sizeof(uint32_t)];
  Tmc2660_SGSCConf_t struct_data;
}Tmc2660_SGSCConf_u;

/* DRVCONF  111 TST. SLPH1-0, SLPL1-0,0, DISS2G, TS2G1-0, SDOFF,VSENSE,RDSEL1-0*/                      
/* TMC2660C ONLY: b3:0 = OTSENS, SHRTSENS, EN_PFD, ENS2VS */  
typedef struct
{
 //LSB end:
  uint32_t unused_12      :12;//unused (0nly lower 20 bits are significant)
#ifdef USE_TMC2660C_DRIVERS
  uint32_t en_s2vs        :1 ;//0=short to VS, clock failsafe disabled, 1=enabled
  uint32_t en_pfd         :1 ;//0=no addl motor dampening, 1=addl motor dampening
  uint32_t shrt_sens      :1 ;//0=short to GND low sensitivity, 1=high sensitivity
  uint32_t overtemp_sens  :1 ;//0=shutdown at 150C, 1=shutdown at 136C
#else
  uint32_t reserved2      :4 ;//
#endif
  uint32_t readout_select :2 ;// 0=step position, 1=sg level, 2=sg and coolStep 
                              //current level, 3=reserved   
  uint32_t vsense_fscale  :1 ;//0=sense resistor full scale volt 305, 1= 165 
  uint32_t sd_disable     :1 ;// 
  uint32_t short_to_gnd_tim:2 ;//0=3.2uS, 1=1.6uS, 2=1.2uS, 3=0.8uS
  uint32_t short_to_gnd   :1 ;//0=enabled, 1= disabled
  uint32_t reserved       :1 ;//
  uint32_t slope_ctrl_ls  :2 ;//slope control low side
  uint32_t slope_ctrl_hs  :2 ;//slope control high side
  uint32_t test_mode      :1 ;//reserved test mode.0=normal op, 1 = test
  uint32_t addr           :3;//register address bits  
 // MSB end: the unused 32-bit padding bits are ignored
}Tmc2660_DrvConf_t;

typedef union
{
  uint32_t u32_data;
  uint8_t  u8_data[sizeof(uint32_t)];
  Tmc2660_DrvConf_t struct_data;
}Tmc2660_DrvConf_u;
#if 0
/* "The most significant bit is sent first. A minimum
of 20 SCK clock cycles is required for a bus transaction with the TMC2660.
If more than 20 clocks are driven, the additional bits shifted into SDI are shifted out on SDO after a
20-clock delay through an internal shift register."
*/
/* Readback response data struct: */
typedef struct
{
  //MSB end: bits arrive from this end first
   uint32_t data             :10 ;//ustep counter or sg val0|5, or coolstep val
   uint32_t reserved         :2 ;//  
   uint32_t standstill       :1 ;//0=none,1=no active edge on step input for 
   uint32_t open_load        :1;//0=none, 1 no chopper event in last period 
                                //2 exp 20 clock cycles 
   uint32_t short_t_gnd      :2;//0=none,1=short to gnd on high side transiters 
   uint32_t overtemp_wrn     :1;//0=none,1=warning active   
   uint32_t overtemp_shut_dn :1;//0=none,1=overtemperatue shutdown active
   uint32_t stall            :1 ;//0=none,1=sg threshold reached, stall active 
   uint32_t unused_12        :12;//unused (0nly lower 20 bits are significant)  
//LSB end: Bits arrive at this end last
}Tmc2660_ReadResponse_t;
#else
/* Readback response data struct: */
typedef struct
{
  //LSB end: bits at this end arive last
   uint32_t unused_12        :12;//unused (0nly lower 20 bits are significant) 
   uint32_t stall            :1 ;//0=none,1=sg threshold reached, stall active 
   uint32_t overtemp_shut_dn :1;//0=none,1=overtemperatue shutdown active
   uint32_t overtemp_wrn     :1;//0=none,1=warning active 
   uint32_t short_t_gnd      :2;//0=none,1=short to gnd on high side transiters, B, A 
   uint32_t open_load        :2;//0=none, 1 no chopper event in last period B, A
   uint32_t standstill       :1 ;//0=none,1=no active edge on step input for 
                                 //2 exp 20 clock cycles 
   uint32_t reserved         :2 ;//     
   uint32_t data             :10 ;//ustep counter or sg val0|5, or coolstep val 
//MSB end: bits at this end arrive back to the Master first
}Tmc2660_ReadResponse_t;
#endif
typedef union
{
  Tmc2660_ReadResponse_t struct_data;
  uint32_t u32_data;
  uint8_t  u8_data[sizeof(uint32_t)];  
}Tmc2660_ReadResponse_u;

/* Provide a union of 10-bit read data:ustep counter or sg val0|5, or coolstep val */
typedef struct
{
  uint32_t ignore :22;
  uint32_t ustep_polarity:1;
  uint32_t ustep_position:9;  
}Tmc2660MicroStepBitsS;

typedef struct
{
  uint32_t ignore :22;
  uint32_t stall_guard_val:10; 
}Tmc2660StallGuardBitsS;

typedef struct
{
  uint32_t ignore :22;
  uint32_t stall_guard_5to9:5; 
  uint32_t coolstep:5; 
}Tmc2660Sg5To9AndCoolStepBitsS;

typedef struct
{
  uint32_t ignore :22;
  uint32_t data:10; 
}Tmc2660ResponseTenDataBitS;

typedef union 
{
  Tmc2660MicroStepBitsS ustep_bits;
  Tmc2660StallGuardBitsS stall_guard_bits; 
  Tmc2660Sg5To9AndCoolStepBitsS sg_and_coolstep_bits;
  Tmc2660ResponseTenDataBitS tenbits;//provide unspecified ten bits as well
}Tmc2660Readback10BitDataU;

/* */
typedef union 
{  
  uint32_t u32_data;
  uint16_t u16_data[(sizeof(uint32_t)/ sizeof(uint16_t))];
  uint8_t  u8_data[sizeof(uint32_t)]; 
  Tmc2660_ReadResponse_t struct_data;
  Tmc2660MicroStepBitsS ustep_bits;
  Tmc2660StallGuardBitsS stall_guard_bits; 
  Tmc2660Sg5To9AndCoolStepBitsS sg_and_coolstep_bits;
  Tmc2660ResponseTenDataBitS tenbits; 
}Tmc2660ReadbackMasterU;

typedef struct
{
   uint32_t dummy_data :30;
   uint32_t addr       :2;//register address bits
}Tmc2660_ReadDummyData_t;

/* Actually, there is no such thing as dummy data. Whatever is commanded in must be a harmless copy of the present setup of drv_conv!*/
typedef union
{
  uint32_t u32_data;
  uint8_t  u8_data[sizeof(uint32_t)];
  Tmc2660_ReadDummyData_t struct_data;
}Tmc2660_ReadDummyData_t_u;

/* Define a queue entry struct or runtime adjustment of trinamics parameeters */
typedef struct
{
  MotorAxisIdE mtr_axis_id;
  SpiSlaveSelectIdE ssl_id;
  uint32_t u32_data;
}tmc2660RuntimeAdjustInfoS;

/* Define and allocate the runtime adjust queue:  */
#define TMC2660_MAX_RUNTIME_ADJUST_Q_ENTRIES 20 //arbitrary large size 
typedef struct
{
 tmc2660RuntimeAdjustInfoS entry[TMC2660_MAX_RUNTIME_ADJUST_Q_ENTRIES]; 
 int wr_ndx;
 int rd_ndx;
 int unread_count;
} tmc2660RuntimeAdjustQueueS;
tmc2660RuntimeAdjustQueueS  tmc2660_set_queue;

#ifdef DEPLOY_TRINAMICS_TUNE_JSON_CMDS
volatile Tmc2660TuneParamS tn[NUM_MOTORS];//global due to config array usage
/* Allocated a circular history array of TN readback values */
#define PREV_TN_COUNT 10
typedef struct
{
  Tmc2660ReadbackInfoS item[PREV_TN_COUNT][NUM_MOTORS]; 
  int ndx[NUM_MOTORS];
  Boolean_t change_flg[NUM_MOTORS][READ_COUNT];
}Tmc2660RebackHistoryS;
Tmc2660RebackHistoryS rdbk_prev;
#endif
volatile TMC26660_SelectMotorE tmc2660_select_motor_id = SELECT_NONE; 
/* Allocate an array of readback registers */
static Tmc2660_ReadResponse_u tmc2660_read_response[TMC2660_REG_COUNT];

/* Allocate spi command and response buffers--for initialization--just one reg applied to all axes */
static Tmc2660_DrvConf_u            tmc2660_drv_conf;
static Tmc2660_SGSCConf_u           tmc2660_sg_conf;
static Tmc2660_SmartEn_u            tmc2660_coolstep_ctrl;
static Tmc2660_ChopConfig_u         tmc2660_chopper_conf;
static Tmc2660_DrvCtrlStepDirMode_u tmc2660_drv_ctrl_step_dir;
volatile static Tmc2660_DrvCtrlSpiMode_u     tmc2660_spi_mode_cmd;
 
static int8_t sg_thresh_val = SG_CFG_THRESHOLD_START_VAL;

/* 
 * Allocate the above again for each separate motor/axes for runtime adjustment 
*/
volatile static Tmc2660_DrvConf_u            tmc2660_drv_conf_adjust[MTR_AXIS_ID_COUNT];
volatile static Tmc2660_SGSCConf_u           tmc2660_sg_conf_adjust[MTR_AXIS_ID_COUNT];
volatile static Tmc2660_SmartEn_u            tmc2660_coolstep_ctrl_adjust[MTR_AXIS_ID_COUNT];
volatile static Tmc2660_ChopConfig_u         tmc2660_chopper_conf_adjust[MTR_AXIS_ID_COUNT];
volatile static Tmc2660_DrvCtrlStepDirMode_u tmc2660_drv_ctrl_step_dir_adjust[MTR_AXIS_ID_COUNT];
volatile static Tmc2660_DrvCtrlSpiMode_u     tmc2660_spi_mode_cmd_adjust[MTR_AXIS_ID_COUNT];
 
volatile static Tmc2660ReadbackMasterU tmc2660_readback_data_adjust[MTR_AXIS_ID_COUNT];
 
static volatile uint8_t tmc2660_sg_tune_active_flag = false;
//volatile uint8_t tmc2660_spi_tx_completed_flag = false;
volatile uint8_t tmc2660_spi_rx_completed_flag = false;
static volatile uint8_t tmc2660_stall_event_flag = false;
void tmc2660_set_stall_event(void){tmc2660_stall_event_flag=true;}
void tmc2660_clear_stall_event(void){tmc2660_stall_event_flag=false;}
static volatile MotorAxisIdE tmc2660_mtr_axis_id;
static volatile SpiSlaveSelectIdE tmc2660_mtr_axis_ssl_id;
static volatile void *tmc2660_hspi;

/* Allocate lookup table of valid uStep settings */
const uint16_t MICROSTEPS_SETTINGS[USTEPS_COUNT] =
{
  256,
  128,
  64,
  32,
  16,
  8,
  4,
  2,
  1
};

/* Allocate a lookup for slave chip selects and motor ids */
const SpiSlaveSelectIdE TMC2660_SPI_SSL_ID[MTR_AXIS_ID_COUNT]=
{
    SSL_X_MTRDRV,
    SSL_Y_MTRDRV,
    SSL_Z_MTRDRV,
    SSL_Y2_MTRDRV, 
};
#if 0
void tmc2660_select_motor_id(TMC26660_SelectMotorE id)
{
  if (id<SELECT_MOTOR1)
  {
    id=SELECT_MOTOR1;
  }
  else if(id>SELECT_MOTOR4)
  {
    id=SELECT_MOTOR4;
  }
  select_motor_id=id;
}
#endif
void tmc2660_notify_spi_rx_complete(void){tmc2660_spi_rx_completed_flag=true;}
stat_t tmc2660_runtime_refresh_response_data(MotorAxisIdE motor_id, uint32_t response, Tmc2660ReadSelIdE select_id);
 /************************************************************************** 
 * 
 *  Function: get_motor_id_from_group_id
 *
 *  Description: Provide a util for mapping motor axis id from json/text mode
 *   Where group id is motor id {'1', '2', '3'..  
 ************************************************************************** */  
MotorAxisIdE get_motor_id_from_group_id(uint8_t group_id)
{
  MotorAxisIdE result = MTR_AXIS_ID_COUNT;//default: out of range
  switch(group_id)
  { 
      case '1':
        result = MTR_AXIS_X;
        break;
        
      case '2':
        result = MTR_AXIS_Y;
        break;
        
      case '3':
        result = MTR_AXIS_Z;
        break;
        
      case '4':
         result = MTR_AXIS_Y2;  //4th is MTR_AXIS_Y2==MTR_AXIS_A
         break;
      //case 'B':  
      //case 'b':
      //  result = MTR_AXIS_B;
      //  break;
        
      default: 
        break;
  }/* End switch */
      return result;
}/* End Function */

/**************************************************************************
*
* Function: tmc2660_set_queue_unread_entry_count 
*
* Description: Provide an accessor for tasks to determine unread 
*             entries to process: 
*
**************************************************************************/
int tmc2660_set_queue_unread_entry_count(void) 
{
  return  tmc2660_set_queue.unread_count;
}
 /************************************************************************** 
 * 
 *  Function: tmc2660_set_queue_add 
 *
 *  Description:  
 ***************************************************************************/  
int tmc2660_set_queue_add(MotorAxisIdE mtr_axis_id, SpiSlaveSelectIdE ssl_id, uint32_t u32_data)
{
  int result = 0;
  if (motion_ctrl_tmc2660_setup_completed() == true)
  {
     if (tmc2660_set_queue.wr_ndx >= TMC2660_MAX_RUNTIME_ADJUST_Q_ENTRIES)
     {
       tmc2660_set_queue.wr_ndx = TMC2660_MAX_RUNTIME_ADJUST_Q_ENTRIES;
     }
     tmc2660_set_queue.entry[tmc2660_set_queue.wr_ndx].ssl_id=ssl_id;
     tmc2660_set_queue.entry[tmc2660_set_queue.wr_ndx].u32_data=u32_data;
     tmc2660_set_queue.entry[tmc2660_set_queue.wr_ndx].mtr_axis_id=mtr_axis_id; 
     tmc2660_set_queue.unread_count++;
     tmc2660_set_queue.wr_ndx++;
  }
  else 
  {
    __NOP();//__no_operation();;//to do: ad an error code
  }
  return result;
}
 /************************************************************************** 
 * 
 *  Function: tmc2660_set_queue_get 
 *
 *  Description:  
 ***************************************************************************/  
int tmc2660_set_queue_get(MotorAxisIdE *mtr_axis_id,SpiSlaveSelectIdE *ssl_id, uint32_t *u32_data)
{
  int result = 0;
  if (tmc2660_set_queue.rd_ndx >= TMC2660_MAX_RUNTIME_ADJUST_Q_ENTRIES)
  {
      tmc2660_set_queue.rd_ndx = TMC2660_MAX_RUNTIME_ADJUST_Q_ENTRIES;
  }
  
  *ssl_id = tmc2660_set_queue.entry[tmc2660_set_queue.rd_ndx].ssl_id;
  *u32_data = tmc2660_set_queue.entry[tmc2660_set_queue.rd_ndx].u32_data;
  *mtr_axis_id = tmc2660_set_queue.entry[tmc2660_set_queue.wr_ndx].mtr_axis_id;
  
  if(tmc2660_set_queue.unread_count>0)
  {
     tmc2660_set_queue.unread_count--;
  }
  
  tmc2660_set_queue.rd_ndx++;  
  return result;
}
/****************************************************************
 *
 * Function: tmc2660_mtr_rngchk_microsteps
 * 
******************************************************************/
BOOL tmc2660_mtr_rngchk_microsteps(uint16_t value)
{
  BOOL result = false;
  
  if (
      (value==MICROSTEPS_SETTINGS[ USTEPS_256])
  ||  (value==MICROSTEPS_SETTINGS[ USTEPS_128]) 
  ||  (value==MICROSTEPS_SETTINGS[ USTEPS_64]) 
  ||  (value==MICROSTEPS_SETTINGS[ USTEPS_32]) 
  ||  (value==MICROSTEPS_SETTINGS[ USTEPS_16]) 
  ||  (value==MICROSTEPS_SETTINGS[ USTEPS_8]) 
  ||  (value==MICROSTEPS_SETTINGS[ USTEPS_4]) 
  ||  (value==MICROSTEPS_SETTINGS[ USTEPS_2]) 
  ||  (value==MICROSTEPS_SETTINGS[ USTEPS_1])) 
  {
      /* Accept, in range */
    result =true;
  } 
  return result;
}

/****************************************************************
 *
 * Function: tmc2660_mtr_map_usteps_to_tn_setting
 *  Description: Maps user intiuitive usetep setting to trinaics register setting equivalent
 * 
******************************************************************/
int tmc2660_mtr_map_usteps_to_tn_setting(uint16_t value)
{
  int mapping = USTEPS_COUNT;//default: out of range
  
  switch(value)
  {
  case 256: 
    mapping=USTEPS_256; 
    break;
    
  case 128:
    mapping=USTEPS_128; 
    break;
    
  case 64:
    mapping=USTEPS_64; 
    break;
    
  case 32:
    mapping=USTEPS_32; 
    break;
    
  case 16:
    mapping=USTEPS_16; 
    break;
    
  case 8:
    mapping=USTEPS_8; 
    break;
    
  case 4:
    mapping=USTEPS_4; 
    break;
    
  case 2:
    mapping=USTEPS_2; 
    break;
    
  case 1:
    mapping=USTEPS_1; 
    break;
    
  default:
    //to do: error handling. 
    __NOP();//__no_operation();;
    break;
  }
  
  return mapping;
}
 /****************************************************************
 *
 * Function: tmc2660_mtr_select
 * 
 * Description: assigns file scope spi access parameters of selected motor axis
 * Returns: status: errcode if input args are out of range, 
 *                   0 otherwise  
 ***************************************************************/
int tmc2660_mtr_select(MotorAxisIdE _id, SPI_HandleTypeDef *hspi,SpiSlaveSelectIdE spi_ssl)
{
  int status=0;
  volatile MotorAxisIdE id =_id;
  if (id<MTR_AXIS_ID_COUNT)
  { 
    
    tmc2660_hspi = hspi;
    tmc2660_mtr_axis_ssl_id =spi_ssl;
  }
  else
  {
     status=-1;//ERR_TMC2666_MTR_AXIS_ID_OUT_OF_RANGE;
  }
  return status;
}

/* Define accessor functions for each command and readback response */
Tmc2660_ReadResponse_u tmc2660_get_read_response(Tmc2660_command_idE reg_id)
{
  return tmc2660_read_response[reg_id]; 
}

int tmc2660_set_chopper_cfg(uint8_t addr,
         uint8_t blanking_time,
         uint8_t chopper_mode,
         uint8_t randm_off_time,  
         uint8_t hyst_decr_intv,
         uint8_t hyst_end_val,
         uint8_t hyst_start_val,
         uint8_t msft_off_time)                                
{
  int status=0;
  tmc2660_chopper_conf.u32_data=0;
  tmc2660_chopper_conf.struct_data.blanking_time  = blanking_time;
  tmc2660_chopper_conf.struct_data.chopper_mode   = chopper_mode;  
  tmc2660_chopper_conf.struct_data.randm_off_time = randm_off_time;
  tmc2660_chopper_conf.struct_data.hyst_decr_intv = hyst_decr_intv;  
  tmc2660_chopper_conf.struct_data.hyst_start_val = hyst_start_val;
  tmc2660_chopper_conf.struct_data.hyst_end_val   = hyst_end_val;
  tmc2660_chopper_conf.struct_data.msft_off_time  =  msft_off_time;
   //tmc2660_chopper_conf.struct_data.addr        = ADDR_CHOPPER_CFG;//
  tmc2660_chopper_conf.struct_data.addr           = addr;//ADDR_CHOPPER_CFG;
  tmc2660_chopper_conf_adjust[MTR_AXIS_X].u32_data=tmc2660_chopper_conf.u32_data;
  tmc2660_chopper_conf_adjust[MTR_AXIS_Y].u32_data=tmc2660_chopper_conf.u32_data;
  tmc2660_chopper_conf_adjust[MTR_AXIS_Z].u32_data=tmc2660_chopper_conf.u32_data;
  tmc2660_chopper_conf_adjust[MTR_AXIS_Y2].u32_data=tmc2660_chopper_conf.u32_data;
  //4-6-2023 Note: MTR_AXIS_A ==MTR_AXIS_Y2==4th motor when deployed, either 4th axis on DCNC or Y2 on LFP
  return status;
}

int tmc2660_set_driver_cfg(int8_t addr,
      uint8_t test_mode,
      uint8_t slope_ctrl_hs,  
      uint8_t slope_ctrl_ls,
      uint8_t short_to_gnd,
      uint8_t short_to_gnd_time,                           
      uint8_t sd_disable,  
      uint8_t vsense_fscale,
#ifdef USE_TMC2660C_DRIVERS
      uint8_t readout_select,
      uint8_t overtemp_sens,
      uint8_t shrt_sens,
      uint8_t en_pfd,
      uint8_t en_s2vs)
#else
      uint8_t readout_select)
#endif
{
  int status=0;
  tmc2660_drv_conf.u32_data=0;//added 11-5-2019
  tmc2660_drv_conf.struct_data.addr           = addr;//ADDR_DRV_CFG;
  tmc2660_drv_conf.struct_data.test_mode      = test_mode;
  tmc2660_drv_conf.struct_data.slope_ctrl_hs  = slope_ctrl_hs;  
  tmc2660_drv_conf.struct_data.slope_ctrl_ls  = slope_ctrl_ls;
  tmc2660_drv_conf.struct_data.short_to_gnd   = short_to_gnd;
  tmc2660_drv_conf.struct_data.short_to_gnd_tim= short_to_gnd_time;  
  tmc2660_drv_conf.struct_data.sd_disable     = sd_disable;
  tmc2660_drv_conf.struct_data.vsense_fscale  = vsense_fscale;  
  tmc2660_drv_conf.struct_data.readout_select = readout_select;
#ifdef USE_TMC2660C_DRIVERS
  tmc2660_drv_conf.struct_data.overtemp_sens  = overtemp_sens;  
  tmc2660_drv_conf.struct_data.shrt_sens      = shrt_sens;
  tmc2660_drv_conf.struct_data.en_pfd         = en_pfd;  
  tmc2660_drv_conf.struct_data.en_s2vs        = en_s2vs;
#endif
  tmc2660_drv_conf_adjust[MTR_AXIS_X].u32_data=tmc2660_drv_conf.u32_data;
  tmc2660_drv_conf_adjust[MTR_AXIS_Y].u32_data=tmc2660_drv_conf.u32_data;
  tmc2660_drv_conf_adjust[MTR_AXIS_Z].u32_data=tmc2660_drv_conf.u32_data;
  tmc2660_drv_conf_adjust[MTR_AXIS_Y2].u32_data=tmc2660_drv_conf.u32_data;
  return status;
}

int tmc2660_set_stallguard_cfg(int8_t addr,
                               uint8_t sg_filt_en,
                               uint8_t sg_thresh_val,
                               uint8_t curr_scale                              
                              )
{
  int status=0;
  tmc2660_sg_conf.u32_data=0;//added 11-5-2019
  tmc2660_sg_conf.struct_data.addr              = addr;// ADDR_SG_CFG;
  tmc2660_sg_conf.struct_data.sg_filt_en        = sg_filt_en;
  tmc2660_sg_conf.struct_data.sg_thresh_val     = sg_thresh_val;
  tmc2660_sg_conf.struct_data.curr_scale        = curr_scale;
  tmc2660_sg_conf_adjust[MTR_AXIS_X].u32_data=tmc2660_sg_conf.u32_data;
  tmc2660_sg_conf_adjust[MTR_AXIS_Y].u32_data=tmc2660_sg_conf.u32_data;
  tmc2660_sg_conf_adjust[MTR_AXIS_Z].u32_data=tmc2660_sg_conf.u32_data;
  tmc2660_sg_conf_adjust[MTR_AXIS_Y2].u32_data=tmc2660_sg_conf.u32_data;
  return status;
} 

int tmc2660_set_step_dir_cfg(int8_t addr,
              uint8_t step_intpol,
              uint8_t dbl_edge_pulse,
              uint8_t ustep_resol                              
              )
{
  int status=0;
 //tmc2660_drv_ctrl_step_dir.struct_data.unused_12=0xcba;
 //tmc2660_drv_ctrl_step_dir.struct_data.reserved_8=0xA5;
 //tmc2660_drv_ctrl_step_dir.struct_data.reserved_4=0x5;
  tmc2660_drv_ctrl_step_dir.u32_data=0;//added 11-5-2019
  tmc2660_drv_ctrl_step_dir.struct_data.addr             = addr;//ADDR_DRV_CTRL ;
  tmc2660_drv_ctrl_step_dir.struct_data.step_intpol      = step_intpol;
  tmc2660_drv_ctrl_step_dir.struct_data.dbl_edge_pulse   = dbl_edge_pulse;
  tmc2660_drv_ctrl_step_dir.struct_data.ustep_resol      = ustep_resol; 
  tmc2660_drv_ctrl_step_dir_adjust[MTR_AXIS_X].u32_data=tmc2660_drv_ctrl_step_dir.u32_data;
  tmc2660_drv_ctrl_step_dir_adjust[MTR_AXIS_Y].u32_data=tmc2660_drv_ctrl_step_dir.u32_data;
  tmc2660_drv_ctrl_step_dir_adjust[MTR_AXIS_Z].u32_data=tmc2660_drv_ctrl_step_dir.u32_data;  
  tmc2660_drv_ctrl_step_dir_adjust[MTR_AXIS_Y2].u32_data=tmc2660_drv_ctrl_step_dir.u32_data; 
  return status;
}

int tmc2660_set_coolstep_ctrl(int8_t addr,
                        uint8_t se_min_cur,
                        uint8_t cur_decr_speed,
                        uint8_t se_max,
                        uint8_t cur_incr_size,
                        uint8_t se_min)
{
  int status=0;
  tmc2660_coolstep_ctrl.u32_data=0;//added 11-5-2019
  tmc2660_coolstep_ctrl.struct_data.addr           =  addr;//ADDR_COOLSTEP_CTRL; 
  tmc2660_coolstep_ctrl.struct_data.se_min_cur     = se_min_cur;
  tmc2660_coolstep_ctrl.struct_data.cur_decr_speed = cur_decr_speed;  
  tmc2660_coolstep_ctrl.struct_data.se_max         = se_max;
  tmc2660_coolstep_ctrl.struct_data.cur_incr_size  = cur_incr_size;  
  tmc2660_coolstep_ctrl.struct_data.se_min         = se_min; 
  tmc2660_coolstep_ctrl_adjust[MTR_AXIS_X].u32_data=tmc2660_coolstep_ctrl.u32_data;
  tmc2660_coolstep_ctrl_adjust[MTR_AXIS_Y].u32_data=tmc2660_coolstep_ctrl.u32_data;
  tmc2660_coolstep_ctrl_adjust[MTR_AXIS_Z].u32_data=tmc2660_coolstep_ctrl.u32_data;  
  tmc2660_coolstep_ctrl_adjust[MTR_AXIS_Y2].u32_data=tmc2660_coolstep_ctrl.u32_data;
  return status;
}

int tmc2660_set_spi_mode_cmd( int8_t addr,
                              uint8_t polarity_a,
                              uint8_t polarity_b,  
                              uint8_t current_a,
                              uint8_t current_b
                              )
{
  int status=0;
  tmc2660_spi_mode_cmd.u32_data=0;//added 11-5-2019
  tmc2660_spi_mode_cmd.struct_data.addr        = addr;//ADDR_DRV_CTRL;
  tmc2660_spi_mode_cmd.struct_data.polarity_a  = polarity_a;
  tmc2660_spi_mode_cmd.struct_data.polarity_b  = polarity_b;
  tmc2660_spi_mode_cmd.struct_data.current_a   = current_a;
  tmc2660_spi_mode_cmd.struct_data.current_b   = current_b;
  tmc2660_spi_mode_cmd_adjust[MTR_AXIS_X].u32_data=tmc2660_spi_mode_cmd.u32_data;
  tmc2660_spi_mode_cmd_adjust[MTR_AXIS_Y].u32_data=tmc2660_spi_mode_cmd.u32_data;
  tmc2660_spi_mode_cmd_adjust[MTR_AXIS_Z].u32_data=tmc2660_spi_mode_cmd.u32_data;  
  tmc2660_spi_mode_cmd_adjust[MTR_AXIS_Y2].u32_data=tmc2660_spi_mode_cmd.u32_data;
  return status;
}

/* Enumerate readback data options */
typedef enum
{
  READBACK_MICROSTEP_COUNTER,
  READBACK_SG_VALUE,
  READBACK_SG_AND_CS,
  READBACK_SELECT_COUNT
}Tmc2660ReadbackDataSelectE;

/****************************************************************
 *
 * Function:  tmc2660_set_readout_option
 * 
 * Description: updates prior DRVCONF register with change in
 *              designated readout data. All other config data
 *               remains the same.
 * 
 * Rreturns: status: errcode if input args are out of range, 
 *                   0 otherwise  
 ***************************************************************/
int tmc2660_set_readout_option(Tmc2660ReadbackDataSelectE selection)
{
  int status = 0;
  if(selection<READBACK_SELECT_COUNT)
  {
    tmc2660_drv_conf.struct_data.readout_select=selection;
  }
  else
  {
    status = -1;//ERR_TMC2666_READBACK_SELECT_RANGE;
  }
  return status;
}

/* Enumerate stallguarge tuning states */
typedef enum
{
  SG_TUNE_IDLE,
  SG_TUNE_INIT,
  SG_TUNE_POLL_INIT_COMPLETE,
  SG_TUNE_READ_SG,
  SG_TUNE_POLL_READ_SG_COMPLETE,
  SG_TUNE_WRITE_SG,
  SG_TUNE_POLL_WRITE_SG_COMPLETE, 
  SG_TUNE_COMPLETE,
  SG_TUNE_ERROR,
  SG_TUNE_STATE_COUNT
}Tmc2660SGTuneStateE;
Tmc2660SGTuneStateE tmc2660_tune_state =  SG_TUNE_IDLE;

/* Enumerate tune run state machine states */
typedef enum
{
  SPI_TASK_IDLE,
  SPI_WRITE_COMMAND,
  SPI_POLL_WRITE_COMPLETE,
  SPI_READ_COMMAND,
  SPI_POLL_READ_COMPLETE,
  SPI_TASK_STATE_COUNT
}Tmc2660SpiTaskE;

Tmc2660SpiTaskE tmc2660_spi_task=SPI_TASK_IDLE;
Tmc2660SetupRegStateE tmc2660_setup_regs_state=SETUP_STATE_IDLE;
  /* Allocate sample slave data */
 /****************************************************************
 *
 * Function: spiSlaveSelect(
 * 
 * Description: Asserts/Dessert designated SSL line
 *   
 ***************************************************************/
 int spiAssertSSL(SpiSlaveSelectIdE _id, GPIO_PinState state)
 {
   int status = 0;
   volatile SpiSlaveSelectIdE id=_id;
   
   switch(id)
   {

   case SSL_X_MTRDRV: 
     hal_write_pin(X_MTRDRV_SPI_nCS_GPIO_PortId,X_MTRDRV_SPI_CSn_Pin,state);      
     break;
     
   case SSL_Y_MTRDRV:    
     hal_write_pin(Y_MTRDRV_SPI_nCS_GPIO_PortId,Y_MTRDRV_SPI_CSn_Pin,state);   
     break;
     
   case SSL_Z_MTRDRV:    
     hal_write_pin(Z_MTRDRV_SPI_nCS_GPIO_PortId,Z_MTRDRV_SPI_CSn_Pin,state);  
     break; 
#ifdef DEPLOY_SECOND_YAXIS_STEPPER    
   case SSL_Y2_MTRDRV:    
     hal_write_pin(Y2_MTRDRV_SPI_nCS_GPIO_PortId,Y2_MTRDRV_SPI_CSn_Pin,state); 
     break;
#endif
     
#ifdef DEPLOY_ENCODER_SPI4_BUS
   case SSL_X_ENC:    
     hal_write_pin(GPIO_PORTC,GPIO_PIN_1,state);     
     break;
     
   case SSL_Y_ENC:    
     hal_write_pin(GPIO_PORTF,GPIO_PIN_5,state);     
     break;
     
   case SSL_Z_ENC:    
     hal_write_pin(GPIO_PORTI,GPIO_PIN_10,state);     
     break;
     
   case SSL_A_ENC:   //4th axis
     hal_write_pin(GPIO_PORTD,GPIO_PIN_5,state);     
     break;
#endif 
     
   default: 
     break;
   }/* End Switch*/
   return status;
 }/* End Function */

/****************************************************************
 *
 * Function:  tmc2660_register_init
 * 
 * Description: Inits control registers with startup values
 * 
 * Rreturns: status: errcode if input args are out of range, 
 *                   0 otherwise    
 ***************************************************************/
int tmc2660_register_init(void)
{
  int status=0;
  
   /* Init Chopper Config register: */
   status = tmc2660_set_chopper_cfg( 
           ADDR_CHOPPER_CFG,  
           CHOP_CFG_BLANKING_TIME,
           CHOP_CFG_MODE,
           CHOP_CFG_RANDOM_OFF_TIME,  
           CHOP_CFG_HYST_DECR_OR_FAST_DECAY_MODE,
           CHOP_CFG_HYST_END_VAL_OR_SINE_OFFSET,
           CHOP_CFG_HYST_START_VAL_OR_FAST_DECAY,
           CHOP_CFG_OFF_TIME_OR_MOSFET_DISABLE
           );

  /* Init Driver Config register: */
  status = tmc2660_set_driver_cfg(  
            ADDR_DRV_CFG,                      
            DRVR_CFG_TEST_MODE_OFF,
            DRVR_CFG_SLOPE_CONTROL_HS,  
            DRVR_CFG_SLOPE_CONTROL_LS,
            DRVR_CFG_SHORT_TO_GROUND_PROT_DIS,
            DRVR_CFG_SHORT_TO_GND_DETECT_TMR,
            DRVR_CFG_SD_ENABLE,  //enabled stepdir
            DRVR_CFG_VSENSE_FSCALE,
#ifdef USE_TMC2660C_DRIVERS
            DRVR_CFG_READOUT_SELECT,
            DRVR_CFG_OTSENS,
            DRVR_CFG_SHRTSENS,
            DRVR_CFG_EN_PFD,
            DRVR_CFG_EN_S2VS
#else
            DRVR_CFG_READOUT_SELECT//default==SG_READBACK
#endif
            );
  /* Init StallGuard Config register: */ 
  status = tmc2660_set_stallguard_cfg( 
            ADDR_SG_CFG,                          
            SG_CFG_FILTER_ENABLE,
            SG_CFG_THRESHOLD_VAL,
            SG_CFG_ISCALE                                                            
            ); 
   /* Init Step/Dir Config register: */ 
  status = tmc2660_set_step_dir_cfg( 
            ADDR_DRV_CTRL,                        
            SD_CFG_INTERPOL,
            SD_CFG_EDGES_PER_PULSE,
            SD_CFG_USTEP_RESOLUTION   
            ); 
  /* Init "SMARTEN"==CoolStep Config register: */ 
  status = tmc2660_set_coolstep_ctrl( 
            ADDR_COOLSTEP_CTRL,                         
            CS_CTRL_SE_IMIN,
            CS_CTRL_I_DECREM_SPEED,
            CS_CTRL_SE_MAX_THRESH,
            CS_CTRL_I_INCREM_SIZE,
            CS_CTRL_SE_MIN_THRESH 
            );
   /* Init SPI Mode Coil cmd register: (NOT USED)*/ 
  status = tmc2660_set_spi_mode_cmd( 
            ADDR_SPI_MODE,                        
            SPI_MODE_POLARITY_COIL_A ,
            SPI_MODE_POLARITY_COIL_B,  
            SPI_MODE_IFLOW_COIL_A,
            SPI_MODE_IFLOW_COIL_B
            ); 
   tmc2660_setup_regs_state = SETUP_STATE_START;  
   memset((void*)&tmc2660_set_queue,0,sizeof(tmc2660_set_queue));
   memset((void*)&rdbk_prev,0,sizeof(rdbk_prev));
  return status;
}/*End function */

/****************************************************************
 *
 * Function:  tmc2660_print_readout_info
 * 
 * Description:  
 *  
 ***************************************************************/
void tmc2660_print_readout_info(void)
{
 // tmc2660_read_response[TMC2660_REG_DRV_CFG].u32_data>>=4;//* data must be aligned */
#define DEBUG_TEXT_LEN 128
 static volatile char debug_text[DEBUG_TEXT_LEN];
  sprintf((char*)debug_text,"Stall:%d, OTW:%d, OTSD:%d, SHRT2GND: %d, STNSTL:%d, OPENLD:%d, DATA:%d\n",
          tmc2660_read_response[TMC2660_REG_DRV_CFG].struct_data.stall,
          tmc2660_read_response[TMC2660_REG_DRV_CFG].struct_data.overtemp_wrn,
          tmc2660_read_response[TMC2660_REG_DRV_CFG].struct_data.overtemp_shut_dn,
          tmc2660_read_response[TMC2660_REG_DRV_CFG].struct_data.short_t_gnd,
          tmc2660_read_response[TMC2660_REG_DRV_CFG].struct_data.open_load,
          tmc2660_read_response[TMC2660_REG_DRV_CFG].struct_data.standstill,    
          tmc2660_read_response[TMC2660_REG_DRV_CFG].struct_data.data);
  comms_uart_debug_print( (char*)debug_text);
}
 /****************************************************************
 *
 * Function:  tmc2660_setup_registers_task
 * 
 * Description: Writes over the SPI bus, the initial values to all 
 *    TMC2660 device's control/config registers, for a given stepper 
 *    motor input arg: spi_ssl designates which (motor/axis id) is
 *   is receiving the full set of setupt commands.
 *
 *   Note: This is actually a state machine that is invoked
 *   by the motion control task durng the startup initialization
 *   of the trinamics motor control driver chips, one per motor/axis.
 ***************************************************************/
int tmc2660_setup_registers_task(SPI_HandleTypeDef *hspi, MotorAxisIdE motor_axis_id)
{
  static uint8_t debug_print_interval_counter=0;
 
#if 1
  tmc2660_mtr_axis_ssl_id = TMC2660_SPI_SSL_ID[motor_axis_id];
#else
   tmc2660_mtr_axis_ssl_id=SSL_X_MTRDRV;
#endif
  int spi_status = STAT_OK; 

  
  switch(tmc2660_setup_regs_state)
  {
    case SETUP_STATE_IDLE:
         break;
         
    case SETUP_STATE_START:
         /* Shift compile time 20-bit command payload to tmc2660's expected 
            position: */
         tmc2660_sg_conf.u32_data  = tmc2660_sg_conf_adjust[motor_axis_id].u32_data >> SHIFT_STEPS;;           
         tmc2660_drv_conf.u32_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data >> SHIFT_STEPS;    
         tmc2660_coolstep_ctrl.u32_data = tmc2660_coolstep_ctrl_adjust[motor_axis_id].u32_data >> SHIFT_STEPS;                         
         tmc2660_drv_ctrl_step_dir.u32_data = tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].u32_data >> SHIFT_STEPS;
         tmc2660_chopper_conf.u32_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data >> SHIFT_STEPS; 

         /* Start with the Chop config register */
         tmc2660_setup_regs_state=SETUP_CHOPCONF;
         
         /* Clear out the tmc2660 response buffer: */
         memset(&tmc2660_read_response[0],0xFF, sizeof(tmc2660_read_response));
         break; 
         
     case SETUP_CHOPCONF:
         spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_RESET);    
         spi_status =(int)USART2_SPI_WriteRead(&tmc2660_chopper_conf,4/*sizeof(uint32_t)*/,
                 &tmc2660_read_response[TMC2660_REG_CHOPPER_CFG],4/*sizeof(uint32_t)*/);
         tmc2660_setup_regs_state = POLL_CHOPCONF_COMPLETE;         
         break;
        
    case POLL_CHOPCONF_COMPLETE:
         if (tmc2660_spi_rx_completed_flag == true)
         {
            tmc2660_spi_rx_completed_flag=0;
            spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_SET); 
            tmc2660_setup_regs_state = SETUP_SGCSCONF;
         }
         break; 
         
    case SETUP_SGCSCONF:      
         spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_RESET);           
         spi_status =(int)USART2_SPI_WriteRead(&tmc2660_sg_conf,sizeof(uint32_t),
                 &tmc2660_read_response[TMC2660_REG_SG_CFG], sizeof(uint32_t));

         tmc2660_setup_regs_state = POLL_SGCSCONF_COMPLETE;      
        break; 
         
    case POLL_SGCSCONF_COMPLETE:
         if (tmc2660_spi_rx_completed_flag==true)
         {
            spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_SET);  
            tmc2660_spi_rx_completed_flag=0;
            tmc2660_setup_regs_state = SETUP_DRVCONF;
         }              
         break; 
         
    case SETUP_DRVCONF:      
         spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_RESET);           
         spi_status =(int)USART2_SPI_WriteRead(&tmc2660_drv_conf,sizeof(uint32_t),
                 &tmc2660_read_response[TMC2660_REG_DRV_CFG], sizeof(uint32_t));         

         tmc2660_setup_regs_state = POLL_DRVCONF_COMPLETE; 
         break;
         
    case POLL_DRVCONF_COMPLETE:
         if (tmc2660_spi_rx_completed_flag==true)
         {      
           spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_SET); 
           tmc2660_setup_regs_state = SETUP_DRVCTRL_STEP_DIR;          
           tmc2660_spi_rx_completed_flag=0;                     
         } 
        break;  
          
    case SETUP_DRVCTRL_STEP_DIR:     
         spiAssertSSL(  tmc2660_mtr_axis_ssl_id,GPIO_PIN_RESET);
         spi_status =(int)USART2_SPI_WriteRead(&tmc2660_drv_ctrl_step_dir,sizeof(uint32_t),
                 &tmc2660_read_response[TMC2660_REG_DRVCTRL], sizeof(uint32_t));          

         tmc2660_setup_regs_state=POLL_DRVCTRL_STEP_DIR_COMPLETE;
         break;
    
    case POLL_DRVCTRL_STEP_DIR_COMPLETE:
         if (tmc2660_spi_rx_completed_flag==true)
         {
            spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_SET);
            tmc2660_spi_rx_completed_flag=0;
            tmc2660_setup_regs_state = SETUP_SMARTEN;
         }
         break;
         
    case SETUP_SMARTEN:            
         spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_RESET);           
         spi_status =(int)USART2_SPI_WriteRead(&tmc2660_coolstep_ctrl,sizeof(uint32_t),
                 &tmc2660_read_response[TMC2660_REG_COOLSTEP_CTRL], sizeof(uint32_t));          
        
         tmc2660_setup_regs_state = POLL_SMARTEN_COMPLETE;
         break;
          
    case POLL_SMARTEN_COMPLETE:
         if (tmc2660_spi_rx_completed_flag==true)
         {
            tmc2660_spi_rx_completed_flag=0;
            spiAssertSSL(tmc2660_mtr_axis_ssl_id, GPIO_PIN_SET);
            spi_status = TMC2660_SETUP_COMPLETE; 
            tmc2660_setup_regs_state = TMC2660_SETUP_COMPLETE;
         }
         break;
         
    case TMC2660_SETUP_COMPLETE: 
       spi_status = TMC2660_SETUP_COMPLETE;         
       tmc2660_setup_regs_state = TMC2660_START_READ_STATUS;         
       break;
       
  case TMC2660_START_READ_STATUS:      
       spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_RESET);    
       spi_status =(int)USART2_SPI_WriteRead(&tmc2660_drv_conf,sizeof(uint32_t),
                 &tmc2660_read_response[TMC2660_REG_READ_STATUS], sizeof(uint32_t));       
        
       tmc2660_setup_regs_state = TMC2660_POLL_READ_STATUS_COMPLETE;
       spi_status = TMC2660_SETUP_COMPLETE;      
       break;
      
  case TMC2660_POLL_READ_STATUS_COMPLETE: 
     if (tmc2660_spi_rx_completed_flag==true)
     {
        tmc2660_spi_rx_completed_flag=0;
        spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_SET);
        spi_status = TMC2660_SETUP_COMPLETE; 
#define DEBUG_INTERVAL_COUNT 250
        tmc2660_setup_regs_state = TMC2660_START_READ_STATUS;
        if (debug_print_interval_counter++ >DEBUG_INTERVAL_COUNT)
        {
            debug_print_interval_counter=0;
//           tmc2660_print_readout_info();     
        }        
     }   
     break;
    
  default:
        break;
  
  }/* END SWITCH*/
  return spi_status;
}/* END FUNCTION */

/****************************************************************
 *
 * Function:  tmc2660_spi_comm_task
 * 
 * Description: Writes over the SPI bus, the runtime-adjusted values   
 * to a single designated register, for a given stepper 
 *    motor input arg: 
 *
 * Note: This is actually a state machine that is invoked
 * by the motion control task during the startup initialization
 * of the Trinamics motor control driver chips, one per motor/axis.
 ***************************************************************/
int tmc2660_spi_comm_task(void)
{
  int status=0;
#define READ_STATUS_INTERVAL_COUNT 30 //5ms scheduling period x 30 = 150ms polling interval, where one axis is selected
  static int read_status_interval_counter=0;
  static int  read_status_motor_id = MTR_AXIS_X;
  static  int tmc2660_read_select_id = READ_SELECT_USEPS;  
  volatile int spi_status = HAL_OK; 
  static SpiSlaveSelectIdE ssl_id;
  uint32_t u32_cmd_data;
  static uint32_t u32_response_data;
  static  MotorAxisIdE motor_axis_id;
  
  static enum
  {
    TASK_IDLE,
    SEND_SPI_COMMAND,
    POLL_SPI_COMPLETE  
  } adjust_state = TASK_IDLE;
  
  /* For testing, provide a means to poll only a selected axis*/
  if (tmc2660_select_motor_id !=  SELECT_NONE )
  {
     read_status_motor_id=tmc2660_select_motor_id-1;
  }  
  switch(adjust_state)
  {
     case TASK_IDLE:
     case SEND_SPI_COMMAND:
         if (tmc2660_set_queue_unread_entry_count()>0) 
         {
            /* Dequeue one entry */
            tmc2660_set_queue_get( &motor_axis_id, &ssl_id, &u32_cmd_data);
            adjust_state=SEND_SPI_COMMAND;
         }
         /* send the command immediately */
         if ( adjust_state==SEND_SPI_COMMAND)
         {
            spiAssertSSL(ssl_id,GPIO_PIN_RESET);  
     
            spi_status =(int)USART2_SPI_WriteRead(&u32_cmd_data,4/*sizeof(uint32_t)*/,
                 &u32_response_data, 4/*sizeof(uint32_t)*/);
            
            /* Copy response data to respective motor drive's persistent response data store: */
            tmc2660_readback_data_adjust[motor_axis_id].u32_data=u32_response_data;
        
            adjust_state = POLL_SPI_COMPLETE ;  
         }
         else
         {
           /* Periodically read out the status info from the trinamics motors */
          
           if (read_status_interval_counter++ >= READ_STATUS_INTERVAL_COUNT)
           {
               read_status_interval_counter=0;
              
              /* Select a motor to read out status: */
              if (read_status_motor_id >= MTR_AXES_COUNT)
              {
                read_status_motor_id = MTR_AXIS_X;
              }
#if 1// 7-7-2022 debug only def TMC2660_POLL_SG_ONLY  
 tmc2660_read_select_id = READ_SELECT_USEPS;//READ_SELECT_STALL_GUARD_VALUE;             
#else
              /* Select which status to read out: */
              if (tmc2660_read_select_id >= READ_SELECT_COUNT)
               
              {
                tmc2660_read_select_id = READ_SELECT_USEPS;//READ_SELECT_COOLSTEP;//READ_SELECT_STALL_GUARD_VALUE;//;
              }
#endif              
              /* Apply the selections to a read out:*/
              ssl_id = TMC2660_SPI_SSL_ID[read_status_motor_id];
              motor_axis_id=(MotorAxisIdE)read_status_motor_id;
              tmc2660_drv_conf_adjust[motor_axis_id].struct_data.readout_select=tmc2660_read_select_id;
              tmc2660_drv_conf.u32_data =0;
              
              /* Align the 20-bit command bits to the msb right justified: */
              tmc2660_drv_conf.u32_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data; 
              tmc2660_drv_conf.u32_data >>= SHIFT_STEPS;
              
              /* Send the command: */
              spiAssertSSL(ssl_id,GPIO_PIN_RESET);
              
              spi_status =(int)USART2_SPI_WriteRead(&tmc2660_drv_conf,4/*sizeof(uint32_t)*/,
                 &u32_response_data,
                 4/*sizeof(uint32_t)*/);
         
               /* Prepare for next polling interval: */
#ifndef TMC2660_POLL_SG_ONLY              
              tmc2660_read_select_id++;
#endif 
if (tmc2660_select_motor_id== SELECT_NONE )
{
              read_status_motor_id++;
}              
              adjust_state = POLL_SPI_COMPLETE ;  
           }
          
         }
          
         break;
        
     case POLL_SPI_COMPLETE:
         if (tmc2660_spi_rx_completed_flag == true)
         {
            tmc2660_spi_rx_completed_flag=false;

            /* Copy response data to respective motor drive's persistent response data store: */
            u32_response_data=ByteSwap32(u32_response_data);
            tmc2660_readback_data_adjust[motor_axis_id].u32_data=u32_response_data;
#if 1//7-7-2022 debug only def REPORT_TMC2660_OVER_TEMP//TMC2660_ENABLE_STATUS_PRINTING        
            /* Apply data to config array data stores */
            tmc2660_runtime_refresh_response_data(motor_axis_id,u32_response_data,(Tmc2660ReadSelIdE )tmc2660_drv_conf_adjust[motor_axis_id].struct_data.readout_select);           
#endif
            spiAssertSSL(ssl_id,GPIO_PIN_SET); 
            
            adjust_state = TASK_IDLE;
         }
         break;  
  }/* End switch */
  
  return status;
}/* End Function */

 /****************************************************************
 *
 * Function:  tmc2660_stallguard_tune_task
 * 
 * Description:  State machine for tuning stall guard sensitivity
                 parameter.
 * 
 * sets file scope state variables    
 *   
 ***************************************************************/
 int tmc2660_stallguard_tune_task(void)
 {
   int status = 0;
   volatile int spi_status = 0;
   uint16_t sg_readout = 0;
   
   switch(tmc2660_tune_state)
   {
     case SG_TUNE_IDLE:
       if (tmc2660_sg_tune_active_flag == true )
       {
          tmc2660_tune_state=SG_TUNE_INIT;
       }
       break; 
        
     case SG_TUNE_INIT: 
        /* setup the readback option to the SG data */
        tmc2660_set_readout_option(READBACK_SG_VALUE); 
        
        spiAssertSSL(tmc2660_mtr_axis_ssl_id, GPIO_PIN_RESET);
        spi_status =(int)USART2_SPI_WriteRead(&tmc2660_drv_conf,sizeof(uint32_t),
                 &tmc2660_read_response[TMC2660_REG_DRV_CFG], sizeof(uint32_t));
        
        spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_SET);     
        tmc2660_tune_state =  SG_TUNE_POLL_INIT_COMPLETE;
        break; 
        
     case SG_TUNE_POLL_INIT_COMPLETE:
         if (tmc2660_spi_rx_completed_flag==true)
         {
           tmc2660_tune_state =SG_TUNE_READ_SG;
         }
         break;
          
     case SG_TUNE_READ_SG:
         spiAssertSSL(tmc2660_mtr_axis_ssl_id,GPIO_PIN_SET);         
         tmc2660_tune_state = SG_TUNE_POLL_READ_SG_COMPLETE;
         break;
           
     case SG_TUNE_POLL_READ_SG_COMPLETE:
        if (tmc2660_spi_rx_completed_flag==true)
        {  
            sg_readout = tmc2660_read_response[TMC2660_REG_SG_CFG].struct_data.data;
            if (tmc2660_stall_event_flag==true)
            {
                 if(sg_readout > SG_MIN_STALL_VAL)
                 {
                    if (sg_thresh_val <SG_CFG_THRESHOLD_MAXVAL)
                    {
                       sg_thresh_val++;
                       tmc2660_sg_conf.struct_data.sg_thresh_val=sg_thresh_val;
                       tmc2660_tune_state = SG_TUNE_WRITE_SG;
                    }
                    else
                    {
                      /* Error: SG threshold maxed out before desired tuned result */
                      status= -1;//ERR_SG_TUNE_MAXED_OUT_BEFORE_ACHIEVING_TARGET;
                      tmc2660_tune_state = SG_TUNE_ERROR;
                    }
                }
                else
                {
                   /* Tuned result reached */
                   tmc2660_tune_state = SG_TUNE_COMPLETE;
                }
           }
           else/* not stalled yet */
           {
              if (sg_readout <= SG_MIN_STALL_VAL)
              {
                  if (sg_thresh_val >SG_CFG_THRESHOLD_MINVAL)
                  {
                     sg_thresh_val--;
                     tmc2660_sg_conf.struct_data.sg_thresh_val=sg_thresh_val;
                     tmc2660_tune_state = SG_TUNE_WRITE_SG;
                  }
                  else
                  {
                    /* Error: SG threshold zeroed out before desired tuned result */
                    status= -1;//ERR_SG_TUNE_MINNED_OUT_BEFORE_ACHIEVING_TARGET;
                    tmc2660_tune_state = SG_TUNE_ERROR;
                  }                
              }
           }
        }         
        break;
        
     case SG_TUNE_WRITE_SG:         
         spiAssertSSL(tmc2660_mtr_axis_ssl_id, GPIO_PIN_RESET);       
         spi_status =(int)USART2_SPI_WriteRead(&tmc2660_sg_conf,sizeof(uint32_t),
                 &tmc2660_read_response[TMC2660_REG_SG_CFG], sizeof(uint32_t));
        
         spiAssertSSL(tmc2660_mtr_axis_ssl_id, GPIO_PIN_SET);         
         tmc2660_tune_state = SG_TUNE_POLL_WRITE_SG_COMPLETE;
         break;
         
     case SG_TUNE_POLL_WRITE_SG_COMPLETE: 
        if (tmc2660_spi_rx_completed_flag==true)
        {
           tmc2660_tune_state = SG_TUNE_READ_SG;
        }       
        break; 
         
     case SG_TUNE_COMPLETE:
        /* To do: write an error message */
        tmc2660_tune_state=SG_TUNE_IDLE;
        break;
        
     case SG_TUNE_ERROR:
         /*To do: write an error message */
         tmc2660_tune_state=SG_TUNE_IDLE;
         break;
         
     default: 
       status = -1;//ERR_TMC2666_UNDEFINED_STALLGUARD_TUNE_STATE;
        break;
    }//*end switch */
    return status;
 }
 

//------------------
//template function for json and text mode "tn" runtime, using  nvObj_t *nv as single parameter:

 
typedef enum
{
  //Set values:
  SET_CS_UPPER_THRESH,
  SET_CS_LOWER_THRESH,
  SET_CS_COIL_I_INCR_UP,
  SET_CS_COIL_I_DECR_SPEED,  
  SET_CS_STANDSTILL_I_MIN,
  SET_SG_THRESH,
  SET_SG_FILTER_ENABLE,  
  SET_CHOP_MODE,
  SET_CHOP_BLANKING_TIME,
  SET_CHOP_RNDM_OFF_TIME,  
  SET_CHOP_FAST_DECAY_TIME, 
  SET_CHOP_FAST_DECAY_MODE,
  SET_CHOP_SINE_OFFSET,
  SET_CHOP_HYST_START,
  SET_CHOP_HYST_END, 
  SET_CHOP_HYST_DECR_INTVL,    
 
  SET_CHOP_OFF_TIME,
  SET_HI_SIDE_SLOPE,
  SET_LO_SIDE_SLOPE,
  SET_SHRT_TO_GND_DETECT,
  SET_SHRT_TO_GND_DELAY_TIME, 
  SET_TEST_MODE,
  SET_USTEP_INTERPOLATION,
  SET_EDGES_PER_PULSE,
  SET_STEP_DIR_MODE,
  SET_USTEP_RESOLUTION,
  SET_SG_ISCALE,
  SET_VSENSE_FSCALE,
#ifdef USE_TMC2660C_DRIVERS 
  SET_OTSENS,
  SET_SHRTSENS,
  SET_EN_PFD,
  SET_EN_S2VS,
#endif

  //Get values:
  GET_COOLSTEP_VAL,    
  GET_STALL_VAL,  
  GET_A_B_SHORT_TO_GND_FLG,  
  GET_A_B_OPEN_LOAD_FLG,  
  GET_OVERTEMP_WARN_FLG,    
  GET_OVERTEMP_SHUTDN_FLG,  
  GET_STANDSTILL_FLG,  
  GET_USTEP_POSITION, 
  TMC2660_ITEM_ID_COUNT  
}Tmc2660ItemIdE;
typedef struct
{
  int minval;
  int maxval;
}Tmc2660ItemRangeS;

static const Tmc2660ItemRangeS TMC2660_ITEM_RANGE[TMC2660_ITEM_ID_COUNT]=
{
  //set values:
  {CS_CTRL_SE_THRESH_MIN, CS_CTRL_SE_THRESH_MAX},// SET_CS_UPPER_THRESH,
  {CS_CTRL_SE_THRESH_MIN, CS_CTRL_SE_THRESH_MAX },// SET_CS_LOWER_THRESH,
  {CS_CTRL_SEUP_COIL_INCR_MIN,CS_CTRL_SEUP_COIL_INCR_MAX},// SET_CS_COIL_I_INCR_UP,
  {CS_CTRL_SEUP_COIL_INCR_MIN,CS_CTRL_SEUP_COIL_INCR_MAX},// SET_CS_COIL_I_DECR_SPEED,  
  {CS_CTRL_STANDSTILL_AMPS_MIN,CS_CTRL_STANDSTILL_AMPS_MAX },// SET_CS_STANDSTILL_I_MIN,
  {SG_CFG_THRESHOLD_MINVAL, SG_CFG_THRESHOLD_MAXVAL },// SET_SG_THRESH,
  {0 , 1},// SET_SG_FILTER_ENABLE,  
  {CHOP_MODE_STD_SPREAD_CYCLE , CHOP_MODE_CONST_TIME_OFF},// SET_CHOP_MODE,
  {BLANKING_TIME_16_CLOCK_PERIODS, BLANKING_TIME_54_CLOCK_PERIODS},// SET_CHOP_BLANKING_TIME,
  {CHOP_TOFF_FIXED, CHOP_TOFF_RANDOM },// SET_CHOP_RNDM_OFF_TIME,  
  {CHOP_FAST_DECAY_TIME_OFF_FREEWHEELS ,CHOP_FAST_DECAY_MAX_OFF_TIME },// SET_CHOP_FAST_DECAY_TIME, 
  {CHOP_FAST_DECAY_MODE_COMPARATOR_END,CHOP_FAST_DECAY_MODE_TIME_END},// SET_CHOP_FAST_DECAY_MODE, 
  {CHOP_SINE_OFFSET_MIN,CHOP_SINE_OFFSET_MAX},// SET_CHOP_SINE_OFFSET  
  {CHOP_HYST_START_MIN,CHOP_HYST_START_MAX },// SET_CHOP_HYST_START,
  {CHOP_HYST_END_MIN,CHOP_HYST_END_MAX },// SET_CHOP_HYST_END, 
  {CHOP_HYST_DECR_INTRVL_16_CYCLES,CHOP_HYST_DECR_INTRVL_64_CYCLES},//SET_CHOP_HYST_DECR_INTVL,  

  {CHOP_TOFF_FIXED, CHOP_TOFF_RANDOM},// SET_CHOP_OFF_TIME,
  {DRVR_CFG_SLOPE_CONTROL_MIN, DRVR_CFG_SLOPE_CONTROL_MAX},// SET_HI_SIDE_SLOPE,
  {DRVR_CFG_SLOPE_CONTROL_MIN, DRVR_CFG_SLOPE_CONTROL_MAX },// SET_LO_SIDE_SLOPE,
  {0 ,1 },// SET_SHRT_TO_GND_DETECT,
  {SHRT_TO_GND_MIN_DLY,SHRT_TO_GND_MAX_DLY},// SET_SHRT_TO_GND_DELAY_TIME, 
  {0 ,1 },// SET_TEST_MODE,
  {0 ,1 },// SET_USTEP_INTERPOLATION,
  {SD_CFG_RISING_EDGE_ONLY,SD_CFG_RISING_AND_FALLING_EDGES },// SET_EDGES_PER_PULSE,
  {DRVR_CFG_SD_MODE_STEP_DIR, DRVR_CFG_SD_MODE_SPI_INTERFACE },// SET_STEP_DIR_MODE,
  {SD_USER_UNITS_USTEP_MIN, SD_USER_UNITS_USTEP_MAX },//1 WHOLE STEP TO 256 MICROSTEPS SET_USTEP_RESOLUTION
  {SG_CFG_ISCALE_MIN, SG_CFG_ISCALE_MAX},// SET_SG_ISCALE,
  {VAL_310_MV, VAL_165_MV},// SET_VSENSE_FSCALE, 
#ifdef USE_TMC2660C_DRIVERS
  {0, 1},// SET_OTSENS, 
  {0, 1},// SET_SHRTSENS, 
  {0, 1},// SET_EN_PFD, 
  {0, 1},// SET_EN_S2VS, 
#endif
  //Get values:
  { CS_READBACK_MIN, CS_READBACK_MAX}, //GET_COOLSTEP_VAL,    
  {STALL_VAL_READBACK_MIN , STALL_VAL_READBACK_MAX}, //GET_STALL_VAL,  
  {0 , 1}, //GET_A_B_SHORT_TO_GND_FLG,  
  {0 , 1}, //GET_A_B_OPEN_LOAD_FLG,  
  {0 , 1}, //GET_OVERTEMP_WARN_FLG,    
  {0 , 1}, //GET_OVERTEMP_SHUTDN_FLG,  
  {0 , 1}, //GET_STANDSTILL_FLG,  
  {0 , 1}  //GET_USTEP_POSITION, 
};

 static volatile FpScrU fpscr_u;
/***************************************************************************
 * 
 * Function: tmc2660_set_helper
 *
 * Description: called by wrapper functions that identify the distinctive
 * items for update
 * 
 **************************************************************************/ 
int tmc2660_set_helper (nvObj_t *nv, Tmc2660ItemIdE id)
{
  stat_t result = STAT_OK;
  int setting = (int)nv->value;
  int setting2=0;
  MotorAxisIdE motor_axis_id=get_motor_id_from_group_id(nv->group[0]);  
  uint32_t u32_send_data = U32_UN_INITIALIZED;   

  if (id<TMC2660_ITEM_ID_COUNT)
  {
    
     RANGE_CHECK_REJECT(setting, TMC2660_ITEM_RANGE[id].minval, TMC2660_ITEM_RANGE[id].maxval, result)   
     if (result == STAT_OK)
     {    
        /* Range check motor axis id */ 
        if (motor_axis_id < MTR_AXES_COUNT)
        {
            /* Range check/correct and assign to respective persistent data store*/
            switch(id)
            {
             /* Coolstep register update: */           
             case SET_CS_UPPER_THRESH:
               if (tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.se_max != setting)
               { 
                  tn[motor_axis_id].cs_set_se_max_thresh=setting;
                  tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.se_max=setting;
                  u32_send_data = tmc2660_coolstep_ctrl_adjust[motor_axis_id].u32_data;
               }
               break;
               
             case SET_CS_LOWER_THRESH:
               if (tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.se_min != setting)
               {             
                  tn[motor_axis_id].cs_set_se_min_thresh=setting;
                  tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.se_min=setting;
                  u32_send_data = tmc2660_coolstep_ctrl_adjust[motor_axis_id].u32_data;
               }
               break;
               
             case SET_CS_COIL_I_INCR_UP: 
               if (tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.cur_incr_size != setting)
               {             
                  tn[motor_axis_id].cs_set_i_incr_step_size=setting;
                  tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.cur_incr_size=setting;
                  u32_send_data = tmc2660_coolstep_ctrl_adjust[motor_axis_id].u32_data;
               }
               break;
               
             case SET_CS_COIL_I_DECR_SPEED:
               if (tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.cur_decr_speed != setting)
               {             
                  tn[motor_axis_id].cs_set_i_decr_speed=setting;
                  tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.cur_decr_speed=setting;
                  u32_send_data = tmc2660_coolstep_ctrl_adjust[motor_axis_id].u32_data;
               }
               break; 
               
             case SET_CS_STANDSTILL_I_MIN:
               if (tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.se_min_cur != setting)
               {             
                  tn[motor_axis_id].cs_set_i_standstill=setting;
                  tmc2660_coolstep_ctrl_adjust[motor_axis_id].struct_data.se_min_cur=setting;
                  u32_send_data = tmc2660_coolstep_ctrl_adjust[motor_axis_id].u32_data;
               }
               break;
               
            /*StallGuard register update: */             
             case SET_SG_THRESH:
               if (tmc2660_sg_conf_adjust[motor_axis_id].struct_data.sg_thresh_val != setting)
               {            
                  tn[motor_axis_id].sg_set_stall_threshold=setting;
                  tmc2660_sg_conf_adjust[motor_axis_id].struct_data.sg_thresh_val=setting;
                  u32_send_data = tmc2660_sg_conf_adjust[motor_axis_id].u32_data;
               }
               break;
               
             case SET_SG_ISCALE: 
               if (tmc2660_sg_conf_adjust[motor_axis_id].struct_data.curr_scale != setting)
               {             
                  tn[motor_axis_id].sg_set_i_scale=setting;
                  tmc2660_sg_conf_adjust[motor_axis_id].struct_data.curr_scale=setting;
                  u32_send_data = tmc2660_sg_conf_adjust[motor_axis_id].u32_data;
               }
               break; 
               
             case SET_SG_FILTER_ENABLE:
               if (tmc2660_sg_conf_adjust[motor_axis_id].struct_data.sg_filt_en != setting)
               {            
                  tn[motor_axis_id].sg_set_stall_filter_enable=setting;
                  tmc2660_sg_conf_adjust[motor_axis_id].struct_data.sg_filt_en=setting;
                  u32_send_data = tmc2660_sg_conf_adjust[motor_axis_id].u32_data;
               }
               break;
               
   /* Chopper register update: */             
             case SET_CHOP_MODE:
               if (tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.chopper_mode != setting)
               {            
                  tn[motor_axis_id].chpr_set_mode=setting;
                  tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.chopper_mode = setting;
                  u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
               }
               break;
               
             case SET_CHOP_BLANKING_TIME:
               if (tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.blanking_time != setting)
               {           
                  tn[motor_axis_id].chpr_set_blanking_time=setting;
                  tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.blanking_time = setting;
                  u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
               }
               break;
               
             case SET_CHOP_RNDM_OFF_TIME:
               if (tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.randm_off_time != setting)
               {            
                  tn[motor_axis_id].chpr_set_rndm_off_time=setting;
                  tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.randm_off_time = setting;
                  u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
               }
               break;
               
             /* Chopper Mode: 0 == SpreadCycle: */ 
            case SET_CHOP_HYST_DECR_INTVL:
               if (tn[motor_axis_id].chpr_set_mode==CHOP_MODE_STD_SPREAD_CYCLE)
               {           
                  if (tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_decr_intv != setting)
                  {           
                     tn[motor_axis_id].chpr_set_chm0_hyst_decr_intrvl=setting;
                     tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_decr_intv= setting;
                     u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
                  }
               }
               else
               {
                 //To do: provide an error message for not being in th correct chopper mode for this command
                 __NOP();//__no_operation();;
               }            
               break;
              /* Chopper Mode: 0 == SpreadCycle: */  
             case SET_CHOP_HYST_START:
               if (tn[motor_axis_id].chpr_set_mode==CHOP_MODE_STD_SPREAD_CYCLE)
               {
                  if (tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_start_val != setting)
                  {            
                     tn[motor_axis_id].chpr_set_chm0_hyst_start=setting;
                     tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_start_val=setting;
                     u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
                  }
               }
               else
               {
                 //To do: provide an error message for not being in th correct chopper mode for this command
                 __NOP();//__no_operation();;
               }            
               break;
               
              /* Chopper Mode: 0 == SpreadCycle: */  
             case SET_CHOP_HYST_END:
               if (tn[motor_axis_id].chpr_set_mode==CHOP_MODE_STD_SPREAD_CYCLE)
               {            
                  if (tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_end_val != setting)
                  {            
                     tn[motor_axis_id].chpr_set_chm0_hyst_end=setting;
                     tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_end_val = setting; 
                     u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
                  }
               }
               else
               {
                 //To do: provide an error message for not being in th correct chopper mode for this command
                 __NOP();//__no_operation();;
               }            
               break;
               
              /* Chopper Mode: 1== Constant Time Off: */     
              case SET_CHOP_FAST_DECAY_TIME:
               if (tn[motor_axis_id].chpr_set_mode==CHOP_MODE_CONST_TIME_OFF)
               {
                 /* Bitfiled-intense, yet a consistent bitfield approach I think, provides the most readable presentation */
                 U8DataU new_u;
                 U8DataU old_u;
                 new_u.u8_byte = setting;
                 old_u.u8_byte = 0;
                 old_u.fast_decay_data.lsb3=tmc2660_chopper_conf_adjust[motor_axis_id].chm1_struct_data.fast_decay_time_lsb3;
                 old_u.fast_decay_data.msb1=tmc2660_chopper_conf_adjust[motor_axis_id].chm1_struct_data.fast_decay_time_msb1;
                 
                  if ( new_u.u8_byte != old_u.u8_byte)
                  {            
                     tn[motor_axis_id].chpr_set_chm1_fast_decay_time=setting;
                     tmc2660_chopper_conf_adjust[motor_axis_id].chm1_struct_data.fast_decay_time_lsb3=new_u.fast_decay_data.lsb3;
                     tmc2660_chopper_conf_adjust[motor_axis_id].chm1_struct_data.fast_decay_time_msb1=new_u.fast_decay_data.msb1;                  
                     u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
                  }
               }
               else
               {
                 //To do: provide an error message for not being in th correct chopper mode for this command
                 __NOP();//__no_operation();;
               }
               break; 
               
            /* Chopper Mode: 1== Constant Time Off: */   
            case SET_CHOP_FAST_DECAY_MODE:
               if (tn[motor_axis_id].chpr_set_mode==CHOP_MODE_CONST_TIME_OFF)
               {
                  if (tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_decr_intv != setting)
                  {           
                     tn[motor_axis_id].chpr_set_chm1_fast_decay_mode=setting;
                     tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_decr_intv=setting;
                     u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
                  }
               }
               else
               {
                 //To do: provide an error message for not being in th correct chopper mode for this command
                 __NOP();//__no_operation();;
               }            
               break;
               
            /* Chopper Mode: 1== Constant Time Off: */      
            case SET_CHOP_SINE_OFFSET:
               if (tn[motor_axis_id].chpr_set_mode==CHOP_MODE_CONST_TIME_OFF)
               {
                  if (tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_end_val != setting)
                  {           
                     tn[motor_axis_id].chpr_set_chm1_sine_offset=setting;
                     tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.hyst_end_val=setting;
                     u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
                  }
               }
               else
               {
                 //To do: provide an error message for not being in th correct chopper mode for this command
                 __NOP();//__no_operation();;
               }            
               break;
               
             case SET_CHOP_OFF_TIME:
               if (tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.msft_off_time != setting)
               {            
                  tn[motor_axis_id].chpr_set_off_time=setting;
                  tmc2660_chopper_conf_adjust[motor_axis_id].struct_data.msft_off_time = setting;
                  u32_send_data = tmc2660_chopper_conf_adjust[motor_axis_id].u32_data;
               }
               break; 
               
   /* DRVCONF register updates: */           
             case SET_HI_SIDE_SLOPE:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.slope_ctrl_hs != setting)
               {            
                  tn[motor_axis_id].pwr_set_slope_hs=setting;            
                  tmc2660_drv_conf_adjust[motor_axis_id].struct_data.slope_ctrl_hs = setting;
                  u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data;              
               }
               break;
             
             case SET_LO_SIDE_SLOPE:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.slope_ctrl_ls != setting)
               {            
                   tn[motor_axis_id].pwr_set_slope_ls=setting;
                   tmc2660_drv_conf_adjust[motor_axis_id].struct_data.slope_ctrl_ls = setting;
                   u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data;
               } 
               break;
               
             case SET_SHRT_TO_GND_DETECT:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.short_to_gnd != setting)
               {            
                   tn[motor_axis_id].prtct_set_shrt2gnd=setting;
                   tmc2660_drv_conf_adjust[motor_axis_id].struct_data.short_to_gnd = setting;
                   u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data;
               } 
               break;
               
             case SET_SHRT_TO_GND_DELAY_TIME:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.short_to_gnd_tim != setting)
               {            
                   tn[motor_axis_id].prtct_set_shrt2gnd_dly=setting;
                   tmc2660_drv_conf_adjust[motor_axis_id].struct_data.short_to_gnd_tim = setting;
                   u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data;
               }
               break;
                 
             case SET_TEST_MODE:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.test_mode != setting)
               {            
                  tn[motor_axis_id].diag_set_test_mode=setting;
                  tmc2660_drv_conf_adjust[motor_axis_id].struct_data.test_mode= setting;
                  u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data; 
               }
               break;
               
              case SET_VSENSE_FSCALE://1-6-2021: copy paste error had the assinment going to the wrong struct. Now corrected
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.vsense_fscale != setting)
               {            
                  tn[motor_axis_id].drv_set_vsense=setting;
                  tmc2660_drv_conf_adjust[motor_axis_id].struct_data.vsense_fscale= setting;
                  u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data;
               }
               break; 
#ifdef USE_TMC2660C_DRIVERS
              case SET_OTSENS:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.overtemp_sens != setting)
               {            
                  tn[motor_axis_id].drv_set_otsens=setting;
                  tmc2660_drv_conf_adjust[motor_axis_id].struct_data.overtemp_sens= setting;
                  u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data;
               }
               break; 
              
              case SET_SHRTSENS:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.shrt_sens != setting)
               {            
                  tn[motor_axis_id].drv_set_shrtsens=setting;
                  tmc2660_drv_conf_adjust[motor_axis_id].struct_data.shrt_sens= setting;
                  u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data;
               }
               break; 

              case SET_EN_PFD:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.en_pfd != setting)
               {            
                  tn[motor_axis_id].drv_set_en_pfd=setting;
                  tmc2660_drv_conf_adjust[motor_axis_id].struct_data.en_pfd= setting;
                  u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data;
               }
               break; 

              case SET_EN_S2VS:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.en_s2vs != setting)
               {            
                  tn[motor_axis_id].drv_set_en_s2vs=setting;
                  tmc2660_drv_conf_adjust[motor_axis_id].struct_data.en_s2vs= setting;
                  u32_send_data = tmc2660_drv_conf_adjust[motor_axis_id].u32_data;
               }
               break; 
#endif
   /* DRVCTRL register updates: */   
              case SET_USTEP_RESOLUTION:
                fpscr_u._u32 =(uint32_t)__get_FPSCR();
                if (fpscr_u._struct.neg_flg==1)
                {
                  fpscr_u._struct.inval_op_c_flg=1;
                __set_FPSCR(fpscr_u._u32);
                }
               setting2=tmc2660_mtr_map_usteps_to_tn_setting(setting);
               if (tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].struct_data.ustep_resol != setting2)
               {                    
                  RANGE_CHECK_REJECT(setting2, USTEPS_256,USTEPS_1, result);  
                  if (result == STAT_OK)
                  {
                      tn[motor_axis_id].drv_set_mstep_resol=setting;
                      st_set_mi(nv);
                      tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].struct_data.ustep_resol= setting2;
                      u32_send_data = (uint32_t)tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].u32_data; 
                  }
                  else
                  {
                    __NOP();//__no_operation();;//to do: err handle
                  }
               }
               break;
               
             case SET_USTEP_INTERPOLATION:
               if (tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].struct_data.step_intpol != setting)
               {  
                 
                  tn[motor_axis_id].drv_set_step_interpl=setting;
                  tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].struct_data.step_intpol= setting;
                  u32_send_data = tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].u32_data; 
               }
               break;
        
             case SET_EDGES_PER_PULSE:
               if (tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].struct_data.dbl_edge_pulse != setting)
               {            
                  tn[motor_axis_id].drv_set_dbl_edge_step=setting;
                  tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].struct_data.dbl_edge_pulse = setting;
                  u32_send_data = tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].u32_data;
               }
               break;
               
             case SET_STEP_DIR_MODE:
               if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.sd_disable != setting)
               {           
                  tn[motor_axis_id].drv_set_stpdir_off=setting;
                  tmc2660_drv_conf_adjust[motor_axis_id].struct_data.sd_disable = setting;
                   u32_send_data = tmc2660_drv_ctrl_step_dir_adjust[motor_axis_id].u32_data; 
               }
               break;
                                      
             default: 
               //to do: undefined case--data corruption!
               result=STAT_INTERNAL_RANGE_ERROR;
               break;
               
          }/* End switch */
          
          /* Result can be error if a range check failed in the switch statement section */
          if (result == STAT_OK)        
          {   
              /* The u32_send_data buffer acts a flag when there is no new data to sen out */
              if (u32_send_data != U32_UN_INITIALIZED)
              {
                u32_send_data >>= SHIFT_STEPS;
                /* Copy command to updater task's queue: */
                tmc2660_set_queue_add(motor_axis_id,TMC2660_SPI_SSL_ID[motor_axis_id], u32_send_data);
              }
          }      
          else
          {
             __NOP();//__no_operation();;//to do: error handling
          }
  
        } /* End If */   
        else//Out of range axis==group id
        {
            result = STAT_INPUT_VALUE_RANGE_ERROR;/*sme:STAT_INPUT_VALUE_UNSUPPORTED*/;
        }//
     }//End If RANGE_CHECK_REJECT check status was OK
     else//Rejected out of range user input  
     {
       // error status already already set for return status for out of range parameter value
       //For reporting purposes, re-assign the out of range value to the existing, range-safe value of the config item
       //This will show to the user that present value was not changed, and the out of range value was rejected
       get_ui8(nv);
     }
     
   } /* End If */   
   else//Out of range tmc2660 item id
   {
     result = STAT_INTERNAL_RANGE_ERROR;
   }
  
  return result;
  
}/* End Function */
//----------------------
/***************************************************************************
 * 
 * Functions: stepper_motor_runtime_adjust 
 * 
 * Description: Provides interface to individual stepper a motor adjust:      
 *  a. Microsteps per WholeStep: None=8, half=7, 4=5, 8=5, 16=4, 32=3, 64=2, 128=1, 256=0  
 *  b. Stallguard sensitivity -64(most sensitive)to +63 (least sensitive),
 *       Enforced limits: -10..+64
 *  c. Idle current level: 0 = 1/2 amp, 1 = 1/4 amp
 *       Default: 0=1/2 amp
 *  d. Vsense 0 = 305 mv, 1 = 165mv, default = 305 mv
 *  e. Current Scaling 0 to 32, 15= default
 * Assumptions/Requirements: 
 ****************************************************************************/ 

/***************************************************************************
 * 
 * Function: tmc2660_set_microsteps 
 * Note: this replicates tinyG, or is redundant wrt to tinyG: st_set_mi
 * axis id is encoded in the motor group id, "1","2","3" == x, y, z
 * so calling function is assumed to have already checked the 
 * group id to select the axis
 ****************************************************************************/ 
stat_t tmc2660_set_microsteps(nvObj_t *nv)
{
   stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {      
        set_ui16(nv);      
    }
    else
    {
        result= tmc2660_set_helper(nv, SET_USTEP_RESOLUTION);
    }
   return result;
}/* End Function*/


/***************************************************************************
 * 
 * Function: tmc2660_set_sg_iscale 
 * 
 ****************************************************************************/ 
stat_t tmc2660_set_sg_iscale( nvObj_t *nv)
{
   stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result=  tmc2660_set_helper(nv,SET_SG_ISCALE );        
    }
    return result;
  
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_mode
 * two modes:  
 *    0 = standard "spread cycle"
 *    1 = Constant T-off with fast decay time
 ****************************************************************************/ 
stat_t tmc2660_set_chpr_mode(nvObj_t *nv)
{
   stat_t result = STAT_OK;
#if 1    
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result = tmc2660_set_helper(nv,SET_CHOP_MODE );
    }
#else
  switch (value)
  {
   case CHOP_MODE_STD_SPREAD_CYCLE: 
     break;
   case  CHOP_MODE_CONST_TIME_OFF: 
     break;
   default:
  //data range error!
     result = ERR_TMC2666_UNDEFINED_CHOPPER_MODE;
  break;
  } /* end switch */
  /* End switch */
#endif
  return result;
  
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_cs_upper_thrsh 
 * 
 ****************************************************************************/ 
stat_t tmc2660_set_cs_upper_thrsh(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result = tmc2660_set_helper(nv,SET_CS_UPPER_THRESH );
    }
  return result;
  
}/* End Function */
/***************************************************************************
 * 
 * Function: tmc2660_set_cs_lower_thrsh 
 * 
 ****************************************************************************/ 
stat_t tmc2660_set_cs_lower_thrsh(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result = tmc2660_set_helper(nv, SET_CS_LOWER_THRESH);
    }
      
  return result;
  
}/* End Function */
/***************************************************************************
 * 
 * Function: tmc2660_set_vsense_fscale
 * 
 ****************************************************************************/ 
stat_t tmc2660_set_vsense_fscale(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_VSENSE_FSCALE );
    }
  return result;
  
}/* End Function */

#ifdef USE_TMC2660C_DRIVERS
/***************************************************************************
 * 
 * Function: tmc2660_set_otsens
 * 
 ****************************************************************************/ 
stat_t tmc2660_set_otsens(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_OTSENS );
    }
  return result;
  
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_shrtsens
 * 
 ****************************************************************************/ 
stat_t tmc2660_set_shrtsens(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_SHRTSENS );
    }
  return result;
  
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_en_pfd
 * 
 ****************************************************************************/ 
stat_t tmc2660_set_en_pfd(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_EN_PFD );
    }
  return result;
  
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_en_s2vs
 * 
 ****************************************************************************/ 
stat_t tmc2660_set_en_s2vs(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_EN_S2VS );
    }
  return result;
  
}/* End Function */
#endif
                                                
/***************************************************************************
 * 
 * Function: tmc2660_set_step_dir_mode 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_step_dir_mode(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv, SET_STEP_DIR_MODE );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_test_mode 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_test_mode(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv, SET_TEST_MODE);
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_edges_per_pulse 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_edges_per_pulse(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_EDGES_PER_PULSE );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_ustep_interpol 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_ustep_interpol(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_USTEP_INTERPOLATION );
    }
  return result; 
}/* End Function */
/***************************************************************************
 * 
 * Function: tmc2660_set_ustep_interpol 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_ustep_resolution(nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_USTEP_RESOLUTION );
    }
      
  return result; 
}/* End Function */
/***************************************************************************
 * 
 * Function: tmc2660_set_slope_ls 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_slope_ls( nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_LO_SIDE_SLOPE );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_slope_hs 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_slope_hs( nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_HI_SIDE_SLOPE );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_upper_thresh
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_upper_thresh( nvObj_t *nv)
{
    stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
      result =tmc2660_set_helper(nv,SET_CS_UPPER_THRESH );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_lower_thresh
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_lower_thresh( nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CS_LOWER_THRESH );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_cs_coil_incr_up
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_cs_coil_incr_up( nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CS_COIL_I_INCR_UP );
    }
  return result; 
}/* End Function */
/***************************************************************************
 * 
 * Function: tmc2660_set_cs_coil_incr_dn
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_cs_coil_incr_dn(nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CS_COIL_I_DECR_SPEED );
    }
  return result; 
}/* End Function */
/***************************************************************************
 * 
 * Function: tmc2660_set_cs_standstill_imin 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_cs_standstill_imin( nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CS_STANDSTILL_I_MIN );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_sg_thresh 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_sg_thresh( nvObj_t *nv)
{
  stat_t result = STAT_OK;
   if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_SG_THRESH );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_ 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_sg_filt_enable( nvObj_t *nv)
{
  stat_t result = STAT_OK;
    if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv, SET_SG_FILTER_ENABLE);
    }
  return result; 
}/* End Function */
/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_chm1_fast_decay_time 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_chpr_chm1_fast_decay_time( nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CHOP_FAST_DECAY_TIME );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_chm1_fast_decay_mode 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_chpr_chm1_fast_decay_mode( nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv, SET_CHOP_FAST_DECAY_MODE);
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_chm0_hyst_decr_intrvl
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_chpr_chm0_hyst_decr_intrvl( nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CHOP_HYST_DECR_INTVL );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_chm0_hyst_start
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_chpr_chm0_hyst_start(nvObj_t *nv) 
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CHOP_HYST_START );
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_chm0_hyst_end
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_chpr_chm0_hyst_end(nvObj_t *nv) 
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CHOP_HYST_END);
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_blanking_time
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_chpr_blanking_time( nvObj_t *nv)
{
  stat_t result = STAT_OK;
 if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CHOP_BLANKING_TIME);
    }
  return result; 
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_rndm_off_time
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_chpr_rndm_off_time ( nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CHOP_RNDM_OFF_TIME);
    }
  return result;  
}/* End Function */


/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_chm1_sine_offset
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_chpr_chm1_sine_offset ( nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CHOP_SINE_OFFSET);  
    }
  return result;  
}/* End Function */
 
/***************************************************************************
 * 
 * Function: tmc2660_set_chpr_off_time
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_chpr_off_time ( nvObj_t *nv)
{
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_CHOP_OFF_TIME); 
    }
  return result; 
}/* End Function */
 
/***************************************************************************
 * 
 * Function: tmc2660_set_short_to_gnd_detect 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_short_to_gnd_detect(nvObj_t *nv)
{
#if 1
  stat_t result = STAT_OK;
  if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_SHRT_TO_GND_DETECT); 
    }
#else
  STAT_OK;
  int setting = (int)nv->value;
  MotorAxisIdE motor_axis_id=get_motor_id_from_group_id(nv->group[0]);
  
  /* Range check motor axis id*/ 
  if (motor_axis_id < MTR_AXES_COUNT)
  {
     /* Range check, clamp if needed, binary-range data*/    
     if (setting < 0)
     { 
       nv->value=0;  
     }
     else if (setting >1)
     {
       nv->value = 1;
     }
     setting=(int)nv->value;
     
     /* Assign to persistent data store */
     tmc2660_drv_conf_adjust[motor_axis_id].struct_data.short_to_gnd=setting;
        
     /* Copy to updater task's queue: */
     tmc2660_set_queue_add(motor_axis_id,
               TMC2660_SPI_SSL_ID[motor_axis_id],
                tmc2660_drv_conf_adjust[motor_axis_id].u32_data);
 }  
 else//out of range axis==group id
 {
    __NOP();//__no_operation();;
 }
#endif   
  return result;
  
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_set_short_to_gnd_delay 
 *
 * Description: motor
 * 
 **************************************************************************/ 
stat_t tmc2660_set_short_to_gnd_dly_time(nvObj_t *nv)
{
  stat_t result = STAT_OK;
  
#if 1
   if (config_setting_defaults() == true)
    {
      set_ui8(nv);
    }
    else
    {
        result=tmc2660_set_helper(nv,SET_SHRT_TO_GND_DELAY_TIME); 
    }
#else
  MotorAxisIdE motor_axis_id=get_motor_id_from_group_id(nv->group[0]);
  int setting = (int)nv->value;
  
  if (motor_axis_id < MTR_AXES_COUNT)
  {
     /* Range check, clamp if needed*/    
     if (setting < SHRT_TO_GND_MIN_DLY)
     { 
       nv->value=SHRT_TO_GND_MIN_DLY;  
     }
     else if (setting >SHRT_TO_GND_MAX_DLY)
     {
       nv->value = SHRT_TO_GND_MAX_DLY;
     }
     
     /* Apply the value */
     setting = (int)nv->value;
     
     /* If not setting defaults at startup,
      *  And, if new value is different than the present value 
      *  Then queue up the new value for runtime adjustment.
      */ 
     if (tmc2660_drv_conf_adjust[motor_axis_id].struct_data.short_to_gnd_tim!= setting)
     {
       /* Assign to persistent data store*/
        tmc2660_drv_conf_adjust[motor_axis_id].struct_data.short_to_gnd_tim=setting;
        
        /* Copy to updater task's queue: */
        tmc2660_set_queue_add(motor_axis_id,
                                         TMC2660_SPI_SSL_ID[motor_axis_id],
                                         tmc2660_drv_conf_adjust[motor_axis_id].u32_data);
     }
  }
  else
  {
    result =-1;//to do: err code, err message
  }
#endif
  return result;
  
}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_runtime_examine_response_data
 *
 * Description:  Compares latest readout with previous, and flags 
 * any differences
 * 
 **************************************************************************/ 
int tmc2660_runtime_examine_response_data(MotorAxisIdE motor_id)
{  
#define CHANGE_MESSAGE_TEXT_LEN 250
  static int SEND_UART_MS_UPDATE_INTERVAL_COUNT=5;//25;
  static int send_msg_update_interval_counter=0;
  static char change_message_text[CHANGE_MESSAGE_TEXT_LEN];
  const char *AXIS_NAME[MTR_AXES_COUNT]={"X0","Y1","Z0","Y2"};
  volatile static Tmc2660ReadbackMasterU master_data_u;
  static uint32_t mtr_drv_flg_event_count[MTR_AXES_COUNT]={0,0,0,0};
  int ndx = rdbk_prev.ndx[motor_id];
  float feedrate_fval = get_feedrate();
  int feedrate_ival=int(feedrate_fval);
#if 0//problems: uneven distribution of axes when set <5 
  if (feedrate_ival>=2000)
  {
    SEND_UART_MS_UPDATE_INTERVAL_COUNT=0;
  }
#endif
  if (cycle_homing_active() ==true)
  {
     feedrate_fval=cycle_get_homing_velocity();
     feedrate_ival=int(feedrate_fval);
  }
#if 0//tbd
  else (cm_probe_cycle_active()==true)
  {
  }
#endif
  else if (feedrate_ival ==0)
  {
   feedrate_ival= VELOCITY_MAX;
  }
 /* Flag any differences between the latest reading and the previous: */
  
#define TMC2660_PRINT_STALL_AND_COOLSTEP_AND_OT_WARN_AND_SHUTDOWN

  rdbk_prev.ndx[motor_id]++;
  if (rdbk_prev.ndx[motor_id]>=PREV_TN_COUNT)
  {
    rdbk_prev.ndx[motor_id]=0;
  }
  ndx=rdbk_prev.ndx[motor_id];
  
  rdbk_prev.item[ndx][motor_id].drv_get_mstep_position  =    tn[motor_id].drv_get_mstep_position; 
  rdbk_prev.item[ndx][motor_id].sg_get_stall_val        =    tn[motor_id].sg_get_stall_val;   
  rdbk_prev.item[ndx][motor_id].cs_get_coolstep_value   =    tn[motor_id].cs_get_coolstep_value;  
  rdbk_prev.item[ndx][motor_id].diag_get_stndstill_indic=    tn[motor_id].diag_get_stndstill_indic;
  rdbk_prev.item[ndx][motor_id].prtct_get_a_b_shrt2gnd  =    tn[motor_id].prtct_get_a_b_shrt2gnd; 
  rdbk_prev.item[ndx][motor_id].prtct_get_ovrtmp_wrn    =    tn[motor_id].prtct_get_ovrtmp_wrn;   
  rdbk_prev.item[ndx][motor_id].prtct_get_ovrtmp_shtdn  =    tn[motor_id].prtct_get_ovrtmp_shtdn; 
  rdbk_prev.item[ndx][motor_id].sg_get_stall_flag       =    tn[motor_id].sg_get_stall_flag;      
  rdbk_prev.item[ndx][motor_id].prtct_get_openld_a_b    =    tn[motor_id].prtct_get_openld_a_b; 
  
  if (true)
  {
   
    master_data_u.u16_data[1]= rdbk_prev.item[ndx][motor_id].drv_get_mstep_position;
#ifdef TMC2660_ENABLE_PERIODIC_PRINTING_READBACKS  
    if (send_msg_update_interval_counter++>SEND_UART_MS_UPDATE_INTERVAL_COUNT)
    {
         mtr_drv_flg_event_count[MTR_AXIS_X]  = main_get_x_stall_event_count();     
         mtr_drv_flg_event_count[MTR_AXIS_Y]  = main_get_y1_stall_event_count();     
         mtr_drv_flg_event_count[MTR_AXIS_Z]  = main_get_z_stall_event_count();        
         mtr_drv_flg_event_count[MTR_AXIS_Y2] = main_get_y2_stall_event_count();     
         send_msg_update_interval_counter = main_get_x_stall_event_count();

#ifdef  TMC2660_PRINT_STALL_AND_COOLSTEP_AND_OT_WARN_AND_SHUTDOWN
         //if ((tn[motor_id].prtct_get_ovrtmp_wrn!=0)||(tn[motor_id].prtct_get_ovrtmp_shtdn!=0))
         {
//          sprintf(change_message_text,"tmc2660 stepper %s: coolstep=%d stallguard=%d\n",         
//          sprintf(change_message_text,"axis:%s OTPW=%d, OTPShtdn=%d\n", 
//          sprintf(change_message_text,"axis:%s, coolstep=%d,  stall_val=%d,  stall_flg=%d, stndstll=%d, OTPW=%d, OTPShtdn=%d\n", 
//          sprintf(change_message_text,"axis:%s, coolstep=%d,  stall_val=%d,  stall_flg=%d, OTPW=%d, OTPShtdn=%d\n", 
//          sprintf(change_message_text,"axis:%s,OTPW=%d, OTPShtdn=%d, SGT=%d, stall_flg=%d, SG=%d\n",            
           sprintf(change_message_text,"axis:%s,feedrate=%d, SGT=%d, stall_flg=%d, stall_count=%d, SG=%d\n",  
             AXIS_NAME[motor_id],
             feedrate_ival,
             //tn[motor_id].cs_get_coolstep_value,
             //tn[motor_id].prtct_get_ovrtmp_wrn,           
             //tn[motor_id].prtct_get_ovrtmp_shtdn,            
             tn[motor_id].sg_set_stall_threshold,            
             tn[motor_id].sg_get_stall_flag,
             mtr_drv_flg_event_count[motor_id],
             tn[motor_id].sg_get_stall_val);
             //tn[motor_id].diag_get_stndstill_indic,
             
            comms_uart7_send_text_msg((uint8_t*)change_message_text, strlen(change_message_text));
         }
#else// print all
       sprintf(change_message_text,"tmc2660 stepper %d, %d changes: usteps=%d, polarity=%d, coolstep=%d stallguard=%d,stall_flg=%d, stndstll=%d, shr2nd=%d, OTPW=%d, OTPShtdn=%d, OpnLd=%d\n",
          
          motor_id,
          change_flg_count, 
          master_data_u.ustep_bits.ustep_position,  //tn[motor_id].drv_get_mstep_position, 
          master_data_u.ustep_bits.ustep_polarity,
          tn[motor_id].cs_get_coolstep_value,
          tn[motor_id].sg_get_stall_val,
          tn[motor_id].sg_get_stall_flag, 
          tn[motor_id].diag_get_stndstill_indic,
          tn[motor_id].prtct_get_a_b_shrt2gnd,
          tn[motor_id].prtct_get_ovrtmp_wrn,           
          tn[motor_id].prtct_get_ovrtmp_shtdn,         
          tn[motor_id].prtct_get_openld_a_b);
         
          comms_uart7_send_text_msg((uint8_t*)change_message_text, strlen(change_message_text));
#endif
       
    }
#endif    
  }  
  return 0;

}/* End Function */

/***************************************************************************
 * 
 * Function: tmc2660_runtime_refresh_response_data
 *
 * Description: Copies latest tmc2660 response /readback date to persistent 
 * data stores accessed from the the config array.
 * 
 **************************************************************************/ 
stat_t tmc2660_runtime_refresh_response_data(MotorAxisIdE motor_id, uint32_t _response, Tmc2660ReadSelIdE select_id)
{
  stat_t status = 0;
  Tmc2660Readback10BitDataU ten_bit_data; 
  static volatile Tmc2660_ReadResponse_u response;
#ifdef REPORT_TMC2660_OVER_TEMP
  enum{OT_WARNING, OT_SHUTDOWN, OT_MSG_COUNT};
  static bool ot_active_flag[OT_MSG_COUNT][MTR_AXIS_ID_COUNT]={0};
  const char *OT_msg[OT_MSG_COUNT][MTR_AXIS_ID_COUNT]=
  {
    "Stepper Axis X OverTemp Warning",
    "Stepper Axis Y1 OverTemp Warning",   
    "Stepper Axis Z OverTemp Warning",
    "Stepper Axis Y2 OverTemp Warning", 
    "Stepper Axis X OverTemp Shutdown",
    "Stepper Axis Y1 OverTemp Shutdown",   
    "Stepper Axis Z OverTemp Shutdown",
    "Stepper Axis Y2 OverTemp Shutdown"     
  };
#endif
  response.u32_data = _response;
  ten_bit_data.tenbits.data = response.struct_data.data;
  
  /* Assign the selectable twm bit data per the selection */
   tn[motor_id].drv_get_mstep_position=0;
   tn[motor_id].sg_get_stall_val=0;
   tn[motor_id].cs_get_coolstep_value =0; 

  switch (select_id)
  {
     case READ_SELECT_USEPS:
       tn[motor_id].drv_get_mstep_position=response.struct_data.data;
       break;
       
     case READ_SELECT_STALL_GUARD_VALUE:
       tn[motor_id].sg_get_stall_val=response.struct_data.data; 
       break;
       
     case READ_SELECT_COOLSTEP:
       tn[motor_id].cs_get_coolstep_value =ten_bit_data.sg_and_coolstep_bits.coolstep;       
       break;
    
     default://corrupt data!
       __NOP();//__no_operation();;
       break;
  }/* End switch */
  /* Assign the other bit field data */
  tn[motor_id].diag_get_stndstill_indic   =response.struct_data.standstill;
  tn[motor_id].prtct_get_a_b_shrt2gnd     =response.struct_data.short_t_gnd;
  tn[motor_id].prtct_get_ovrtmp_wrn       =response.struct_data.overtemp_wrn;  
  tn[motor_id].prtct_get_ovrtmp_shtdn     =response.struct_data.overtemp_shut_dn;
  tn[motor_id].sg_get_stall_flag          =response.struct_data.stall; 
  tn[motor_id].prtct_get_openld_a_b       =response.struct_data.open_load;
#ifdef REPORT_TMC2660_OVER_TEMP
  if (tn[motor_id].prtct_get_ovrtmp_shtdn==true)
  { 
    if(ot_active_flag[OT_SHUTDOWN][motor_id]!=true)
    {
      ot_active_flag[OT_SHUTDOWN][motor_id]=true;
      rpt_exception(STAT_STEPPER_OT_SHUTDOWN,(char*)OT_msg[OT_SHUTDOWN][motor_id]);
    }
  }
  else 
  {
    ot_active_flag[OT_SHUTDOWN][motor_id]=false;
    
    if(tn[motor_id].prtct_get_ovrtmp_wrn==true)
    {
      if(ot_active_flag[OT_WARNING][motor_id]!=true)
      {
         ot_active_flag[OT_WARNING][motor_id]=true;
         rpt_exception(STAT_STEPPER_OT_WARNING,(char*)OT_msg[OT_WARNING][motor_id]);
      }
    }
    else
    {
      ot_active_flag[OT_WARNING][motor_id]=false;
    }
  }
#endif  
#ifdef TMC2660_ENABLE_STATUS_PRINTING  
  status = tmc2660_runtime_examine_response_data(motor_id);
  if (status >0)
  {
    __NOP();//__no_operation();;//Allow for a breakpoint
  }
#endif
  return status;
}/* End function */

stat_t tmc2660_set_select(nvObj_t *nv)
{
  stat_t result=STAT_OK;
  /* out of range values get assigned 0== "None" */
  if (nv->value_int<0)
  {
    nv->value_int=0;
  }
  else
  if (nv->value_int>SELECT_MOTOR4)
  {
    nv->value_int=0;
  }
  set_ui8(nv);;
  return result;
}
#ifdef __TEXT_MODE
void tn_print_select(nvObj_t *nv)
{   
  const char fmt_select_id[] = "[tnselect]:selected mtr axis id=%s \n";  
   const char *mtr_axis[SELECT_MOTOR_COUNT]={"NONE","X","Y1","Z","Y2"};
   int id = nv->value_int;
   if(id>=SELECT_MOTOR_COUNT)
   {
     id=0;
   }
   fprintf_P( STDERR_SUBSTITUTE, fmt_select_id, mtr_axis[id]);
   comms_mgr_write_msg(STDERR_SUBSTITUTE);     
}
/***************************************************************************
 * Provide for JSON command paradigm:
 * print functions and their related format strings
 * 
* ****************************************************************************/ 
void _print_tn_item_ui8(nvObj_t *nv, const char *format)
{
   fprintf_P( STDERR_SUBSTITUTE, format,  nv->token, nv->group, (uint8_t)nv->value);
   comms_mgr_write_msg(STDERR_SUBSTITUTE);
}

void _print_tn_item_i8(nvObj_t *nv, const char *format)
{
   fprintf_P( STDERR_SUBSTITUTE, format,  nv->token, nv->group, (int8_t)nv->value);
   comms_mgr_write_msg(STDERR_SUBSTITUTE);
}
/*
 * SME: augment the print funcs to access adjacent 8-bit values in a struct
 *with a single format print function. Assumes two adjacenet byte fields.
 *
*/
void _print_tn_item_ui16_special(nvObj_t *nv, const char *format)
{
  U16DataU u16_data_u;
  u16_data_u.u16_data=(uint16_t)nv->value;//Assumes two adjacent byte fields.
  fprintf_P( STDERR_SUBSTITUTE, format,  nv->token, nv->group, u16_data_u.b_data.low_byte,u16_data_u.b_data.high_byte);
  comms_mgr_write_msg(STDERR_SUBSTITUTE);
}
 
void _print_tn_item_u16_ustep(nvObj_t *nv, const char *format)
{
  volatile uint8_t usteps_count=0, direction=0;
 
  union 
  {
    uint16_t u16_data; 
    uint8_t u8_data[sizeof(uint16_t)];
    Tmc2660_ustepInfoS struct_data;
  }u16_data_u;
  
#if 1//debug only
   volatile uint16_t u16_data;
   uint32_t u32_data;
   //usteps_count = 0xA5;
   //direction = 1;
   u16_data_u.struct_data.ustep_counts = usteps_count;
   u16_data_u.struct_data.dir = direction;
   u16_data=u16_data_u.u16_data;
#endif 
   //nv->value = u16_data;   
   u32_data  = (uint32_t)nv->value; 


  u16_data_u.u16_data=(uint16_t) u32_data;
  usteps_count=u16_data_u.struct_data.ustep_counts;
  direction =  u16_data_u.struct_data.dir;
  
  fprintf_P( STDERR_SUBSTITUTE, format,  nv->token, nv->group,usteps_count,direction);
  comms_mgr_write_msg(STDERR_SUBSTITUTE);
}
void _print_tn_item_ui16(nvObj_t *nv, const char *format)
{
   fprintf_P( STDERR_SUBSTITUTE, format,  nv->token, nv->group, (uint16_t)nv->value);
   comms_mgr_write_msg(STDERR_SUBSTITUTE);
}

void _print_tn_item_i16(nvObj_t *nv, const char *format)
{
   fprintf_P( STDERR_SUBSTITUTE, format,  nv->token, nv->group, (int16_t)nv->value);
   comms_mgr_write_msg(STDERR_SUBSTITUTE);
}

static const char fmt_tncstl[] = "[%s] m%s %16d[0..15]SEMIN lower coolstep threshold, coolstep disabled==0\n";
static const char fmt_tncsth[] = "[%s] m%s %16d[0..15]SEMAX upper coolstep threshold, offset from lower thrshld, related to stallguard\n"; 
void tn_print_cs_thrsl(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tncstl);}
void tn_print_cs_thrsh(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tncsth);}

static const char fmt_tnseup[]  = "[%s] m%s %16d[0..3]SEUP current increment step size 0=1, 1=2, 2=4, 3=8\n";
static const char fmt_tnsedn[]  = "[%s] m%s %16d[0..3]SEDN current decrement sample times 0=32x, 1=8x, 2=2x, 3=1x\n";
void tn_print_cs_coil_i_up(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnseup);}
void tn_print_cs_coil_i_dn(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnsedn);}

static const char fmt_tnseimin[]  = "[%s] m%s %16d[0..1]SEIMIN lower limit of scaling coil current, 0=1/2 of CS, 1=1/4 of CS\n";
void tn_print_standstill_imin(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnseimin);}

static const char fmt_coolstep_val[]  = "[%s] m%s %16d[0..31]SE actual coolstep scaling val\n";
void tn_print_coolstep_val(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_coolstep_val);}

static const char fmt_tnsgr[] = "[%s] m%s %16d[0..1023]SG StallGuard reading(Inversely proportional to load\n";
void tn_print_stall_val(nvObj_t *nv){_print_tn_item_ui16(nv,fmt_tnsgr);}

static const char fmt_tnstall_flg[] = "[%s] m%s %16d[0..1]SGT StallGuard Threshold met flag, 0=false, 1=true\n";
void tn_print_stall_flg(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnstall_flg );}

 
static const char fmt_tnshrt2gnd[] = "[%s] m%s %16d[0..3]S2GA,S2GB Short2Gnd, 0=None, 1=S2GB, 2=S2GA, 3=S2GA, S2GB\n"; 
void tn_print_shrt2gnd_ab(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnshrt2gnd );}
 
static const char fmt_tnol_ab_flgs[] = "[%s] m%s %16d[0..3]OLA,OLB, Open Load, 0=None, 1=OLB, 2=OLA, 3=OLB, OLA \n"; 
void tn_print_ol_ab_flgs(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnol_ab_flgs );}

static const char fmt_tnstndstll_flg[] = "[%s] m%s %16d[0..1]STST Standstill flag, 0=false, 1=true\n";
void tn_print_stndstll_flg(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnstndstll_flg);}

static const char fmt_tnustep_pos[] = "[%s] m%s %16d[0..511] MSTEP position, %d[0..1]Direction, 0=Fwd, 1=Rev\n";
void tn_print_ustep_pos(nvObj_t *nv){_print_tn_item_u16_ustep(nv,fmt_tnustep_pos );}

static const char fmt_tnsg_thrsh[] = "[%s] m%s %16d[-64..+63]SGT StallGuard Threshold Val, (advised to stay above -10)\n";
void tn_print_sg_thrsh(nvObj_t *nv){_print_tn_item_i8(nv,fmt_tnsg_thrsh );}

static const char fmt_tnfilt_en[] = "[%s] m%s %16d[0..1]SFILT StallGuard Filtering Enable, 0=disable, 1=enable,\n";
void tn_print_filt_en(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnfilt_en );}

static const char fmt_tnchop_mode[] = "[%s] m%s %16d[0..1]CHM Chopper Mode, 0=Spreadcycle, 1= Constant Off Time\n";
void tn_print_chpr_mode(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnchop_mode );}

static const char fmt_tnblnkng_time[] = "[%s] m%s %16d[0-3]TBL Chopper Blanking Time clock cycles, 0=16,1=24,2=36,3=54\n";
void tn_print_blnkng_time(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnblnkng_time );}

static const char fmt_tnrndm_off_time[] = "[%s] m%s %16d[0..1]RNDTF Select Random Off Time, 0=Fixed, 1=Random\n";
void tn_print_rndm_off_time(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnrndm_off_time );}

static const char fmt_tnfast_decay_time[] = "[%s] m%s %16d[0..15]HSTRT Chopper Fast Decay Time, 0=slow, 1-15=duration, precondition: CHM=1\n";
void tn_print_fast_decay_time(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnfast_decay_time );}

static const char fmt_tnfast_decay_mode[] = "[%s] m%s %16d[0..1]HDEC Chopper Fast Decay Mode  0=Terminate by Current Comparator, 1= Terminate by Timer, precondition: CHM=1\n";
void tn_print_chpr_chm1_fast_decay_mode(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnfast_decay_mode );}

static const char fmt_tnchop_hyst_sine_offset[] = "[%s] m%s %16d[0..15]HEND Chopper Sinewave Offset, 0-2 negative, 3 no offset, 4-15 positive offset,  precondition CH1=1 \n";
void tn_print_chpr_chm1_sine_offset(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnchop_hyst_sine_offset );}

static const char fmt_tnchop_hyst_decr_intrvl[] = "[%s] m%s %16d[0-3]HDEC Chopper Hyst Decr Interval clock periods,0=16,1=32,2=48,3=64 precondition: CHM=0\n";
void tn_print_chpr_chm0_hyst_decr_intrvl(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnchop_hyst_decr_intrvl );}

static const char fmt_tnchop_hyst_strt[] = "[%s] m%s %16d[0..7] HSTRT Chopper Hyst Start Val, 0=1,1=2...7=8 Precondition: Spreadcycle (CHM=0)\n";
void tn_print_chpr_chm0_hyst_start(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnchop_hyst_strt);}

static const char fmt_tnchop_hyst_end[] = "[%s] m%s %16d[0-15]HEND Chopper Hyst End val, precondition: CHM=0\n";
void tn_print_chpr_chm0_hyst_end(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnchop_hyst_end );}

static const char fmt_tn_chop_off_time[] = "[%s] m%s %16d[0..15]TOFF Set Chopper Off Time/MOSFET Disable val 0=Disable, 1 to 15= slow decay time t=1/(f_clck)*((Toff*32)+12)\n";
void tn_print_chpr_off_time(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tn_chop_off_time );}

static const char fmt_tnslope_ctrl_hs[] = "[%s] m%s %16d[0..3]SLPH Mosfet High-side Slope, Min, 1=Min+Temper compens, 2=Med+Temper compens,3=Max\n";
void tn_print_slope_ctrl_hs(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnslope_ctrl_hs );}

static const char fmt_tnslope_ctrl_ls[] = "[%s] m%s %16d[0..3]SLPL Mosfet Low-side Slope0=Min, 3=Max\n";
void tn_print_slope_ctrl_ls(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnslope_ctrl_ls );}

static const char fmt_tns2g_detct_tmr[] = "[%s] m%s %16d[0..3]TS2G Shrt2Gnd Detect Delay uSecs: 0=3.2uS, 1=1.6uS, 2=1.2uS, 3=0.8uS\n"; 
void tn_print_s2g_detct_tmr(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tns2g_detct_tmr );}

static const char fmt_tns2g_detct[] = "[%s] m%s %16d[0..1]DISS2G S2G Protection Detect : 0=Enable, 1=Disable\n";
void tn_print_s2g_detct(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tns2g_detct);}

static const char fmt_tntest_mode_off[] = "[%s] m%s %16d[0..3] TST Test Mode: 0=Off/Normal Advised to stay OFF\n";
void tn_print_test_mode_off(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tntest_mode_off );}

static const char fmt_tnsd_interpol[] = "[%s] m%s %16d[0..1]INTPOL Step/Dir Interpolation: 0=Disable, 1=Enable\n";
void tn_print_sd_interpol(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnsd_interpol );}

static const char fmt_tnedges_per_puls[] = "[%s] m%s %16d[0..1]DEDGE Edges Per Step Pulse: 0= Rising only, 1= Rising and Falling\n";
void tn_print_edges_per_puls(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnedges_per_puls );}

static const char fmt_tnsd_en[] = "[%s] m%s %16d[0..1]SDOFF Step/Dir Mode: 0=Enable, 1=Disable \n";
void tn_print_sd_en(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnsd_en );}

static const char fmt_tni_scale[] = "[%s] m%s %16d[0..31]CS Set Current Scaling, 0=1/31,1=2/32, 2=3/32,...,31=32/32\n";
void tn_print_i_scale(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tni_scale );}

static const char fmt_tnotp_wrn_shtdn_flgs[] = "[%s] m%s %16d [0..1]OTPW Over Temperature Warning flag, 0=false. 1=true, %d [0..1]OTP Shutdown 0=false, 1=true\n";
void tn_print_otp_ot_flgs(nvObj_t *nv){_print_tn_item_ui16_special(nv,fmt_tnotp_wrn_shtdn_flgs );}
 
static const char fmt_tnvsense_fscale[] = "[%s] m%s %16d[0..1]VSENSE resister voltage-based current scaling, 0=set 0.31V, 1= set to 0.165 \n";
void tn_print_vsense_fscale(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnvsense_fscale );}

#ifdef USE_TMC2660C_DRIVERS
static const char fmt_tnotsens[] = "[%s] m%s %16d[0..1]Overtemperature sensitivity, 0=shutdown at 150C, 1=sensitive shutdown at 136C \n";
void tn_print_otsens(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnotsens );}

static const char fmt_tnshrtsens[] = "[%s] m%s %16d[0..1]Short to GND detection sensitivity, 0=low sensitivity, 1=high sensitivity \n";
void tn_print_shrtsens(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnshrtsens );}

static const char fmt_tnen_pfd[] = "[%s] m%s %16d[0..1]Enable Passive fast decay / 5V undervoltage threshold, 0=no additional dampening, 1=dampening \n";
void tn_print_en_pfd(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnen_pfd );}

static const char fmt_tnen_s2vs[] = "[%s] m%s %16d[0..1]Short to VS and CLK fail protection, 0=disabled, 1=enabled \n";
void tn_print_en_s2vs(nvObj_t *nv){_print_tn_item_ui8(nv,fmt_tnen_s2vs );}
#endif

#endif