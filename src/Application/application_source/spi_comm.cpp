/*******************************************************************************
* File:    spi_comm.c
*
* Description: 
*
*     
*
*******************************************************************************/
#include "main.h"
#include "bantam_hal.h"
#include "user_types.h"
#include "globdefs.h"

#include "g2core.h" 
#include "g2_config.h"
#include "tmc2660_mtr_ctrlr.h" 
#include "spi_comm.h"
// IAR #include "intrinsics.h"//sme: nop   
#include <string.h>//memset
#include "kernel.h"

/* Allocate buffers */
#define SPI_TX_BUF_LEN 80//Arbitrary, refine later.
#define SPI_RX_BUF_LEN 80//Arbitrary, refine later.
uint8_t spiTxBuf[SPI_TX_BUF_LEN];
uint8_t spiRxBuf[SPI_RX_BUF_LEN];
 
 
/* Enumerate non-blocking SPI comm state machine */
typedef enum
{
  SPI_STATE_IDLE,
  SPI_WRITE,
  SPI_POLL_WRITE_COMPLETE,
  SPI_READ,
  SPI_POLL_READ_COMPLETE
}SpiSendReceiveStateE;

 
 /* if using interrutpts: */ 
volatile uint8_t spi_rx_completed_flag = 0;
volatile uint8_t spi_tx_completed_flag = 0;
 
 HAL_StatusTypeDef spi_status = HAL_OK; 
 
 typedef struct
 {
   uint8_t red;
   uint8_t green;
   uint8_t blue;
   uint8_t white;
 }SpotlightLedColorSettingS;



#ifdef DEPLOY_FUTURE_EXPANSION_BRD
 /****************************************************************************
 * 
 * Function:  spi_comm_task 
 * 
 * Description:   
 * Assumptions/Requirement: 
 *****************************************************************************/
 /***************************************************************************
 * 
 * Function:  spi_comm_init 
 * 
 * Description:   
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
void spi_comm_init(void)
{

}
void spi_comm_task(void)
{
  static volatile enum
  {
    SPI_MGR_INIT, 
    SPI_MGR_TEST,
    SPI_MGR_RUN,
    SPI_MGR_STATE_COUNT
  }spi_mgr_state = SPI_MGR_INIT;
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_in();
#endif 
  //hal_toggle_pin(GPIO_DBG2_GPIO_PortId,GPIO_DBG2_Pin );
  /* Apply the   */
  switch(spi_mgr_state )
  {
     case SPI_MGR_INIT: 
       spi_comm_init();       
       spi_mgr_state = SPI_MGR_RUN;//SPI_MGR_TEST;
       break;
    
     case SPI_MGR_TEST: 

       break;
    
     case SPI_MGR_RUN:
       /* 
        * To do: 1.Setup a state machine for periodic interrogation of the power board using the defined commands
        *        2. Setup a state machine for non-periodic accessesof th power board
        */
        // deploy when ready pwr_brd_spi_comm();
        break;
            
     default:
       /* to do: raise error: corrupt data:*/
      break;
  }/* End Switch */
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_out();
#endif       
}/* End Task */
#endif