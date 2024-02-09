/******************************************************************
*
* File: comms_mgr.c
*
* @brief: 
*
******************************************************************/
#include "main.h"
#include "bantam_hal.h"
#include "user_types.h"
#include "globdefs.h"
#include "definitions.h"
#include "tasks.h"
#include "g2_util.h"
#include "g2core.h"
#include "g2_xio.h"// - extended IO functions
#include "g2_canonical_machine.h"
#include "g2_controller.h"
#include "g2_text_parser.h"
#include "system_bantam.h"
#include "comms_mgr.h"
#include "motion_ctrl.h"
#include "kernel.h"
#include "g2_report.h"

/* Incoming general message buf */
typedef struct
{
  uint8_t buf[COMMS_MGR_RX_CMD_BUF_ENTRIES][COMMS_MGR_RX_CMD_BUF_LEN];
  int write_ndx;//producer owns this
  int read_ndx;//consumer owns this 
  int new_arrival_count;//incremented by "writer", rx int callback, decremented by "reader"  comms_mgr task, >0 is length 
  int cmdFlush_ndx;
  uint8_t new_arrival_busy_flag;
}CmdInfoQueueS;
 
typedef struct
{
  uint8_t buf[COMMS_SPECIAL_CHAR_BUFLEN];
  int write_ndx;//producer owns this
  int read_ndx;//consumer owns this 
  int new_arrival_count;//incremented by "writer", rx int callback, decremented by "reader"  comms_mgr task, >0 is length       
}SpecialCharInfoQueueS;
 
/* Out going, "response" messages.
 *  text info prints can be queued or simply directly written to the tx port 
*/
typedef struct
{
  uint8_t buf[COMMS_MGR_TX_CMD_BUF_ENTRIES][COMMS_MGR_TX_RESPONSE_BUF_LEN];
  int write_ndx;//producer owns this
  int read_ndx;//consumer owns this 
  int new_arrival_count;// producer increments, consumer decrements this
  uint8_t new_arrival_flag;//set by producer e.g JSON response, reset by consumer, comms_mgr task     
}ResponseInfoQueueS;

SpecialCharInfoQueueS cmd_special_char_rx_queue;

static CmdInfoQueueS cmd_chan_rx_queue;//general incoming msg queue
CmdInfoQueueS json_chan_rx_queue;//JSON command are "control" and must be executed immediately
static ResponseInfoQueueS response_chan_tx_queue;
volatile UsbdConnectStatusE usbd_connect_status, usbd_prev_connect_status = USBD_STATUS_UNDEFINED;
static int comms_wd_downcount_start_value=COMMS_WD_DOWNCOUNT_START_VALUE;
static int comms_wd_down_counter=COMMS_WD_DOWNCOUNT_START_VALUE;
static bool comms_lost_flag=false;
bool comms_timed_out(void){return (comms_lost_flag==true);}
bool usb_comms_disconnected(void){ return ((usbd_connect_status==USBD_STATUS_DISCONNECTED)&&( usbd_prev_connect_status==USBD_STATUS_CONNECTED));} 

volatile int json_1st_char_instance_count=0;//12-20-2022
volatile int rx_buf_overrun_instance_counter=0;//12-20-2022


/* COMMS_MGR_DEPLOY_GROUP_PRINTING */
/* Allocate a tx buffer large enough to accomodate the longest 
 * conceivable concatenation of a group's collective text report strings 
*/
#define COMMS_MGR_GROUP_TEXT_MAX_LEN 5000 //2048 too small for trinamics group textrefine as needed

/* Outgoing, Define a buffer for containing the 
 *  concatenation of a group of NV Object's info text 
*/
typedef struct
{
  uint8_t buf[COMMS_MGR_GROUP_TEXT_MAX_LEN];
  int len;
  int write_ndx;
  uint8_t group_started_flag;
  uint8_t group_ready_flag;
  uint8_t group_completed_flag;
  uint8_t group_item_count;  
}TextModeResponsePrintGroupInfoS;  
static TextModeResponsePrintGroupInfoS comms_mgr_print_group;

/* Enumerate comms_mgr task states */
typedef enum 
{
  COMMS_MGR_IDLE,
  COMMS_MGR_INIT,
  COMMS_MGR_NORMAL_HOST_COMM_MODE,
  COMMS_MGR_STATE_COUNT
} CommsMgrStateE;

#ifdef ROSECOMB_MVP
UartIntfType exp_uart;

void exp_uart_rx_cb(uintptr_t context)
{
  // Increment length
  exp_uart.rx[exp_uart.write_ndx].len++;
  
  // CR or LF reached, mark transfer done and increment pointers
  if ((exp_uart.rx[exp_uart.write_ndx].len > 1) &&
      ((exp_uart.rx[exp_uart.write_ndx].buf[exp_uart.rx[exp_uart.write_ndx].len - 1] == CR_CHAR) ||
       (exp_uart.rx[exp_uart.write_ndx].buf[exp_uart.rx[exp_uart.write_ndx].len - 1] == NEWLINE_CHAR))) {
    
    exp_uart.rx[exp_uart.write_ndx].rdy = true;
    exp_uart.write_ndx = ((exp_uart.write_ndx + 1) % EXP_UART_NUM_BUFS);
    
    // Start next read at start of new buffer
    UART0_Read(&exp_uart.rx[exp_uart.write_ndx].buf, 1);
    
  } else {
    
    // Restart read at the next location in the current buffer
    UART0_Read(&exp_uart.rx[exp_uart.write_ndx].buf[exp_uart.rx[exp_uart.write_ndx].len], 1);
  }
}

bool uart_mgr_exp_uart_connected(void) {
  return exp_uart.enabled;
}
#endif

 static devflags_t flags;                        // bitfield for device state flags (these are not)
 void comms_mgr_setUSBAsConnectedAndReady(void) { flags |= ( DEV_IS_CONNECTED | DEV_IS_READY); };
 bool comms_mgr_isUSBConnected() { return flags & DEV_IS_CONNECTED; }
 bool comms_mgr_isUSBNotConnected() { return !(flags & DEV_IS_CONNECTED); }
 void comms_mgr_clearUSBFlags() { flags = DEV_FLAGS_CLEAR; }       
 void comms_mgr_assign_cmd_flush_ndx(void)
{
   cmd_chan_rx_queue.cmdFlush_ndx=cmd_chan_rx_queue.write_ndx;
   json_chan_rx_queue.cmdFlush_ndx=json_chan_rx_queue.write_ndx; 
}
 void comms_mgr_enable_usb_interrupt(void)
 {
   SYS_INT_Enable();
 }
void comms_mgr_disable_usb_interrupt(void)
{
 SYS_INT_Disable();
}
void comms_mgr_flushToCommand(void)//int special_ndx)
{ 
  if (cmd_chan_rx_queue.cmdFlush_ndx != PRIORITY_NDX_UNASSIGNED)
  { 
     SYS_INT_Disable();
#ifdef ROSECOMB_MVP
     // Deassert RTS before flushing the queues
     if (exp_uart.enabled) {
       EXP_UART_RTS_Clear();
     }
#endif
     cmd_chan_rx_queue.new_arrival_count=0;      
     while (cmd_chan_rx_queue.read_ndx !=cmd_chan_rx_queue.cmdFlush_ndx)
     {
       cmd_chan_rx_queue.buf[cmd_chan_rx_queue.read_ndx++][0]=0;
       if(cmd_chan_rx_queue.read_ndx>=COMMS_MGR_RX_CMD_BUF_ENTRIES)
       {
         cmd_chan_rx_queue.read_ndx=0;//handle wrap-around
       }                 
     }
     cmd_chan_rx_queue.cmdFlush_ndx = PRIORITY_NDX_UNASSIGNED;
#ifdef ROSECOMB_MVP
     // OK now we're ready again
     if (exp_uart.enabled) {
       EXP_UART_RTS_Set();
     }
#endif
 
     json_chan_rx_queue.new_arrival_count=0;      
     while (json_chan_rx_queue.read_ndx !=json_chan_rx_queue.cmdFlush_ndx)
     {
       json_chan_rx_queue.buf[json_chan_rx_queue.read_ndx++][0]=0;
       if(json_chan_rx_queue.read_ndx>=COMMS_MGR_RX_CMD_BUF_ENTRIES)
       {
         json_chan_rx_queue.read_ndx=0;//handle wrap-around
       }                 
     }
     json_chan_rx_queue.cmdFlush_ndx = PRIORITY_NDX_UNASSIGNED;  
  
     SYS_INT_Enable();
  }
}
void comms_mgr_flushRead(void)
{
   __disable_irq();//gcc
   memset((void *)&cmd_chan_rx_queue,0, sizeof(cmd_chan_rx_queue));
   cmd_chan_rx_queue.cmdFlush_ndx=PRIORITY_NDX_UNASSIGNED;
   memset((void *)&json_chan_rx_queue,0, sizeof(json_chan_rx_queue));
   json_chan_rx_queue.cmdFlush_ndx=PRIORITY_NDX_UNASSIGNED;
   memset((void *)&cmd_special_char_rx_queue,0, sizeof(cmd_special_char_rx_queue));   
   __enable_irq();//gcc
}

void comms_mgr_flush(void)
{ 
   SYS_INT_Disable();
   memset((void *)&cmd_chan_rx_queue,0, sizeof(cmd_chan_rx_queue)); 
   memset((void *)&response_chan_tx_queue,0, sizeof(response_chan_tx_queue)); 
   memset((void *)&cmd_special_char_rx_queue,0, sizeof(cmd_special_char_rx_queue)); 
   memset((void *)&json_chan_rx_queue,0, sizeof(json_chan_rx_queue));
   SYS_INT_Enable();
}
 

/* Allocate COMMS task state var */
#define COMMS_MGR_DEFAULT_RUN_STATE COMMS_MGR_NORMAL_HOST_COMM_MODE
static CommsMgrStateE comms_mgr_state = COMMS_MGR_IDLE;

static uint32_t comms_mgr_delay_start_time_ms=0;
void comms_mgr_set_state_idle(void){comms_mgr_state =COMMS_MGR_IDLE;}
void comms_mgr_set_state_run(void){comms_mgr_state =COMMS_MGR_NORMAL_HOST_COMM_MODE;}
bool comms_mgr_usb_connected(void){return (usbd_connect_status==USBD_STATUS_CONNECTED);}
extern "C" void comms_mgr_set_usb_connection_state(bool state );
void comms_mgr_set_usb_connection_state(bool state )
{ 
    usbd_connect_status=USBD_STATUS_DISCONNECTED;
    if (state==true)
    {
    usbd_connect_status=USBD_STATUS_CONNECTED;
    }
}
 

#define WDOG_DEFAULT_UPDATE_PERIOD_SECS  (uint16_t)1
uint16_t comms_wdog_update_period_secs = WDOG_DEFAULT_UPDATE_PERIOD_SECS;
uint16_t comms_wdog_timeout_secs=COMMS_DEFAULT_WD_TIME_SECS;
bool comms_wdog_enabled(void){return (comms_wdog_update_period_secs>0);}

/***************************************************************************
 * 
 * Function:  comms_print_wdtimeout 
 * 
 * Description:  Prints the wdog timeout time in seconds in text mode
 *
 ***************************************************************************/
void comms_print_wdtimeout(nvObj_t *nv)
{
    const char fmt_wdog[] = "[sys wdtime] comms watchdog timeout interval seconds :%d\n";
    nv->value_int=comms_wdog_timeout_secs;
    text_print_int(nv,fmt_wdog);
}
/***************************************************************************
 * 
 * Function:  coms_get_wdog_timeout 
 * 
 * Description: Gets the wdog timeout time interval in seconds
 *  
 ***************************************************************************/
stat_t comms_get_wdog_timeout(nvObj_t *nv)
{
    nv->value_int = comms_wdog_timeout_secs;
    nv->valuetype = TYPE_INTEGER;
     return STAT_OK;
}
/***************************************************************************
 * 
 * Function:  comms_set_wdog_timeout 
 * 
 * Description: Sets the wdog timeout time interval in seconds
 * 
 ***************************************************************************/
stat_t comms_set_wdog_timeout(nvObj_t *nv) 
{
    nv->valuetype = TYPE_INTEGER; 
    /*Silently range check/correct value as needed */
    if (nv->value_int<COMMS_MIN_WD_TIMEOUT_SECS)
    {
      nv->value_int=COMMS_MIN_WD_TIMEOUT_SECS;
    }
    else if (nv->value_int>COMMS_MAX_WD_TIMEOUT_SECS)
    {
      nv->value_int=COMMS_MAX_WD_TIMEOUT_SECS;
    }
    comms_wdog_timeout_secs=(uint8_t)nv->value_int;
    comms_wd_downcount_start_value=comms_wdog_timeout_secs*COMMS_TASK_SCHED_FREQUENCY;
   return STAT_OK;
}

/***************************************************************************
 * 
 * Function:  comms_get_wdog 
 * 
 * Description:  Watchdog is disables with vale 0, enables with integer values >0
 *               where 1 = 1 second interval
 *  
 ****************************************************************************/
stat_t comms_get_wdog(nvObj_t *nv)
{   
    nv->value_int = comms_wdog_update_period_secs;
    nv->valuetype = TYPE_INTEGER;
    return STAT_OK;
}
/***************************************************************************
 * 
 * Function:  comms_set_wdog 
 * 
 * Description:  Watchdog is disables with value 0, enabled with integer values n >0
 *               where n = 1,2,3,...COMMS_MAX_WD_TIMEOUT_SECS second interval
 *                 
 *  
 ****************************************************************************/
stat_t comms_set_wdog(nvObj_t *nv)
{   
   nv->valuetype = TYPE_INTEGER;
   if(nv->value_int<0)
   {
     nv->value_int=0;
   }
   if(nv->value_int> COMMS_MAX_WD_TIMEOUT_SECS)
   {
     nv->value_int= COMMS_MAX_WD_TIMEOUT_SECS;
   }
  
    comms_wdog_update_period_secs=nv->value_int;
    comms_wd_downcount_start_value=comms_wdog_update_period_secs*COMMS_MIN_WD_TIMEOUT_SECS*COMMS_TASK_SCHED_FREQUENCY;
    return STAT_OK;
}
/***************************************************************************
 * 
 * Function:  comms_print_wdog 
 * 
 * Description:   
 *  
 ****************************************************************************/
void comms_print_wdog(nvObj_t *nv)
{
  const char fmt_wdog[] = "[sys wdog] wdog:%d [0=Disabled, >0 = Enabled at 1,2,3,4 etc, seconds update interval]\n";
  nv->value_int=comms_wdog_update_period_secs;
  text_print_int(nv,fmt_wdog);
}    

 /***************************************************************************
 * 
 * Function:  comms_mgr_init 
 * 
 * Description:   
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
void comms_mgr_init(void)
{   
  memset((void *)&cmd_chan_rx_queue,0, sizeof(cmd_chan_rx_queue));  
  memset((void *)&response_chan_tx_queue,0, sizeof(response_chan_tx_queue));
  memset((void *)&json_chan_rx_queue,0, sizeof(json_chan_rx_queue));
  memset((void *)&comms_mgr_print_group,0,sizeof(comms_mgr_print_group));
  json_1st_char_instance_count=0;//added 12-21-2022 fro issue
  cmd_chan_rx_queue.cmdFlush_ndx=PRIORITY_NDX_UNASSIGNED; 
  json_chan_rx_queue.cmdFlush_ndx=PRIORITY_NDX_UNASSIGNED;
  comms_mgr_state = COMMS_MGR_IDLE;
  comms_mgr_delay_start_time_ms = sys_get_time_ms();
#ifdef ROSECOMB_MVP
  // Initialize structures
  for (int i = 0; i < EXP_UART_NUM_BUFS; i++) {
    exp_uart.rx[i].len = 0;
    exp_uart.rx[i].rdy = false;
  }

  // Clear active flags and indices
  exp_uart.enabled = false;
  exp_uart.write_ndx = 0;
  exp_uart.read_ndx = 0;
  exp_uart.start_time = 0;
#endif
}/* End Function */ 
 /****************************************************************
 *
 * Function:  comms_mgr_start_multiline_print
 * 
 * Description:  Marks the next entry ndx in the circular tx queue
 *  as the start entry of a group of related items to print out in
 *  preserved sequence FIFO, not cut in on by any other 
 *  print request, while the group is being printed.
 *  Assumes: response_chan_tx_queue.write_ndx== next vailable circ buf entry
 *  Returns Error code if a new call is made while the present group
    is still being processed.
 *   
 ***************************************************************/
 int comms_mgr_start_multiline_print(void)
 {
    int result = 0;
    if (comms_mgr_print_group.group_started_flag ==TRUE)
    {
      result = -1;//ERR_COMMS_MGR_START_GROUP_PRINT_ALREADY_STARTED;
    }
    else
    {
      comms_mgr_print_group.group_started_flag =TRUE;    
      
      /*  Start response on a newline: prepend a newline char on the 0th ndx:*/
      comms_mgr_print_group.buf[0]='\n';
      
      /* Response text starts on 1th ndx: */
      comms_mgr_print_group.write_ndx = 1;
      comms_mgr_print_group.len       = comms_mgr_print_group.write_ndx;      
    }
    return result;
 }
/****************************************************************************
 * 
 * Function:   comms_mgr_queue_tx_msg
 * 
 * Description: gets strnlen and copies text msg into next entry of tx to 
 * out queue. Accomodates text longer than a single queue entry: spill over to 
 * the next entry,etc
 * Assumptions/Requireement: 
 *  returns length of msg placed on queue
 *****************************************************************************/
int comms_mgr_queue_tx_msg( char * msg)  
{  
  /* Point to the next available entry in the queue: */
  char * dst_bufptr = 0; 
    
  /* Protect against buffer overflow: */
  int len = strlen(msg);
   
  volatile int num_queue_entries_needed=1; 
  volatile int spillover_len = 0;
  volatile int queue_entry_len = len;
  volatile int msg_offset = 0;
  
  if (len >=COMMS_MGR_TX_RESPONSE_BUF_LEN)
  {
    num_queue_entries_needed = len/COMMS_MGR_TX_RESPONSE_BUF_LEN; 
    spillover_len = len % COMMS_MGR_TX_RESPONSE_BUF_LEN;
    queue_entry_len =  COMMS_MGR_TX_RESPONSE_BUF_LEN;   
  }
  
  while (num_queue_entries_needed >= 0)
  { 
       /* 5/26/2020: prevent older longer msg text from getting sent out along with shorter present text:*/
       memset(&response_chan_tx_queue.buf[response_chan_tx_queue.write_ndx][0],0,COMMS_MGR_TX_RESPONSE_BUF_LEN);
    
       dst_bufptr = 
      (char * )&response_chan_tx_queue.buf[response_chan_tx_queue.write_ndx][0];
       /* Copy msg onto next queue entry: */
      
       memcpy(dst_bufptr,&msg[msg_offset],queue_entry_len);
     
       /* Increment the new arrival for the comms mgr task*/      
       response_chan_tx_queue.new_arrival_count++;
     
       /* Maintain circular buffer limits*/
       response_chan_tx_queue.write_ndx++;
       if(response_chan_tx_queue.write_ndx >=COMMS_MGR_TX_CMD_BUF_ENTRIES)
       {
         response_chan_tx_queue.write_ndx=0;
       }
       
       /* Move the offset past what has already been copied into the queue:*/
       msg_offset += queue_entry_len;         
       num_queue_entries_needed--;
       
       /* Check for any end of message spillover bytes */
       if (num_queue_entries_needed ==0)
       {
          if (spillover_len>0)
          {
            queue_entry_len = spillover_len;
          }
          else
          {
            break;
          }
       }   
  }/* End while */
   
  return len;
}/* End Function */

 /****************************************************************
 *
 * Function:  comms_mgr_end_multiline_print
 * 
 * Description:  Marks the next entry ndx in the circular tx queue
 *  as the last entry of a group of related items to print out in
 *  preserved sequence FIFO, not cut in on by any other 
 *  print request, while the group is being printed.
 * Assumes: response_chan_tx_queue.write_ndx== next available circ buf entry
 *  Returns:
 *   
 ***************************************************************/
 int comms_mgr_end_multiline_print(void)
 {
    int result = 0;
    if (comms_mgr_print_group.group_started_flag !=TRUE)
    {
      result = -1;//ERR_COMMS_MGR_END_GROUP_PRINT_NOT_STARTED;
    }
    else
    {
      /*Print the concatenated string right here, or place on queue  */    
      comms_mgr_print_group.len=comms_mgr_print_group.write_ndx;
      comms_mgr_print_group.write_ndx=0;//reset
      //print right here:      
      comms_mgr_print_group.group_started_flag = FALSE;//reset
      comms_mgr_queue_tx_msg((char*)&comms_mgr_print_group.buf[0]);   
    }    
    return result;
 } 

/****************************************************************************
 * 
 * Function:   comms_mgr_write_msg
 * 
 * Description: writes text msg immediately out
 *
 * Assumptions/Requireement: 
 *  returns length of msg written out immediately, not queued
 *****************************************************************************/
int comms_mgr_write_msg(char * msg) 
{
  int len = strlen(msg); 

  /* If this item is part of a multi line group print then concatenate the 
   * entire groups text onto a single output string
  */
  if (comms_mgr_print_group.group_started_flag == TRUE)
  {
    /* Concatenate directly here, don't put on queue */
    comms_mgr_print_group.len+=len;//can trap if msg exceeds max len of buf here
    if (comms_mgr_print_group.len <COMMS_MGR_GROUP_TEXT_MAX_LEN)
    {
      strncpy(&comms_mgr_print_group.buf[comms_mgr_print_group.write_ndx], msg, len);
      comms_mgr_print_group.write_ndx += len;
      comms_mgr_print_group.group_item_count++;  
    }
    else
    {
      __NOP();//__no_operation();//to do: notify message is truncated due to excessive size
    }
  }  
  else    
  {
    /*Place on queue, let comms task send it out*/ 
    comms_mgr_queue_tx_msg(msg);
     
  }/* End else */

  return comms_mgr_print_group.len;
}/* End Function */

/****************************************************************************
 * 
 * Function: comms_mgr_USBconnectedStateChanged
 *
 * Description: interrupt callback 
 *
 * Assumptions/Requireement: 
 *
 * Returns: none
 *
*****************************************************************************/
void comms_mgr_USBconnectedStateChanged(bool connected) 
{
   volatile static int steps_remaining_count=0;
  if (connected) 
  {
      if (comms_mgr_isUSBNotConnected()) 
      {
        if(usbd_prev_connect_status != USBD_STATUS_UNDEFINED) 
        {
           comms_mgr_flush(); // toss anything that has been written so far.
        }
        usbd_prev_connect_status=USBD_STATUS_CONNECTED;
        comms_mgr_setUSBAsConnectedAndReady();         
        _controller_set_connected(true);  
          
      } // flags & DEV_IS_DISCONNECTED
  } 
  else 
  {  // disconnected
      if (comms_mgr_isUSBConnected()) 
      {
          //USB0 has just disconnected
          comms_mgr_clearUSBFlags();
          comms_mgr_flush();      
          _controller_set_connected(false);   
      } // flags & DEV_IS_CONNECTED
  }
 }  

 /****************************************************************************
 * 
 * Function:  comms_mgr_task
 * 
 * Description:   Manages queued outgoing messages. (Incoming messages are
 *                handled by polling the rx queue loaded by the rx isr callback
 *
 * Assumptions/Requirement: 
 *
 *****************************************************************************/
void comms_mgr_task(void)
{
     volatile static int len, queue_entry_len = 0;
     volatile USBD_StatusTypeDef ret_stat=USBD_OK;
     volatile static int bad_tx_send_status_count=0;//cleared on each good tx send command
     volatile static char * tx_queue_bufp=0;
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_in();
#endif  
     
     //chabo drivers
     usb_middleware_task();
     usb_hsdrv_task();
     usb_cdc_task();

     /* Execute the task state machine:  */
     switch(comms_mgr_state)
     { 
        case COMMS_MGR_IDLE: 
          if(sys_time_ms_limit_reached(comms_mgr_delay_start_time_ms,COMMS_MGR_STARTUP_DELAY_TIME_MS)==true)
          {
            comms_mgr_state=COMMS_MGR_INIT;
          }
          break;
          
        case COMMS_MGR_INIT:
          comms_mgr_state=COMMS_MGR_NORMAL_HOST_COMM_MODE;          
          break;
          
        case COMMS_MGR_NORMAL_HOST_COMM_MODE:       
          
          /* Process incoming UART RX comms for Rosecomb */
#ifdef ROSECOMB_MVP
             
          // Disconnect from Rosecomb if USB becomes available or we lose connectivity
          if (((usbd_connect_status == USBD_STATUS_CONNECTED) || (ROSECOMB_DETn_Get() != ROSECOMB_DETECTED)) && exp_uart.enabled) {

              //SYS_CONSOLE_PRINT("Connected to USB or lost Rosecomb\r\n");
            
              // Deassert RTS signal and tell Rosecomb we're in USB mode
              EXP_UART_RTS_Clear();
              CHABO_MODE_Set();
              
              // Clear active flag
              exp_uart.enabled = false;
  
              // Abort the current read
              UART0_ReadAbort();
              
              // Flush queues
              for (int i = 0; i < EXP_UART_NUM_BUFS; i++) {
                exp_uart.rx[i].len = 0;
               exp_uart.rx[i].rdy = false;
              }
              exp_uart.write_ndx = 0;
              exp_uart.read_ndx = 0;
  
              // Flush the comms buffers
              comms_mgr_flush();  
              
              // Feedhold and kill any current job
              cm_request_feedhold(FEEDHOLD_TYPE_ACTIONS, FEEDHOLD_EXIT_CYCLE); 
              cm_request_job_kill();
              
              // De-initialize the expansion UART interrupt callbacks to talk to Rosecomb
              UART0_WriteCallbackRegister(NULL, 0);
              UART0_ReadCallbackRegister(NULL, 0);
              
              
          // Connect to Rosecomb via expansion UART if present and USB is unavailable
          } else if (usbd_connect_status != USBD_STATUS_CONNECTED && (ROSECOMB_DETn_Get() == ROSECOMB_DETECTED) && !exp_uart.enabled) {

              //SYS_CONSOLE_PRINT("Connected to Rosecomb UART\r\n"); 
              
              // Set active flag
              exp_uart.enabled = true;

              // Initialize the expansion UART RX interrupt callback to talk to Rosecomb
              UART0_ReadCallbackRegister(exp_uart_rx_cb, 0);

              // Set up initial read
              UART0_Read(&exp_uart.rx[exp_uart.write_ndx].buf, 1);
              
              // Tell controller we're up and running
              _controller_set_connected(true);

              // Tell Rosecomb we're in UART mode and ready!
              CHABO_MODE_Clear();
              EXP_UART_RTS_Set();
              
              // Start UART timeout for RX transfer
              exp_uart.start_time = SYS_TIME_CounterGet();

          // Received (full or partial) buffer of data from Rosecomb, process it
          } else if (exp_uart.enabled && (exp_uart.rx[exp_uart.read_ndx].rdy)) {

              // Copy RX read buffer into comms buffers for processing
              comms_mgr_receive_callback_helper(exp_uart.rx[exp_uart.read_ndx].buf, exp_uart.rx[exp_uart.read_ndx].len);

              // DEBUG: Print out RX buffer
              //exp_uart.rx[exp_uart.read_ndx].buf[exp_uart.rx[exp_uart.read_ndx].len] = 0;
              //SYS_CONSOLE_PRINT("RX>%s\r\n", exp_uart.rx[exp_uart.read_ndx].buf);

              // Clear flags for full buffer and increment pointer
              exp_uart.rx[exp_uart.read_ndx].rdy = false;
              exp_uart.rx[exp_uart.read_ndx].len = 0;
              exp_uart.read_ndx = ((exp_uart.read_ndx + 1) % EXP_UART_NUM_BUFS);

              // Reset UART timeout for next RX transfer
              exp_uart.start_time = SYS_TIME_CounterGet();  
          }
#endif
       
          /* Check if Response to RX'd messages have been placed on the outgoing queue */  
#ifdef ROSECOMB_MVP
         if (usbd_connect_status==USBD_STATUS_CONNECTED || exp_uart.enabled)
#else
          if (usbd_connect_status==USBD_STATUS_CONNECTED)
#endif
          {        
              /*  Check whether Comms are still valid--not lapsed beyond a countdown var*/
              if(comms_wd_down_counter > 0)// only respond once per timeout
              if(--comms_wd_down_counter <= 0)
              {
                /* timed out!*/
                comms_lost_flag=true;
              }
             
              if (response_chan_tx_queue.new_arrival_count>0)
              {   
                                               
               /* Point to next item in queue to send out: */ 
               tx_queue_bufp =(volatile char *)&response_chan_tx_queue.buf[response_chan_tx_queue.read_ndx][0];                  
               len=strlen((const char *)tx_queue_bufp);
                  
               /* Break out if there is nothing to send out */
               if (len==0)
               {
                 __NOP();//__no_operation();
                 break;
               }
               
               queue_entry_len  = (len > COMMS_MGR_TX_RESPONSE_BUF_LEN ? COMMS_MGR_TX_RESPONSE_BUF_LEN :len);

#ifdef ROSECOMB_MVP
               // Send over UART instead of USB if using Rosecomb
               if (exp_uart.enabled) {
                 
                 // Wait for previous transmit to finish (BLOCKING)
                 while(UART0_WriteIsBusy()) { 
                   __NOP();
                 }
                 
                 // Transmit data
                 UART0_Write((char*)tx_queue_bufp, queue_entry_len);
                 
                 // DEBUG: Print out TX buffer
                 //SYS_CONSOLE_PRINT("TX>%s\r\n", tx_queue_bufp);
                 
               } else {
#endif
                 usb_cdc_transmit((char*)tx_queue_bufp, queue_entry_len );
#ifdef ROSECOMB_MVP
               }
#endif               
              
               /* Perform queue maintenance: */
               if (ret_stat== USBD_OK)
               {
                  response_chan_tx_queue.read_ndx++;
                  if(response_chan_tx_queue.read_ndx>=COMMS_MGR_TX_CMD_BUF_ENTRIES)
                  {
                     response_chan_tx_queue.read_ndx=0;
                  }  
             
                  /* New arrival count, to decrement until all new arrivals are sent out */  
                  if (response_chan_tx_queue.new_arrival_count>0)//clamp to zero, 
                  {
                     response_chan_tx_queue.new_arrival_count--;  
                  }                                              
                 
                  /* Clear bad status count when get good status */
                  if (bad_tx_send_status_count!=0)
                  {
                     bad_tx_send_status_count=0;//can set a breakpoint here on this event
                  }
               }
               else
               {
                  /* To Do: do more error announcement */
                  bad_tx_send_status_count++;//Can set a breakpoint here on this event
               }    
          }
                        
          }/* End if new arrivals need to be sent out */

          break; 
                    
         default:
             /* to do: raise error: corrupt data:*/
             break;
     }/* End Switch */
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_out();
#endif      
     
}/* End Function */
/****************************************************************************
 * 
 * Function:  comms_mgr_get_next_msg
 * 
 * Description:  Polled by client, controller task.
 *
 *   Accesses/maintains rx receive queue for unread received 
 *   command (JSON), control chars { '~', '!', '%', '^d'} or Data/Command (Gcode), 
 *   or $text mode.
 *   Copies received data to provided buf and then erases the source msg entry
 *
 *  1. Special processing input arg: flag, control_only_flag:
 *   The function: _dispatch_kernel()is called in two places
 *   in the main controller loop: 
 *     i. In the first call, near the top of the loop, the flags arg is set 
 *        to control_only_flag=true. JSON and control chars are processed.
 *        Any Textmode or gcode mode content messages identified in first call
 *        are not processed, but instead are preserved (buffer indexing is managed appropriately)
 *        for processing during the second call. 
 *    ii. In the second call, toward the end of the loop, the flags are set to
 *        accept all messages. This is where Gcode and Text mode $<command> are
 *        intended to be processed, however no restriction is placed against control commands.

 *   2. Special buffer processing for single character command processing:
 *   Only one special control character command can be processed within a single cycle
 *   of the controller loop. These chars can be the first char in a message that is followed by gcode
 *
 *  
 *  Returns num bytes of msg or -1 for error
 *   Assumptions/Requirement: 
 *
 *****************************************************************************/
int comms_mgr_get_next_msg(char * client_bufp,  bool control_only_flag )
{   
  static int max_msg_len=0;
  volatile bool json_flag=false;
  volatile bool gcode_flag=false;
  volatile bool text_flag=false;
  bool preserve_buffer_flag=false;//needed if there is more content past a special control char
  volatile int i, j, len=0;  
  volatile int len_copy=0;
  char c;
  
  char *rx_bufptr=0;
  int active_entry_ndx = 0; //Assigned to either cmd_chan_rx_queue.read_ndx or a more recent arrival per the cmd_chan_rx_queue.write_ndx 
  j=0;
  if (control_only_flag==true) 
  {
      if (cmd_special_char_rx_queue.new_arrival_count>0) 
      {        
        cmd_special_char_rx_queue.new_arrival_count--;
        c=cmd_special_char_rx_queue.buf[cmd_special_char_rx_queue.read_ndx];
        client_bufp[0]=c;

        cmd_special_char_rx_queue.read_ndx++;
        if (cmd_special_char_rx_queue.read_ndx>COMMS_SPECIAL_CHAR_BUFLEN)
        {
          cmd_special_char_rx_queue.read_ndx=0;
        }
        return len=1;
    }   
    else if (json_chan_rx_queue.new_arrival_count>0)
    {
        SYS_INT_Disable();//added 12-14-2022        
        json_chan_rx_queue.new_arrival_count--;
        SYS_INT_Enable(); //added 12-14-2022     
       /* Init the buf pntr */
       rx_bufptr=(char *)&json_chan_rx_queue.buf[json_chan_rx_queue.read_ndx][0];
       /* clear out 0th element to indicate buffer was completely read */     
       len=strlen((const char *)rx_bufptr);      
       strncpy(client_bufp,rx_bufptr,len);
       json_chan_rx_queue.buf[json_chan_rx_queue.read_ndx][0]=0; //leave  some remnant for debugging
    
       /* Advance the queue entry ndx: */                     
       json_chan_rx_queue.read_ndx++;
            
       if( json_chan_rx_queue.read_ndx>=COMMS_MGR_RX_CMD_BUF_ENTRIES)
       {
          json_chan_rx_queue.read_ndx=0;
       }
       return len;       
    }
  }
  /* General processing which ends up being text or gcode since control chars and JSON are processed in the "control_only" condition above*/
  if ((cmd_chan_rx_queue.new_arrival_count>0)) // At least one new message has accumulated in the rx buf  
  {
      active_entry_ndx=cmd_chan_rx_queue.read_ndx;         
  
      /* Init the buf pntr */
      rx_bufptr=(char *)&cmd_chan_rx_queue.buf[active_entry_ndx][0];
      len=strlen((const char *)rx_bufptr);
      if (max_msg_len<len)
      {
        max_msg_len=len;//diagnostics
      }
      if (len>0)
      {      
          j=0;          
          for (i=0; i<len; i++)
          {             
             c = cmd_chan_rx_queue.buf[active_entry_ndx][i];               
             client_bufp[j++] = c;
             
             if (c=='{')//we must check for json here, in order to apply process of elimination, a broad-sweeping heuristic, 
                        //that all not-specifically control message content is non-control, aka text_mode|gcode_mode
             {
               json_flag=true;
             }

             /* Check for specific instances of all defined non-gcode "text" chars */
             else if (strchr("$?Hh", *rx_bufptr) != NULL)
             {
               text_flag=true;
             }
          
             /* Arriving here, must indicate Gcode is present: */
             else
             {
                  if (json_flag !=true)
                  {
                     gcode_flag=true;
                  }
                  else
                  {
                      __NOP();
                  }
             }
             if ((control_only_flag==true)/*This function is being called from the control-only section of the main controller loop, and content is not control */
             /* Preserve this present buffer for _dispatch_command function */      
             && (( text_flag==true)||(gcode_flag==true)))
             {
               preserve_buffer_flag = true;
               len = 0;
               break;
             }                       
          }/* End for */

    }//End if len>0 12-15-2022 place endif ahead of buffer maintenance, to correctly handle a len==0 msg
      
    /* Buffer will not be processed when control_only_flag==true
    * and non-control content (textmode or Gcode) has been received
    */          
    if (preserve_buffer_flag != true)
    {
       /* decrement only when new arrival will be processed [even len==0 msg), including gcode 
       or text that was parsed when control_only_flag ==true */           
      SYS_INT_Disable();
      cmd_chan_rx_queue.new_arrival_count--;  
#ifdef ROSECOMB_MVP
        // Assert RTS signal to Rosecomb once Chabo can handle two full sets of slots on command buffer
        if (exp_uart.enabled && (cmd_chan_rx_queue.new_arrival_count < (COMMS_MGR_RX_CMD_BUF_ENTRIES - (2 * COMMS_MGR_ROSECOMB_TX_ENTRIES)))) {
          EXP_UART_RTS_Set();
        }
#endif
      SYS_INT_Enable();

      /* clear out 0th element to indicate buffer was completely read */
      cmd_chan_rx_queue.buf[active_entry_ndx][0]=0; //leave  some remnant for debugging

      /* Advance the queue entry ndx: */                     
      cmd_chan_rx_queue.read_ndx++;

      if( cmd_chan_rx_queue.read_ndx>=COMMS_MGR_RX_CMD_BUF_ENTRIES)
      {
         cmd_chan_rx_queue.read_ndx=0;
      }
    }//End if preserve buffer flag
  }//End if cmd_chan_rx_queue.new_arrival_count>0
 
  len_copy=len;
  client_bufp[len]=0; //null terminate the msg
  return len;  
}/* End Function */
/*************************************************************************************
 * 
 * Function:  comms_mgr_receive_callback_helper
 *
 ************************************************************************************/
void comms_mgr_receive_callback_helper(uint8_t *buf, uint32_t len)
{
    static char c = 0;
    static int i=0;
    static int len_rx_buf= sizeof(cmd_chan_rx_queue.buf[0]);
    static int buf_start_ndx = 0;//used to concatenate incoming messages that have been cut into 64 byte max segments 
    bool newline_char_flag=false;
    int buf_ndx=0;
    bool json_char_flag=false;   

#define SINGLE_CHAR_CMD_LEN 2
    char prev_char = 0;
    /* Re-init the wd down counter to full count each time a message is received*/
    comms_wd_down_counter=comms_wd_downcount_start_value;//COMMS_WD_DOWNCOUNT_START_VALUE;
    comms_lost_flag=false;
    char *uppercase_str=strupr((char *) buf);

  
    /* Copy packet into receive buf: */
    for (i=0; i< len; i++)
    {      
         prev_char=c;       
         c=buf[i];       
         if (c=='{')
         {
            if (i==0)//12-14-2022  We only need true control-only JSON here, not when embedded in m100 gcode
            {
                json_char_flag=true;
                json_1st_char_instance_count++;
            }          
         }
  
      /* special control chars can be included in gcode messages but they must be processed in the "control" part of the main loop */
         switch (c)
         {                        
           case CHAR_ALARM_KILL_JOB:
           case CHAR_CYCLE_START:  
           case CHAR_FEEDHOLD:  
           case CHAR_QUEUE_FLUSH:  
           case CHAR_RESET:                       
             cmd_special_char_rx_queue.buf[ cmd_special_char_rx_queue.write_ndx]=c;
             cmd_special_char_rx_queue.write_ndx++;
             cmd_special_char_rx_queue.new_arrival_count++;
             if (cmd_special_char_rx_queue.write_ndx>COMMS_SPECIAL_CHAR_BUFLEN)
             {
               cmd_special_char_rx_queue.write_ndx=0;
             }
            return;
             
           break;                  
           case CHAR_WDOG: 
             //return;/* watchdog chars do not go any further than the rx interrupt*/
             break;                
         default:
           break;
         }                    
        if ((c== NEWLINE_CHAR)&& (prev_char == CHAR_WDOG)||(c==CHAR_WDOG))
        {
         /* Watchdog char has a newline char. Filter out this 'message' */
         return;
        }

       // Guard against rx buffer over-run */
       if (buf_start_ndx+buf_ndx < len_rx_buf-1 )
       { 
           /* Assign the next char into the rx buffer*/
           cmd_chan_rx_queue.buf[cmd_chan_rx_queue.write_ndx][buf_start_ndx+buf_ndx]=c;
    
           /* Mark instance of end of msg char: new_line== 0x0A, == '\n' */
           if((c== NEWLINE_CHAR)||(c== '\r')) 
           {  
              //7-29-2022-JSON message capture and copy to separate rx buf
              /* Null terminate this string on top of the new line */
              cmd_chan_rx_queue.buf[cmd_chan_rx_queue.write_ndx][buf_start_ndx+buf_ndx] = 0;//NULL;
                                
               if (json_char_flag==true)
               {
                   /* Copy this message to the json rx buf queue */
                  strcpy( &json_chan_rx_queue.buf[json_chan_rx_queue.write_ndx],&cmd_chan_rx_queue.buf[cmd_chan_rx_queue.write_ndx]);
                  
                  /* Note: Do not increment the write index here-- 
                   * we allow this buffer to be over written with the next non-json message
                   */
                  
                  /* Flag the new arrival for the commsMsg Task */
                  json_chan_rx_queue.new_arrival_count++;

                  /* Maintain circular rx queue */
                  json_chan_rx_queue.write_ndx++;
                  if (json_chan_rx_queue.write_ndx >= COMMS_MGR_RX_CMD_BUF_ENTRIES)
                  {
                      json_chan_rx_queue.write_ndx=0;
                  }  
               }
               else
               {
                    /* Flag the new arrival for the commsMsg Task */
                    cmd_chan_rx_queue.new_arrival_count++;
                    
                    /* Maintain circular rx queue */
                    cmd_chan_rx_queue.write_ndx++;          
                    if(cmd_chan_rx_queue.write_ndx >= COMMS_MGR_RX_CMD_BUF_ENTRIES)
                    {
                        cmd_chan_rx_queue.write_ndx=0;
                    }
#ifdef ROSECOMB_MVP
                    // Dessert RTS signal to Rosecomb if Chabo cannot handle a full set of command slots
                    SYS_INT_Disable();
                    if (cmd_chan_rx_queue.new_arrival_count >= (COMMS_MGR_RX_CMD_BUF_ENTRIES - COMMS_MGR_ROSECOMB_TX_ENTRIES)) {
                      EXP_UART_RTS_Clear();   
                    }
                    SYS_INT_Enable();
#endif
                  if(cmd_chan_rx_queue.buf[cmd_chan_rx_queue.write_ndx][0] !=0)
                  {     
                      /* latest msg has overwritten an earlier, unread msg*/
                      rpt_exception(STAT_COMMS_RX_QUEUE_OVERWRITTEN,(char*)"USB RX QUEUE Unread Msg[s] Overwritten!");
                  }
                   
               }

              buf_ndx=0;
              buf_start_ndx=0; 
              newline_char_flag = true;                
           } 
           else
           {
               buf_ndx++; 
           }
       }
       else//ran out of buffer! should never happen
       {
           rx_buf_overrun_instance_counter++;
          __NOP();//__no_operation();
       }
                
    }/* End For i....*/
    
    /* If there was not a new line character, there are more packets that comprise the present message.*/
    if (( newline_char_flag != true )||(buf_ndx>0))//buf_ndx>0) added 12-4-2020  ;
    {
        /* Advance the offset into the present buffer, to where the next packet will start*/
        buf_start_ndx += buf_ndx;//12-4-2020 len ;
    }      
}
 
