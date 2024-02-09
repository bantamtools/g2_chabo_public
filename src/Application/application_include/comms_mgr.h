/******************************************************************
*
* File: comms_mgr.h
*
* @brief: 
*
******************************************************************/
#ifndef _COMMS_MGR_H
#define	_COMMS_MGR_H

#include "main.h"
#include "user_types.h"
#include "globdefs.h"
#include "g2core.h"
#include "g2_config.h"
#include "bantam_hal.h"

#define APPLY_M100_EXCEPTION
#define SEPARATE_JSON_RX_QUEUE
#define COMMS_MIN_WD_TIMEOUT_SECS 4//3
#define COMMS_MAX_WD_TIMEOUT_SECS 255
#define COMMS_DEFAULT_WD_TIME_SECS COMMS_MIN_WD_TIMEOUT_SECS

//based on kernel 30 ms scheduling interval of comms task to approximate the configured watchdog timeout in seconds
#define COMMS_TASK_SCHED_FREQUENCY 33// based on 30 ms scheduling periods, Frequency is 33/second THIS MUST ALWAYS EXACTLY MATCH ACTUAL KERNEL's SCHEDULING INTERVAL!
#define COMMS_WD_DOWNCOUNT_START_VALUE (COMMS_DEFAULT_WD_TIME_SECS*COMMS_TASK_SCHED_FREQUENCY)
#define COMMS_MGR_STARTUP_DELAY_TIME_MS 10
#define COMMS_MGR_FIRST_RX_SR_MSG_NDX 2//third msg received at startup is the status report setup message

/* Define safe/realistic rx command (JSON) and data (Gcode) packet buffer lengths */
#define COMMS_MGR_RX_CMD_BUF_LEN 512//ARBITRARY, ADJUST AS NEEDED
#ifdef ROSECOMB_MVP
#define COMMS_MGR_ROSECOMB_TX_ENTRIES 30                                   // Number of buffer slots on Rosecomb TX side
#define COMMS_SPECIAL_CHAR_BUFLEN     (10 + COMMS_MGR_ROSECOMB_TX_ENTRIES) // Be sure can hold a full set of Rosecomb TX slots
#define COMMS_MGR_RX_CMD_BUF_ENTRIES  (60 + COMMS_MGR_ROSECOMB_TX_ENTRIES) // Be sure can hold a full set of Rosecomb TX slots

#define COMMS_MGR_TX_RESPONSE_BUF_LEN 512//ARBITRARY, ADJUST AS NEEDED
#define COMMS_MGR_TX_CMD_BUF_ENTRIES  30                                   // Number of buffer slots on Rosecomb TX side
#else
#define COMMS_SPECIAL_CHAR_BUFLEN 10//arbitrarilly larger than expected arrivals
#define COMMS_MGR_RX_CMD_BUF_ENTRIES 60//cd 30//ARBITRARY, ADJUST AS NEEDED

#define COMMS_MGR_TX_RESPONSE_BUF_LEN 512//ARBITRARY, ADJUST AS NEEDED
#define COMMS_MGR_TX_CMD_BUF_ENTRIES 50//ARBITRARY, ADJUST AS NEEDED
#endif
#define PRIORITY_NDX_UNASSIGNED -1

/* Allocate data for uart */
#define UART_MAX_BUFLEN 512
#define UART_RXLEN 1//ST RX INTERRUPT MUST BE RE-INIT'D ON EACH RECEPTION OF A SINGLE BYTE
#define UART7_TX_RESPONSE_BUF_LEN 2048//ARBITRARY, ADJUST AS NEEDED
#define UART7_TX_BUF_SIZE UART7_TX_RESPONSE_BUF_LEN
#define UART7_RX_BUF_SIZE UART_MAX_BUFLEN

#define UART3_TX_BUF_SIZE UART_MAX_BUFLEN
#define UART3_RX_BUF_SIZE UART_MAX_BUFLEN

#define PAD_BYTES 40

typedef enum
{
  UART_MSG_MODE_TEXT,
  UART_MSG_MODE_BINARY,
  UART_MSG_MODE_COUNT
}UartMsgModeE;

 typedef struct
 {
   uint8_t ready_flag;
   uint8_t buf[UART_MAX_BUFLEN];
   int16_t ndx;
   int16_t len;  
 }UartRxBufInfoS;
 /* Define and allocate a tx response queue for sending out echoes of modbus read/write requests and responses*/

#define UART7_TX_CMD_BUF_ENTRIES 10//ARBITRARY, ADJUST AS NEEDED
typedef struct
{
  uint8_t buf[UART7_TX_CMD_BUF_ENTRIES][UART7_TX_RESPONSE_BUF_LEN];
  int write_ndx;//producer owns this
  int read_ndx;//consumer owns this 
  int new_arrival_count;// producer increments, consumer decrements this
  uint8_t new_arrival_flag;//set by producer e.g JSON response, reset by consumer, comms_mgr task     
}Uart7ResponseInfoQueueS;

extern  Uart7ResponseInfoQueueS uart7_tx_queue;
extern  UartRxBufInfoS uart3_rx,uart7_rx; 
extern uint8_t uart7_tx_buf[UART7_TX_BUF_SIZE+PAD_BYTES];

#ifdef ROSECOMB_MVP
#define ROSECOMB_DETECTED   0
#define EXP_UART_BUF_SIZE   COMMS_MGR_RX_CMD_BUF_LEN
#define EXP_UART_NUM_BUFS   COMMS_MGR_RX_CMD_BUF_ENTRIES
#define EXP_UART_TIMEOUT_MS 25

typedef struct
{
    uint8_t buf[EXP_UART_BUF_SIZE];
    int len;
    bool rdy;
} UartBufferType;

typedef struct
{
  bool enabled;
  int write_ndx;
  int read_ndx; 
  uint32_t start_time;
  UartBufferType rx[EXP_UART_NUM_BUFS];
}UartIntfType;

bool uart_mgr_exp_uart_connected(void);
#endif

/* COMMS_MGR_DEPLOY_GROUP_PRINTING*/
int comms_mgr_start_multiline_print(void);
int comms_mgr_end_multiline_print(void);
void comms_mgr_receive_callback_helper(uint8_t *buf, uint32_t len);
void comms_mgr_init(void);
void comms_mgr_task(void);
int comms_mgr_get_next_msg(char *client_bufp, bool control_only_flag);
int comms_mgr_replace_fprintf(char * fmt_str, int len);
int comms_mgr_write_msg(char * client_bufp);
extern char *comms_mgr_stderr_bufptr;
uint16_t usb_process_single_char_cmd(uint8_t *buffer,size_t size);
bool comms_mgr_usb_connected(void);
void comms_mgr_set_state_idle(void);
void comms_mgr_set_state_run(void);
bool comms_wdog_enabled(void);    
void comms_mgr_assign_cmd_flush_ndx(void);
void comms_print_wdog(nvObj_t *nv); 
stat_t comms_get_wdog(nvObj_t *nv);
stat_t comms_set_wdog(nvObj_t *nv);
void comms_print_wdtimeout(nvObj_t *nv);
stat_t comms_get_wdog_timeout(nvObj_t *nv); 
stat_t comms_set_wdog_timeout(nvObj_t *nv);
bool comms_timed_out(void);
bool usb_comms_disconnected(void);
void comms_mgr_setUSBAsConnectedAndReady(void );
bool comms_mgr_isUSBConnected(void );
bool comms_mgr_isUSBNotConnected(void );
bool comms_mgr_isReady(void );
void comms_mgr_connectedStateChanged(bool connected);
void comms_mgr_clearUSBFlags(void );
void comms_mgr_flush(void);
void comms_mgr_flushRead(void);
void comms_mgr_assign_cmd_flush_ndx(void);//10-14-2021
void comms_mgr_flushToCommand(void);//int special_ndx);
void comms_mgr_enable_usb_interrupt(void);
void comms_mgr_disable_usb_interrupt(void);
void comms_mgr_sol2_conflict_kill_job(void);
void comms_uart7_send_text_msg(uint8_t *msg, uint16_t len);
int comms_uart_debug_print(char * msg);
void comms_uart_init_usart3(void);
void uart_mgr_task(void); //TEMP

#define USB_FS_MAX_PKT_SIZE 64
#define STDERR_BUF comms_mgr_stderr_bufptr

#endif	/*_COMMS_MGR_H */
