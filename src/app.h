/*
 * app.h
 *
 *  Created on: May 23, 2022
 *      Author: matt
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"
#include "system/console/sys_console.h"
#include "definitions.h"
#include "user.h"
#include "ctype.h"

#include "stepper.h"
#include "sk6812.h"
#include "bantam_hal.h"
#include "main.h"


// Definitions
#define USB_READ_BUFFER_SIZE    512
#define CDC_READBUF_SIZE        1024
#define CDC_WRITEBUF_SIZE       1024
#define SDCARD_MOUNT_NAME       SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0
#define SDCARD_DEV_NAME         SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0
#define SDCARD_FILE_NAME        "CHABO.txt"
#define SDCARD_DIR_NAME         "Dir1"
#define SDCARD_APP_DATA_LEN     512
#define FAN_PWM_MAX             3000
#define UART_RX_LEN             1     // Receive size in char
#define UART_RX_MSG_LEN         256
#define USER_CMD_COUNT          12    // Total number of user commands
#define USER_CMD_MAX_LEN        5
#define CMD_STR_LEN             30    // Total length of the command string
#define CMD_STR_MAX_ARGS        4     // Maximum number of command string arguments

// User command mapping
#define ESTP_CMD                0
#define ITR_CMD                 1
#define PRB_CMD                 2
#define FAN_CMD                 3
#define MOT_CMD                 4
#define LS_CMD                  5
#define SPD_CMD                 6
#define ESC_CMD                 7
#define SPOT_CMD                8
#define STAT_CMD                9
#define HELP_CMD                10
#define HELP2_CMD               11

// Type Definitions
typedef enum {
  APP_STATE_INIT=0,         // Applications state machine's initial state
  APP_STATE_SERVICE_TASKS,
  //TODO: Define states used by the application state machine
} APP_STATES;

typedef enum {
  USB_STATE_INIT=0,                   // USB state machine's initial state
  USB_STATE_WAIT_FOR_CONFIGURATION,   // USB waits for device configuration
  USB_STATE_SCHEDULE_READ,            // Wait for a character receive
  USB_STATE_WAIT_FOR_READ_COMPLETE,   // A character is received from host
  USB_STATE_SCHEDULE_WRITE,           // Wait for the TX to get completed
  USB_STATE_WAIT_FOR_WRITE_COMPLETE,  // Wait for the write to complete
  USB_STATE_ERROR                     // USB error state
} USB_STATES;

typedef enum {
  SD_MOUNT_WAIT = 0,      // The SD app waits for SD card to be mounted
  SD_SET_CURRENT_DRIVE,   // Set the current drive
  SD_OPEN_FIRST_FILE,     // The SD app opens the file to read
  SD_CREATE_DIRECTORY,    // Create directory
  SD_OPEN_SECOND_FILE,    // The SD app opens the file to write
  SD_READ_WRITE_TO_FILE,  // The SD app reads from a file and writes to another file
  SD_CLOSE_FILE,          // The SD app closes the file
  SD_IDLE,                // The SD app closes the file and idles
  SD_ERROR                // An SD app error has occurred
} SD_STATES;

typedef struct {
  APP_STATES state;
  SYS_CONSOLE_HANDLE consoleHandle;
} APP_DATA;

typedef struct {
  USB_DEVICE_HANDLE deviceHandle;                     // Device layer handle returned by device layer open function
  USB_STATES state;                                   // USB current state
  USB_CDC_LINE_CODING setLineCodingData;              // Set line coding data
  bool isConfigured;                                  // Device configured state
  USB_CDC_LINE_CODING getLineCodingData;              // Get line coding data
  USB_CDC_CONTROL_LINE_STATE controlLineStateData;    // Control line state
  USB_DEVICE_CDC_TRANSFER_HANDLE readTransferHandle;  // Read transfer handle
  USB_DEVICE_CDC_TRANSFER_HANDLE writeTransferHandle; // Write transfer handle
  bool isReadComplete;                                // True if a character was read
  bool isWriteComplete;                               // True if a character was written
  bool sofEventHasOccurred;                           // Flag determines SOF event occurrence
  uint16_t breakData;                                 // Break data
  uint8_t * cdcReadBuffer;                            // Application CDC read buffer
  uint8_t * cdcWriteBuffer;                           // Application CDC write buffer
  uint32_t numBytesRead;                              // Number of bytes read from host 
} USB_APP_DATA;

typedef struct {
  SYS_FS_HANDLE fileHandle;       // SYS_FS file handle for 1st file
  SYS_FS_HANDLE fileHandle1;      // SYS_FS file handle for 2nd file
  SD_STATES state;                // SD card's current state
  int32_t nBytesRead;
  volatile bool sdCardMountFlag;  // Flag to indicate SD card mount status
} SD_APP_DATA;

// Function Prototypes
uint8_t is_uint(char *);
uint8_t is_pos_float(char *);
uint8_t is_input_valid(int, char **, const int *);
uint8_t is_finput_valid(int, char **, const float *);
void gpio_init(void);
void uart_init(void);
void uart_mgr_task_matt(void);
void halo_init(void);
bool usb_state_reset(void);
void usb_cdc_init(void);
void usb_cdc_task(void);
void sd_card_init(void);
void sd_card_task(void);
void start_systime_interrupt(void);
void check_and_update_chabo_pid(void);

void parse_user_command(char *);
#ifdef __cplusplus
extern "C" void APP_Initialize(void);
extern "C" void APP_Tasks(void);
extern "C" int usb_cdc_transmit(char * bufp, int len);
#else
void APP_Initialize(void);
void APP_Tasks(void);
int usb_cdc_transmit(char * bufp, int len);
#endif

#endif /* INC_APP_H */
