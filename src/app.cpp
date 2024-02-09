/*
 * app.c
 *
 *  Created on: May 23, 2022
 *      Author: matt
 */

#include "app.h"
#include "system_bantam.h"
#include "comms_mgr.h"

extern "C" void App_Initialize(void);
extern "C" void App_Tasks(void);
// User command variables
const char USER_COMMANDS[USER_CMD_COUNT][USER_CMD_MAX_LEN] = {
  "estp",
  "itr",
  "prb",
  "fan",
  "mot",
  "ls",
  "spd",
  "esc",
  "spot",
  "stat",
  "help",
  "?"
};

// USB, SD card and user command buffers
uint8_t cdcReadBuffer[CDC_READBUF_SIZE];
uint8_t cdcWriteBuffer[CDC_WRITEBUF_SIZE];
uint8_t BUFFER_ATTRIBUTES dataBuffer[SDCARD_APP_DATA_LEN];
char user_command_str[UART_RX_MSG_LEN];

// UART/Modbus variables
typedef struct {
  uint8_t msg_ready_flag;
  unsigned char buf[UART_RX_MSG_LEN];
  int ndx;
  int len;
} uart_rx_type;

uart_rx_type uart_rx;

// LED variables
uint16_t led0_count = 0;
uint16_t led1_count = 0;

// Application data structures for App, USB and SD card
APP_DATA appData;
USB_APP_DATA usbAppData;
SD_APP_DATA sdAppData;

// LED strip array:two pwm-driven LED device types:  
extern PwmLedDeviceInfoS pwm_led_device_info[PWM_LED_DEVICE_COUNT];
extern void SYS_Tasks_restricted ( void );
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
//wrapper for comms_mgr_task to write
int usb_cdc_transmit(char * bufp, int len)
{
   
    int result =
    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
              &usbAppData.writeTransferHandle,
              bufp, len,
              USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
    return result;
}
// Heartbeat timer interrupt callback
void TC1_CH0_TimerInterruptHandler(TC_TIMER_STATUS status, uintptr_t context)
{
#if 1//sme substitute heartbeat leds with bantam system time callback
    systick_interrupt_callback_helper(); 
#else //original, matt's
  // Toggle LED0 every 0.5s, LED1 every 1s for heartbeat
  if (++led0_count == 500) {
   GPIO_DBG0_Toggle2();// GPIO_DBG0_Toggle();
    led0_count = 0;
  }
  if (++led1_count == 1000) {
    led1_count = 0;
    GPIO_DBG1_Toggle2();// GPIO_DBG1_Toggle();
  }
#endif
}

// USB CDC device events - application event handler
USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
  USB_DEVICE_CDC_INDEX index,
  USB_DEVICE_CDC_EVENT event,
  void * pData,
  uintptr_t userData
)
{
  USB_APP_DATA * appDataObject;
  USB_CDC_CONTROL_LINE_STATE * controlLineStateData;
  USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE * eventDataRead;

  appDataObject = (USB_APP_DATA *)userData;

  switch(event)
  {
    case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

      /* This means the host wants to know the current line
       * coding. This is a control transfer request. Use the
       * USB_DEVICE_ControlSend() function to send the data to
       * host.  */

      USB_DEVICE_ControlSend(appDataObject->deviceHandle,
              &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

      break;

    case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

      /* This means the host wants to set the line coding.
       * This is a control transfer request. Use the
       * USB_DEVICE_ControlReceive() function to receive the
       * data from the host */

      USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
              &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

      break;

    case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

      /* This means the host is setting the control line state.
       * Read the control line state. We will accept this request
       * for now. */

      controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
      appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
      appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

      USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

      break;

    case USB_DEVICE_CDC_EVENT_SEND_BREAK:

      /* This means that the host is requesting that a break of the
       * specified duration be sent. Read the break duration */

      appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *)pData)->breakDuration;

      /* Complete the control transfer by sending a ZLP  */
      USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

      break;

    case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

      /* This means that the host has sent some data*/
      eventDataRead = (USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE *)pData;
      appDataObject->isReadComplete = true;
      appDataObject->numBytesRead = eventDataRead->length; 
      break;

    case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

      /* The data stage of the last control transfer is
       * complete. For now we accept all the data */

      USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
      break;

    case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

      /* This means the GET LINE CODING function data is valid. We don't
       * do much with this data in this demo. */
      break;

    case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

      /* This means that the data write got completed. We can schedule
       * the next read. */

      appDataObject->isWriteComplete = true;
      break;

    default:
      break;
  }

  return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

// Application USB device layer event handler
void APP_USBDeviceEventHandler 
(
  USB_DEVICE_EVENT event, 
  void * eventData, 
  uintptr_t context 
)
{
  USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

  switch(event)
  {
    case USB_DEVICE_EVENT_SOF:

      usbAppData.sofEventHasOccurred = true;

      break;

    case USB_DEVICE_EVENT_RESET:

      usbAppData.isConfigured = false;

      break;

    case USB_DEVICE_EVENT_CONFIGURED:

      /* Check the configuration. We only support configuration 1 */
      configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;

      if ( configuredEventData->configurationValue == 1)
      {

          /* Register the CDC Device application event handler here.
           * Note how the appData object pointer is passed as the
           * user data */

          USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t)&usbAppData);

          /* Mark that the device is now configured */
          usbAppData.isConfigured = true;
      }

      break;

    case USB_DEVICE_EVENT_POWER_DETECTED:

      /* VBUS was detected. We can attach the device */
      USB_DEVICE_Attach(usbAppData.deviceHandle);

      break;

    case USB_DEVICE_EVENT_POWER_REMOVED:

      /* VBUS is not available any more. Detach the device. */
      USB_DEVICE_Detach(usbAppData.deviceHandle);

      break;

    case USB_DEVICE_EVENT_SUSPENDED:

      break;

    case USB_DEVICE_EVENT_RESUMED:
    case USB_DEVICE_EVENT_ERROR:
    default:

      break;
  }
}

// Application SD card layer event handler
static void APP_SysFSEventHandler(SYS_FS_EVENT event,void* eventData,uintptr_t context)
{
  switch(event)
  {
    /* If the event is mount then check if SDCARD media has been mounted */
    case SYS_FS_EVENT_MOUNT:
      if(strcmp((const char *)eventData, SDCARD_MOUNT_NAME) == 0)
      {
          sdAppData.sdCardMountFlag = true;
      }
      break;

    /* If the event is unmount then check if SDCARD media has been unmount */
    case SYS_FS_EVENT_UNMOUNT:
      if(strcmp((const char *)eventData, SDCARD_MOUNT_NAME) == 0)
      {
          sdAppData.sdCardMountFlag = false;

          sdAppData.state = SD_MOUNT_WAIT;

      }

      break;

    case SYS_FS_EVENT_ERROR:
    default:
      break;
  }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

// Checks if string is an unsigned integer
uint8_t is_uint(char *s) {
  for (int i = 0; i < strlen(s); i++) {
    if (s[i] < 48 || s[i] > 57) return false; // Look for digits 0-9
  }
  return true;
}

// Checks if string is a float
uint8_t is_pos_float(char *s) {
  for (int i = 0; i < strlen(s); i++) {
    if (((s[i] < 48 || s[i] > 57) && s[i] != 46) || (s[i] == 46 && i == 0)) return false; // Look for digits 0-9 and decimal point (exclude decimal in first position)
  }
  return true;
}

// Sanity checks input command arguments within the given integer bounds
uint8_t is_input_valid(int argc, char **argv, const int *bounds) {

  // Iterate through the list of arguments
  for (int i = 0; i < argc; i++) {

    // Check for null, non-integer arguments and within given bounds
    if (!argv[i + 1] || !is_uint(argv[i + 1]) || atoi(argv[i + 1]) < bounds[2 * i] || atoi(argv[i + 1]) > bounds[(2 * i) + 1]) {
      SYS_CONSOLE_PRINT("ERROR: Missing or invalid argument(s)\r\n");
      return false;
    }
  }

  // Input valid, return
  return true;
}

// Sanity checks input command arguments within the given float bounds
uint8_t is_finput_valid(int argc, char **argv, const float *bounds) {

  // Iterate through the list of arguments
  for (int i = 0; i < argc; i++) {

    // Check for null, non-float arguments and within given bounds
    if (!argv[i + 1] || !is_pos_float(argv[i + 1]) || atof(argv[i + 1]) < bounds[2 * i] || atof(argv[i + 1]) > bounds[(2 * i) + 1]) {
      SYS_CONSOLE_PRINT("ERROR: Missing or invalid argument(s)\r\n");
      return false;
    }
  }

  // Input valid, return
  return true;
}
void start_systime_interrupt(void)
{
   // Start 1ms tick for LED heartbeat
  TC1_CH0_TimerCallbackRegister(TC1_CH0_TimerInterruptHandler, (uintptr_t)NULL);
  TC1_CH0_TimerStart();   
}
// Initializes the debug GPIOs
void gpio_init(void) 
{

  // Turn off LEDs
  GPIO_DBG0_Clear();
  GPIO_DBG1_Clear();
}

// Initializes the UART/Modbus
void uart_init(void) 
{
  // Initialize the debug UART variables
  memset((void*)&uart_rx, 0, sizeof(uart_rx));
  memset((void*)&user_command_str, 0, sizeof(user_command_str));

  // Clear the ready flag
  uart_rx.msg_ready_flag = false;
  
  // Get the system console handle
   appData.consoleHandle = SYS_CONSOLE_HandleGet(SYS_CONSOLE_INDEX_0);
}

// UART management task
void uart_mgr_task_matt(void)
{
  // Read a character if available
  if (SYS_CONSOLE_Read(appData.consoleHandle, &uart_rx.buf[uart_rx.ndx], UART_RX_LEN) > 0) {
    
    // Local echo
    SYS_CONSOLE_PRINT("%c", uart_rx.buf[uart_rx.ndx]);
    
    // Below logic assumes callback is executed on every received char
    switch(uart_rx.buf[uart_rx.ndx++]) 
    {
      case NEWLINE_CHAR:
      case NULL_CHAR:
      case CR_CHAR:
        uart_rx.msg_ready_flag = true;
        uart_rx.len = uart_rx.ndx + 1;
        uart_rx.ndx = 0;
        break;
    }
    //3-30-2023; range check ndx!
      if(uart_rx.ndx >= UART_RX_MSG_LEN)
      {
        uart_rx.ndx=0;  
      }      
    // Received a command string, parse it
    if (uart_rx.msg_ready_flag == true) {
      uart_rx.msg_ready_flag = false;
      memset((void*)&user_command_str, 0, sizeof(user_command_str));
      memcpy(user_command_str, uart_rx.buf, uart_rx.len);
      parse_user_command(user_command_str);
    }
  }
}

// Initialize the halo
void halo_init(void) 
{
   // Reset the LEDs back to black (off)
  //pwm_led_set_all_RGBW(0x00, 0x00, 0x00, 0x00);
  //pwm_led_render();

}

bool usb_state_reset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(usbAppData.isConfigured == false)
    {
        usbAppData.state = USB_STATE_WAIT_FOR_CONFIGURATION;
        usbAppData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        usbAppData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        usbAppData.isReadComplete = true;
        usbAppData.isWriteComplete = true;
        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}

// Initializes the USB CDC subsystem
void usb_cdc_init(void) 
{ 
  /* Place the USB App state machine in its initial state. */
  usbAppData.state = USB_STATE_INIT;
  
  /* Device Layer Handle  */
  usbAppData.deviceHandle = USB_DEVICE_HANDLE_INVALID ;

  /* Device configured status */
  usbAppData.isConfigured = false;

  /* Initial get line coding state */
  usbAppData.getLineCodingData.dwDTERate = 115200;//8-15-2022 9600;   
  usbAppData.getLineCodingData.bParityType = 0;
  usbAppData.getLineCodingData.bParityType = 0;
  usbAppData.getLineCodingData.bDataBits = 8;

  /* Read Transfer Handle */
  usbAppData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

  /* Write Transfer Handle */
  usbAppData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

  /* Initialize the read complete flag */
  usbAppData.isReadComplete = true;

  /*Initialize the write complete flag*/
  usbAppData.isWriteComplete = true;

  /* Reset other flags */
  usbAppData.sofEventHasOccurred = false;

  /* Set up the read buffer */
  usbAppData.cdcReadBuffer = &cdcReadBuffer[0];

  /* Set up the read buffer */
  usbAppData.cdcWriteBuffer = &cdcWriteBuffer[0]; 
}


// USB CDC management task
void usb_cdc_task(void) 
{ 
  /* Update the application state machine based
   * on the current state */
  int i;

  switch(usbAppData.state)
  {
    case USB_STATE_INIT:

      /* Open the device layer */
      usbAppData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

      if(usbAppData.deviceHandle != USB_DEVICE_HANDLE_INVALID)
      {
        /* Register a callback with device layer to get event notification (for end point 0) */
        USB_DEVICE_EventHandlerSet(usbAppData.deviceHandle, APP_USBDeviceEventHandler, 0);

        usbAppData.state = USB_STATE_WAIT_FOR_CONFIGURATION;
      }
      else
      {
        /* The Device Layer is not ready to be opened. We should try
         * again later. */
      }

      break;

    case USB_STATE_WAIT_FOR_CONFIGURATION:

      /* Check if the device was configured */
      if(usbAppData.isConfigured)
      {
        /* If the device is configured then lets start reading */
        usbAppData.state = USB_STATE_SCHEDULE_READ;
      }

      break;

    case USB_STATE_SCHEDULE_READ:

      if(usb_state_reset())
      {
        __NOP();
        break;
      }

      /* If a read is complete, then schedule a read
       * else wait for the current read to complete */

      usbAppData.state = USB_STATE_WAIT_FOR_READ_COMPLETE;
      if(usbAppData.isReadComplete == true)
      {
        usbAppData.isReadComplete = false;
        usbAppData.readTransferHandle =  USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

        USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
                &usbAppData.readTransferHandle, usbAppData.cdcReadBuffer,
                USB_READ_BUFFER_SIZE);

        if(usbAppData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)
        {
          usbAppData.state = USB_STATE_ERROR;
          break;
        }
      }

      break;

    case USB_STATE_WAIT_FOR_READ_COMPLETE:

      if(usb_state_reset())
      {
          __NOP();
        break;
      }

      /* Check if a character was received the isReadComplete flag gets 
       * updated in the CDC event handler. */

      if(usbAppData.isReadComplete)
      {
#if 1//sme:send data to bantam app layer
       comms_mgr_receive_callback_helper(usbAppData.cdcReadBuffer, usbAppData.numBytesRead);
       usbAppData.state = USB_STATE_SCHEDULE_READ;
       break;
      }
#else //original
        usbAppData.state = USB_STATE_SCHEDULE_WRITE;
//#endif        
      }

      break;


    case USB_STATE_SCHEDULE_WRITE:

      if(usb_state_reset())
      {
        break;
      }

      /* Setup the write */
      usbAppData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
      usbAppData.isWriteComplete = false;
      usbAppData.state = USB_STATE_WAIT_FOR_WRITE_COMPLETE;

      /* Else echo each received character by adding 1 [sme]--no, just echo what was sent */
      for(i = 0; i < usbAppData.numBytesRead; i++)
      {
        if((usbAppData.cdcReadBuffer[i] != 0x0A) && (usbAppData.cdcReadBuffer[i] != 0x0D))
        {
          usbAppData.cdcWriteBuffer[i] = usbAppData.cdcReadBuffer[i];//[sme] just echo what was sent + 1;
        }
        usbAppData.cdcWriteBuffer[usbAppData.numBytesRead]='\n';//sme: prevent stray chars at end
        usbAppData.cdcWriteBuffer[usbAppData.numBytesRead+1]='\n';//sme: prevent stray chars at end
      }
      USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
              &usbAppData.writeTransferHandle,
              usbAppData.cdcWriteBuffer, usbAppData.numBytesRead,
              USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);

      break;

    case USB_STATE_WAIT_FOR_WRITE_COMPLETE:

      if(usb_state_reset())
      {
        break;
      }

      /* Check if a character was sent. The isWriteComplete
       * flag gets updated in the CDC event handler */

      if(usbAppData.isWriteComplete == true)
      {
        usbAppData.state = USB_STATE_SCHEDULE_READ;
      }

      break;
#endif
    case USB_STATE_ERROR:
    default:

      break;
  }

}

// Initializes the SD card subsystem
void sd_card_init(void) 
{
  /* Place the App state machine in its initial state. */
  sdAppData.state = SD_MOUNT_WAIT;

  /* Register the File System Event handler */
  SYS_FS_EventHandlerSet((void const*)APP_SysFSEventHandler,(uintptr_t)NULL);
}

// SD card management task
void sd_card_task(void) 
{  
  /* Check the application's current state. */
  switch ( sdAppData.state )
  {
    case SD_MOUNT_WAIT:
      /* Wait for SDCARD to be Auto Mounted */
      if(sdAppData.sdCardMountFlag == true)
      {
        SYS_CONSOLE_PRINT("SD Card Test Starting...\r\n");
        sdAppData.state = SD_SET_CURRENT_DRIVE;
      }
      break;

    case SD_SET_CURRENT_DRIVE:
      if(SYS_FS_CurrentDriveSet(SDCARD_MOUNT_NAME) == SYS_FS_RES_FAILURE)
      {
        /* Error while setting current drive */
        sdAppData.state = SD_ERROR;
      }
      else
      {
        /* Open a file for reading. */
        sdAppData.state = SD_OPEN_FIRST_FILE;
      }
      break;

    case SD_OPEN_FIRST_FILE:
      sdAppData.fileHandle = SYS_FS_FileOpen(SDCARD_FILE_NAME,
              (SYS_FS_FILE_OPEN_READ));
      if(sdAppData.fileHandle == SYS_FS_HANDLE_INVALID)
      {
        /* Could not open the file. Error out*/
        SYS_CONSOLE_PRINT("SD ERROR: Could not open file.  Did you create CHABO.txt on the SD?\r\n");
        sdAppData.state = SD_ERROR;
      }
      else
      {
        /* Create a directory. */
        sdAppData.state = SD_CREATE_DIRECTORY;
      }
      break;

    case SD_CREATE_DIRECTORY:
      /* Delete the files under Dir1 directory (if any) */
      SYS_FS_FileDirectoryRemove(SDCARD_DIR_NAME "/" SDCARD_FILE_NAME);

      /* Delete the Dir1 directory if it exists */
      SYS_FS_FileDirectoryRemove(SDCARD_DIR_NAME);

      if(SYS_FS_DirectoryMake(SDCARD_DIR_NAME) == SYS_FS_RES_FAILURE)
      {
        /* Error while setting current drive */
        sdAppData.state = SD_ERROR;
      }
      else
      {
        /* Open a second file for writing. */
        sdAppData.state = SD_OPEN_SECOND_FILE;
      }
      break;

    case SD_OPEN_SECOND_FILE:
      /* Open a second file inside "Dir1" */
      sdAppData.fileHandle1 = SYS_FS_FileOpen(SDCARD_DIR_NAME "/" SDCARD_FILE_NAME,
              (SYS_FS_FILE_OPEN_WRITE));

      if(sdAppData.fileHandle1 == SYS_FS_HANDLE_INVALID)
      {
        /* Could not open the file. Error out*/
        sdAppData.state = SD_ERROR;
      }
      else
      {
        /* Read from one file and write to another file */
        sdAppData.state = SD_READ_WRITE_TO_FILE;
      }

    case SD_READ_WRITE_TO_FILE:

      sdAppData.nBytesRead = SYS_FS_FileRead(sdAppData.fileHandle, (void *)dataBuffer, SDCARD_APP_DATA_LEN);

      if (sdAppData.nBytesRead == -1)
      {
        /* There was an error while reading the file.
         * Close the file and error out. */

        SYS_FS_FileClose(sdAppData.fileHandle);
        sdAppData.state = SD_ERROR;
      }
      else
      {
        /* If read was success, try writing to the new file */
        if(SYS_FS_FileWrite(sdAppData.fileHandle1, (const void *)dataBuffer, sdAppData.nBytesRead) == -1)
        {
          /* Write was not successful. Close the file
           * and error out.*/
          SYS_FS_FileClose(sdAppData.fileHandle1);
          sdAppData.state = SD_ERROR;
        }
        else if(SYS_FS_FileEOF(sdAppData.fileHandle) == 1)    /* Test for end of file */
        {
          /* Continue the read and write process, until the end of file is reached */

          sdAppData.state = SD_CLOSE_FILE;
        }
      }
      break;

    case SD_CLOSE_FILE:
      /* Close both files */
      SYS_FS_FileClose(sdAppData.fileHandle);
      SYS_FS_FileClose(sdAppData.fileHandle1);

      /* The test was successful. Lets idle. */
      SYS_CONSOLE_PRINT("SD Card Test Completed!\r\n");
      sdAppData.state = SD_IDLE;
      break;

    case SD_IDLE:
      /* The application comes here when the demo has completed successfully.*/
      break;

    case SD_ERROR:
      /* The application comes here when the demo has failed. */
      break;

    default:
      break;
    }
}

// Checks and updates the Chabo product ID
void check_and_update_chabo_pid(void) {

  uint32_t pid = CHABO_PID;
  uint32_t address = CHABO_PID_ADDRESS;
            
  while(EFC_IsBusy());
 
  //uint32_t read_val;
  //memcpy(&read_val, (void *)address, sizeof(read_val));
  //SYS_CONSOLE_PRINT("Chabo Product ID -> %d\r\n", read_val);
    
  // Update product ID if register does not match
  if (memcmp(&pid, (void *)address, sizeof(&pid)) != 0) {
    
    //SYS_CONSOLE_PRINT("*** UPDATING PID!!! ***\r\n");
      
    // Erase the sector
    EFC_SectorErase(address);
      
    while(EFC_IsBusy());
      
    // Program product ID into page
    EFC_PageWrite(&pid, address);
      
  }  
}

// User command parser
void parse_user_command(char *buf) 
{
 
#define INITIAL_PWM_DUTY_CYCLE (float)0.135
#define PWM_DUTY_STEP_INCREMENT 0.10
  static float spindle_pwm_duty_cycle_fraction = INITIAL_PWM_DUTY_CYCLE;
  static float spindle_pwm_duty_cycle_increment =PWM_DUTY_STEP_INCREMENT;
  static bool vary_the_duty_cycle_flag=true;
  int len = strlen(buf);
  int i, j;
  char cmd_str[CMD_STR_LEN];
  char *strp = 0;
  char *arg[CMD_STR_MAX_ARGS + 1] = {0, 0, 0};
  uint8_t match_found_flag = false;
#define MAX_BOUNDS_CHECK_ELEMENTS 8//sme: changing file to .cpp wrecks C language feature, here is a workaround
  int bounds_check_array[MAX_BOUNDS_CHECK_ELEMENTS]={0};
  // Sanity length check, truncate
  if (len > CMD_STR_LEN) 
  {
    len = CMD_STR_LEN;
  }

  // Force all chars to lower case
  for (i = 0; i < len; i++) 
  {
    cmd_str[i] = tolower(buf[i]);
  }

  // Identify the command set
  for (i = 0; i < USER_CMD_COUNT; i++) 
  {
    // Check command string against current token
    strp = strstr(cmd_str, USER_COMMANDS[i]);

    // Found a command
    if (strp != NULL) 
    {
      // Flag a match
      match_found_flag = 1;

      // Grab any arguments (max four for current command set)
      arg[0] = strtok(cmd_str, " \r\n");
      for(j = 0; arg[j] != NULL && j < CMD_STR_MAX_ARGS; j++) 
      {
        arg[j + 1] = strtok(NULL, " \r\n");
      }

      // Process user command
      switch (i) 
      {
       // Read/print e-stop input
        case ESTP_CMD: SYS_CONSOLE_PRINT("E-Stop = %u\r\n", ESTOP_Get()); break;

        // Read/print all interlock inputs
        case ITR_CMD:
          SYS_CONSOLE_PRINT("Interlock Loop 0 = %u\r\n", ITRLK_LOOP0_Get());
          SYS_CONSOLE_PRINT("Interlock Loop 1 = %u\r\n", ITRLK_LOOP1_Get());
          break;

        // Read/print tool touchoff probe input
        case PRB_CMD: SYS_CONSOLE_PRINT("Probe = %u\r\n", PROBE_IN_Get()); break;

        // Set fan PWM output to given value
        case FAN_CMD:
          bounds_check_array[0]=0;
          bounds_check_array[1]=1;
          bounds_check_array[2]=0;
          bounds_check_array[3]=100;          
          if(is_input_valid(2, arg,bounds_check_array))
          { 
            // Enable the appropriate PWM output
            if (atoi(arg[1]) == 1) 
            {  // Flip polarity
              PWM1_ChannelsStart(PWM_CHANNEL_1_MASK);
              PWM1_ChannelDutySet(PWM_CHANNEL_1, FAN_PWM_MAX - ((FAN_PWM_MAX * atoi(arg[2])) / 100));
            } 
            else 
            {  // Flip polarity
              PWM1_ChannelsStart(PWM_CHANNEL_0_MASK);
              PWM1_ChannelDutySet(PWM_CHANNEL_0, FAN_PWM_MAX - ((FAN_PWM_MAX * atoi(arg[2])) / 100));
            }
          }
          break;

        // Configure given stepper motor on/off
        case MOT_CMD:
          bounds_check_array[0]=X_AXIS;
          bounds_check_array[1]=Z_AXIS;
          bounds_check_array[2]=0;
          bounds_check_array[3]=1;
          if(is_input_valid(2, arg, bounds_check_array))
          { 
            switch (atoi(arg[1])) 
            {
              case X_AXIS: SYS_CONSOLE_PRINT("Set X Stepper = %s\r\n", arg[2]); break;
              case Y1_AXIS: SYS_CONSOLE_PRINT("Set Y1 Stepper = %s\r\n", arg[2]); break;
              case Y2_AXIS: SYS_CONSOLE_PRINT("Set Y2 Stepper = %s\r\n", arg[2]); break;
              case Z_AXIS: SYS_CONSOLE_PRINT("Set Z Stepper = %s\r\n", arg[2]); break;
              
                default: 
                  break;
            }

            // Turn stepper on
            if (atoi(arg[2]) == 1) 
            {
              stepper_on(atoi(arg[1]));
            // Turn stepper off
            } 
            else 
            {
              stepper_off(atoi(arg[1]));
            }
          }
          break;

        // Read/print all limit switch (NC) inputs
        case LS_CMD:
          SYS_CONSOLE_PRINT("X Limit Switch SME = %u\r\n", X_ENDSTOP_SW_NC_Get2());           
          SYS_CONSOLE_PRINT("X Limit Switch SME2 = %u\r\n", X_ENDSTOP_SW_NC_Get3());           
          //SYS_CONSOLE_PRINT("X Limit Switch SME3 = %u\r\n", X_ENDSTOP_SW_NC_Get4());           

          SYS_CONSOLE_PRINT("X Limit Switch = %u\r\n", X_ENDSTOP_SW_NC_Get());
          SYS_CONSOLE_PRINT("Y Limit Switch = %u\r\n", Y_ENDSTOP_SW_NC_Get());
          SYS_CONSOLE_PRINT("Z Limit Switch = %u\r\n", Z_ENDSTOP_SW_NC_Get());
          SYS_CONSOLE_PRINT("A Limit Switch = %u\r\n", A_ENDSTOP_SW_NC_Get());
          SYS_CONSOLE_PRINT("B Limit Switch = %u\r\n", B_ENDSTOP_SW_NC_Get());
          break;

        // Turn the spindle PWM on/off
        case SPD_CMD:
          bounds_check_array[0]=0;
          bounds_check_array[1]=1;
          if(is_input_valid(1, arg, bounds_check_array)) 
          {
            SYS_CONSOLE_PRINT("Set Spindle PWM = %s\r\n", arg[1]);
            if (atoi(arg[1]) == 1) 
            {
              PWM0_ChannelsStart(PWM_CHANNEL_3_MASK);
              if (vary_the_duty_cycle_flag==true)
              {
                 PWM0_increment_spindle_speed();//PWM0_SetSpindleDutyCycle(spindle_pwm_duty_cycle_fraction);
                 //spindle_pwm_duty_cycle_fraction+=spindle_pwm_duty_cycle_increment;
              }             
            } 
            else 
            {
              PWM0_ChannelsStop(PWM_CHANNEL_3_MASK);
              //if (vary_the_duty_cycle_flag==true)
              //{             
              //  spindle_pwm_duty_cycle_increment *= -1.0;
              //}
            }
          }
          break;
          
        // Turn the ESC enable
        case ESC_CMD:
          bounds_check_array[0]=0;
          bounds_check_array[1]=1;
          if(is_input_valid(1, arg, bounds_check_array))
          {  
            SYS_CONSOLE_PRINT("Set ESC Enable = %s\r\n", arg[1]);
            if (atoi(arg[1]) == 1) 
            {
#if 1//sme investigate reset on ESC set:
              hal_write_pin(0,ESC_EN_PIN,(GPIO_PinState) 1);
#else
              ESC_EN_Set();//resets cpu on ESC set:
#endif              
            } 
            else 
            {
              ESC_EN_Clear();
            }
          }
          break;

        // Write spotlight RGBW value / run color loop test
        case SPOT_CMD:

          // Run color loop test
          if(strstr(arg[1], "test")) 
          {
               pwm_led_color_test(pwm_led_device_info[SPOT_LEDS]);

               // Write specific RGBW value to LEDs
          } 
          else
          {
                bounds_check_array[0]=0;
                bounds_check_array[1]=255;
                bounds_check_array[2]=0;
                bounds_check_array[3]=255;              
                bounds_check_array[4]=0;
                bounds_check_array[5]=255;
                bounds_check_array[6]=0;
                bounds_check_array[7]=255;
                
                if(is_input_valid(4, arg, bounds_check_array))
                { 
                    SYS_CONSOLE_PRINT("Set Spotlight LEDs = %s %s %s %s\r\n", arg[1], arg[2], arg[3], arg[4]);
                    pwm_led_set_all_RGBW(pwm_led_device_info[SPOT_LEDS], atoi(arg[1]), atoi(arg[2]), atoi(arg[3]), atoi(arg[4]));
                    pwm_led_render(pwm_led_device_info[SPOT_LEDS]);
                }
          }
          break;
          
        // Write status LED RGB value / run color loop test
        case STAT_CMD:
          // Run color loop test
          if(strstr(arg[1], "test")) 
          {
             pwm_led_color_test(pwm_led_device_info[STAT_LED]);

             // Write specific RGB value to LEDs
          } 
          else 
          {
                bounds_check_array[0]=0;
                bounds_check_array[1]=255;
                bounds_check_array[2]=0;
                bounds_check_array[3]=255;              
                bounds_check_array[4]=0;
                bounds_check_array[5]=255;
              
                if(is_input_valid(3, arg,bounds_check_array)) 
                {
                  SYS_CONSOLE_PRINT("Set Status LED = %s %s %s\r\n", arg[1], arg[2], arg[3]);
                  pwm_led_set_all_RGBW(pwm_led_device_info[STAT_LED], atoi(arg[1]), atoi(arg[2]), atoi(arg[3]), 0x00);
                  pwm_led_render(pwm_led_device_info[STAT_LED]);
                }
              }
          break;
          
        // Print out help menu
        case HELP_CMD:
        case HELP2_CMD:

          SYS_CONSOLE_PRINT("\r\n\r\nBring-Up FW Commands:\r\n\r\n");
          SYS_CONSOLE_PRINT("help         = help (or ?)\r\n");
          SYS_CONSOLE_PRINT("estp         = read/print e-stop\r\n");
          SYS_CONSOLE_PRINT("itr          = read/print all interlocks\r\n");
          SYS_CONSOLE_PRINT("prb          = read/print tool touchoff probe input\r\n");
          SYS_CONSOLE_PRINT("fan m n      = set FANm PWM to n percent [0-100]\r\n");
          SYS_CONSOLE_PRINT("mot m n      = set motor m [0-3 = X/Y1/Y2/Z] to n [0-1 = off/on]\r\n");
          SYS_CONSOLE_PRINT("ls           = read/print all limit switch inputs\r\n");
          SYS_CONSOLE_PRINT("spd m        = set the spindle PWM to m [0-1 = off/on]\r\n");
          SYS_CONSOLE_PRINT("esc m        = set the ESC enable to m [0-1 = off/on]\r\n");
          SYS_CONSOLE_PRINT("spot r g b w = set spotlight to RGBW value given [0-255]\r\n");
          SYS_CONSOLE_PRINT("spot test    = runs led color loop test on spotlight once\r\n");
          SYS_CONSOLE_PRINT("stat r g b   = set status LED to RGB value given [0-255]\r\n");
          SYS_CONSOLE_PRINT("stat test    = runs led color loop test on status LED once\r\n");
          SYS_CONSOLE_PRINT("\r\n");
          break;

        default: break;
      }
    }
  }

  // Reply with an error message when command not found
  if (!match_found_flag) 
  {
    SYS_CONSOLE_PRINT("ERROR: Unrecognized command\r\n");
  }
 
}

void APP_Initialize(void) 
{
  // Place the App state machine in its initial state
  appData.state = APP_STATE_INIT;
  
  // Place the USB CDC, UART and SD card subsystems in their initial states
  usb_cdc_init();
  sd_card_init();
}

void APP_Tasks(void) 
{
  static bool init_stepper_matt=true;
  // Check the application's current state
  switch ( appData.state )
  {
    // Application's initial state
    case APP_STATE_INIT:
    {
      bool appInitialized = true;
      
      // Add some settling time
      delay_ms(100);

      // Initialize GPIOs, UART, steppers, and LEDs
      gpio_init();
      uart_init();
   
      pwm_led_device_init();

      // Make sure all steppers are off
      for (int i = 0; i < STEPPER_NUM_AXES; i++) 
      {
        stepper_off(i);
      }

      // Print welcome message
      SYS_CONSOLE_PRINT("Welcome to the Chabo Bring-Up FW!\r\n");

      // Start servicing tasks after initialization
      if (appInitialized)
      {
          appData.state = APP_STATE_SERVICE_TASKS;
      }
      break;
    }

    case APP_STATE_SERVICE_TASKS:
    {
      uart_mgr_task();
      usb_cdc_task();   // USB CDC 
      sd_card_task();
      break;
    }

    // The default state should never be executed
    default:
    {
        // TODO: Handle error in application's state machine
        break;
    }
  }
}
