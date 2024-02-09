/******************************************************************
*
* File: sd_card_file_mgr.c
*
* @brief: 
*
******************************************************************/
#include "main.h"
#include "user_types.h"
#include "globdefs.h"
#include "system_bantam.h"//tim14 provides system time for velocity and rpm calcs
#include "app.h"
#include "tasks.h"
#include "sd_card_file_mgr.h"
#include "kernel.h"
#include "g2_persistence.h"
//extern void sdmmmc_task(void);
typedef enum
{
  NO_ERROR,
  F_MOUNT_FAILED,
  F_OPEN_FAILED,
  F_READ_FAILED,
  F_WRITE_FAILED,
  FILE_OP_FAIL_COUNT
}FileOpFailCodeE;

#define READFILE_BUFLEN 500  
FATFS myFatFS;
FIL myFile;
FIL myWriteFile;
FIL myReadFILE;

UINT myBytes;
char myReadFileDataBuf[READFILE_BUFLEN];
uint8_t myReadData[READFILE_BUFLEN];
UINT bytesRead;

char myWriteFileName[]="WRITE1.TXT";
char myReadFileName[]="READ1.TXT";

#define MODE_MOUNT_IMMEDIATELY 1
#define MODE_DELAYED_MOUNT 0
uint8_t rtext[READFILE_BUFLEN];  
char SDPath[4]={0,0,0,0};
extern void SYS_Tasks_restricted ( void );

/**************************************************************************
* 
* Function: sdCardTest
*
* Description:
*
**************************************************************************/
void sdCardTest(void)
{
   char myData[]="Hello from SD Card, Disk0, SDMMC in 4 byte mode with DMA";
   int len=strlen(myData);
   static int opt=MODE_MOUNT_IMMEDIATELY;
  //  static bool return_early_flag=true;
#define FILE_STATUS_DELAY 1000    
   
   if (f_mount(&myFatFS, SDPath, opt)==FR_OK)
   {                      
       if(f_open(&myFile, myWriteFileName, FA_WRITE|FA_CREATE_ALWAYS) == FR_OK)
       {    
          if (f_write(&myFile, myData, len, &myBytes) ==FR_OK)        
          {
            sys_blocking_delay_ms(FILE_STATUS_DELAY);
          }
          f_close(&myFile);
    }
    else
    {
       __NOP();//__no_operation();
    }
   
    if( f_open(&myReadFILE, myReadFileName, FA_READ | FA_OPEN_ALWAYS) == FR_OK) 
    {
      if (f_read(&myReadFILE, myReadFileDataBuf,sizeof(myReadFileDataBuf), &bytesRead)== FR_OK)
      {
          if (strcmp(myReadFileDataBuf,myData)!=0)
          {
             __NOP();//to do: fault handling     
          }
          f_close(&myReadFILE);  
      }
      else
      {
        __NOP();//__no_operation();
      }
    }
    else
    {
      __NOP();//__no_operation();
    }    
  }
 
}/* End function*/
/****************************************************************
 *
 * Function: FS_FileOperations
 * 
 * Description: Exercise reading/writing of SD Card
 *   
 ***************************************************************/
void FS_FileOperations(void) 
{
    static FATFS mySDFatFs;  /* File system object for SD card logical drive */
    static FIL MyFile;     /* File object */
    static bool create_always=true;
    static int mount_option=MODE_MOUNT_IMMEDIATELY;
   
  /* USER CODE END 2 */
    volatile int res;    /* FatFs function common result code */
    uint32_t byteswritten, bytesread, file_pos=0;   /* File write/read counts */
    uint8_t wtext[] = "\nThis is the sd card working with FatFs with SDMMC IN 4 BITS MODE\n\n"; /* File write buffer */
    uint8_t wtext2[] = "\nThis IS LINE 2\n\n"; /* File write buffer */
    uint8_t wtext3[] = "\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n\n"; /* File write buffer */
 
    /* Register the file system object to the FatFs module */
    res=f_mount(&mySDFatFs, (TCHAR const*)SDPath, mount_option);
    if(res == FR_OK)
    {
                    
       /* Create and Open a new text file object with write access */
       if(create_always==true)
       {
           res = f_open(&MyFile, "TEST1.TXT", FA_CREATE_ALWAYS | FA_WRITE|FA_READ); 
       }
       else
       {           
           res = f_open(&MyFile, "TEST1.txt",FA_READ|FA_WRITE);  
       }
       if(res == FR_OK)  //f_mount worked
       {     
          /* Read data from the text file */
          res = f_read(&MyFile, rtext, sizeof(rtext),(UINT*) &bytesread);
          file_pos= MyFile.fptr; 
          file_pos=f_lseek(&MyFile,file_pos); 

          /* Write data to the text file */
          res = f_write(&MyFile, wtext, sizeof(wtext),  (UINT*)&byteswritten);
          res = f_write(&MyFile, wtext2, sizeof(wtext2), (UINT*)&byteswritten); 
          res = f_write(&MyFile, wtext3, sizeof(wtext3), (UINT*)&byteswritten); 
                
          if((byteswritten > 0) && (res == FR_OK))
          {
            /* Close the open text file */
            f_close(&MyFile);
            
            /* Open the text file object with read access */
            if(f_open(&MyFile, "TEST1.TXT", FA_READ) == FR_OK)
            {
                /* Read data from the text file */
                res = f_read(&MyFile, rtext, sizeof(rtext),(UINT*)&bytesread);
              
                if((bytesread > 0) && (res == FR_OK))
                {
                  /* Close the open text file */
                  f_close(&MyFile);                                        
                }
                else
                {
                   res =F_READ_FAILED;//file read failed
                }
            }
            else
            {
                res = F_OPEN_FAILED;//f_open failed
            }
          }
          else
          {
              res = F_WRITE_FAILED; //f_write failed
          }
        }
        else
        {
            res =F_MOUNT_FAILED; //f_mount failed
        }
    }  
    if ( res !=0) //something failed
    {
        res = FILE_OP_FAIL_COUNT; 
    }

}



 /***************************************************************************
 * 
 * Function:  sd_card_file_mgr_init 
 * 
 * Description:   
 * 
 * Assumptions/Requirement: 
 ****************************************************************************/
void sd_card_file_mgr_init(void)
{
  
}
 /****************************************************************************
 * 
 * Function:  sd_card_file_mgr__task 
 * 
 * Description:   

 * Assumptions/Requirement: 

 *****************************************************************************/
void sd_card_file_mgr_task(void)
{
  static bool no_card_detected_flag=false;
  static bool uart5_already_inited_flag=false;
  static bool fs_mgr_sd_detected_flag=false;
  static bool drv_sdmmc_is_attached_flag=false;
  enum
  {
    SDCARD_TASK_INIT,
    SDCARD_TASK_FILE_OPS_TEST,
    SDCARD_TASK_PERSISTENCE,
    SDCARD_STATE_COUNT
  };
  static volatile int sdcard_state =  
     SDCARD_TASK_PERSISTENCE;
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_in();
#endif     

     fs_mgr_sd_detected_flag=SysFsSdCardInUse();  
     if ((fs_mgr_sd_detected_flag == true))
     {
       /* Apply the   */
       switch( sdcard_state )
       { 
           case SDCARD_TASK_INIT:
             //sdcard_state=SDCARD_TASK_FILE_OPS_TEST;           
             break;
              
           case SDCARD_TASK_FILE_OPS_TEST: 
             //sdCardTest();
             FS_FileOperations(); 
             sdcard_state= SDCARD_TASK_PERSISTENCE;//SDCARD_STATE_COUNT;
             break;
             
           case SDCARD_TASK_PERSISTENCE:
            write_persistent_values_callback();
            break;
            
           default:
               /* to do: raise error: corrupt data:*/
               break;
       }/* End Switch */
     
     } 
     else
     {
       if (no_card_detected_flag==false)//first entry it is false
       {
          no_card_detected_flag=true;              
       }
     }
#ifdef KERNEL_TASK_CHECKIN_CHECKOUT 
     kernel_task_check_out();
#endif        
}/* End Task */
